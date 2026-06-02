"""
OAK-4P-New-B036801 Mei Omni Camera Model -> Isaac Sim Generalized Projection LUT Generator

Reads Kalibr calibration data (camera_model: omni) from fisheye_cams.yaml,
generates rayEnterDirectionTexture and rayExitPositionTexture EXR files
for use with OmniLensDistortionLutAPI in NVIDIA Isaac Sim.

Mei Unified Omnidirectional Model:
  1. Unit sphere projection:  Xs = P / ||P||
  2. Xi shift:                Xs' = Xs + [0, 0, xi]^T
  3. Perspective projection:  m = [Xs'_x / Xs'_z, Xs'_y / Xs'_z]^T
  4. Radtan distortion + intrinsics -> pixel
"""

import argparse
import os
import yaml
import numpy as np
import OpenEXR
import Imath


# ============================================================
# EXR I/O
# ============================================================

def save_exr(path: str, img_data: np.ndarray):
    """Save a float32 numpy array (H, W, C) as 32-bit float EXR."""
    if img_data.ndim != 3:
        raise ValueError("img_data must be 3D: (H, W, C)")

    h, w, c = img_data.shape
    channel_names = ['R', 'G', 'B', 'A'][:c]

    channel_map = {}
    channel_types = {}
    for i, name in enumerate(channel_names):
        channel_map[name] = img_data[:, :, i].astype(np.float32).tobytes()
        channel_types[name] = Imath.Channel(Imath.PixelType(Imath.PixelType.FLOAT))

    header = OpenEXR.Header(w, h)
    header['channels'] = channel_types
    header['compression'] = Imath.Compression(Imath.Compression.ZIP_COMPRESSION)

    out = OpenEXR.OutputFile(path, header)
    out.writePixels(channel_map)
    out.close()
    print(f"  Saved: {path}  ({w}x{h}, {c}ch)")


# ============================================================
# Octahedral Encoding / Decoding  (matches NVIDIA's convention)
# ============================================================

def oct_to_unit_vector(x, y):
    """Decode octahedral 2D coords ([-1,1]^2) to unit direction vectors."""
    dirX, dirY = np.meshgrid(x, y)
    dirZ = 1.0 - np.abs(dirX) - np.abs(dirY)

    sx = 2.0 * np.heaviside(dirX, 1.0) - 1.0
    sy = 2.0 * np.heaviside(dirY, 1.0) - 1.0

    tmpX = dirX.copy()
    dirX = np.where(dirZ <= 0, (1.0 - np.abs(dirY)) * sx, dirX)
    dirY = np.where(dirZ <= 0, (1.0 - np.abs(tmpX)) * sy, dirY)

    norm = 1.0 / np.sqrt(dirX**2 + dirY**2 + dirZ**2)
    dirX *= norm
    dirY *= norm
    dirZ *= norm
    return dirX, dirY, dirZ


def get_octahedral_directions(width, height):
    """Generate octahedral direction grid for a texture of given size."""
    x = np.linspace(-1, 1, width)
    y = np.linspace(-1, 1, height)
    return oct_to_unit_vector(x, y)


# ============================================================
# Mei Omni Model: Forward projection  (3D point -> pixel)
# ============================================================

def mei_project(points_3d_x, points_3d_y, points_3d_z,
                xi, fx, fy, cx, cy, dist_coeffs):
    """
    Forward Mei omni projection: 3D direction -> 2D pixel (NDC).

    Args:
        points_3d_{x,y,z}: arrays of 3D direction components
        xi: Mei mirror parameter
        fx, fy: focal lengths (pixels)
        cx, cy: principal point (pixels, absolute)
        dist_coeffs: [k1, k2, p1, p2] radtan distortion

    Returns:
        u_ndc, v_ndc: normalized device coordinates in [0,1]
        valid_mask: boolean mask for valid projections
    """
    k1, k2, p1, p2 = dist_coeffs

    # Step 1: normalize to unit sphere
    norm = np.sqrt(points_3d_x**2 + points_3d_y**2 + points_3d_z**2)
    norm = np.maximum(norm, 1e-12)
    xs = points_3d_x / norm
    ys = points_3d_y / norm
    zs = points_3d_z / norm

    # Step 2: xi shift along z-axis
    zs_shifted = zs + xi

    # Valid: only points in front of the shifted projection center
    valid_mask = zs_shifted > 1e-8

    # Step 3: perspective projection to normalized plane
    mx = np.where(valid_mask, xs / zs_shifted, 0.0)
    my = np.where(valid_mask, ys / zs_shifted, 0.0)

    # Step 4: radtan distortion
    r2 = mx**2 + my**2
    radial = 1.0 + k1 * r2 + k2 * r2**2
    mx_d = mx * radial + 2.0 * p1 * mx * my + p2 * (r2 + 2.0 * mx**2)
    my_d = my * radial + p1 * (r2 + 2.0 * my**2) + 2.0 * p2 * mx * my

    # Step 5: intrinsics -> pixel coordinates
    u_px = fx * mx_d + cx
    v_px = fy * my_d + cy

    # Convert to NDC [0, 1]
    # width and height will be applied by caller
    return u_px, v_px, valid_mask


def mei_project_to_ndc(dirX, dirY, dirZ, width, height,
                       xi, fx, fy, cx, cy, dist_coeffs):
    """Forward projection returning NDC coords and valid mask."""
    u_px, v_px, valid = mei_project(dirX, dirY, dirZ,
                                    xi, fx, fy, cx, cy, dist_coeffs)
    u_ndc = u_px / width
    v_ndc = v_px / height

    in_bounds = valid & (u_ndc >= 0) & (u_ndc <= 1) & (v_ndc >= 0) & (v_ndc <= 1)

    u_ndc = np.where(in_bounds, u_ndc, -1.0)
    v_ndc = np.where(in_bounds, v_ndc, -1.0)
    return u_ndc, v_ndc


# ============================================================
# Mei Omni Model: Inverse projection  (pixel -> 3D ray direction)
# ============================================================

def mei_unproject(u_ndc, v_ndc, width, height,
                  xi, fx, fy, cx, cy, dist_coeffs,
                  undistort_iters=50):
    """
    Inverse Mei omni projection: NDC pixel -> 3D ray direction.

    Uses iterative undistortion (Newton-like fixed point iteration)
    to remove radtan distortion, then analytically inverts the
    xi-shifted perspective projection.

    Returns:
        dirX, dirY, dirZ: normalized 3D direction arrays
    """
    k1, k2, p1, p2 = dist_coeffs

    # NDC [0,1] -> pixel coordinates
    u_px = u_ndc * width
    v_px = v_ndc * height

    # Inverse intrinsics: pixel -> normalized distorted plane
    mx_d = (u_px - cx) / fx
    my_d = (v_px - cy) / fy

    # Iterative undistortion (fixed-point iteration)
    mx = mx_d.copy()
    my = my_d.copy()
    for _ in range(undistort_iters):
        r2 = mx**2 + my**2
        radial = 1.0 + k1 * r2 + k2 * r2**2
        dx_tang = 2.0 * p1 * mx * my + p2 * (r2 + 2.0 * mx**2)
        dy_tang = p1 * (r2 + 2.0 * my**2) + 2.0 * p2 * mx * my
        mx = (mx_d - dx_tang) / radial
        my = (my_d - dy_tang) / radial

    # Inverse Mei: from normalized plane (mx, my) back to unit sphere point
    #
    # Forward: mx = xs/(zs+xi), my = ys/(zs+xi), with xs^2+ys^2+zs^2=1
    # Let lambda = zs + xi, then xs = lambda*mx, ys = lambda*my, zs = lambda - xi
    # Substituting into xs^2+ys^2+zs^2=1:
    #   lambda^2*(r2+1) - 2*xi*lambda + xi^2 - 1 = 0
    # Or equivalently solving for zs directly:
    #   (r2+1)*zs^2 + 2*xi*r2*zs + xi^2*r2 - 1 = 0

    r2_undist = mx**2 + my**2

    a = r2_undist + 1.0
    b = 2.0 * xi * r2_undist
    c = xi**2 * r2_undist - 1.0
    discriminant = b**2 - 4.0 * a * c

    # For xi > 1, pixels beyond r2 > 1/(xi^2-1) have negative discriminant
    # and fall outside the valid Mei projection domain
    valid = discriminant > 0

    discriminant = np.maximum(discriminant, 0.0)
    zs = (-b + np.sqrt(discriminant)) / (2.0 * a)

    scale = zs + xi
    dirX = mx * scale
    dirY = my * scale
    dirZ = zs

    # Normalize to unit vector
    norm = np.sqrt(dirX**2 + dirY**2 + dirZ**2)
    norm = np.maximum(norm, 1e-12)
    dirX /= norm
    dirY /= norm
    dirZ /= norm

    # Invalid pixels: zero vector signals "no ray" to the RTX renderer,
    # which produces clean black instead of sampling a stray scene direction.
    dirX = np.where(valid, dirX, 0.0)
    dirY = np.where(valid, dirY, 0.0)
    dirZ = np.where(valid, dirZ, 0.0)

    return dirX, dirY, dirZ


# ============================================================
# Texture Generation
# ============================================================

def generate_enter_direction_texture(tex_w, tex_h, img_w, img_h,
                                     xi, fx, fy, cx, cy, dist_coeffs):
    """
    Generate rayEnterDirectionTexture (NDC -> 3D ray direction).
    Texture UV = NDC, RGB = normalized direction vector.
    """
    u_ndc = np.linspace(0, 1, tex_w)
    v_ndc = np.linspace(0, 1, tex_h)
    U, V = np.meshgrid(u_ndc, v_ndc)

    dirX, dirY, dirZ = mei_unproject(U, V, img_w, img_h,
                                     xi, fx, fy, cx, cy, dist_coeffs)

    # OpenCV camera: X=right, Y=down, Z=forward
    # RTX camera:    X=right, Y=up,   Z=backward
    dirY = -dirY
    dirZ = -dirZ

    img_data = np.stack([dirX, dirY, dirZ], axis=-1).astype(np.float32)
    return img_data


def generate_exit_position_texture(tex_w, tex_h, img_w, img_h,
                                   xi, fx, fy, cx, cy, dist_coeffs):
    """
    Generate rayExitPositionTexture (3D direction -> NDC via octahedral encoding).
    Texture UV = octahedral encoded direction, RG = NDC position.
    """
    dirX, dirY, dirZ = get_octahedral_directions(tex_w, tex_h)

    # RTX camera: X=right, Y=up, Z=backward
    # OpenCV camera: X=right, Y=down, Z=forward
    # Inverse transform: RTX(X, Y, Z) -> OpenCV(X, -Y, -Z)
    dirX_cv = dirX
    dirY_cv = -dirY
    dirZ_cv = -dirZ

    u_ndc, v_ndc = mei_project_to_ndc(dirX_cv, dirY_cv, dirZ_cv,
                                       img_w, img_h,
                                       xi, fx, fy, cx, cy, dist_coeffs)

    img_data = np.stack([u_ndc, v_ndc], axis=-1).astype(np.float32)
    return img_data


# ============================================================
# Main: Read calibration and generate textures for all cameras
# ============================================================

def load_calibration(yaml_path):
    """Load Kalibr omni calibration from fisheye_cams.yaml."""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    cameras = {}
    for cam_key in sorted(data.keys()):
        cam = data[cam_key]
        if cam.get('camera_model') != 'omni':
            print(f"  Skipping {cam_key}: camera_model={cam.get('camera_model')}, expected 'omni'")
            continue

        intrinsics = cam['intrinsics']
        # Kalibr omni intrinsics format: [xi, fx, fy, cx, cy]
        cameras[cam_key] = {
            'name': cam.get('rostopic', cam_key),
            'xi': intrinsics[0],
            'fx': intrinsics[1],
            'fy': intrinsics[2],
            'cx': intrinsics[3],
            'cy': intrinsics[4],
            'dist_coeffs': cam['distortion_coeffs'],
            'width': cam['resolution'][0],
            'height': cam['resolution'][1],
        }
    return cameras


def generate_camera_lut(cam_name, cam_params, output_dir, tex_scale=2):
    """Generate LUT EXR texture pair for one camera."""
    print(f"\n{'='*60}")
    print(f"Generating LUT for: {cam_name}")
    print(f"  Resolution: {cam_params['width']}x{cam_params['height']}")
    print(f"  xi={cam_params['xi']:.6f}")
    print(f"  fx={cam_params['fx']:.4f}, fy={cam_params['fy']:.4f}")
    print(f"  cx={cam_params['cx']:.4f}, cy={cam_params['cy']:.4f}")
    print(f"  distortion (radtan): {cam_params['dist_coeffs']}")
    print(f"{'='*60}")

    img_w = cam_params['width']
    img_h = cam_params['height']
    xi = cam_params['xi']
    fx = cam_params['fx']
    fy = cam_params['fy']
    cx = cam_params['cx']
    cy = cam_params['cy']
    dist = cam_params['dist_coeffs']

    # Texture resolution: at least sensor resolution, scaled up for sub-pixel accuracy
    enter_w = img_w * tex_scale
    enter_h = img_h * tex_scale
    # Exit (octahedral) texture: square, high-res for angular precision
    exit_size = max(img_w, img_h) * tex_scale

    # --- rayEnterDirectionTexture ---
    print(f"  Generating rayEnterDirectionTexture ({enter_w}x{enter_h})...")
    enter_data = generate_enter_direction_texture(
        enter_w, enter_h, img_w, img_h, xi, fx, fy, cx, cy, dist)

    enter_path = os.path.join(output_dir, f"{cam_name}_rayEnterDirection.exr")
    save_exr(enter_path, enter_data)

    # --- rayExitPositionTexture ---
    print(f"  Generating rayExitPositionTexture ({exit_size}x{exit_size})...")
    exit_data = generate_exit_position_texture(
        exit_size, exit_size, img_w, img_h, xi, fx, fy, cx, cy, dist)

    exit_path = os.path.join(output_dir, f"{cam_name}_rayExitPosition.exr")
    save_exr(exit_path, exit_data)

    # --- Suggested maskRadius from generated rayEnter EXR ---
    try:
        from compute_mask_radius import compute_mask_from_enter_exr

        r_mask = int(round(compute_mask_from_enter_exr(enter_path, center_x=cx, center_y=cy)))
        print(
            f"  suggested maskRadius: {r_mask}px @ calibration ({cx:.1f}, {cy:.1f}), "
            f"renderable dirZ<0.2 "
            f"(bake: --mask_radius {cam_name}={r_mask} --mask_center calibration)"
        )
    except Exception as e:
        print(f"  (maskRadius hint skipped: {e})")

    # --- Print Isaac Sim configuration ---
    print(f"\n  === Isaac Sim OmniLensDistortionLutAPI Configuration ===")
    print(f"  omni:lensdistortion:model = \"lut\"")
    print(f"  omni:lensdistortion:lut:nominalWidth  = {img_w}.0")
    print(f"  omni:lensdistortion:lut:nominalHeight = {img_h}.0")
    print(f"  omni:lensdistortion:lut:opticalCenter = ({img_w/2.0}, {img_h/2.0})")
    print(f"    (set to geometric center because cx/cy is baked into EXR)")
    print(f"  omni:lensdistortion:lut:rayEnterDirectionTexture = \"{enter_path}\"")
    print(f"  omni:lensdistortion:lut:rayExitPositionTexture   = \"{exit_path}\"")

    return enter_path, exit_path


def verify_roundtrip(cam_params, num_samples=1000):
    """Verify forward->inverse and inverse->forward consistency."""
    img_w = cam_params['width']
    img_h = cam_params['height']
    xi = cam_params['xi']
    fx, fy = cam_params['fx'], cam_params['fy']
    cx, cy = cam_params['cx'], cam_params['cy']
    dist = cam_params['dist_coeffs']

    # Sample random pixel positions
    np.random.seed(42)
    u_ndc = np.random.uniform(0.05, 0.95, num_samples)
    v_ndc = np.random.uniform(0.05, 0.95, num_samples)

    # Pixel -> 3D direction
    dX, dY, dZ = mei_unproject(u_ndc, v_ndc, img_w, img_h,
                                xi, fx, fy, cx, cy, dist)

    # Filter out pixels that fell outside the valid Mei projection domain
    # (unproject returns (0,0,1) for invalid pixels)
    valid_unproject = ~((dX == 0) & (dY == 0) & (dZ == 1))

    # 3D direction -> pixel
    u_px, v_px, valid_proj = mei_project(dX, dY, dZ, xi, fx, fy, cx, cy, dist)
    u_ndc2 = u_px / img_w
    v_ndc2 = v_px / img_h

    valid = valid_unproject & valid_proj

    u_err = np.abs(u_ndc[valid] - u_ndc2[valid]) * img_w
    v_err = np.abs(v_ndc[valid] - v_ndc2[valid]) * img_h
    total_err = np.sqrt(u_err**2 + v_err**2)

    n_invalid = num_samples - np.sum(valid)
    print(f"  Roundtrip verification ({np.sum(valid)}/{num_samples} valid, {n_invalid} outside Mei FOV):")
    print(f"    Mean reprojection error: {np.mean(total_err):.6f} px")
    print(f"    Max reprojection error:  {np.max(total_err):.6f} px")
    print(f"    Median reprojection error: {np.median(total_err):.6f} px")
    return np.mean(total_err)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--yaml",
        default="docs/oak_camera/calibration/fisheye_cams.yaml",
        help="Kalibr omni calibration yaml path",
    )
    parser.add_argument(
        "--output_dir",
        default="assets/cameras/oak_camera_texture",
        help="Output directory for generated EXR textures",
    )
    args = parser.parse_args()

    yaml_path = args.yaml
    output_dir = args.output_dir

    os.makedirs(output_dir, exist_ok=True)

    print("Loading calibration from:", yaml_path)
    cameras = load_calibration(yaml_path)
    print(f"Found {len(cameras)} omni cameras: {list(cameras.keys())}")

    cam_name_map = {
        'cam0': 'CAM_A',
        'cam1': 'CAM_B',
        'cam2': 'CAM_C',
        'cam3': 'CAM_D',
    }

    for cam_key, cam_params in cameras.items():
        cam_name = cam_name_map.get(cam_key, cam_params['name'])

        # Roundtrip verification first
        verify_roundtrip(cam_params)

        generate_camera_lut(cam_name, cam_params, output_dir, tex_scale=1)

    print(f"\n{'='*60}")
    print("All textures generated successfully!")
    print(f"Output directory: {os.path.abspath(output_dir)}")
    print(f"\nNext steps in Isaac Sim:")
    print(f"  1. Apply OmniLensDistortionLutAPI to your camera prim")
    print(f"  2. Set model to 'lut'")
    print(f"  3. Load the generated EXR textures")
    print(f"  4. Set opticalCenter to geometric center (width/2, height/2)")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
