# ---------------------------- Home 000 ----------------------------
# 1. 采集轨迹s
cd /home/fufa/projects2026/SimDataGen
./tools/demo_data/run_record_camera_rig_trajectory.sh \
  --scene_usd /home/fufa/projects2026/SimDataGen/asset_extern/home_000/interior_template.usdc \
  --camera_usd /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
  --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/home_000 \
  --init_pose 1 1 1.5 0 0 0 \
  --occupancy-resolution 0.1 \
  --viewport-camera CAM_A \
  --viewport-cameras CAM_B CAM_C CAM_D CAM_Front CAM_Back

cd /home/fufa/projects2026/SimDataGen
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
python3 tools/demo_data/keyboard_camera_rig_teleop.py --step-size 0.1

# ---------------------------- TaoBao03 014 SkylineRestaurant ----------------------------

cd /home/fufa/projects2026/SimDataGen
./tools/demo_data/run_record_camera_rig_trajectory.sh \
  --scene_usd /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao03/014_SkylineRestaurant/SkylineRestaurant_P.usd \
  --camera_usd /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
  --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/SkylineRestaurant \
  --init_pose 17 8 1.5 0 0 0 \
  --occupancy-resolution 0.25 \
  --viewport-camera CAM_A \
  --viewport-cameras CAM_B CAM_C CAM_D CAM_Front CAM_Back

cd /home/fufa/projects2026/SimDataGen
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
python3 tools/demo_data/keyboard_camera_rig_teleop.py --step-size 0.25

# ---------------------------- TaoBao03 112 Bunker ----------------------------

cd /home/fufa/projects2026/SimDataGen
./tools/demo_data/run_record_camera_rig_trajectory.sh \
  --scene_usd /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao03/112_Bunker/Demonstration.usd \
  --camera_usd /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
  --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/112_Bunker \
  --init_pose 13 76 17.5 0 0 0 \
  --occupancy-resolution 0.5 \
  --viewport-camera CAM_A \
  --viewport-cameras CAM_B CAM_C CAM_D CAM_Front CAM_Back
  

cd /home/fufa/projects2026/SimDataGen
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
python3 tools/demo_data/keyboard_camera_rig_teleop.py --step-size 0.5

# ---------------------------- TaoBao04 102 AsianArch ----------------------------
1. 采集轨迹
cd /home/fufa/projects2026/SimDataGen
./tools/demo_data/run_record_camera_rig_trajectory.sh \
  --scene_usd /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao04/102_AsianArch/ExampleGathering.usd \
  --camera_usd /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
  --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/102_AsianArch \
  --init_pose 74 17 23.5 0 0 0 \
  --occupancy-resolution 1 \
  --viewport-camera CAM_A \
  --viewport-cameras CAM_B CAM_C CAM_D CAM_Front CAM_Back
  

cd /home/fufa/projects2026/SimDataGen
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
python3 tools/demo_data/keyboard_camera_rig_teleop.py --step-size 1

# ---------------------------- TaoBao11 AsianVillage ----------------------------
# 1. 采集轨迹
cd /home/fufa/projects2026/SimDataGen
./tools/demo_data/run_record_camera_rig_trajectory.sh \
  --scene_usd /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao11/AsianVillage/Asian_Village.usd \
  --camera_usd /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
  --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/AsianVillage \
  --init_pose 28 "-106" "-33.5" 0 0 90 \
  --occupancy-resolution 1 \
  --viewport-camera CAM_A \
  --viewport-cameras CAM_B CAM_C CAM_D CAM_Front CAM_Back
  

cd /home/fufa/projects2026/SimDataGen
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
python3 tools/demo_data/keyboard_camera_rig_teleop.py --step-size 1


# ---------------------------- TaoBao11 ForestHourse ----------------------------
1. 采集轨迹
cd /home/fufa/projects2026/SimDataGen
./tools/demo_data/run_record_camera_rig_trajectory.sh \
  --scene_usd /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao11/ForestHourse/Map_Houses_A.usd \
  --camera_usd /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
  --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/ForestHourse \
  --init_pose 39 "145" "-18" 0 0 0 \
  --occupancy-resolution 1 \
  --viewport-camera CAM_A \
  --viewport-cameras CAM_B CAM_C CAM_D CAM_Front CAM_Back
  

cd /home/fufa/projects2026/SimDataGen
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
python3 tools/demo_data/keyboard_camera_rig_teleop.py --step-size 1

# ---------------------------- TaoBao11 RuinedCrypt ----------------------------
1. 采集轨迹
cd /home/fufa/projects2026/SimDataGen
./tools/demo_data/run_record_camera_rig_trajectory.sh \
  --scene_usd /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao11/RuinedCrypt/RuinedCrypt_01_P.usd \
  --camera_usd /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
  --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/RuinedCrypt \
  --init_pose "-9.9" "-4.8" "1" 0 0 0 \
  --occupancy-resolution 0.5 \
  --viewport-camera CAM_A \
  --viewport-cameras CAM_B CAM_C CAM_D CAM_Front CAM_Back
  

cd /home/fufa/projects2026/SimDataGen
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
python3 tools/demo_data/keyboard_camera_rig_teleop.py --step-size 0.5


#   ---------------------------- TaoBao11 temple ----------------------------
1. 采集轨迹
cd /home/fufa/projects2026/SimDataGen
./tools/demo_data/run_record_camera_rig_trajectory.sh \
  --scene_usd /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao11/temple/Demonstration.usd \
  --camera_usd /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
  --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/temple \
  --init_pose "19" "-3.4" "-16" 0 0 0 \
  --occupancy-resolution 1 \
  --viewport-camera CAM_A \
  --viewport-cameras CAM_B CAM_C CAM_D CAM_Front CAM_Back
  

cd /home/fufa/projects2026/SimDataGen
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
python3 tools/demo_data/keyboard_camera_rig_teleop.py --step-size 0.5