# 2. 生成数据
./tools/demo_data/run_gen_data_from_trajectory.sh \
 --scene_usd_url /home/fufa/projects2026/SimDataGen/extern_asserts/Intime/home_000/interior_template.usdc \
 --camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA_regular_dog.usd \
 --trajectory_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/home_000_dog \
 --trajectory_tags 1 \
 --output_dir /home/fufa/projects2026/SimDataGen/workdir/tractory_data/home_000_dog \
 --point_stride 1

 ./tools/demo_data/run_gen_data_from_trajectory.sh \
 --scene_usd_url /home/fufa/projects2026/SimDataGen/extern_asserts/Intime/home_000/interior_template.usdc \
 --camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA_regular_human.usd \
 --trajectory_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/home_000_human \
 --trajectory_tags 3 \
 --output_dir /home/fufa/projects2026/SimDataGen/workdir/tractory_data/home_000_human \
 --point_stride 1

 ./tools/demo_data/run_gen_data_from_trajectory.sh \
 --scene_usd_url /home/fufa/projects2026/SimDataGen/extern_asserts/TaoBao03/112_Bunker/Demonstration.usd \
 --camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA_regular_dog.usd \
 --trajectory_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/112_Bunker_dog \
 --trajectory_tags 0 \
 --output_dir /home/fufa/projects2026/SimDataGen/workdir/tractory_data/112_Bunker_dog \
 --point_stride 1

  ./tools/demo_data/run_gen_data_from_trajectory.sh \
 --scene_usd_url /home/fufa/projects2026/SimDataGen/extern_asserts/TaoBao03/014_SkylineRestaurant/SkylineRestaurant_P.usd \
 --camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA_regular_human.usd \
 --trajectory_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/SkylineRestaurant_human \
 --trajectory_tags 0 \
 --output_dir /home/fufa/projects2026/SimDataGen/workdir/tractory_data/SkylineRestaurant_human \
 --point_stride 1

# ./tools/demo_data/run_gen_data_from_trajectory.sh \
#   --scene_usd_url /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao04/102_AsianArch/ExampleGathering.usd \
#   --camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
#   --trajectory_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/102_AsianArch \
#   --trajectory_tags 1 \
#   --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/102_AsianArch \
#   --point_stride 1
  
# ./tools/demo_data/run_gen_data_from_trajectory.sh \
#   --scene_usd_url /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao03/112_Bunker/Demonstration.usd \
#   --camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
#   --trajectory_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/112_Bunker \
#   --trajectory_tags 0 \
#   --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/112_Bunker \
#   --point_stride 1
 
# ./tools/demo_data/run_gen_data_from_trajectory.sh \
#   --scene_usd_url /home/fufa/projects2026/SimDataGen/asset_extern/kujiale/kujiale_0004/kujiale_0004.usda \
#   --camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
#   --trajectory_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/kujiale_0004 \
#   --trajectory_tags 0 \
#   --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/kujiale_0004 \
#   --point_stride 1
 
# ./tools/demo_data/run_gen_data_from_trajectory.sh \
#   --scene_usd_url /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao03/014_SkylineRestaurant/SkylineRestaurant_P.usd \
#   --camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
#   --trajectory_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/SkylineRestaurant \
#   --trajectory_tags 0 \
#   --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/SkylineRestaurant \
#   --point_stride 1
  
#  ./tools/demo_data/run_gen_data_from_trajectory.sh \
#   --scene_usd_url /home/fufa/projects2026/SimDataGen/asset_extern/Intime/home_009/home_009.usdc \
#   --camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
#   --trajectory_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/home_009 \
#   --trajectory_tags 0 \
#   --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/home_009 \
#   --point_stride 1
  
#  ./tools/demo_data/run_gen_data_from_trajectory.sh \
#   --scene_usd_url /home/fufa/projects2026/SimDataGen/asset_extern/kujiale/kujiale_0030/kujiale_0030.usda \
#   --camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
#   --trajectory_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/kujiale_0030 \
#   --trajectory_tags 0 \
#   --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/kujiale_0030 \
  # --point_stride 1

#  ./tools/demo_data/run_gen_data_from_trajectory.sh \
#   --scene_usd_url /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao11/AsianVillage/Asian_Village.usd \
#   --camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
#   --trajectory_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/AsianVillage \
#   --trajectory_tags 1 \
#   --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/AsianVillage \
#   --point_stride 1

#  ./tools/demo_data/run_gen_data_from_trajectory.sh \
#   --scene_usd_url /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao11/ForestHourse/Map_Houses_A.usd \
#   --camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
#   --trajectory_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/ForestHourse \
#   --trajectory_tags 0 \
#   --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/ForestHourse \
#   --point_stride 1

# ./tools/demo_data/run_gen_data_from_trajectory.sh \
#   --scene_usd_url /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao11/RuinedCrypt/RuinedCrypt_01_P.usd \
#   --camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
#   --trajectory_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/RuinedCrypt \
#   --trajectory_tags 0 \
#   --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/RuinedCrypt \
#   --point_stride 1

# ./tools/demo_data/run_gen_data_from_trajectory.sh \
#   --scene_usd_url /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao11/temple/Demonstration.usd \
#   --camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
#   --trajectory_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory/temple \
#   --trajectory_tags 1 \
#   --output_dir /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/temple \
#   --point_stride 1

 # 3. 验证
# ./scripts/batch_vis_to_mcap.sh /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/home_000
# ./app/python.sh tools/check_data/make_rgb_depth_video.py  --input workdir/trajectory_data/home_000/

# ./scripts/batch_vis_to_mcap.sh /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/102_AsianArch
# ./app/python.sh tools/check_data/make_rgb_depth_video.py  --input workdir/trajectory_data/102_AsianArch/

# ./scripts/batch_vis_to_mcap.sh /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/112_Bunker
# ./app/python.sh tools/check_data/make_rgb_depth_video.py  --input workdir/trajectory_data/112_Bunker/

# ./scripts/batch_vis_to_mcap.sh /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/kujiale_0004
# ./app/python.sh tools/check_data/make_rgb_depth_video.py  --input workdir/trajectory_data/kujiale_0004/

# ./scripts/batch_vis_to_mcap.sh /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/SkylineRestaurant
# ./app/python.sh tools/check_data/make_rgb_depth_video.py  --input workdir/trajectory_data/SkylineRestaurant/

# ./scripts/batch_vis_to_mcap.sh /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/home_009
# ./app/python.sh tools/check_data/make_rgb_depth_video.py  --input workdir/trajectory_data/home_009/

# ./scripts/batch_vis_to_mcap.sh /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/kujiale_0030
# ./app/python.sh tools/check_data/make_rgb_depth_video.py  --input workdir/trajectory_data/kujiale_0030/

# ./scripts/batch_vis_to_mcap.sh /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/temple
# ./app/python.sh tools/check_data/make_rgb_depth_video.py  --input workdir/trajectory_data/temple/

# ./scripts/batch_vis_to_mcap.sh /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/ForestHourse
# ./app/python.sh tools/check_data/make_rgb_depth_video.py  --input workdir/trajectory_data/ForestHourse/

# ./scripts/batch_vis_to_mcap.sh /home/fufa/projects2026/SimDataGen/workdir/trajectory_data/RuinedCrypt
# ./app/python.sh tools/check_data/make_rgb_depth_video.py  --input workdir/trajectory_data/RuinedCrypt/