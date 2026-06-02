#!/usr/bin/env bash
# 按 task 批量提交火山任务（多 task，多 seed）。
# 用法:
#   export VOLC_AK VOLC_SK VOLC_PASSWD
#   $0
#
# 说明:
# - 在 TASKS=(...) 里通过注释/取消注释选择要跑的 task。
# - 在 for 循环中的一条命令里统一维护提交参数。
#
# 示例:
#   $0

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
# echo "PROJECT_DIR($0): ${PROJECT_DIR}"
SCRIPT_DIR="${PROJECT_DIR}/scripts/scripts"
# echo "SCRIPT_DIR($0): ${SCRIPT_DIR}"
ONE_TASK_SCRIPT="${PROJECT_DIR}/scripts/submit_all_seed_one_task.sh"
# echo "ONE_TASK_SCRIPT($0): ${ONE_TASK_SCRIPT}"
CAMERA_NAME=${1:-"oak_camera_4lut_2H30YA"}
# echo "CAMERA_NAME($0): ${CAMERA_NAME}"

if [[ ! -x "${ONE_TASK_SCRIPT}" ]]; then
  echo "错误: 未找到可执行脚本 ${ONE_TASK_SCRIPT}" >&2
  exit 1
fi

# 每项为 task_name，对应脚本为 ${SCRIPT_DIR}/${task}.sh
TASKS=(
  # # Intime_Home场景
  # intime_factory_000  # OKK
  # intime_home_000     # OKK
  # intime_home_001     # OKK
  # intime_home_002     # OKK
  # intime_home_003     # OKK
  # intime_home_004     # OKK
  # intime_home_005     # OKK
  # intime_home_006     # OKK
  # intime_home_007     # OKK
  # intime_home_008     # OKK
  # intime_home_009     # OKK
  # intime_home_010     # OKK

  # # Taobao场景
  # taobao_AI_vol33_scene_04    # OKK
  # taobao_AIUE_V01_002         # OKK
  # taobao_AIUE_V01_004         # OKK
  # taobao_AIUE_V01_005         # OKK
  # taobao_AIUE_V03_001         # OKK
  # taobao_AIUE_V03_002         # OKK
  # taobao_ModularSwimmingPool  # OKK
  # taobao_NewScandinavian      # OKK
  # taobao_NightClub            # OKK
  # taobao_OfficeMeetingRoom2   # OKK
  # taobao_Old_Laboratory2      # OKK
  # taobao_ParkingGarage        # OKK
  # taobao_PostSovietFlat2      # OKK
  # taobao_PostSovietKitchen    # OKK
  # taobao_QAModularParking     # OKK
  # taobao_QAOffice             # OKK
  # taobao_QAPolice             # OKK
  # taobao_ResearchCenter       # OKK
  # taobao_RetroOffice          # OKK
  # taobao_SchoolGym            # OKK
  # taobao_ShootingRange        # OKK
  # taobao_ShowRoom             # OKK
  # taobao_StylizedRoom         # OKK
  # taobao_VictorianLivingRoom  # OKK

  # # TaoBao02场景
  # taobao02_AI_vol3_scene_01   # OKK
  # taobao02_AI_vol3_scene_03   # OKK
  # taobao02_AI_vol3_scene_04   # OKK
  # taobao02_AI_vol4_01         # OKK
  # taobao02_AI_vol4_02         # OKK
  # taobao02_AI_vol4_03         # OKK
  # taobao02_AI_vol4_04         # OKK
  # taobao02_AI_vol4_05         # OKK
  # taobao02_AIUE_V01_001       # OKK
  # taobao02_AIUE_V01_003       # OKK
  # taobao02_AIUE_V02_001       # OKK
  # taobao02_AIUE_V02_002       # OKK
  # taobao02_AIUE_V02_003       # OKK
  # taobao02_B110_House_Furniture                # OKK

  # # TaoBao03场景
  # taobao03_001_ModernHomeHouseInteriorw        # OKK
  # taobao03_002_SG_Condominium                  # OKK
  # taobao03_004_TeaHourse                       # OKK
  # taobao03_006_OfficeCorridorEnvironment       # OKK
  # taobao03_009_OvalOffice                      # OKK
  # taobao03_010_ModernOffice                    # OKK
  # taobao03_012_CityStairwellandAtticScene      # OKK
  # taobao03_013_SwimmingPool                    # OKK
  # taobao03_013_SwimmingPool02                  # OKK
  # taobao03_013_SwimmingPool03                  # OKK
  # taobao03_013_SwimmingPool04                  # OKK
  # taobao03_014_SkylineRestaurant               # OKK
  # taobao03_016_Warehouse                       # OKK
  # taobao03_017_SchoolLab                       # OKK
  # taobao03_017_SchoolLab02                     # OKK
  # taobao03_018_Fluorescence                    # OKK
  # taobao03_106_Showroom                        # OKK
  # taobao03_108_Bazaar                          # OKK
  # taobao03_110_UrbanProps                      # OKK
  # taobao03_111_Obsidium02                      # OKK
  # taobao03_111_Obsidium04                      # OKK
  # taobao03_111_Obsidium05                      # OKK
  # taobao03_112_Bunker                          # OKK
  # taobao03_115_C065LoftOffice_workspace        # OKK
  # taobao03_117_VictorianInterior               # OKK

  # # TaoBao04场景
  # taobao04_021_OperaHall                       # OKK
  # taobao04_027_FuelBar                         # OKK
  # taobao04_028_Stationer                       # OKK
  # taobao04_029_ShoppingMall                    # OKK
  # taobao04_037_ModularUrban                    # OKK
  # taobao04_101_GaragePack                      # OKK
  # taobao04_102_AsianArch                       # OKK
  # taobao04_107_Airport                         # OKK

  # # TaoBao05场景
  # taobao05_150_TV_Sport_Studio                 # OKK
  # taobao05_152_NewsStudio                      # OKK
  # taobao05_154_TVStudio                        # OKK
  # taobao05_155_Studio7                         # OKK
  # taobao05_156_BasketballHeavenStudio          # OKK
  # taobao05_157_CoffeeBakery                    # OKK
  # taobao05_159_ArchinteriorsV2S1ue53           # OKK
  # taobao05_160_Archinteriors                   # OKK
  # taobao05_161_ArchinteriorsV10S04             # OKK
  # taobao05_162_SciFiCorridor                   # OKK
  # taobao05_164_BigTriplexHouseVilla            # OKK
  # taobao05_165_ModularBank                     # OKK
  # taobao05_166_Palace                          # OKK
  # taobao05_169_StudioApart                     # OKK
  # taobao05_170_Classroom                       # OKK

  # # TaoBao06场景
  # taobao06_200_JapaneseSchool                  # OKK
  # taobao06_201_AIUE_vol8_01                    # OKK
  # taobao06_202_AIUE_vol8_02                    # OKK
  # taobao06_203_AIUE_vol8_03                    # OKK
  # taobao06_204_AIUE_vol8_04                    # OKK
  # taobao06_205_AIUE_vol8_05                    # OKK
  # taobao06_207_Sci_Fi_Living_Room              # OKK
  # taobao06_208_Archinteriors                   # OKK
  # taobao06_212_train                           # OKK
  # taobao06_213_Modular_Brutalist_Pack          # OKK
  # taobao06_214_Modular_Bank_Pack               # OKK
  # taobao06_215_WeatherStudio                   # OKK
  # taobao06_217_VP_Real_Food_and_Coffee_Bakery  # OKK
  # taobao06_218_Archinteriors                   # OKK
  # taobao06_219_SoftwareServer                  # OKK
  # taobao06_220_OldWestModularBarn              # OKK 
  # taobao06_221_ShowroomKit                     # OKK
  # taobao06_222_ASubwayTrain                    # OKK
  # taobao06_223_MedievalTavernModularkit        # OKK
  # taobao06_224_HospitalityPack                 # OKK
  # taobao06_225_BlueMetro                       # OKK
  # taobao06_226_LoftOffice_workspace            # OKK
  # taobao06_227_LoftOffice_kichen               # OKK
  # taobao06_228_LoftOffice_cabinet              # OKK

  # # TaoBao07场景
  # taobao07_250_MoonBase                        # OKK
  # taobao07_251_XiangYuZhongxin                 # OKK
  # taobao07_253_Carpenters                      # OKK
  # taobao07_257_SciFiFacility                   # OKK
  # taobao07_259_Factory                         # OKK

  # # TaoBao08 场景
  # taobao08_00_100_LivingRoom_12                            # OKK
  # taobao08_00_102_Bedroom_8                                # OKK
  # taobao08_00_103_InteriorDesign11                         # OKK
  # taobao08_00_104_InteriorDesign9                          # OKK
  # taobao08_00_109_InteriorDesign6                          # OKK
  # taobao08_00_110_InteriorDesign4                          # OKK
  # taobao08_00_111_InteriorDesign3                          # OKK
  # taobao08_00_112_InteriorDesign1                          # OKK
  # taobao08_00_114_Bathroom                                 # OKK
  # taobao08_00_116_MaleAndFemaleRestroom                    # OKK
  # taobao08_00_118_ModernMinimalistKitchenInterior          # OKK
  # taobao08_00_11_NavyBlueBathroom                          # OKK
  # taobao08_00_123_ModernKitchen                            # OKK
  # taobao08_00_124_Bedroom_6                                # OKK
  # taobao08_00_126_FashionPhotoshootStudio                  # OKK
  # taobao08_00_127_AristocratLivingRoom                     # OKK
  # taobao08_00_129_LivingRoomAndKitchenFullset              # OKK
  # taobao08_00_12_ModernBedroom                             # OKK
  # taobao08_00_130_RoyalEleganceLivingRoom                  # OKK
  # taobao08_00_132_FuturisticNeonConferenceRoom             # OKK
  # taobao08_00_133_ModernGym                                # OKK  
  # taobao08_00_13_ModernBedroom                             # OKK
  # taobao08_00_141_VictorianEleganceLivingRoom              # OKK
  # taobao08_00_142_BedroomKidsMonkey                        # OKK
  # taobao08_00_143_ElegantClassicLivingRoom                 # OKK
  # taobao08_00_144_BathroomWithCityView                     # OKK
  # taobao08_00_145_CompactElegantBathroom                   # OKK
  # taobao08_00_147_ModernZenMeditationSpace                 # OKK
  # taobao08_00_149_VintageRoyalLivingRoom                   # OKK
  # taobao08_00_150_BedroomDesign                            # OKK
  # taobao08_00_152_ElegantChildrensRoom                     # OKK
  # taobao08_00_154_CompactKitchen                           # OKK
  # taobao08_00_155_LivingRoom                               # OKK
  # taobao08_00_157_ModernSerenityLivingRoom                 # OKK
  # taobao08_00_159_Bathroom                                 # OKK
  # taobao08_00_15_ApartmentHighQuality                      # OKK
  # taobao08_00_160_ModernInteriorDesign                     # OKK
  # taobao08_00_162_WarmModernInteriorScene                  # OKK
  # taobao08_00_163_BedroomKid01                             # OKK
  # taobao08_00_164_CardioStation                            # OKK
  # taobao08_00_166_ModernMinimalistLivingRoomInterior       # OKK
  # taobao08_00_169_ModernBedroom                            # OKK
  # taobao08_00_16_ModernBedroom                             # OKK
  # taobao08_00_170_OfficeFloor                              # OKK
  # taobao08_00_171_KidsRoom                                 # OKK
  # taobao08_00_172_AbandonedClassroomInterior               # OKK
  # taobao08_00_173_ModernBedroomInSunset                    # OKK
  # taobao08_00_176_ModernElegantBathroom                    # OKK
  # taobao08_00_177_FunctionalBathroom                       # OKK
  # taobao08_00_178_LowPolyRoom                              # OKK
  # taobao08_00_179_JapaneseStyleRoom                        # OKK
  # taobao08_00_17_ModernBedroom                             # OKK
  # taobao08_00_184_GourmetArea                              # OKK
  # taobao08_00_185_MinimalistLivingSpace                    # OKK
  # taobao08_00_186_PinkDreamKidsBedRoom                     # OKK
  # taobao08_00_188_BeachBar                                 # OKK
  # taobao08_00_19_CompactApartment                          # OKK
  # taobao08_00_22_LivingRoomDesign                          # OKK
  # taobao08_00_23_ModernApartmentHighQuality                # OKK
  # taobao08_00_26_BedroomMorden                             # OKK
  # taobao08_00_28_CompactLibrary                            # OKK
  # taobao08_00_29_Coliving                                  # OKK  
  # taobao08_00_2_ModernAbstractLivingRoom                   # OKK
  # taobao08_00_30_ModernApartmentLivingAndDining            # OKK
  # taobao08_00_32_ResidentialPartyRoom                      # OKK
  # taobao08_00_33_MinimalistLivingRoom                      # OKK, slow
  # taobao08_00_35_SmallFactoryOffice                        # OKK
  # taobao08_00_37_ModernLivingRoomLoft                      # OKK
  # taobao08_00_39_FlatStudio                                # OKK
  # taobao08_00_190_ModernCozyLivingRoom                     # OKK
  # taobao08_00_1_LuxuryVintageHotelBathroom                 # OKK
  # taobao08_00_3_ModernCurvedWoodenLivingRoom               # OKK, slow
  # taobao08_00_41_ModernBedroom                             # OKK
  # taobao08_00_42_LingvingModern                            # OKK
  # taobao08_00_44_LuxuryKitchenDesign                       # OKK
  # taobao08_00_45_MinimalLivingRoom_ChrismasTree            # OKK
  # taobao08_00_46_LivingRoom                                # OKK
  # taobao08_00_47_Podcast_ArtboardSilvioPinheiro            # OKK
  # taobao08_00_4_ModernOrganicLivingRoom                    # OKK
  # taobao08_00_52_LuxuriousBathroom                         # OKK
  # taobao08_00_53_3dStudio                                  # OKK
  # taobao08_00_55_BedroomKid                                # OKK
  # taobao08_00_57_InteriorDesign_29                         # OKK
  # taobao08_00_58_BedroomKid_14                             # OKK
  # taobao08_00_59_ElegantBathroom                           # OKK
  # taobao08_00_5_LivingRoom                                 # OKK
  # taobao08_00_60_ModernKitchenAndDinerScene                # OKK
  # taobao08_00_61_InteriorDesign_28                         # OKK
  # taobao08_00_62_InteriorDesign_27                         # OKK
  # taobao08_00_63_DecoratedApartment                        # OKK
  # taobao08_00_64_LuxuriousIndigoBathroom                   # OKK
  # taobao08_00_65_BlueAndWhiteKitchenWithDiningArea         # OKK
  # taobao08_00_66_ArchitecturalRoomGenovaMadridSpain        # OKK
  # taobao08_00_67_ModernLivingRoom                          # OKK
  # taobao08_00_68_InteriorDesign_26                         # OKK
  # taobao08_00_6_SceneModernKitchen                         # OKK
  # taobao08_00_72_ElegantLivingRoom                         # OKK
  # taobao08_00_74_LuxuryKitchen                             # OKK
  # taobao08_00_76_LivingRoomWithIntegratedKitchen           # OKK
  # taobao08_00_78_HouseInterior                             # OKK
  # taobao08_00_79_CompactLivingRoom                         # OKK
  # taobao08_00_7_MinimalistCoffeeShop3dScene                # OKK
  # taobao08_00_80_InteriorDesign_23                         # OKK
  # taobao08_00_83_InteriorDesign_19                         # OKK
  # taobao08_00_84_HomeTheater                               # OKK
  # taobao08_00_85_InteriorDesign_16                         # OKK
  # taobao08_00_86_InteriorDesign_15                         # OKK 
  # taobao08_00_87_ModernBathroom                            # OKK 
  # taobao08_00_88_MiniPartyHall                             # OKK 
  # taobao08_00_89_LivingRoomInteriorDesign                  # OKK
  # taobao08_00_8_ModernApartmentLivingRoom                  # OKK
  # taobao08_00_90_ConceptLivingRoom                         # OKK
  # taobao08_00_91_LivingRoomFullset01                       # OKK
  # taobao08_00_92_ModernBathroomInterior                    # OKK
  # taobao08_00_93_OfficeBoardroom                           # OKK
  # taobao08_00_94_InteriorDesign5                           # OKK
  # taobao08_00_95_InteriorScene                             # OKK
  # taobao08_00_96_ContemporaryLivingRoom2                   # OKK
  # taobao08_00_97_ContemporaryLivingRoom                    # OKK
  # taobao08_00_98_ModernOfficeFullSet01                     # OKK
  # taobao08_00_99_ContemporaryLivingroom                    # OKK
  # taobao08_00_9_GreenWoodenKitchen                         # OKK
  # taobao08_00_122_ContemporaryLiving_DiningInterior        # OKK
  # taobao08_00_134_TheWaitingTable                          # OKK  
  # taobao08_00_158_CompactStoveAndBar                       # OKK
  # taobao08_00_167_ModernClassicLivingRoom_WarmTones        # OKK
  # taobao08_00_25_BedroomHighQuality                        # OKK
  # taobao08_00_161_RetroKitchenScene                        # OKK
  # taobao08_00_168_StaircaseInABuilding                     # OKK
  # taobao08_00_174_OutdoorKitchen                           # OKK
  # taobao08_00_187_KidsBedroomScene                         # OKK
  # taobao08_00_31_LivingAndDiningRoom                       # OKK
  # taobao08_01_101_KitchenSet008                            # OKK
  # taobao08_01_106_ModernConferenceRoom                     # OKK
  # taobao08_01_108_ModernLivingRoomHall                     # OKK
  # taobao08_01_113_ClassicArtDecoHall                       # OKK
  # taobao08_01_115_WineClub                                 # OKK
  # taobao08_01_125_MinimalArchvizInterior                   # OKK
  # taobao08_01_131_OrganizationalResearchHub_Office         # OKK
  # taobao08_01_135_ClothingStore                            # OKK, slow
  # taobao08_01_139_DesignStudio                             # OKK
  # taobao08_01_148_ClassicArtGallery                        # OKK
  # taobao08_01_14_BlackNGold                                # OKK
  # taobao08_01_180_ComercialMall                            # OKK
  # taobao08_01_20_BedroomInForest                           # OKK
  # taobao08_01_27_ModernOfficeInterior                      # OKK  
  # taobao08_01_49_ModernDecoLivingRoom                      # OKK
  # taobao08_01_50_ModernPurpleLivingRoom                    # OKK
  # taobao08_01_51_ModernWoodenLivingRoom                    # OKK
  # taobao08_01_54_CompleteCoffeeShopScene                   # OKK
  # taobao08_01_56_MiniMarket                                # OKK
  # taobao08_01_69_ClassicLuxurySwimmingPool                 # OKK
  # taobao08_01_70_LuxurySwimmingPool                        # OKK
  # taobao08_01_71_UltramodernLuxurySwimmingPool             # OKK
  # taobao08_01_73_SchoolHallwayAndClassroom                 # OKK
  # taobao08_01_75_LivingRoomFullSet002                      # OKK

  # # TaoBao09 场景
  # taobao09_01_AsiaRestaurant                               # OKK
  # taobao09_04_wuxia                                        # OKK
  # taobao09_11_changlang                                    # OKK
  # taobao09_14_KyotoAlley                                   # OKK
  # taobao09_15_WarRoom                                      # OKK
  # taobao09_16_MJH                                          # OKK
  # taobao09_16_MJH03                                        # OKK
  # taobao09_16_MJH04                                        # OKK
  # taobao09_16_MJH05                                        # OKK
  # taobao09_16_MJH06                                        # OKK
  # taobao09_16_MJH08                                        # OKK
  # taobao09_16_MJH09                                        # OKK
  # taobao09_16_MJH10                                        # OKK
  # taobao09_16_MJH11                                        # OKK
  # taobao09_16_MJH16                                        # OKK
  # taobao09_16_MJH20                                        # OKK
  # taobao09_20_JapaneseHouse                                # OKK
  # taobao09_23_Japanese_Castle                              # OKK
  # taobao09_24_ShintoShrine                                 # OKK
  # taobao09_26_Asian_Modular                                # OKK
  # taobao09_27_Hong_Kong_Street                             # OKK
  # taobao09_28_ChinaModular                                 # OKK
  # taobao09_33_SICKA_DYNASTY                                # OKK
  # taobao09_35_AsianTemple                                  # OKK
  # taobao09_36_Japanese_village                             # OKK
  # taobao09_38_JapaneseVillage                              # OKK  
  # taobao09_42_Building_Props                               # OKK
  # taobao09_44_JapaneseOnsen                                # OKK
  # taobao09_49_PolygonSamurai                               # OKK
  # taobao09_51_LakeWoodenHouse                              # OKK

  # # TaoBao10 场景
  # taobao10_002_AE37_003_Blender                            # OKK
  # taobao10_003_AE37_004_Blender                            # OKK
  # taobao10_005_AE37_009                                    # OKK
  # taobao10_007_GT_Grid_Armada_Military_Base                # OKK

  # # Kujiale (InteriorAgent) 场景
  # kujiale_0003                                              # OKK
  # kujiale_0004                                              # OKK
  # kujiale_0008                                              # OKK
  # kujiale_0009                                              # OKK
  # kujiale_0020                                              # OKK
  # kujiale_0021                                              # OKK  
  # kujiale_0022                                              # OKK
  # kujiale_0024                                              # OKK
  # kujiale_0025                                              # OKK
  # kujiale_0026                                              # OKK
  # kujiale_0030                                              # OKK
  # kujiale_0031                                              # OKK
  # kujiale_0032                                              # OKK
  # kujiale_0033                                              # OKK
  # kujiale_0034                                              # OKK
  # kujiale_0035                                              # OKK
  # kujiale_0036                                              # OKK
  # kujiale_0037                                              # OKK
  # kujiale_0038                                              # OKK
  # kujiale_0040                                              # OKK
  # kujiale_0042                                              # OKK
  # kujiale_0043                                              # OKK
  # kujiale_0065                                              # OKK
  # kujiale_0066                                              # OKK
  # kujiale_0067                                              # OKK

  # # Slow
  # taobao08_01_48_SceneKitchenIndoorCoffee                  # OKK
  # taobao03_036_WillsRoom                                   # OKK
  # taobao08_01_81_ConvenienceStore                          # OKK
  # taobao04_103_Shoppingmall01                              # OKK, 一个无商品商场 + 外面


  # # 报错场景
  # taobao10_001_AE37_001_Blender                            # OKK     
  # taobao10_004_AE37_007                                    # OKK
  # taobao08_01_10_ModernLuxuriousKitchenInterior            # OKK
  # taobao08_00_82_InteriorDesign_22                         # OKK
  # taobao06_216_Triplex_House_Villa                         # OKK
  # taobao_OutdoorFurniture                                  # OKK
  # taobao_UtopianCity               
  # taobao03_020_AbandonedHK         
  # taobao06_211_city_of_gods       
  # taobao08_01_117_AntiqueBrownAndGreenApartment          
  # taobao08_00_151_MysticForestPathInBlender             
  # taobao08_00_43_LivingRoomScene
  # taobao09_05_ModularJapaneseTemple
  # taobao08_01_105_LivingRoomInterior
  # taobao10_006_AE38_005_Blender            
  # taobao03_114_Abandoned_Library                           # submit
  # taobao03_116_SciFiWorld                                  # submit
  # taobao04_103_Shoppingmall                                # submit, 两个商场，有商品、无商品              
  # taobao04_103_Shoppingmall02                              # submit, 一个有商品商场
  # taobao05_151_Galaxy                                      # submit
  # taobao08_00_77_LivingRoom_25                             # submit
  # taobao08_00_18_ModernBedroom                             # submit
  # taobao08_01_138_BoxingRing                               # submit                

)


for task_name in "${TASKS[@]}"; do
  if [[ ! -f "${SCRIPT_DIR}/${task_name}.sh" ]]; then
    echo "错误: task_name '${task_name}' 不存在，对应脚本缺失: ${SCRIPT_DIR}/${task_name}.sh" >&2
    exit 1
  fi
done

cd "$(dirname "$0")"

for task_name in "${TASKS[@]}"; do
  echo "提交 task: ${task_name}"
  # 往L4 Task队列中提交任务
  # bash submit_all_seed_one_task.sh ${CAMERA_NAME} 100 6 10 60 "${task_name}" "q-20260429225420-jrwjn" True ml.gni3.48xlarge 8 48 NVIDIA-L4 ml.gni3 1
  
  # 往L4 Develop队列提交任务
  # bash submit_all_seed_one_task.sh ${CAMERA_NAME} 50 24 40 10 "${task_name}" "q-20260507093353-7r9k8" True ml.gni3.48xlarge 8 48 NVIDIA-L4 ml.gni3 1

  # 往L20 Develop队列提交任务
  # bash submit_all_seed_one_task.sh ${CAMERA_NAME} 50 1 40 10 "${task_name}" "q-20251110132321-bx8th" True ml.gni3cl.45xlarge 16 96 NVIDIA-L20 ml.gni3cl 1
  
  # 往L20 Task队列提交任务
  # bash submit_all_seed_one_task.sh ${CAMERA_NAME} 50 1 200 10 "${task_name}" "q-20260507105650-5lk49" True ml.gni3cl.45xlarge 64 384 NVIDIA-L20 ml.gni3cl 4

  # 往预约的L20中提交任务
  # bash submit_all_seed_one_task.sh ${CAMERA_NAME} 50 24 40 10 "${task_name}" "rp-20260518074302-f2l7s" False ml.gni3cl.11xlarge 16 96 NVIDIA-L20 ml.gni3cl 1

done