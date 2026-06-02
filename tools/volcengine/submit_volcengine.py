from __future__ import print_function
import argparse

import volcenginesdkcore
import volcenginesdkmlplatform20240701 as mlpsdk
from volcenginesdkcore.rest import ApiException

def _str2bool(value):
    """argparse 不能用 type=bool：bool('False') 在 Python 里为 True。"""
    if isinstance(value, bool):
        return value
    v = value.strip().lower()
    if v in ("1", "true", "t", "yes", "y", "on"):
        return True
    if v in ("0", "false", "f", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError("期望布尔值: true/false、1/0、yes/no 等")

def create_ml_job(
    command,                             # 必传参数：核心执行命令
    job_name='sample-task',              # 任务名称
    resource_queue_id='q-20250418****',  # 资源队列ID（需替换为你的实际ID）
    instance_type_id='ml.g2a.xlarge',    # 实例类型（如ml.g2a.xlarge）
    ak=None,                             # 访问密钥AK
    sk=None,                             # 密钥SK
    region='cn-beijing',                 # 区域（默认北京）
    image_url='vemlp-cn-beijing.cr.volces.com/preset-images/python:3.12-ubuntu22.04', # 任务使用的镜像
    private_image_username=None, # 镜像如果为私有镜像，其登录的用户名
    private_image_password=None, # 镜像如果为私有镜像，其登录的密码
    tos_name=None,       # 如需要挂载对象存储，对象存储的桶名
    tos_mount_path=None, # 对象存储挂载到任务中的路径
    tos_prefix=None,     # 将对象存储的哪个路径挂载至任务中
    vepfs_name=None,     # 如需挂载vePFS，其名称
    vepfs_volume_id=None,   # 如需挂载vePFS，其ID
    vepfs_mount_path=None,  # vePFS挂载到任务的路径
    vepfs_prefix=None,      # 将vePFS的哪个路径挂载至任务重
    cpu_number=16,       # 指定要使用的CPU核数
    memory_number=64,    # 指定要使用的内存，单位为GB
    gpu_type="NVIDIA-L20",  # 指定显卡的类型
    gpu_count=1,         # 指定要使用的GPU个数
    family="ml.gni3cl",  # 指定显卡的家族
    zone_id="cn-beijing-c", # 地址服务器的可用区,如何查看请见代码块下方zone_id详解
    is_flexible=True,       # 是否使用灵活配比模式, 预约火山资源时使用False
):
    # 1. 初始化配置（优先用外部传入的AK/SK，无则用默认值）
    config = volcenginesdkcore.Configuration()
    config.ak = ak if ak else "AKLTYzdhZGI0NDI5Z****"    # 替换为你的默认AK
    config.sk = sk if sk else "TldRMk9XTXpNR0V3T0R****"  # 替换为你的默认SK
    config.region = region
    volcenginesdkcore.Configuration.set_default(config)

    # 2. 初始化API实例
    api_instance = mlpsdk.MLPLATFORM20240701Api()

    # 3. 构建任务请求
    # 资源配置（队列、实例规格等）
    if is_flexible:
        resource_config=mlpsdk.ResourceConfigForCreateJobInput(
            resource_queue_id=resource_queue_id,
            roles=[
                mlpsdk.RoleForCreateJobInput(
                    name='worker',  # 角色名（固定为worker即可）
                    replicas=1,     # 实例数量（如不需要多机分布式计算，写1即可，如需分布式计算，需要几机就写几）
                    resource=mlpsdk.ResourceForCreateJobInput(
                        # 以下注释的两行为使用整机资源运行任务，instance_type_id为机器类型，如使用整机运行，则需要解开以下两行的注释，并注释以下未注释的所有参数
                        # instance_type_id=instance_type_id,
                        # type='mlp_flavor',  # 资源类型（固定为mlp_flavor）
                        
                        #以下为灵活配比模式，可以选择需要的资源运行任务，注意其填写的最大资源值为单机上限 
                        type="Flexible",  # 指定为灵活配比
                        zone_id=zone_id,  # 地址服务器的可用区,如何查看请见代码块下方zone_id详解
                        flexible_resource_claim={
                            "Cpu": cpu_number,          # 指定要使用的CPU核数
                            "MemoryGiB": memory_number, # 指定要使用的内存，单位为GB
                            "GpuType": gpu_type,        # 指定显卡的类型
                            "Family": family,           # 指定显卡的家族
                            "GpuCount": gpu_count       # 指定要使用的GPU个数
                        },
                    )
                )
            ]
        )
    else:
        resource_config=mlpsdk.ResourceConfigForCreateJobInput(
            resource_reservation_plan_id=resource_queue_id,
            priority=6,
            roles=[
                mlpsdk.RoleForCreateJobInput(
                    name='worker',  # 角色名（固定为worker即可）
                    replicas=1,     # 实例数量（默认1，可根据需求调整）
                    resource=mlpsdk.ResourceForCreateJobInput(
						instance_type_id=instance_type_id,
						type="Preset",
						zone_id=zone_id,
					)
                )
            ]
        )
    # 运行时配置
    runtime_config=mlpsdk.RuntimeConfigForCreateJobInput(
        command=command,     # 设置任务默认运行的命令
        framework='Custom',  # 框架（根据你的实际需求调整，如PyTorch, MPI, TensorFlow, Ray, Custom）

        # 使用公共镜像
        # image=mlpsdk.ImageForCreateJobInput(
        #     type='Public',  # 公共镜像
        #     url=image_url  # 固定公共镜像地址
        # )

        # 使用私有镜像
        image=mlpsdk.ImageForCreateJobInput(
            type='VolcEngine',  # 指定镜像类型为使用传到火山镜像仓库的参数
            url=image_url,      # 指定镜像地址
            credential=mlpsdk.CredentialForCreateJobInput(
                registry_username=private_image_username,  # 指定镜像仓库用户名
                registry_token=private_image_password      # 指定镜像仓库密码
            )
        )
    )
    # 存储配置（挂载TOS、vepfs，按需调整）
    if is_flexible:
        storage_config=mlpsdk.StorageConfigForCreateJobInput(
            credential=mlpsdk.ConvertCredentialForCreateJobInput(
                access_key=config.ak,       # 对象存储所需的权限ak
                secret_access_key=config.sk # 对象存储所需的权限sk
            ),
            storages=[
                mlpsdk.StorageForCreateJobInput(
                    type='Tos',                  # 指定为挂载对象存储
                    mount_path=tos_mount_path,   # 指定将对象存储挂载到任务的路径
                    config=mlpsdk.ConfigForCreateJobInput(
                        tos=mlpsdk.TosForCreateJobInput(
                            bucket=tos_name,     # 指定对象存储名称
                            prefix=tos_prefix    # 将对象存储的指定目录挂载到任务中
                        )
                    )
                ),
                mlpsdk.StorageForCreateJobInput(
                    type='Vepfs',                  # 指定为挂载vePFS
                    mount_path=vepfs_mount_path,   # 指定将vePFS挂载到任务的路径
                    config=mlpsdk.ConfigForCreateJobInput(
                        vepfs=mlpsdk.VepfsForCreateJobInput(
                            id=vepfs_volume_id,    # 指定vePFS的id
                            file_system_name=vepfs_name,  # 指定vePFS的名称
                            sub_path=vepfs_prefix         # 将vePFS的指定目录挂载到任务中
                        )
                    )
                ),
                # mlpsdk.StorageForCreateJobInput(
                #     type='Vepfs',                    # 指定为挂载vePFS（如需多个则写多个即可，如此）
                #     mount_path='/data/vepfs/public', # 指定将vePFS挂载到任务的/data/vepfs/public路径下
                #     config=mlpsdk.ConfigForCreateJobInput(
                #         vepfs=mlpsdk.VepfsForCreateJobInput(
                #             id=vepfs_volume_id,      # 指定vePFS的id
                #             file_system_name=vepfs_name, # 指定vePFS的名称
                #             sub_path='/public'           # 将vePFS的/public挂载到任务中/data/vepfs/public路径下
                #         )
                #     )
                # )
            ]
        )
    else:
        storage_config=mlpsdk.StorageConfigForCreateJobInput(
            credential=mlpsdk.ConvertCredentialForCreateJobInput(
                access_key=config.ak,       # 对象存储所需的权限ak
                secret_access_key=config.sk # 对象存储所需的权限sk
            ),
            storages=[
                mlpsdk.StorageForCreateJobInput(
                    type='Vepfs',                  # 指定为挂载vePFS
                    mount_path=vepfs_mount_path,   # 指定将vePFS挂载到任务的路径
                    config=mlpsdk.ConfigForCreateJobInput(
                        vepfs=mlpsdk.VepfsForCreateJobInput(
                            id=vepfs_volume_id,    # 指定vePFS的id
                            file_system_name=vepfs_name,  # 指定vePFS的名称
                            sub_path=vepfs_prefix         # 将vePFS的指定目录挂载到任务中
                        )
                    )
                )
            ]
        )
    # 创建任务请求
    create_job_req = mlpsdk.CreateJobRequest(
        name=job_name,
        # 资源配置（队列、实例规格等）
        resource_config=resource_config,
        # 运行时配置
        runtime_config=runtime_config,
        # 存储配置（挂载TOS、vepfs，按需调整）
        storage_config=storage_config
    )

    try:
        resp = api_instance.create_job(create_job_req)
        print(f"✅ 任务创建成功！任务名称：{job_name}，响应：{resp}")
        return resp
    except ApiException as e:
        print(f"❌ 任务创建失败！错误信息：{e}")
        return None


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--task_name", type=str, default="training-task", help="任务名称")
    parser.add_argument("--resource_queue_id", type=str, default="q-20251110132321-bx8th", help="资源队列ID")
    parser.add_argument("--command", type=str, default="/root/vepfs/isaacsim/scripts/home_000_zedx01.sh", help="核心执行命令")
    parser.add_argument(
        "--is_flexible",
        type=_str2bool,
        default=True,
        help="是否使用灵活配比模式；命令行请传 true/false（勿用 argparse type=bool）",
    )

    parser.add_argument("--instance_type_id", type=str, default="ml.gni3cl.45xlarge", help="实例类型ID")
    parser.add_argument("--tos_name", type=str, default="drobotics-ailab", help="对象存储名称")
    parser.add_argument("--tos_mount_path", type=str, default="/root/tos")
    parser.add_argument("--tos_prefix", type=str, default="", help="对象存储前缀")
    parser.add_argument("--vepfs_name", type=str, default="drobotics-ailab", help="vePFS名称")
    parser.add_argument("--vepfs_volume_id", type=str, default="vepfs-cnbj18d820ed29da", help="vePFS ID")
    parser.add_argument("--vepfs_mount_path", type=str, default="/root/vepfs")
    parser.add_argument("--vepfs_prefix", type=str, default="/fa.fu", help="vePFS前缀")
    parser.add_argument("--image_url", type=str, default="d-robotics-image-dev-cn-beijing.cr.volces.com/isaacsim/isaacsim5.1:cuda12.8.1", help="镜像URL")
    parser.add_argument("--private_image_username", type=str, default="D-Robotics@2108796092", help="私有镜像用户名")
    parser.add_argument("--private_image_password", type=str, default="*****yourpassword*****", help="私有镜像密码")
    parser.add_argument("--ak", type=str, default="AKLTMDljNmNhNmFkYTM1NGUxNzkxZWIxM*********", help="访问密钥AK")
    parser.add_argument("--sk", type=str, default="WkRjMU9Ea3hZbVJsTTJabU5HVm1OVGxtWVR**************", help="密钥SK")
    parser.add_argument("--region", type=str, default="cn-beijing", help="区域")
    parser.add_argument("--cpu_number", type=int, default=16, help="CPU核数")
    parser.add_argument("--memory_number", type=int, default=96, help="内存, 单位为GB")
    parser.add_argument("--gpu_type", type=str, default="NVIDIA-L20", help="显卡类型")
    parser.add_argument("--gpu_count", type=int, default=1, help="GPU个数")
    parser.add_argument("--family", type=str, default="ml.gni3cl", help="显卡家族")
    parser.add_argument("--zone_id", type=str, default="cn-beijing-c", help="地址服务器的可用区")
    args = parser.parse_args()

    print(f"=====😁😁😁提交任务参数{args.task_name}😁😁😁=====")
    print("所有参数:", vars(args))

    create_ml_job(
        command=f"bash {args.command}",  
        job_name=args.task_name, 
        resource_queue_id=args.resource_queue_id, 
        instance_type_id=args.instance_type_id, 
        ak=args.ak,  
        sk=args.sk,
        region=args.region,
        image_url=args.image_url,
        private_image_username=args.private_image_username,
        private_image_password=args.private_image_password,
        tos_name=args.tos_name,
        tos_mount_path=args.tos_mount_path,
        tos_prefix=args.tos_prefix,
        vepfs_name=args.vepfs_name,
        vepfs_volume_id = args.vepfs_volume_id,
        vepfs_mount_path=args.vepfs_mount_path,
        vepfs_prefix=args.vepfs_prefix,
        cpu_number=args.cpu_number,
        memory_number=args.memory_number,
        gpu_type=args.gpu_type,
        gpu_count=args.gpu_count,
        family=args.family,
        zone_id=args.zone_id,
        is_flexible=args.is_flexible
    )
    print(f"===========================================")
