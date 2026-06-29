from __future__ import print_function
import volcenginesdkcore
import volcenginesdkmlplatform20240701 as mlpsdk
from volcenginesdkcore.rest import ApiException
import os

DEFAULT_EFS_MOUNTS = [
    # (mount_path, access_point_id, access_point_name)
    ("/data/efs/users", "ap-ab8d0375", "users"),
    ("/data/efs/datasets", "ap-6dcfe989", "datasets"),
    ("/data/efs/public", "ap-0fa23446", "public"),
]


def _make_efs_ap_storage(
    efs_name, efs_id, efs_mount_path, efs_read_only,
    access_point_id, access_point_name, access_point_domain,
):
    # CreateJob 要求 Type=EfsAP 
    efs_config = mlpsdk.ConfigForCreateJobInput()
    efs_config.swagger_types = dict(
        efs_config.swagger_types,
        efs='object',
        efs_ap='object',
    )
    efs_config.attribute_map = dict(
        efs_config.attribute_map,
        efs='Efs',
        efs_ap='EfsAP',
    )
    efs_config.efs = {
        "Id": efs_id,
        "FileSystemName": efs_name,
    }
    efs_config.efs_ap = {
        "Id": efs_id,
        "AccessPointId": access_point_id,
        "AccessPointName": access_point_name,
        "EnabledIam": False,
        "AccessPointDomain": access_point_domain,
    }
    return mlpsdk.StorageForCreateJobInput(
        type='EfsAP',
        mount_path=efs_mount_path,
        read_only=efs_read_only,
        config=efs_config,
    )


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
    tos_name=None, # 如需要挂载对象存储，对象存储的桶名
    tos_mount_path=None, # 对象存储挂载到任务中的路径
    tos_prefix=None, # 将对象存储的哪个路径挂载至任务中
    vepfs_name=None, # 如需挂载vePFS，其名称
    vepfs_volume_id=None, # 如需挂载vePFS，其ID
    vepfs_mount_path=None, # vePFS挂载到任务的路径
    vepfs_prefix=None, # 将vePFS的哪个路径挂载至任务重
    efs_name=None, # 如需挂载EFS，其文件系统名称(FileSystemName)
    efs_id=None, # 如需挂载EFS，其ID
    efs_mounts=None, # EFS多挂载，每项为(mount_path, access_point_id, access_point_name)
    efs_type="Premium", # 保留参数，VolcEfs接入点挂载不使用
    efs_addr="", # EFS AccessPointDomain，接入点挂载域名
    efs_read_only=False,
):
    # 1. 初始化配置（优先用外部传入的AK/SK，无则用默认值）
    config = volcenginesdkcore.Configuration()
    config.ak = ak if ak else ""    # 替换为你的默认AK
    config.sk = sk if sk else ""  # 替换为你的默认SK
    config.region = region
    volcenginesdkcore.Configuration.set_default(config)

    # 2. 初始化API实例
    api_instance = mlpsdk.MLPLATFORM20240701Api()
    req_tos = mlpsdk.TosForCreateJobInput(
        bucket=tos_name,
        prefix=tos_prefix,
    )
    req_cfs = mlpsdk.CfsForCreateJobInput(
        file_system_name="drobotics-ailab",
        tos=req_tos,
    ) 
    req_config = mlpsdk.ConfigForCreateJobInput(
    cfs=req_cfs,
    )

    # 3. 构建存储配置
    storages = [
        mlpsdk.StorageForCreateJobInput(
            type='Cfs',
            mount_path=tos_mount_path,
            config=req_config
        ),
        mlpsdk.StorageForCreateJobInput(
            type='Vepfs',
            mount_path=vepfs_mount_path,
            config=mlpsdk.ConfigForCreateJobInput(
                vepfs=mlpsdk.VepfsForCreateJobInput(
                    id=vepfs_volume_id,
                    file_system_name=vepfs_name,
                    sub_path=vepfs_prefix
                )
            )
        ),
        # mlpsdk.StorageForCreateJobInput(
        #     type='Vepfs',
        #     mount_path='/data/vepfs/public',
        #     config=mlpsdk.ConfigForCreateJobInput(
        #         vepfs=mlpsdk.VepfsForCreateJobInput(
        #             id=vepfs_volume_id,
        #             file_system_name=vepfs_name,
        #             sub_path='/public'
        #         )
        #     )
        # ),
    ]
    # EfsAP 接入点挂载: /data/efs/users、datasets、public
    if efs_id or efs_name:
        for mount_entry in (efs_mounts or DEFAULT_EFS_MOUNTS):
            if len(mount_entry) == 3:
                mount_path, access_point_id, access_point_name = mount_entry
            else:
                mount_path, access_point_id = mount_entry[:2]
                access_point_name = mount_path.rstrip('/').split('/')[-1]
            storages.append(
                _make_efs_ap_storage(
                    efs_name, efs_id, mount_path, efs_read_only,
                    access_point_id, access_point_name, efs_addr,
                )
            )

    # 4. 构建任务请求
    create_job_req = mlpsdk.CreateJobRequest(
        name=job_name,
        # 资源配置（队列、实例规格等）
        resource_config=mlpsdk.ResourceConfigForCreateJobInput(
            resource_queue_id=resource_queue_id,
            # preemptible=True,
            priority=6,
            roles=[
                mlpsdk.RoleForCreateJobInput(
                    name='worker',  # 角色名（固定为worker即可）
                    replicas=1,  # 实例数量（如不需要多机分布式计算，写1即可，如需分布式计算，需要几机就写几）
                    resource=mlpsdk.ResourceForCreateJobInput(
                        # 以下注释的两行为使用整机资源运行任务，instance_type_id为机器类型，如使用整机运行，则需要解开以下两行的注释，并注释以下未注释的所有参数
                        # instance_type_id=instance_type_id,
                        # type='mlp_flavor',  # 资源类型（固定为mlp_flavor）
                        
                        #以下为灵活配比模式，可以选择需要的资源运行任务，注意其填写的最大资源值为单机上限 
                        type="Flexible",  # 指定为灵活配比
                        zone_id="cn-beijing-b",  # 地址服务器的可用区,如何查看请见代码块下方zone_id详解
                        flexible_resource_claim={
                           "Cpu": 105,  # 指定要使用的CPU核数
                           "MemoryGiB": 1800, # 指定要使用的内存，单位为GB
                        #    "GpuType":"NVIDIA-H20-SXM5-96GB", # 指定显卡的类型 H20
                        #    "GpuType":"NVIDIA-L20", # 指定显卡的类型NVIDIA-L20
                           "GpuType":"NVIDIA-A800-SXM4-80GB", # 指定显卡的类型  A800
                        #    "Family": "ml.hpcpni3l", # 指定显卡的家族 H20
                            # "Family": "ml.gni3cl.45xlarge", # 指定显卡的家族 L20
                            "Family": "ml.hpcpni2l", # 指定显卡的家族 A800
                           "GpuCount": 8 # 指定要使用的GPU个数
                        },
                    )
                )
            ]
        ),
        # 运行时配置
        runtime_config=mlpsdk.RuntimeConfigForCreateJobInput(
            command=command,  # 设置任务默认运行的命令
            framework='PyTorch',  # 框架（根据你的实际需求调整，如PyTorch, MPI, TensorFlow, Ray, Custom）

            # 使用公共镜像
            # image=mlpsdk.ImageForCreateJobInput(
            #     type='Public',  # 公共镜像
            #     url=image_url  # 固定公共镜像地址
            # )

            # 使用私有镜像
            image=mlpsdk.ImageForCreateJobInput(
                type='VolcEngine',  # 指定镜像类型为使用传到火山镜像仓库的参数
                url=image_url,  # 指定镜像地址
                credential=mlpsdk.CredentialForCreateJobInput(
                    registry_username=private_image_username,  # 指定镜像仓库用户名
                    registry_token=private_image_password     # 指定镜像仓库密码
                )
            )
        ),
        # 存储配置（挂载CFS、vepfs、EFS，按需调整）
        storage_config=mlpsdk.StorageConfigForCreateJobInput(
            credential=mlpsdk.ConvertCredentialForCreateJobInput(
                access_key=config.ak,
                secret_access_key=config.sk
            ),
            storages=storages
        )
    )
    try:
        resp = api_instance.create_job(create_job_req)
        print(f"✅ 任务创建成功！任务名称：{job_name}，响应：{resp}")
        return resp
    except ApiException as e:
        print(f"❌ 任务创建失败！错误信息：{e}")
        return None


if __name__ == "__main__":
    # 以下参数含义及在个人信息获取可以参照2.3节
    task_name = 'rsgen-igligen'    #####``
    config = "/data/vepfs/users/xianbao01.hou/temp/DRRM-VODPP/configs/vodpp_train/dit_fm_dinov3cnnl_1v_adapter4_dit1024.yaml"
    resource_queue_id = 'q-20260108184533-vm9mz'
    # resource_queue_id = 'q-20251104161904-5jb7z' 
    # resource_queue_id = 'q-20251110132124-k5dt9'
    # instance_type_id = 'ml.hpcpni3l.48xlarge'
    # instance_type_id = 'ml.gni3cl.45xlarge'
    instance_type_id = 'ml.hpcpni2l.4xlarge'

    tos_name = 'drobotics-ailab'     # 默认值
    tos_mount_path = '/data/tos'
    tos_prefix = 'datasets/'

    vepfs_name = 'd-robotics-vepfs-dev'
    vepfs_volume_id = 'vepfs-cnbjf7015ef11e9c'
    vepfs_mount_path = '/data/vepfs/users/xianbao01.hou'
    vepfs_prefix = '/xianbao01.hou'

    efs_name = 'DRoboticsAILab'
    efs_id = 'efs-cnbjc8a589e0f6d6f'
    efs_type = 'Premium'
    efs_addr = 'cnbjc8a589e0f6d6f.3psq7ep69ltkw6csxyuomq6f5.cn-beijing.efs.ivolces.com'
    efs_read_only = False

    image_url = 'd-robotics-image-dev-cn-beijing.cr.volces.com/d-robotics-images/instada:v1'   ##### 
    private_image_username = 'xianbao01.hou@61215337'
    private_image_password = 'Wcasdfasfdasdfasdfasdfasd'
    ak = "asdf4YTE"
    sk = "sdfasdf"
    region = 'cn-beijing'


    create_ml_job(
        command=(
            "sleep 10d && source /data/vepfs/users/xianbao01.hou/anaconda3/etc/profile.d/conda.sh "
            "&& conda activate rsgen "
            "&& cd /data/vepfs/users/xianbao01.hou/RSGen/igligen "
            f'&& bash train_sdv2.1_fgcontrol.sh'
        ),
        job_name=task_name, 
        resource_queue_id=resource_queue_id, 
        instance_type_id=instance_type_id, 
        ak=ak,  
        sk=sk,
        region=region,
        image_url=image_url,
        private_image_username=private_image_username,
        private_image_password=private_image_password,
        tos_name=tos_name,
        tos_mount_path=tos_mount_path,
        tos_prefix=tos_prefix,
        vepfs_name=vepfs_name,
        vepfs_volume_id=vepfs_volume_id,
        vepfs_mount_path=vepfs_mount_path,
        vepfs_prefix=vepfs_prefix,
        efs_name=efs_name,
        efs_id=efs_id,
        efs_type=efs_type,
        efs_addr=efs_addr,
        efs_read_only=efs_read_only,
    )