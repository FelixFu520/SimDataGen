# 封装isaacsim镜像
```
docker pull nvcr.io/nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04

# 在docker的CUDA官方镜像中，这些内容要重新安装
apt-get update && apt-get install -y libgl1 libxt6 libxml2 libxrandr2 libxinerama1 libxcursor1 libxi6 libvulkan1 zenity  libglu1-mesa

```