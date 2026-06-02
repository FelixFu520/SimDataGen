## 安装isaacsim
### 安装isaacsim和对应资产
官方推荐安装到$HOME/isaacsim文件夹下，但是我这个文件已经创建了，我就安装到$HOME/isaac_sim中了
```
mkdir -p $HOME/isaac_sim
cd $HOME/isaac_sim

# 下载软件和资产
wget https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip
wget https://download.isaacsim.omniverse.nvidia.com/isaac-sim-assets-complete-5.1.0.zip.001
wget https://download.isaacsim.omniverse.nvidia.com/isaac-sim-assets-complete-5.1.0.zip.002
wget https://download.isaacsim.omniverse.nvidia.com/isaac-sim-assets-complete-5.1.0.zip.003

# 合并资产
cat isaac-sim-assets-complete-5.1.0.zip.001 isaac-sim-assets-complete-5.1.0.zip.002 isaac-sim-assets-complete-5.1.0.zip.003 > isaac-sim-assets-5.1.0.zip

# 解压
unzip -d 5.1/ isaac-sim-standalone-5.1.0-linux-x86_64.zip
unzip -d 5.1_asset isaac-sim-assets-5.1.0.zip

# git管理, 有时候会误修改isaacsim软件中代码，所以用git管理下，如果有变化，可以及时发现
cd $HOME/isaac_sim/5.1/
git init
git add .
git commit -m "init"

cd $HOME/isaac_sim/5.1_asset/
git init
git add .
git commit -m "init"

```
### 配置isaacsim默认资产
isaacsim资产可以从网络和本地加载，上一步已经下载了资产，现在配置下，让isaacsim可以识别到
#### 5.1 配置
参考：https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_faq.html

编辑apps/isaacsim.exp.base.kit文件，根据自身情况修改

```
[settings]
persistent.isaac.asset_root.default = "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1"

exts."isaacsim.gui.content_browser".folders = [
    "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1/Isaac/Robots",
    "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1/Isaac/People",
    "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1/Isaac/IsaacLab",
    "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1/Isaac/Props",
    "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1/Isaac/Environments",
    "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1/Isaac/Materials",
    "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1/Isaac/Samples",
    "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1/Isaac/Sensors",
]

exts."isaacsim.asset.browser".folders = [
    "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1/Isaac/Robots",
    "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1/Isaac/People",
    "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1/Isaac/IsaacLab",
    "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1/Isaac/Props",
    "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1/Isaac/Environments",
    "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1/Isaac/Materials",
    "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1/Isaac/Samples",
    "/home/fufa/isaac_sim/5.1_asset/Assets/Isaac/5.1/Isaac/Sensors",
]

```
## 安装本项目
```
cd $HOME
git clone git@github.com:FelixFu520/SimDataGen.git
cd $HOME/SimDataGen

# 链接isaacsim, 我这里链接的是isaacsim5.1，可以根据需要选择不同的版本
ln -s $HOME/isaac_sim/5.1 app   # 链接isaacsim5.1
ln -s $HOME/isaac_sim/5.1_asset asset_internal  # 链接isaacsim5.1资产
ln -s $HOME/isaac_sim/extern_asset asset_extern # 链接生成的场景资产
```