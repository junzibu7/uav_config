# uav_config

## 介绍
来自https://gitee.com/myboyhood/uav_config ，根据自己的项目做了更改。

## 使用方法

### Step1

从Gitee仓库下载代码，根据需求修改配置从文件。

注意，对yaml文件进行项的增加或减少时，需要对uav_config/include/uav_config/read_config_drone.h进行相应的修改。

### Step2

修改uav_config/src/uav_config_parse.cpp中的文件路径。

### Step3

编译：
1. 从终端中进入uav_config文件夹
2. 执行`mkdir build`
3. 执行`cd build`
4. 执行`cmake ..`
5. 执行`make`

此时在build文件夹中会生成一个可执行文件。
# uav_config
