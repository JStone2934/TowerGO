# 环境

使用 scripts/container.sh build/run。基础镜像支持 ARM64，ROS发行版固定Humble。构建期下载依赖，运行期network none、Domain125、独立IPC。

主机仅需可用Docker和sudo权限，不安装或替换主机ROS。运行以宿主用户UID/GID写入data，日志和地图不归root所有。构建镜像记录依赖版本和镜像inspect结果。

升级依赖后必须重跑实验；禁止直接套用Rolling版Nav2参数。
