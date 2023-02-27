# 沙盘项目
## ROS 版本ros2 测试过 foxy humble

## 此分支为发布主分区

### 功能包介绍

## 编译
一键安装ROS 
```
wget http://fishros.com/install -O fishros && . fishros

```

安装依赖
```
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
不重复只编译一个包
```
colcon build --packages-select YOUR_PKG_NAME 
```

只编译一个包
```
colcon build --symlink-install --packages-select YOUR_PKG_NAME 
```

不编译测试单元
```
colcon build --packages-select YOUR_PKG_NAME  --cmake-args -DBUILD_TESTING=0
```