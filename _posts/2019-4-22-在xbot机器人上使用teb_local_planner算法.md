---
layout: post
title: 在xbot机器人上使用teb_local_planner算法
subtitle: 基于ros插件机制更换局部规划算法
date: 2019-04-23

header-img: img/teb.jpg
catalog: true
tags:
  - 机器人
  - 运动规划
---

## 一些概念
自适应蒙特卡洛：
1. 当发现粒子点平均分突变的时候（机器人绑架），通过全局重新撒点进行计算
2. 根据聚集程度判断粒子足够精确的时候，降低需要维护的粒子数目，减少计算开销

代价地图：
1. 静态层static_layer：map_server提供
2. 障碍层obstacle_layer: 激光雷达提供
3. 全局膨胀层global_inflation_layer
4. 局部膨胀层local_inflation_layer

## 使用teb路径规划算法

#### 安装
在xbot机器人上配置使用teb路径规划算法，根据这个[教程][1]来实现

其中关键注意事项：
1. 查看插件是否安装完成：
`rospack plugins --attrib=plugin nav_core`
2. 需要新建teb_local_planner的配置文件：
这里可以直接从tutorial下载使用，日后需要更改参数，可以在这个基础上改动。
3. 同时需要在载入对应的move_base_param时，将base_local_planner改动为： teb_local_planner/TebLocalPlannerROS


## ROS的插件机制
这部分主要参考这篇[博文][2]

所有的movebase插件都是继承nav_core， nav_core定义了插件接口的类型，然后所有三大类插件继承，实现了对应的借口


```

controller_frequency: 5.0   # 局部规划发送控制指令的频率
controller_patience: 3.0    #


planner_frequency: 2.0      # 全局规划频率
planner_patience: 5.0       # 在清空操作之前的等待时间

oscillation_timeout: 10.0 #10
oscillation_distance: 0.2 #0.2

# local planner - default is trajectory rollout
base_local_planner: "dwa_local_planner/DWAPlannerROS" #"base_local_planner/TrajectoryPlannerROS" #

base_global_planner: "navfn/NavfnROS" #alternatives: global_planner/GlobalPlanner, carrot_planner/CarrotPlanner

```

## teb算法详解


## navigation导航详解

navigation stack中核心部分就是五大关键插件，具体包括：
全局规划器： global_planner
全局代价地图： global_costmap
局部规划器： local_planner
局部代价地图： local_costmap
恢复行为： recovery_behaviors

上述的五个借口都是可以通过插件机制做替换，比如本文就是使用teb_local_planner替换local_planner做局部规划。

move_base的核心就是上述的五大接口，其中与外界的交互包括如下内容：

1. 需要一个节点node给movebase发送goal指令，实际是发送给global_planner插件。
2. 向外发送控制指令‘cmd_vel’到base controller，这部分是由local_planner发送指令
3. 如果启动了map_server，那global_costmap是在map_server基础上添加layer,如果没有map_server, 那当前的全局地图的建立实际是源于slam
其次，在调用/map的时候，其实是调用一个service
4. local_costmap的建立要依赖当前传感器
5. costmap出问题的时候，就会进入recovery_behaviors模式
6. costmap主要用于2d导航，每个costmap都可以是很多层叠加生成最终效果。前面基础概念已经提到对应的层
7. 定位的本质是要知道我在map中的位置，我指的是机器人的base_link，所以一切定位的根本目的是要找到base_link和map这两个坐标系的变化关系。
其实我们知道的只有odom和base之间的变化，传统方法认为odom就是map. 的确，如果没有轮子的漂移，map就是odom坐标系。
amcl的方法就是使用传感器纠正在弥补map和odom之间的累计误差
8. 关于tf的一些说明：机器人关节的定义，相互之间的距离，都是通过urdf文件定义的，比如我们机器人上的各种传感器相对于底盘的/tf变化，我们都已经在urdf中申明。
9. local_planner做规划，依赖三部分信息：全局规划器，局部代价地图，源于topic"odom"的Odometry信息，具体的信息格式如下：

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
```

## 当前待解决的问题列表

1. LaserScan和PointCloud是哪个传感器发出的？
2. local_planner怎么利用其他信息进行局部规划？
3. 全局规划和局部规划的划分界限是在哪里？
4. odometry信息的具体含义是什么?源于哪里？进行了怎样的处理？局部规划器怎么使用该信息？
5. 进入recovery_behaviors模式的条件？怎么算costmap出问题？
6. slam建图的时候，怎么判断是静态障碍物和动态障碍物的？
7. odom 坐标系到底是指什么？谁在维护这个frame？odom这个topic发布什么类型的msg？
8. 需要看一下nav_msgs/OccupancyGrid.msg中的Pose具体是什么? 在柴老师视频中，他说这是，当前初始地图相对/map这个frame的变化关系。这里需要验证！
9. 查看/tf这个topic的样子

残留工作： service要学习，action要学习，需要在机器人平台上过一遍。

**回答：**

##### 第一题：LaserScan和PointCloud

这部分参考了文章[ROS之发布传感器数据（LaserScan和PointCloud][3]，文章单独实现了两种数据格式的收发，值得看一看
目前导航功能包集只接受使用sensor_msgs/LaserScan或sensor_msgs/PointCloud消息类型发布的传感器数据

个人分析认为：LaserScan是二维，PointCloud是三维
LaserScan 属于/scan话题
具体信息格式如下：

```
std_msgs/Header header
  uint32 seq
  time stamptf这个topic的样子
  string frame_id
  #
  # 测量的激光扫描角度，逆时针为正
  # 设备坐标帧的0度面向前（沿着X轴方向）
  #
  float32 angle_min        # scan的开始角度 [弧度]
  float32 angle_max        # scan的结束角度 [弧度]
  float32 angle_increment  # 测量的角度间的距离 [弧度]
  float32 time_increment   # 测量间的时间 [秒]
  float32 scan_time        # 扫描间的时间 [秒]
  float32 range_min        # 最小的测量距离 [米]
  float32 range_max        # 最大的测量距离 [米]
  float32[] ranges         # 测量的距离数据 [米] (注意: 值 < range_min 或 > range_max 应当被丢弃)
  float32[] intensities    # 强度数据 [device-specific units] 可以利用其判断材质

======举个例子============

header:
  seq: 9019
  stamp:
    secs: 1556090239
    nsecs: 420321047
  frame_id: "laser"
angle_min: -3.12413907051
angle_max: 3.14159274101
angle_increment: 0.0174532923847
time_increment: 0.000220614529098
scan_time: 0.079200617969
range_min: 0.15000000596
range_max: 12.0
ranges: "<array type: float32, length: 360>"
intensities: "<array type: float32, length: 360>"

```

PointClouds msg格式如下：

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Point32[] points
  float32 x
  float32 y
  float32 z
sensor_msgs/ChannelFloat32[] channels
  string name
  float32[] values

```

PointCloud2的msg格式：

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
#  这个是选择数据格式
sensor_msgs/PointField[] fields
  uint8 INT8=1
  uint8 UINT8=2
  uint8 INT16=3
  uint8 UINT16=4
  uint8 INT32=5
  uint8 UINT32=6
  uint8 FLOAT32=7
  uint8 FLOAT64=8
  string name
  uint32 offset
  uint8 datatype
  uint32 count
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense

```

##### 第二题


##### 第三题


##### 第四题


##### 第五题


##### 第六题


##### 第七题 针对/odom的探究  ×

odom本身代表里程计，实际应用中可以是轮子上的编码器，也可以是摄像头做视觉里程计或IMU。slam核心节点会订阅/tf，这个/tf必须包含/odom和/base_link之间的变化关系，通常这个odom信息的就是上述各式各样的里程计来维护。

这里需要去了解tf具体工作机制。 通常slam定位的工作，就是slam核心节点发布的/tf，这个/tf包含/map frame到/odom frame的信息

我来试着解释一下：我之前是被rviz有些束缚，因为rviz固定使用的是map坐标系。所以我们看到的好象是odom在相对远离map，其实如果我们固定使用odom坐标系，同样可以觉察到map在相对远离。

当把激光雷达纠正这步去掉，只看里程计定位的时候，我们误以为里程计的odom就是map
我们是依据这个假的map信息，在做定位。而建图的效果，完全依赖定位。所以slam的核心就是定位。如何准确给出自己当前在map中的位置，是slam最大的难点。

只使用里程计的定位算法，等于是蒙着眼睛走路。
加上激光雷达的定位算法，等于这个盲人多了一根拐杖。
加上视觉定位的算法，相当于睁开了眼睛。

这里还需要考虑，就比从如amcl使用粒子滤波，他是怎么做这个事情的？是怎么弥补odom和map之间的误差的？

##### 第八题


##### 第九题 TF topic具体内容
**第一部分：** 节点维护的一小段的tf（两个frame之间的link，也就是boardcaster）需要发送的数据格式： TransformStamped.msg:

```
std_msgs/Header header
  uint32 seq              // 序号
  time stamp              // 时间
  string frame_id         // 当前frame
string child_frame_id     // 子frame名称
geometry_msgs/Transform transform   // 向量表示平移，四元数表示旋转
  geometry_msgs/Vector3 translation
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion rotation
    float64 x
    float64 y
    float64 z
    float64 w
```
**第二部分：** 整个tf tree的topic的数据格式：
这部分有两种形式：tf/tfMessage.msg 和 tf2_msgs/tfMessage.msg 只是因为升级了tf就变成了tf2
里面的内容完全一样：
`geometry_msgs/TransformStamped[] transforms `
用一个数组描述一组transformStamped，这样就构成了一个 tf_tree

## 参考文献

[1]:  https://blog.csdn.net/xiekaikaibing/article/details/80197164
[2]:  http://www.cnblogs.com/W-chocolate/p/4328725.html
[3]:  https://blog.csdn.net/zong596568821xp/article/details/79111061


1. https://blog.csdn.net/xiekaikaibing/article/details/80197164
2. http://www.cnblogs.com/W-chocolate/p/4328725.html
3. https://blog.csdn.net/zong596568821xp/article/details/79111061
