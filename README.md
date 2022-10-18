# graph_search

## 1. 创建ROS工作空间，编译代码
首先创建ROS工作空间并初始化，这里假定工作空间命名为catkin_ws
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src/
$ catkin_init_workspace
```
然后将该仓库克隆到本地
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/NKU-PnC-Course/graph_search.git
```
完成GraphSearch类后，编译程序
```
$ cd ~/catkin_ws/
$ catkin_make
```

## 2. 给定任意起点和终点
打开新的终端，source一下工作空间的路径
```
$ source ~/catkin_ws/devel/setup.bash
```

然后运行launch文件
```
$ roslaunch graph_search demo.launch
```

这时候会弹出RViz可视化界面，点击2D Pose Estimate设置起点，2D Nav Goal设置终点，即可运行A*算法程序并可视化搜索到的全局路径和搜索过程中访问的栅格

## 3. 给定指定起点和终点
打开新的终端，source一下工作空间的路径并运行launch文件
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch graph_search demo.launch
```
打开另一新的终端，source一下工作空间的路径并通过程序指定起点和终点
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch graph_search start_and_goal.launch
```
