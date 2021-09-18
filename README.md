# graph_search

完成程序编写后先编译：
```
$ cd ~/catkin_ws/
$ catkin_make
```

然后source一下工作空间：
```
$ source ~/catkin_ws/devel/setup.bash
```

最后运行launch文件
```
$ roslaunch graph_search demo.launch
```

这时候会弹出RViz可视化界面，点击2D Pose Estimate设置起点，2D Nav Goal设置终点

打开新的终端，然后source一下工作空间，通过程序给定给定起点和终点（无需关闭之前的界面）
```
$ roslaunch graph_search start_and_goal.launch
```