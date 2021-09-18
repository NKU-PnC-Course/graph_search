#include "graph_search/graph_search_ros.h"

GraphSearchROS::GraphSearchROS() : planner_(NULL), map_(NULL)
{
  planner_ = new GraphSearch();
  start_x_ = -1;
  start_y_ = -1;
  goal_x_ = -1;
  goal_y_ = -1;

  ros::NodeHandle nh;

  map_sub_ = nh.subscribe("map", 1, &GraphSearchROS::setMap, this);
  start_sub_ = nh.subscribe("initialpose", 1, &GraphSearchROS::setStart, this);
  goal_sub_ = nh.subscribe("move_base_simple/goal", 1, &GraphSearchROS::setGoal, this);

  path_pub_ = nh.advertise<nav_msgs::Path>("path", 1);
  expands_pub_ = nh.advertise<visualization_msgs::Marker>("expands", 1);
}

GraphSearchROS::~GraphSearchROS()
{
  if (planner_)
    delete planner_;

  if (map_)
  {
    for (int i = 0; i < width_; i++)
      delete[] map_[i];
    delete[] map_;
  }
}

void GraphSearchROS::setMap(const nav_msgs::OccupancyGrid::Ptr map)
{
  width_ = map->info.width;
  height_ = map->info.height;
  resolution_ = map->info.resolution;
  origin_x_ = map->info.origin.position.x;
  origin_y_ = map->info.origin.position.y;

  map_ = new unsigned char*[width_];
  for (int i = 0; i < width_; i++)
    map_[i] = new unsigned char[height_];

  for (int i = 0; i < width_; i++)
  {
    for (int j = 0; j < height_; j++)
    {
      int index = i + j * width_;
      map_[i][j] = map->data[index];
    }
  }
}

void GraphSearchROS::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial)
{
  int start_x = CONTXY2DISC(initial->pose.pose.position.x - origin_x_, resolution_);
  int start_y = CONTXY2DISC(initial->pose.pose.position.y - origin_y_, resolution_);

  if (start_x == start_x_ && start_y == start_y_)
  {
    return;
  }
  else
  {
    start_x_ = start_x;
    start_y_ = start_y;
    ROS_INFO("A new start x=%d, y=%d is received.", start_x_, start_y_);
  }

  if (height_ > start_y_ && start_y_ >= 0 && width_ > start_x_ && start_x_ >= 0)
  {
    valid_start_ = true;
    makePlan();
  }
  else
  {
    ROS_INFO("Invalid start x=%d, y=%d", start_x_, start_y_);
  }
}

void GraphSearchROS::setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  // retrieving goal position
  int goal_x = CONTXY2DISC(goal->pose.position.x - origin_x_, resolution_);
  int goal_y = CONTXY2DISC(goal->pose.position.y - origin_y_, resolution_);

  if (goal_x == goal_x_ && goal_y == goal_y_)
  {
    return;
  }
  else
  {
    goal_x_ = goal_x;
    goal_y_ = goal_y;
    ROS_INFO("A new goal x=%d, y=%d is received.", goal_x_, goal_y_);
  }

  if (height_ > goal_y_ && goal_y_ >= 0 && width_ > goal_x_ && goal_x_ >= 0)
  {
    valid_goal_ = true;
    makePlan();
  }
  else
  {
    ROS_INFO("Invalid goal x=%d, y=%d", goal_x_, goal_y_);
  }
}

void GraphSearchROS::publishPath(const std::vector<PointInt>& path)
{
  // publish path
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = "map";
  gui_path.header.stamp = ros::Time::now();
  gui_path.poses.resize(path.size());
  for (int i = 0; i < (int)path.size(); i++)
  {
    double x = DISCXY2CONT(path[i].x, resolution_) + origin_x_;
    double y = DISCXY2CONT(path[i].y, resolution_) + origin_y_;
    gui_path.poses[i].pose.position.x = x;
    gui_path.poses[i].pose.position.y = y;
  }

  path_pub_.publish(gui_path);
}

void GraphSearchROS::publishExpands(const std::vector<PointInt>& expands)
{
  // publish expanded states
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::POINTS;
  marker.scale.x = resolution_ * 0.5;
  marker.scale.y = resolution_ * 0.5;
  marker.scale.z = resolution_ * 0.5;
  marker.color.a = 1.0;
  marker.color.b = 1.0;
  marker.color.g = 0.0;
  marker.color.r = 0.0;
  marker.points.resize(expands.size());
  for (int i = 0; i < (int)expands.size(); i++)
  {
    double x = DISCXY2CONT(expands[i].x, resolution_) + origin_x_;
    double y = DISCXY2CONT(expands[i].y, resolution_) + origin_y_;
    marker.points[i].x = x;
    marker.points[i].y = y;
  }

  expands_pub_.publish(marker);
}

void GraphSearchROS::makePlan()
{
  // if a start as well as goal are defined go ahead and plan
  if (valid_start_ && valid_goal_)
  {
    std::vector<PointInt> path;
    std::vector<PointInt> expands;
    planner_->Search(start_x_, start_y_, goal_x_, goal_y_, map_, width_, height_, 100, path, expands);

    publishPath(path);
    publishExpands(expands);
  }
}