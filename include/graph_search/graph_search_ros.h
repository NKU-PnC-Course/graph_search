#ifndef _GRAPH_SEARCH_ROS_H_
#define _GRAPH_SEARCH_ROS_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include "graph_search/graph_search.h"
#include "graph_search/planner_interface.h"

class GraphSearchROS
{
public:
  GraphSearchROS();
  virtual ~GraphSearchROS();

private:
  void setMap(const nav_msgs::OccupancyGrid::Ptr map);
  void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);
  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);
  void makePlan();
  void publishPath(const std::vector<PointInt>& path);
  void publishExpands(const std::vector<PointInt>& expands);

private:
  /// A subscriber for receiving map updates
  ros::Subscriber map_sub_;
  /// A subscriber for receiving goal updates
  ros::Subscriber goal_sub_;
  /// A subscriber for receiving start updates
  ros::Subscriber start_sub_;

  /// A pointer to the grid the planner runs on
  int width_, height_;
  double origin_x_, origin_y_;
  float resolution_;
  unsigned char** map_;
  /// The start pose set through RViz
  int start_x_, start_y_, goal_x_, goal_y_;
  /// Flags for allowing the planner to plan
  bool valid_start_ = false;
  /// Flags for allowing the planner to plan
  bool valid_goal_ = false;

  PlannerInterface* planner_;
  ros::Publisher path_pub_;
  ros::Publisher expands_pub_;
};

#endif  // _GRAPH_SEARCH_ROS_H_