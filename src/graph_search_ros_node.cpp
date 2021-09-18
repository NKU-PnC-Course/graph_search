#include <ros/ros.h>
#include "graph_search/graph_search_ros.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "graph_search");

  GraphSearchROS graph_search;

  ros::spin();

  return 0;
}