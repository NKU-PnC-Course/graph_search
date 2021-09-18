#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "start_goal_publisher");

  ros::NodeHandle nh;
  ros::Publisher start_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

  // read initial and target pose
  ros::NodeHandle private_nh_("~");
  double initial_pose_x, initial_pose_y, initial_pose_a, goal_pose_x, goal_pose_y, goal_pose_a;
  if (!private_nh_.getParam("initial_pose_x", initial_pose_x))
  {
    ROS_ERROR("Failed to set initial pose");
  }
  if (!private_nh_.getParam("initial_pose_y", initial_pose_y))
  {
    ROS_ERROR("Failed to set initial pose");
  }
  if (!private_nh_.getParam("initial_pose_a", initial_pose_a))
  {
    ROS_ERROR("Failed to set initial pose");
  }
  ROS_INFO("Initial robot pose: (%f, %f, %f)", initial_pose_x, initial_pose_y, initial_pose_a);

  if (!private_nh_.getParam("goal_pose_x", goal_pose_x))
  {
    ROS_ERROR("Failed to set goal pose");
  }
  if (!private_nh_.getParam("goal_pose_y", goal_pose_y))
  {
    ROS_ERROR("Failed to set goal pose");
  }
  if (!private_nh_.getParam("goal_pose_a", goal_pose_a))
  {
    ROS_ERROR("Failed to set goal pose");
  }
  ROS_INFO("Goal robot pose: (%f, %f, %f)", goal_pose_x, goal_pose_y, goal_pose_a);

  geometry_msgs::PoseWithCovarianceStamped start;
  geometry_msgs::PoseStamped goal;

  start.header.frame_id = "map";
  start.pose.pose.position.x = initial_pose_x;
  start.pose.pose.position.y = initial_pose_y;
  start.pose.pose.orientation = tf::createQuaternionMsgFromYaw(initial_pose_a);
  goal.header.frame_id = "map";
  goal.pose.position.x = goal_pose_x;
  goal.pose.position.y = goal_pose_y;
  goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_pose_a);

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    start.header.stamp = ros::Time::now();
    goal.header.stamp = ros::Time::now();
    start_pub.publish(start);
    goal_pub.publish(goal);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}