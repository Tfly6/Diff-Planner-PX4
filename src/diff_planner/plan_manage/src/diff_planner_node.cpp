// #include <ros/ros.h>
// #include <csignal>
// #include <visualization_msgs/Marker.h>

// #include <plan_manage/diff_replan_fsm.h>

// using namespace diff_planner;

// void SignalHandler(int signal) {
//   if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
//     ros::shutdown();
//   }
// }

// int main(int argc, char **argv) {

//   signal(SIGINT, SignalHandler);
//   signal(SIGTERM,SignalHandler);

//   ros::init(argc, argv, "diff_planner_node", ros::init_options::NoSigintHandler);
//   ros::NodeHandle nh("~");

//   DiffReplanFSM rebo_replan;

//   rebo_replan.init(nh);

//   // ros::Duration(1.0).sleep();
//   ros::AsyncSpinner async_spinner(4);
//   async_spinner.start();
//   ros::waitForShutdown();

//   return 0;
// }

#include <rclcpp/rclcpp.hpp>

#include <plan_manage/diff_replan_fsm.h>

using namespace diff_planner;

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("diff_planner_node");

  DiffReplanFSM rebo_replan;

  rebo_replan.init(node);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
