#include "mission_sequencer.hpp"

int main(int argc, char** argv)
{
  std::string name(argv[0]);
  ROS_INFO("%s started", name.c_str());
  ros::init(argc, argv, name.c_str());

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  mission_sequencer::MissionSequencer sequencer(nh, private_nh);

  ros::Rate rate(20.0);

  while (ros::ok())
  {
    sequencer.logic();  // Non-blocking logic ;)
    sequencer.publishPoseSetpoint();
    ros::spinOnce();
    rate.sleep();
  }

  // // Check for new messages and loop
  // ros::spin();

  return 0;
}
