#include "amaze_mission_sequencer.h"


int main(int argc, char **argv)
{
    std::string name(argv[0]);
    ROS_INFO("%s started", name.c_str());
    ros::init(argc, argv, name.c_str());

    ros::NodeHandle nh("~");


    AmazeMissionSequencer follower(nh);

    ros::Rate rate(20.0);    

    while (ros::ok())
    {
        follower.logic(); // Non-blocking logic ;)
        follower.publishPoseSetpoint();
        ros::spinOnce();
        rate.sleep();
    }

    // // Check for new messages and loop
    // ros::spin();

    return 0;
}