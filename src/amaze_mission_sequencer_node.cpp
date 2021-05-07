#include "amaze_mission_sequencer.h"


int main(int argc, char **argv)
{
    // Node handle creation
    ros::init(argc, argv, "amaze_mission_sequencer_node");
    ros::NodeHandle nh("amaze_mission_sequencer_node");

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