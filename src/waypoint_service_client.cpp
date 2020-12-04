#include "ros/ros.h"
#include "amaze_waypoint_following/wp_service.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wp_service_client");
  if (argc != 6)
  {
    ROS_INFO("usage: wp_service_client Use_case X Y Z degrees");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<amaze_waypoint_following::wp_service>("wp_service");
  amaze_waypoint_following::wp_service srv;
  srv.request.mode = atoll(argv[1]);
  srv.request.x = atoll(argv[2]);
  srv.request.y = atoll(argv[3]);
  srv.request.z = atoll(argv[4]);
  srv.request.yaw = atoll(argv[5]);
  if (client.call(srv))
  {
    ROS_INFO("waypoint reached: %d", (bool)srv.response.wp_reached);
  }
  else
  {
    ROS_ERROR("Failed to call service wp_service");
    return 1;
  }

  return 0;
}
