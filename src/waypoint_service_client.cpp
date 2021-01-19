// Copyright (C) 2020 Control of Networked Systems, Universit?t Klagenfurt, Austria
//
// All rights reserved.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

///
/// \brief PX4 offboard waypoint cmd service client
/// Tested in Gazebo SITL and real-world
/// Usage: rosrun amaze_waypoint_following waypoint_service_client 0 0 1 90
///

#include <amaze_waypoint_following/wp_service.h>
#include <ros/ros.h>
#include <cstdlib>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wp_service_client");
  if (argc != 6)
  {
    ROS_INFO("Usage: wp_service_client Use_case X Y Z degrees");
    return EXIT_FAILURE;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<amaze_waypoint_following::wp_service>("wp_service");
  amaze_waypoint_following::wp_service srv;

  srv.request.mode = char(atoll(argv[1]));
  srv.request.x = atof(argv[2]);
  srv.request.y = atof(argv[3]);
  srv.request.z = atof(argv[4]);
  srv.request.yaw = atof(argv[5]);
  // changing atof to atoll would let the client able to request int values only

  if (client.call(srv))
  {
    ROS_INFO("waypoint reached: %d", bool(srv.response.wp_reached));
  }
  else
  {
    ROS_ERROR("Failed to call service wp_service");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
