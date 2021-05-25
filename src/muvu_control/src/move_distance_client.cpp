#include "ros/ros.h"
#include "muvu_control/MoveDistance.h"

int main(int argc, char **argv)
{
  geometry_msgs::Point target_point;
  target_point.x=-5;
  target_point.y=-5;

  ros::init(argc, argv, "move_distance_client");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient
                    <muvu_control::MoveDistance>("move_distance");

  //Instantiate the service class
  muvu_control::MoveDistance srv;

  //Inout the request value
  srv.request.target = target_point;

  //Call the actual service. returns true if succeeded
  if (client.call(srv))
  {
    ROS_INFO("Called the service, and done");
  }
  else
  {
    ROS_ERROR("Failed to call service move_distance");
    return 1;
  }

  target_point.x=10;
  target_point.y=0;

  //Inout the request value
  srv.request.target = target_point;

  //Call the actual service. returns true if succeeded
  if (client.call(srv))
  {
    ROS_INFO("Called the service, and done");
  }
  else
  {
    ROS_ERROR("Failed to call service move_distance");
    return 1;
  }

  return 0;
}
