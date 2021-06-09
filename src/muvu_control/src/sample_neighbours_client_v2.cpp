#include "ros/ros.h"
#include "muvu_control/SampleNeighbours.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_neighbours_client");
  ros::NodeHandle node_handle;

  ros::ServiceClient client = node_handle.serviceClient
                          <muvu_control::SampleNeighbours>("sample_neighbours");

  //Instantiate the service class
  muvu_control::SampleNeighbours srv;

  //Call the actual service. returns true if succeeded
  if (client.call(srv))
  {
    ROS_INFO("Called the service, and done");
    ROS_INFO("X: %f Y: %f", srv.response.neighbours[0].x, srv.response.neighbours[0].y);
    ROS_INFO("X: %f Y: %f", srv.response.neighbours[1].x, srv.response.neighbours[1].y);
    ROS_INFO("X: %f Y: %f", srv.response.neighbours[2].x, srv.response.neighbours[2].y);
    ROS_INFO("X: %f Y: %f", srv.response.neighbours[3].x, srv.response.neighbours[3].y);
  }
  else
  {
    ROS_ERROR("Failed to call service sample_neighbours");
    return 1;
  }
}
