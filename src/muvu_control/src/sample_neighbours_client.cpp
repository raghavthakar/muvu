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
    ROS_INFO("%d", srv.response.neighbours[0].data);
    ROS_INFO("%d", srv.response.neighbours[1].data);
    ROS_INFO("%d", srv.response.neighbours[2].data);
    ROS_INFO("%d", srv.response.neighbours[3].data);

    // ROS_INFO("%d", srv.response.left_free.data);
    // ROS_INFO("%d", srv.response.backward_free.data);
    // ROS_INFO("%d", srv.response.right_free.data);
  }
  else
  {
    ROS_ERROR("Failed to call service sample_neighbours");
    return 1;
  }
}
