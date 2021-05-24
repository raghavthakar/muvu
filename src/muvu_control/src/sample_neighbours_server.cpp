#include "ros/ros.h"
#include "muvu_control/SampleNeighbours.h"
#include "sensor_msgs/LaserScan.h"

class Sampler
{
  ros::NodeHandle node_handle;
  std::string laserscan_topic="/muvu/laser/scan";
  sensor_msgs::LaserScanConstPtr laser_data;
  ros::ServiceServer sample_server;

  bool getObstacles(int ray_num, sensor_msgs::LaserScanConstPtr laser_data)
  {
    //write better detection formula
    if(laser_data->ranges[ray_num]<1.7)
    {
      return true;
    }

    return false;
  }

public:
  Sampler()
  {
    sample_server = node_handle.advertiseService("sample_neighbours",
                                &Sampler::sample, this);
  }

  bool sample(muvu_control::SampleNeighbours::Request &request,
              muvu_control::SampleNeighbours::Response &response)
  {
    laser_data=ros::topic::waitForMessage
                <sensor_msgs::LaserScan>(laserscan_topic);

    //Calculate laser index ranges accurately
    //360 is ray pointing straight
    response.forward_free.data=int(!getObstacles(360, laser_data));
    response.left_free.data=int(!getObstacles(540, laser_data));
    response.backward_free.data=int(!getObstacles(1, laser_data));
    response.right_free.data=int(!getObstacles(180, laser_data));

    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_neighbours_server");
  Sampler sampler;
  ros::spin();
  return 0;
}
