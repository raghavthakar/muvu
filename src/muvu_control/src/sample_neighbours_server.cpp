#include <cmath>
#include "ros/ros.h"
#include "muvu_control/SampleNeighbours.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

#define CELL_UNIT 50

class Sampler
{
  ros::NodeHandle node_handle;
  std::string laserscan_topic="/muvu/laser/scan";
  std::string odom_topic="/muvu/odom";
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

  nav_msgs::OdometryConstPtr getOdometry()
  {
    return ros::topic::waitForMessage<nav_msgs::Odometry>(odom_topic);;
  }

  double getYaw(nav_msgs::OdometryConstPtr o)
  {
    double roll, pitch, yaw;

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(o->pose.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    return yaw;
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
    std_msgs::Bool temp;

    double yaw=getYaw(getOdometry());

    //[POS_X, POS_Y, NEG_X, NEG_Y]
    //REFERENCE ANGLE IS DIRECT ANGLE FOR +VE X FACING
    int reference_angle=round(180*(1-(double)(yaw/3.141)));

    // temp.data=int(!getObstacles(reference_angle, laser_data));
    // response.neighbours.push_back(temp);
    //
    // temp.data=int(!getObstacles(reference_angle+90, laser_data));
    // response.neighbours.push_back(temp);
    //
    // temp.data=int(!getObstacles((reference_angle+180)%360, laser_data));
    // response.neighbours.push_back(temp);
    //
    // temp.data=int(!getObstacles(reference_angle-90, laser_data));
    // response.neighbours.push_back(temp);

    //Check if there is obstacle directly behind


    //anticlockwise, starting from thr back
    temp.data=int(!getObstacles(0, laser_data));
    response.neighbours.push_back(temp);

    //right
    temp.data=int(!getObstacles(90, laser_data));
    response.neighbours.push_back(temp);

    //front
    temp.data=int(!getObstacles(180, laser_data));
    response.neighbours.push_back(temp);

    //left
    temp.data=int(!getObstacles(270, laser_data));
    response.neighbours.push_back(temp);

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
