#include "ros/ros.h"
#include "muvu_control/SampleNeighbours.h"
#include "muvu_control/MoveDistance.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

#define BACKWARD 0
#define RIGHT 1
#define FRONT 2
#define LEFT 3

class Cell
{
  double x;
  double y;

  Cell* child;
  Cell* parent;

public:
  Cell(double x, double y)
  {
    this->x=x;
    this->y=y;
  }
};


class STC
{
  ros::NodeHandle node_handle;

  ros::ServiceClient move_client;
  ros::ServiceClient sample_client;

  nav_msgs::OdometryConstPtr current_odom;
  std::string odom_topic="/muvu/odom";

  double current_yaw;

  muvu_control::MoveDistance move_srv;
  muvu_control::SampleNeighbours sample_srv;

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
  STC()
  {
    move_client = node_handle.serviceClient
                  <muvu_control::MoveDistance>("move_distance");

    sample_client = node_handle.serviceClient
                    <muvu_control::SampleNeighbours>("sample_neighbours");
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "online_stc");

  STC stc;

  return 0;
}
