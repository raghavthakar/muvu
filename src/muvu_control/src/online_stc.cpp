#include "ros/ros.h"
#include "muvu_control/SampleNeighbours.h"
#include "muvu_control/MoveDistance.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

enum {POS_X, POS_Y, NEG_X, NEG_Y};

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

  std::list<Cell> old_cells;

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

  void operate()
  {
    //Add current cell to list of visited cells
    // old_cells.push_back(*curr_cell);

    //sample neighbouring cells
    sample_client.call(sample_srv);

    //Generate a list of free neghbours from server response
    for(int dir = 0; dir < 4; dir++)
    {
      if(dir == POS_X && sample_srv.response.neighbours[POS_X].data==1)
      {
        move_srv.request.target.x+=0.25;
        move_client.call(move_srv);

        operate();
      }
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "online_stc");

  STC stc;

  Cell start_cell(0.25, 0.25);

  stc.operate();

  return 0;
}
