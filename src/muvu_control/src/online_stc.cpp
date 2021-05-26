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
  Cell(nav_msgs::OdometryConstPtr current_odom, dir)
  {
    switch(dir)
    {
      case POS_X:
      {
        x=current_odom->pose.pose.point.x+0.5;
        y=current_odom->pose.pose.point.y;
        break;
      }
      case POS_Y:
      {
        y=current_odom->pose.pose.point.y+0.5;
        x=current_odom->pose.pose.point.x;
        break;
      }
      case NEG_X:
      {
        x=current_odom->pose.pose.point.x-0.5;
        y=current_odom->pose.pose.point.y;
        break;
      }
      case NEG_Y:
      {
        y=current_odom->pose.pose.point.y-0.5;
        x=current_odom->pose.pose.point.x;
        break;
      }
    }
  }
};

class STC
{
  ros::NodeHandle node_handle;

  //Client objects for each service
  ros::ServiceClient move_client;
  ros::ServiceClient sample_client;

  //tracks the current odometry
  nav_msgs::OdometryConstPtr current_odom;
  //topic where odometry is published
  std::string odom_topic="/muvu/odom";

  //Request/response handlers
  muvu_control::MoveDistance move_srv;
  muvu_control::SampleNeighbours sample_srv;

  //global list tracks the cells that have been visited
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

  void operate(Cell* prev_cell, Cell* curr_cell)
  {
    //Add current cell to list of visited cells
    old_cells.push_back(*curr_cell);

    //sample neighbouring cells
    sample_client.call(sample_srv);

    //Get the current odometry
    current_odom=getOdometry();

    //Generate a list of free neghbours from server response
    std::list<Cell> neighbouring_free_cells;

    //If the cell is free then add it to the list of free neighbouring cells
    for(int dir=0; dir<4; dir++)
      if(sample_srv.response.neighbours[dir].data==1)
        //give the current odom as a paramaeter so that it can be
        // used to set coordinates of the child cell
        neighbouring_free_cells.push_back(Cell(current_odom, dir));

    

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "online_stc");

  STC stc;

  Cell* start cell=new Cell(0.25, 0.25);

  stc.operate();

  return 0;
}
