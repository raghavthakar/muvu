#include <cmath>
#include "ros/ros.h"
#include "muvu_control/SampleNeighbours.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

enum {POS_X, POS_Y, NEG_X, NEG_Y};

#define UNIT_CELL 0.50
#define SUBCELL_UNIT 0.25

class Sampler
{
  ros::NodeHandle node_handle;
  std::string laserscan_topic="/muvu/laser/scan";
  std::string odom_topic="/muvu/odom";
  sensor_msgs::LaserScanConstPtr laser_data;
  nav_msgs::OdometryConstPtr current_odom;
  ros::ServiceServer sample_server;

  //returns whether the specifid ray number detects obstacle or not
  bool getObstacle(int ray_num, sensor_msgs::LaserScanConstPtr laser_data)
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

  //returns the direction the robot is facing
  //POS_X, POS_Y, NEG_X, NEG_Y
  int getDirection(double yaw)
  {
    if(yaw>0)
    {
      if(yaw<0.785)
        return POS_X;
      else if(yaw<2.355)
        return POS_Y;
      else
        return NEG_X;
    }

    if(yaw<0)
    {
      if(yaw>-0.785)
        return POS_X;
      else if(yaw>-2.355)
        return NEG_Y;
      else
        return NEG_X;
    }
  }

  //Arranges the back, right, front and left dirs in anticlockwise order
  std::list<int> getDirectionList(int front_direction)
  {
    std::list<int> directions;
    //Gives the direction to the cell, behind, right, then left
    int back_direction=(front_direction+2)%4;
    int right_direction=(front_direction+3)%4;
    int left_direction=(front_direction+1)%4;

    directions.push_back(back_direction);
    directions.push_back(right_direction);
    directions.push_back(front_direction);
    directions.push_back(left_direction);

    return directions;
  }

  //Rounding the coordinates to the nearest slab
  float round(float val, float slab)
  {
    int sign=(val>0)?1:-1;
    val*=sign;
    float head=(int)(val/slab);
    float rem = (val/slab)-head;
    if(rem<0.5)
      head*=slab;
    else
      head=slab*(head+1);
    return sign*head;
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

    current_odom=ros::topic::waitForMessage<nav_msgs::Odometry>(odom_topic);

    //store which direction the robot id facing
    //{POSX, POSY, NEGX, NEGY}
    int front_direction=getDirection(getYaw(getOdometry()));

    //get the array of directions arranged in counter clockwise order
    std::list<int> directions = getDirectionList(front_direction);

    //Loop through the directions, check for obstacle
    int ray_num_factor=0;//factor by which right angle to multiply to get ray_num
    double offset_x, offset_y;//offsets to add to curr location to get point in specified direction
    geometry_msgs::Point neighbour_point;
    for(auto dir: directions)
    {
      //if the cell in dir direction is unoccupied
      if(!getObstacle(ray_num_factor*90, laser_data))
      {
        //construct a nav_msgs::point message corresponding to the cell & push
        //it to the response
        switch(dir)
        {
          case POS_X:
            offset_x=UNIT_CELL;
            offset_y=0;
            ROS_INFO("POSX");
            break;

          case POS_Y:
            offset_x=0;
            offset_y=UNIT_CELL;
            ROS_INFO("POSY");
            break;

          case NEG_X:
            offset_x=-UNIT_CELL;
            offset_y=0;
            ROS_INFO("NEGX");
            break;

          case NEG_Y:
            offset_x=0;
            offset_y=-UNIT_CELL;
            ROS_INFO("NEGY");
            break;
        }
      }

      ROS_INFO("CURR: X: %f Y: %f ", current_odom->pose.pose.position.x,current_odom->pose.pose.position.y);
      ROS_INFO("OFFSETS: X: %f Y: %f ", offset_x, offset_y);

      neighbour_point.x=round(current_odom->pose.pose.position.x, SUBCELL_UNIT)+offset_x;
      neighbour_point.y=round(current_odom->pose.pose.position.y, SUBCELL_UNIT)+offset_y;

      ROS_INFO("NEIGHBOUR: X: %f Y: %f ", neighbour_point.x, neighbour_point.y);

      response.neighbours.push_back(neighbour_point);

      ray_num_factor++;
    }

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
