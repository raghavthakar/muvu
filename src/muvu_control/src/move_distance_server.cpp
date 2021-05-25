#include "ros/ros.h"
#include "muvu_control/MoveDistance.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "std_msgs/Float64.h"
#include <string>
#include <cmath>


class Mover
{
  ros::NodeHandle node_handle;
  std::string odom_topic="/muvu/odom";
  nav_msgs::OdometryConstPtr current_odom;
  ros::ServiceServer move_server;
  ros::Publisher twist_publisher;
  geometry_msgs::Twist twist_message;

  std::vector<double> getOrientation()
  {
    double roll, pitch, yaw;
    std::vector<double> angles;

    //Get the current odom data of the robot
    current_odom=ros::topic::waitForMessage<nav_msgs::Odometry>(odom_topic);

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(current_odom->pose.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    angles.push_back(roll);
    angles.push_back(pitch);
    angles.push_back(yaw);

    return angles;
  }

  double distanceFrom(double target_x, double target_y)
  {
    return sqrt(pow(target_y-current_odom->pose.pose.position.y, 2)
              +pow(target_x-current_odom->pose.pose.position.x, 2));
  }

  double getAngleToTarget(double target_y, double current_y,
                          double target_x, double current_x)
  {
    //Find the angle to rotate to face target
    double angle_to_target = atan((target_y-current_y)/
                                  (target_x-current_x));

    if(target_x-current_x<=0)
      if(target_y-current_y>=0)
        angle_to_target+=3.14;
      else
        angle_to_target-=3.14;

    return angle_to_target;
  }

public:
  Mover()
  {
    move_server = node_handle.advertiseService("move_distance",
                                    &Mover::move, this);

    twist_publisher = node_handle.advertise
                      <geometry_msgs::Twist>("/muvu/cmd_vel", 10);
  }

  bool move(muvu_control::MoveDistance::Request &request,
            muvu_control::MoveDistance::Response &response)
  {
    //Get the current orientation of the robot in space (angles)
    // [roll, pitch, yaw]
    std::vector<double> orientation=getOrientation();

    double angle_to_target=getAngleToTarget(request.target.y, current_odom->
                                  pose.pose.position.y, request.target.x,
                                  current_odom->pose.pose.position.x);

    //ROtate the robot till we face the target
    while(fabs(angle_to_target-orientation[2])>0.01)
    {
      ROS_INFO("%f %f", angle_to_target, orientation[2]);
      twist_message.angular.z=0.1;
      twist_message.linear.x=0;
      twist_publisher.publish(twist_message);

      orientation=getOrientation();

      double angle_to_target=getAngleToTarget(request.target.y, current_odom->
                                    pose.pose.position.y, request.target.x,
                                    current_odom->pose.pose.position.x);
    }

    //Stop the robot
    twist_message.angular.z=0;
    twist_message.linear.x=0;
    twist_publisher.publish(twist_message);

    //Move the robot forwards till we reach the target
    while(fabs(distanceFrom(request.target.x, request.target.y))>0.1)
    {
      //Get the current odom data of the robot
      current_odom=ros::topic::waitForMessage<nav_msgs::Odometry>(odom_topic);

      if(fabs(distanceFrom(request.target.x, request.target.y))>2)
        twist_message.linear.x=0.5;
      else
        twist_message.linear.x=0.1;

      twist_publisher.publish(twist_message);
      ROS_INFO("%f", distanceFrom(request.target.x, request.target.y));
    }

    //Stop the robot
    twist_message.angular.z=0;
    twist_message.linear.x=0;
    twist_publisher.publish(twist_message);

    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_distance_server");
  Mover mover;
  ros::spin();
  return 0;
}
