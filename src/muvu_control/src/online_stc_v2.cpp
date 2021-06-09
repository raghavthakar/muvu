#include "geometry_msgs/Point.h"
#include "muvu_control/MoveDistance.h"
#include "muvu_control/SampleNeighbours.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_datatypes.h"

#define UNIT_CELL 0.5
#define SUBCELL_UNIT 0.25

enum { POS_X, POS_Y, NEG_X, NEG_Y };

class Cell
{
  double x;
  double y;

public:
  Cell()
  {
    this->x=SUBCELL_UNIT;
    this->y=SUBCELL_UNIT;
  }

  Cell(double x, double y)
  {
    this->x=x;
    this->y=y;
  }

  //this function is const because the map data structure reqiures internal
  //comparison using <, which needs to be defined for cell class
  //which is defined in a const functino, which calls getX, which hence needs
  //to be const
  double getX() const
  {
    return x;
  }

  double getY() const
  {
    return y;
  }

  void setX(double x)
  {
    this->x=x;
  }

  void setY(double y)
  {
    this->y=y;
  }

  void display()
  {
    std::cout<<"X: "<<x<<", Y: "<<y<<std::endl;
  }

  //custom operator for the map to store cell object
  bool operator<(const Cell& cell) const
  {
    return cell.getX() < x;
  }

  //custom operator to remove cell from list of cells
  bool operator==(const Cell& cell) const
  {
    return (cell.getX()==x && cell.getY()==y);
  }
};

class STC
{
  ros::NodeHandle node_handle;

  //Client obkect for each service
  ros::ServiceClient move_client;
  ros::ServiceClient sample_client;

  //Request and response handlers
  muvu_control::MoveDistance move_srv;
  muvu_control::SampleNeighbours sample_srv;

  //tp track the current odometry
  nav_msgs::OdometryConstPtr current_odom;

  //topic from where the odometry will be retrieved
  std::string odom_topic="/muvu/odom";

  //global list that tracks all visted cells
  std::list<Cell> old_cells;

  //Map that maps every old cell to its free and new neighbouring cells
  std::map<Cell, std::list<Cell>> cell_free_new_neighbours_map;

  //return the yaw for current odometry
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

public:
  STC()
  {
    move_client =node_handle.serviceClient
                  <muvu_control::MoveDistance>("move_distance");

    sample_client = node_handle.serviceClient
                    <muvu_control::SampleNeighbours>("sample_neighbours");

    std::cout<<"Online STC";
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

  //Tells if the cell is old/visited
  bool isOld(Cell temp_cell)
  {
    for(auto it=old_cells.begin(); it!=old_cells.end(); it++)
    {
      if(temp_cell.getX()==it->getX()
          && temp_cell.getY()==it->getY())
      {
        return true;
      }
    }
    return false;
  }

  //main recursive function
  void operate(Cell parent_cell, Cell curr_cell)
  {
    ROS_INFO("OPERATING");

    for(auto x = old_cells.begin(); x!=old_cells.end(); x++)
    {
      ROS_INFO("CELL: ");
      x->display();
      ROS_INFO("NEIGHBOURS: ");
      auto y=cell_free_new_neighbours_map.find(*x);
      for(auto z=y->second.begin(); z!=y->second.end(); z++)
      {
        z->display();
      }
    }

    Cell temp_cell;

    //placeholder variable to insert into map along with current cell
    std::list<Cell> free_new_neighbours;

    //Get the current odomeetry and round it to nearest SUBCELL_UNIT
    current_odom=ros::topic::waitForMessage<nav_msgs::Odometry>(odom_topic);
    curr_cell.setX(round(current_odom->pose.pose.position.x, SUBCELL_UNIT));
    curr_cell.setY(round(current_odom->pose.pose.position.y, SUBCELL_UNIT));

    //Push the current cell to the list of old cells
    old_cells.push_back(curr_cell);

    //get the free neighbours of current cell
    //posx, posy, negx, negy
    sample_client.call(sample_srv);

    //Get the yaw of the current robot position
    double yaw=getYaw(current_odom);
    // Get the current direction the robot is facing, and decide counterclockwise
    // order from that
    int direction=getDirection(yaw);
    int parent_direction=(direction+2)%4;

    //loop through all the neighbours, add free and new neighbours to list
    //start circular queue form parent direction
    int i=parent_direction;
    do
    {
      ROS_INFO("I is : %d", i);
      // if neighbour is free
      if(sample_srv.response.neighbours[i].data==1)
      {
        //assign a temp cell to this free cell
        //search for this temp cell in old cells
        switch(i)
        {
          case 0://posx
            temp_cell.setX(curr_cell.getX()+UNIT_CELL);
            temp_cell.setY(curr_cell.getY());

            //if the ngihbour is univisited, add to list of free new nghbrs
            if(!isOld(temp_cell))
            {
              ROS_INFO("Free: PosX Unvisited X: %f, Y: %f", temp_cell.getX(), temp_cell.getY());
              free_new_neighbours.push_back(temp_cell);
            }
            //else remove the current cell from the neighbour's free new nghbrs
            else
            {
              //find this neighbour in the map
              auto it=cell_free_new_neighbours_map.find(temp_cell);
              //remove current cell from its free new nghbrs
              it->second.remove(curr_cell);
            }

            break;

          case 1://posy
            temp_cell.setX(curr_cell.getX());
            temp_cell.setY(curr_cell.getY()+UNIT_CELL);

            if(!isOld(temp_cell))
            {
              ROS_INFO("Free: PosY Unvisited X: %f, Y: %f", temp_cell.getX(), temp_cell.getY());
              free_new_neighbours.push_back(temp_cell);
            }
            //else remove the current cell from the neighbour's free new nghbrs
            else
            {
              //find this neighbour in the map
              auto it=cell_free_new_neighbours_map.find(temp_cell);
              //remove current cell from its free new nghbrs
              it->second.remove(curr_cell);
            }

            break;

          case 2://negx
            temp_cell.setX(curr_cell.getX()-UNIT_CELL);
            temp_cell.setY(curr_cell.getY());

            if(!isOld(temp_cell))
            {
              ROS_INFO("Free: NegX Unvisited X: %f, Y: %f", temp_cell.getX(), temp_cell.getY());
              free_new_neighbours.push_back(temp_cell);
            }
            //else remove the current cell from the neighbour's free new nghbrs
            else
            {
              //find this neighbour in the map
              auto it=cell_free_new_neighbours_map.find(temp_cell);
              //remove current cell from its free new nghbrs
              it->second.remove(curr_cell);
            }

            break;

          case 3://negy
            temp_cell.setX(curr_cell.getX());
            temp_cell.setY(curr_cell.getY()-UNIT_CELL);

            if(!isOld(temp_cell))
            {
              ROS_INFO("Free: NegY Unvisited X: %f, Y: %f", temp_cell.getX(), temp_cell.getY());
              free_new_neighbours.push_back(temp_cell);
            }
            //else remove the current cell from the neighbour's free new nghbrs
            else
            {
              //find this neighbour in the map
              auto it=cell_free_new_neighbours_map.find(temp_cell);
              //remove current cell from its free new nghbrs
              it->second.remove(curr_cell);
            }

            break;
        }
      }
      i++;
      i=i%4;
    }while(i!=(parent_direction+4)%4);//instead of going 3->4, shoudl go 3->0

    //add a key to the map for current cell
    //equivalent to pushing an old cell along with its free new neighbours
    cell_free_new_neighbours_map.insert({curr_cell, free_new_neighbours});

    // --------WE HAVE A CELL, LIST OF OLD CELLS, AND LIST OF FREE NIGHBOURS OF THIS CELL-------
    //Store coordinatesof the next cell to go to
    geometry_msgs::Point next_point;
    //iterator to current cell in the map
    auto curr_cell_it=cell_free_new_neighbours_map.find(curr_cell);
    //declare the next cell
    std::list<Cell>::iterator next_cell_it;
    //While the curr cell has free new nieghbours
    while(curr_cell_it->second.size())
    {
      //iterator to first new naighbour in list
      next_cell_it=curr_cell_it->second.begin();
      ROS_INFO("TARGET CELL: ");
      next_cell_it->display();
      //update the next point to the coordinatesof next cell
      next_point.x=next_cell_it->getX();
      next_point.y=next_cell_it->getY();

      //set next point as the service target
      move_srv.request.target=next_point;

      //Call the move service. returns true if succeeded
      if(move_client.call(move_srv))
      {
        ROS_INFO("Called the service, and done");
      }
      else
      {
        ROS_ERROR("Failed to call service move_distance");
      }

      next_cell_it->display();
      current_odom=ros::topic::waitForMessage<nav_msgs::Odometry>(odom_topic);
      ROS_INFO("CURRENT CELL: ");
      ROS_INFO("X; %f", round(current_odom->pose.pose.position.x, SUBCELL_UNIT));
      ROS_INFO("Y; %f", round(current_odom->pose.pose.position.y, SUBCELL_UNIT));
      ROS_INFO("X; %f", current_odom->pose.pose.position.x);
      ROS_INFO("Y; %f", current_odom->pose.pose.position.y);

      //call operate with current cell and next cell
      operate(curr_cell, *next_cell_it);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "online_stc");

  STC stc;

  Cell start_cell(0.25, 0.25);

  stc.operate(start_cell, start_cell);

  return 0;
}
