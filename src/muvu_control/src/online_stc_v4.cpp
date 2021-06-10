#include <list>
#include <algorithm>
#include "geometry_msgs/Point.h"
#include "muvu_control/MoveDistance.h"
#include "muvu_control/SampleNeighbours.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

class Cell
{
  double x;
  double y;

public:
  Cell(double x, double y)
  {
    this->x=x;
    this->y=y;
  }

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

  void display() const
  {
    std::cout<<"X: "<<x<<", Y: "<<y<<std::endl;
  }

  //custom operator for the map to store cell object
  bool operator<(const Cell& obj) const
  {
    return obj.getX() < x;
  }

  //overloaded operator to allow remove cell from list of cells
  bool operator==(const Cell& cell) const
  {
    return (cell.getX()==x && cell.getY()==y);
  }
};

class STC_handler
{
  ros::NodeHandle node_handle;

  //Client object for each service
  ros::ServiceClient sample_client;
  ros::ServiceClient move_client;

  //Response and request handlers
  muvu_control::SampleNeighbours sample_srv;
  muvu_control::MoveDistance move_srv;

  //list that maintains visited cells
  std::map<Cell, int> visited_cells;

public:
  STC_handler()
  {
    //initialising clients for each service
    sample_client = node_handle.serviceClient
                    <muvu_control::SampleNeighbours>("sample_neighbours");

    move_client = node_handle.serviceClient
                  <muvu_control::MoveDistance>("move_distance");
  }

  void STC(Cell curr_cell, int num)
  {
    //mark current cell as old
    visited_cells.insert({curr_cell, 0});

    Cell next_cell(0, 0);

    //Sample the neighbours
    sample_client.call(sample_srv);

    //cycle through the neighbours of current cell
    for(auto neighbour : sample_srv.response.neighbours)
    {
      Cell temp_cell(neighbour.x, neighbour.y);
      //find the neighbour in visited cells map, if not found
      if(visited_cells.find(temp_cell)==visited_cells.end())
      {
        next_cell=temp_cell;
        break;
      }
    }

    //Next cell is now the cell we should go to
    move_srv.request.target.x=next_cell.getX();
    move_srv.request.target.y=next_cell.getY();

    //Call the move service. returns true if succeeded
    if(move_client.call(move_srv))
      ROS_INFO("Called the service, and done");
    else
      ROS_ERROR("Failed to call service move_distance");

    if(num>5)
      return;

    STC(next_cell, num+1);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "online_stc");
  STC_handler handler;
  handler.STC(Cell(0.25, 0.25), 0);
}
