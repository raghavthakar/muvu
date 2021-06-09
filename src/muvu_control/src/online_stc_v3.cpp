#include "geometry_msgs/Point.h"
#include "muvu_control/MoveDistance.h"
#include "muvu_control/SampleNeighbours.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

enum {BACK, RIGHT, FRONT, LEFT};

//represents every unit in the entire map that the robot traverses and covers
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

  //this map stores information about all visited cells and their corresponding
  //free neighbours. {Cell, Free_Neighbours}
  std::map<Cell, std::list<Cell>> cell_map;

public:
  STC_handler()
  {
    //initialising clients for each service
    sample_client = node_handle.serviceClient
                    <muvu_control::SampleNeighbours>("sample_neighbours");

    move_client = node_handle.serviceClient
                  <muvu_control::MoveDistance>("move_distance");
  }

  void STC(Cell parent_cell, Cell curr_cell)
  {
    std::list<Cell> bruh;
    bruh.push_back(Cell(0.25, 0.25));
    bruh.push_back(Cell(0.125, 0.25));
    bruh.push_back(Cell(0.75, 0.-25));
    bruh.push_back(Cell(0.75, 0.75));
    cell_map.insert({Cell(0.75, 0.25), bruh});

    //local list of free neighbours of current cell
    std::list<Cell> free_neighbours;

    //get the array of neighbours
    sample_client.call(sample_srv);

    //we now have an array of 4, depicting where obstacle is present
    //[back, right, front, left]. This is the order of priority of movement
    for(auto neighbour : sample_srv.response.neighbours)
    {
      //Create a temp cell with the coordinates of the neighbour
      Cell neighbour_cell=Cell(neighbour.x, neighbour.y);
      //Display the neighbour cell
      std::cout << "Current's neighbour";
      neighbour_cell.display();
      //Find this neighbour in map of visted cells
      auto neighbour_cell_it=cell_map.find(neighbour_cell);
      //if iterator points to end, means not found ie univisited
      if(neighbour_cell_it==cell_map.end())
      {
        free_neighbours.push_back(neighbour_cell);
      }
      //If the neighbour is visited
      else
      {
        std::cout<<"Neighbours of visited cell: \n";
        for(auto i=neighbour_cell_it->second.begin(); i!=neighbour_cell_it->second.end(); i++)
        {
          i->display();
        }
        std::cout << "-----------------" << '\n';

        //neighbour_cell_it points to the neighbour in cell map
        //remove the current cell from its free neighbours
        neighbour_cell_it->second.remove(curr_cell);

        std::cout<<"Free neighbours of visited cell: \n";
        for(auto i=neighbour_cell_it->second.begin(); i!=neighbour_cell_it->second.end(); i++)
        {
          i->display();
        }
        std::cout << "-----------------" << '\n';
      }
    }
    //Display the free neighbours of current cell
    std::cout<<"Free neighbours of current cell: \n";
    for(auto i=free_neighbours.begin(); i!=free_neighbours.end(); i++)
    {
      i->display();
    }
    //Add the current cell and its neighbours to visited cels map
    cell_map.insert({curr_cell, free_neighbours});
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "online_stc");

  STC_handler handler;

  handler.STC(Cell(0.25, 0.25), Cell(0.25, 0.25));
}
