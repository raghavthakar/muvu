#include <iostream>
#include <vector>
#include <list>

#define LENGTH 1000
#define HEIGHT 1000
#define MIN_UNIT 50

//X GOES FROM LEFT TO RIGHT
//Y GOES FROM TOP TO BOTTOM

using namespace std;

class Map
{
  int top_left_x;
  int top_left_y;
  int bottom_right_x;
  int bottom_right_y;

public:
  Map()
  {
    cout<<"Enter top left co-ordinates (X & Y): ";
    cin>>top_left_x;
    cin>>top_left_y;

    cout<<"Enter bottom right co-ordinates (X & Y): ";
    cin>>bottom_right_x;
    cin>>bottom_right_y;
  }
};

class Cell
{
  int x;
  int y;

  vector<Cell*> connections;

public:
  Cell(int x, int y)
  {
    this->x=x;
    this->y=y;
  }

  int getX()
  {
    return x;
  }

  int getY()
  {
    return y;
  }

  void addConnection(Cell* connect_cell)
  {
    connections.push_back(connect_cell);
  }

  void display()
  {
    cout<<x<<" "<<y<<", ";
  }

  void displayConnections()
  {
    for(vector<Cell*>::iterator it=connections.begin();
        it!=connections.end(); it++)
    {
      (*it)->display();
    }
  }
};

class Graph
{
  list<Cell> all_cells;
  int num_cells;

  //Returns a vector of neighbouring cells that are within the bounds of the map
  vector<Cell*> getFreeNeighbours(Cell curr_cell)
  {
    vector<Cell*> free_neighbours;
    //If space to the left then there is a free cell to the left
    if(curr_cell.getX()>MIN_UNIT)
    {
      for(list<Cell>::iterator all_cells_iterator=all_cells.begin();
          all_cells_iterator!=all_cells.end(); all_cells_iterator++)
      {
        if(all_cells_iterator->getX()==curr_cell.getX()-MIN_UNIT&&
            all_cells_iterator->getY()==curr_cell.getY())
        {
          free_neighbours.push_back(&(*all_cells_iterator));
        }
      }
    }

    //If space to the top then there is a free cell at the top
    if(curr_cell.getY()>MIN_UNIT)
    {
      for(list<Cell>::iterator all_cells_iterator=all_cells.begin();
          all_cells_iterator!=all_cells.end(); all_cells_iterator++)
      {
        if(all_cells_iterator->getX()==curr_cell.getX()&&
            all_cells_iterator->getY()==curr_cell.getY()-MIN_UNIT)
        {
          free_neighbours.push_back(&(*all_cells_iterator));
        }
      }
    }

    //If free space to the left, then there is a free cell to the left
    if(LENGTH-curr_cell.getX()>MIN_UNIT)
    {
      for(list<Cell>::iterator all_cells_iterator=all_cells.begin();
          all_cells_iterator!=all_cells.end(); all_cells_iterator++)
      {
        if(all_cells_iterator->getX()==curr_cell.getX()+MIN_UNIT&&
            all_cells_iterator->getY()==curr_cell.getY())
        {
          free_neighbours.push_back(&(*all_cells_iterator));
        }
      }
    }

    //If free space to the left, then there is a free cell to the left
    if(HEIGHT-curr_cell.getY()>MIN_UNIT)
    {
      for(list<Cell>::iterator all_cells_iterator=all_cells.begin();
          all_cells_iterator!=all_cells.end(); all_cells_iterator++)
      {
        if(all_cells_iterator->getX()==curr_cell.getX()&&
            all_cells_iterator->getY()==curr_cell.getY()+MIN_UNIT)
        {
          free_neighbours.push_back(&(*all_cells_iterator));
        }
      }
    }

    return free_neighbours;
  }

public:
  Graph()
  {
    num_cells=0;

    //Create a grid of cells that fill the entire map
    //O(n^2)
    for(int iy=MIN_UNIT; iy<=LENGTH; iy+=MIN_UNIT)
    {
      for(int ix=MIN_UNIT; ix<=HEIGHT; ix+=MIN_UNIT)
      {
        all_cells.push_back(Cell(ix, iy));
        num_cells++;//track the total number of cells
      }
    }
  }

  void constructGraph()
  {
    //Now initialise the connections vector for each cell
    //For each cell, see if cell to pos_x, pos_y, neg_x and neg_y are valid
    //If within the map range then add a connection to the cell in the
    //connections member vector
    for(list<Cell>::iterator all_cells_iterator=all_cells.begin();
        all_cells_iterator!=all_cells.end(); all_cells_iterator++)
    {
      vector<Cell*> free_neighbours = getFreeNeighbours(*all_cells_iterator);

      for(vector<Cell*>::iterator free_neighbours_iterator=free_neighbours.begin();
          free_neighbours_iterator!=free_neighbours.end();
          free_neighbours_iterator++)
      {
        //add conncection accepts Cell* which can be obatined the following ways:
        all_cells_iterator->addConnection(*free_neighbours_iterator);
      }
    }
  }

  void displayGraph()
  {
    for(list<Cell>::iterator it=all_cells.begin(); it!=all_cells.end(); it++)
    {
      it->display();
      cout<<":";
      it->displayConnections();
      cout<<endl;
    }
  }
};

int main()
{
  Graph main_graph;
  main_graph.constructGraph();
  main_graph.displayGraph();

  return 0;
}
