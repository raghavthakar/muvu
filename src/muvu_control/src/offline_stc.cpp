#include <iostream>
#include <fstream>
#include <vector>
#include <list>

#define LENGTH 1000
#define HEIGHT 1000
#define CELL_UNIT 50
#define SUB_UNIT 25

//X GOES FROM LEFT TO RIGHT
//Y GOES FROM TOP TO BOTTOM

enum cell_type {top_left, top_right, bottom_left, bottom_right};

using namespace std;

class Map
{
public:
  //This structure specifies the four corners of an Obstacle
  struct Obstacle
  {
    int top_left_column;
    int top_left_row;
    int bottom_right_column;
    int bottom_right_row;

    Obstacle(int top_left_column, int top_left_row,
             int bottom_right_column, int bottom_right_row)
    {
      this->top_left_column=top_left_column;
      this->top_left_row = top_left_row;
      this->bottom_right_column=bottom_right_column;
      this->bottom_right_row = bottom_right_row;
    }
  };

  //A list of 4 element integer arrays that specify corners of each Obstacle.
  list<Obstacle> all_obstacles;
  //An iteraot over this list tht can be used by other classes
  list<Obstacle>::iterator obs_it;

  Map()
  {
    int num_obstacles;
    int tlc, tlr, brc, brr;

    cout<<"Enter number of obstacles: ";
    cin>>num_obstacles;

    for(int i=0; i<num_obstacles; i++)
    {
      cout<<"Enter obstacle top left column number: ";
      cin>>tlc;
      cout<<"Enter obstacle top left row number: ";
      cin>>tlr;
      cout<<"Enter obstacle bottom right column number: ";
      cin>>brc;
      cout<<"Enter obstacle bottom right row number: ";
      cin>>brr;

      //push a test Obstacle
      all_obstacles.push_back(Obstacle(tlc, tlr, brc, brr));
    }
  }
};

class SubCell
{
public:
  double x;
  double y;

  double parent_x;
  double parent_y;

  void display()
  {
    cout<<x<<" "<<y<<", ";
  }
};

class Cell
{
  int x;
  int y;

  bool visited;


public:
  //Made public so that neighbours can be traversed through in the DFS
  vector<Cell*> connections;

  //top_left, top_right, bottom_left, bottom_right
  SubCell subcells[4];

  Cell(int x, int y)
  {
    visited=false;

    this->x=x;
    this->y=y;

    subcells[top_left].x=x-SUB_UNIT;
    subcells[top_left].y=y-SUB_UNIT;
    subcells[top_left].parent_x=x;
    subcells[top_left].parent_y=y;

    subcells[top_right].x=x+SUB_UNIT;
    subcells[top_right].y=y-SUB_UNIT;
    subcells[top_right].parent_x=x;
    subcells[top_right].parent_y=y;

    subcells[bottom_left].x=x-SUB_UNIT;
    subcells[bottom_left].y=y+SUB_UNIT;
    subcells[bottom_left].parent_x=x;
    subcells[bottom_left].parent_y=y;

    subcells[bottom_right].x=x+SUB_UNIT;
    subcells[bottom_right].y=y+SUB_UNIT;
    subcells[bottom_right].parent_x=x;
    subcells[bottom_right].parent_y=y;
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

  void markVisited()
  {
    visited=true;
  }

  bool isVisited()
  {
    return visited;
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

  //Tracks all the cells and order of adding them to Spanning tree
  list<Cell*> spanning_tree_cells;

  //Returns a vector of neighbouring cells that are within the bounds of the map
  vector<Cell*> getFreeNeighbours(Cell curr_cell)
  {
    vector<Cell*> free_neighbours;
    //If space to the left then there is a free cell to the left
    if(curr_cell.getX()>CELL_UNIT)
    {
      for(list<Cell>::iterator all_cells_iterator=all_cells.begin();
          all_cells_iterator!=all_cells.end(); all_cells_iterator++)
      {
        if(all_cells_iterator->getX()==curr_cell.getX()-CELL_UNIT&&
            all_cells_iterator->getY()==curr_cell.getY())
        {
          //pointer to location pointed to by iterator
          free_neighbours.push_back(&(*all_cells_iterator));
        }
      }
    }

    //If space to the top then there is a free cell at the top
    if(curr_cell.getY()>CELL_UNIT)
    {
      for(list<Cell>::iterator all_cells_iterator=all_cells.begin();
          all_cells_iterator!=all_cells.end(); all_cells_iterator++)
      {
        if(all_cells_iterator->getX()==curr_cell.getX()&&
            all_cells_iterator->getY()==curr_cell.getY()-CELL_UNIT)
        {
          free_neighbours.push_back(&(*all_cells_iterator));
        }
      }
    }

    //If free space to the left, then there is a free cell to the left
    if(LENGTH-curr_cell.getX()>CELL_UNIT)
    {
      for(list<Cell>::iterator all_cells_iterator=all_cells.begin();
          all_cells_iterator!=all_cells.end(); all_cells_iterator++)
      {
        if(all_cells_iterator->getX()==curr_cell.getX()+CELL_UNIT&&
            all_cells_iterator->getY()==curr_cell.getY())
        {
          free_neighbours.push_back(&(*all_cells_iterator));
        }
      }
    }

    //If free space to the left, then there is a free cell to the left
    if(HEIGHT-curr_cell.getY()>CELL_UNIT)
    {
      for(list<Cell>::iterator all_cells_iterator=all_cells.begin();
          all_cells_iterator!=all_cells.end(); all_cells_iterator++)
      {
        if(all_cells_iterator->getX()==curr_cell.getX()&&
            all_cells_iterator->getY()==curr_cell.getY()+CELL_UNIT)
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

    //Clear the CSV file
    ofstream csv_file;
    csv_file.open("offline_stc_points.csv");


    //Create a grid of cells that fill the entire map
    //O(n^2)
    for(int iy=CELL_UNIT; iy<=LENGTH; iy+=CELL_UNIT)
    {
      for(int ix=CELL_UNIT; ix<=HEIGHT; ix+=CELL_UNIT)
      {
        all_cells.push_back(Cell(ix, iy));
        num_cells++;//track the total number of cells
      }
    }
  }

  Cell* getBegin()
  {
    //returns a pointer to the cel being pointed to by the iterator
    return (&(*all_cells.begin()));
  }

  bool isLegal(Cell curr_cell, Map main_map)
  {
    //checking if the proposed new node lies in an obsacle
    //How to do it: start from top left corner to bottom right, create list of
    //illegal rows and columns (br[0]-tl[0]->rows)
    for(main_map.obs_it=main_map.all_obstacles.begin();
      main_map.obs_it!=main_map.all_obstacles.end(); main_map.obs_it++)
    {
      if(curr_cell.getX()<=main_map.obs_it->bottom_right_column &&
        curr_cell.getX()>=main_map.obs_it->top_left_column)
        if(curr_cell.getY()<=main_map.obs_it->bottom_right_row &&
          curr_cell.getY()>=main_map.obs_it->top_left_row)
          return false;
    }
    return true;
  }

  void constructGraph(Map main_map)
  {
    //Now initialise the connections vector for each cell
    //For each cell, see if cell to pos_x, pos_y, neg_x and neg_y are valid
    //If within the map range then add a connection to the cell in the
    //connections member vector
    for(list<Cell>::iterator all_cells_iterator=all_cells.begin();
        all_cells_iterator!=all_cells.end(); all_cells_iterator++)
    {
      if(isLegal(*all_cells_iterator, main_map))
      {
        //getFreeNeighbours can contain al the logic for obstacle detection
        vector<Cell*> free_neighbours = getFreeNeighbours(*all_cells_iterator);

        for(vector<Cell*>::iterator free_neighbours_iterator=free_neighbours.begin();
            free_neighbours_iterator!=free_neighbours.end();
            free_neighbours_iterator++)
        {
          if(isLegal(**free_neighbours_iterator, main_map))
            //addConncection accepts Cell* which can be obatined the following ways:
            all_cells_iterator->addConnection(*free_neighbours_iterator);
        }
      }
    }
  }

  //Display every cell in the graph along with their connections
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

  void writeToCSV(Cell* curr_cell)
  {
    ofstream csv_file;
    csv_file.open("offline_stc_points.csv", ios::app);

    csv_file<<curr_cell->getX()<<","<<curr_cell->getY()<<endl;
  }

  //Depth first search takes a pointer to cell as parameter and performs DFS
  //starting from it recursively
  void DFS(Cell* curr_cell, Cell* prev_cell)
  {
    //Mark the current cell as visited
    curr_cell->markVisited();
    curr_cell->display();
    for(int i=0; i<4; i++)
    {
      curr_cell->subcells[i].display();
    }

    //Push the current cell to all cells in the spanning tree
    spanning_tree_cells.push_back(curr_cell);
    writeToCSV(curr_cell);

    //Loop through neighbouring cells and recursively visit them if unvisited
    for(vector<Cell*>::iterator neighbour_cell=curr_cell->connections.begin();
        neighbour_cell!=curr_cell->connections.end(); neighbour_cell++)
    {
      if(!(*neighbour_cell)->isVisited())
      {
        cout<<endl;
        //neighbour cell is an iterator to pointer to cell. Hence use *
        DFS(*neighbour_cell, curr_cell);
      }
    }

    //For when we;re tracing the way back from dead end
    spanning_tree_cells.push_back(curr_cell);
    writeToCSV(curr_cell);
  }

  SubCell evaluateMove(int &curr_subcell_type, SubCell current_subcell,
                    list<Cell*>::iterator curr_cell, list<Cell*>::iterator next_cell)
  {
    list<Cell*>::iterator parent_cell;
    switch(curr_subcell_type)
    {
      case top_left:
        cout<<endl<<"top left"<<endl;
        //Compare the current subcell coordinates with the coordinates of
        //possible cells to move to
        //if moving left
        if(current_subcell.x-CELL_UNIT==(*next_cell)->subcells[top_right].x)
        {
          curr_subcell_type=top_right;
          parent_cell=next_cell;
        }
        else
        {
          curr_subcell_type=bottom_left;
          parent_cell=curr_cell;
        }
        break;

      case top_right:
        cout<<endl<<"top right"<<endl;
        if(current_subcell.y-CELL_UNIT==(*next_cell)->subcells[bottom_right].y)
        {
          curr_subcell_type=bottom_right;
          parent_cell=next_cell;
        }
        else
        {
          curr_subcell_type=top_left;
          parent_cell=curr_cell;
        }
        break;

      case bottom_left:
        cout<<endl<<"bottom_left"<<endl;
        if(current_subcell.y+CELL_UNIT==(*next_cell)->subcells[top_left].y)
        {
          curr_subcell_type=top_left;
          parent_cell=next_cell;
        }
        else
        {
          curr_subcell_type=bottom_right;
          parent_cell=curr_cell;
        }
        break;

      case bottom_right:
        cout<<endl<<"bottom right"<<endl;
        if(current_subcell.x+CELL_UNIT==(*next_cell)->subcells[bottom_left].x)
        {
          curr_subcell_type=bottom_left;
          parent_cell=next_cell;
        }
        else
        {
          curr_subcell_type=top_right;
          parent_cell=curr_cell;
        }
        break;
    }
    return (*parent_cell)->subcells[curr_subcell_type];
  }

  void moveTo(SubCell s)
  {

    cout<<endl;
    s.display();
  }

  //Counter clockwise circumnavigation of the spanning tree
  void circumNavigate()
  {
    //use spanning_tree_cells member
    auto curr_cell=spanning_tree_cells.begin();
    auto next_cell=curr_cell;
    next_cell++;

    //start from the top left subcell
    int curr_subcell_type=top_left;
    SubCell current_subcell=(*curr_cell)->subcells[curr_subcell_type];

    //Returns where the robot should move
    SubCell next_subcell=evaluateMove(curr_subcell_type, current_subcell, curr_cell, next_cell);

    moveTo(next_subcell);

    for(int i=0; i<10; i++)
    {
      curr_cell=next_cell;
      next_cell++;
      current_subcell=next_subcell;
      next_subcell=evaluateMove(curr_subcell_type, current_subcell,
                                curr_cell, next_cell);
      moveTo(next_subcell);
    }
  }
};

int main()
{
  Map main_map;
  Graph main_graph;
  main_graph.constructGraph(main_map);
  // main_graph.displayGraph();
  main_graph.DFS(main_graph.getBegin(), NULL);
  main_graph.circumNavigate();
  return 0;
}
