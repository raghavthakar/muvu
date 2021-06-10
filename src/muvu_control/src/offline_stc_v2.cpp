#include <iostream>
#include <fstream>
#include <list>

#define MAP_WIDTH 1000
#define MAP_HEIGHT 1000
#define CELL_UNIT 100
#define SUBUNIT_CELL 50

//To simplify accessing neighbours in these directions
enum {POS_X, NEG_Y, NEG_X, POS_Y};

//To simplify subcell identification
enum {TOP_RIGHT, TOP_LEFT, BOTTOM_LEFT, BOTTOM_RIGHT};

//Holds the data of a single subcell in the Group
struct SubCell
{
  int x;
  int y;

  void display()
  {
    std::cout<<"X: "<<x<<", Y: "<<y<<std::endl;
  }

  bool operator==(const SubCell& subcell)
  {
    return (x==subcell.x && y==subcell.y);
  }
};

class SubCellGroup
{
public:
  //Define an array of 4 subcells {TOP_RIGHT< TOP_LEFT, BOTTOM_LEFT, BOTTOM_RIGHT}
  SubCell subcells[4];

  SubCellGroup(int parent_x, int parent_y)
  {
    subcells[TOP_RIGHT].x=parent_x+SUBUNIT_CELL/2;
    subcells[TOP_RIGHT].y=parent_y-SUBUNIT_CELL/2;

    subcells[TOP_LEFT].x=parent_x-SUBUNIT_CELL/2;
    subcells[TOP_LEFT].y=parent_y-SUBUNIT_CELL/2;

    subcells[BOTTOM_LEFT].x=parent_x-SUBUNIT_CELL/2;
    subcells[BOTTOM_LEFT].y=parent_y+SUBUNIT_CELL/2;

    subcells[BOTTOM_RIGHT].x=parent_x+SUBUNIT_CELL/2;
    subcells[BOTTOM_RIGHT].y=parent_y+SUBUNIT_CELL/2;
  }
};

class Cell
{
  int x;
  int y;

  bool visited=false;

public:
  SubCellGroup* child_subcellgroup;

  //array of 4 pointers to Cell to hold neighbours
  Cell* neighbours[4] = {NULL, NULL, NULL, NULL};

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

  void markVisited()
  {
    visited=true;
  }

  bool isVisited()
  {
    return visited;
  }

  void display() const
  {
    std::cout<<"X: "<<x<<", Y: "<<y<<std::endl;
  }

  bool operator==(Cell& cell)
  {
    return (cell.getX()==x && cell.getY()==y);
  }

  //As the cell in the list of all cells is being modified,
  //we must deal in pointers
  void addConnection(std::list<Cell>::iterator neighbour, int dir)
  {
    neighbours[dir]=&(*neighbour);
  }

  //Sets a pointer to corresponding subcell group
  void setSubCellGroup(SubCellGroup* child_subcellgroup)
  {
    this->child_subcellgroup=child_subcellgroup;
  }

};

class STC_handler
{
  //list of all cells
  std::list<Cell> all_cells;

  //list of all pointers cells in order of spanning tree
  std::list<Cell*> spanning_tree_cells;

  //returns a neighbour in the specified direction to the cell
  std::list<Cell>::iterator getNeighbour(Cell curr_cell, int dir)
  {
    int offset_x=0;
    int offset_y=0;

    offset_x=(dir==POS_X)?CELL_UNIT:
      (dir==NEG_X)?-CELL_UNIT:0;

    offset_y=(dir==POS_Y)?CELL_UNIT:
      (dir==NEG_Y)?-CELL_UNIT:0;

    // std::cout << "Curr cell: " << '\n';
    // curr_cell.display();

    // std::cout <<"offset_x: " <<offset_x<<" Offest Y: "<<offset_y<< '\n';

    Cell search_cell(curr_cell.getX()+offset_x, curr_cell.getY()+offset_y);
    // std::cout << "Search cell: " << '\n';
    // search_cell.display();

    for(auto i=all_cells.begin(); i!=all_cells.end(); i++)
    {
      if(*i==search_cell)
        return i;
    }

    return all_cells.end();
  }

  //Stores the current subcell data

public:
  //constructor sets up the graph and clears the csv file
  STC_handler(Cell start_cell)
  {
    //Add all valid cells to the list of all cells
    //top to bottom
    for(int i=start_cell.getY(); i<MAP_HEIGHT; i+=CELL_UNIT)
    {
      //left to right
      for(int ii=start_cell.getX(); ii<MAP_WIDTH; ii+=CELL_UNIT)
      {
        //if within map boundaries
        if(i-SUBUNIT_CELL>=0 && i+SUBUNIT_CELL<=MAP_HEIGHT
          && ii-SUBUNIT_CELL>=0 && ii+SUBUNIT_CELL<=MAP_WIDTH)
        {
          //Add to list of all cells
          all_cells.push_back(Cell(ii, i));
        }
      }
    }

    //Cycle through the list of all cells and connect them to neighbours
    for(auto valid_cell=all_cells.begin(); valid_cell!=all_cells.end(); valid_cell++)
    {
      //find the pointer to cell in list that is +x neighbour of this cell
      auto neighbour=getNeighbour(*valid_cell, POS_X);
      if(neighbour!=all_cells.end())
      {
        valid_cell->addConnection(neighbour, POS_X);
        neighbour->addConnection(valid_cell, NEG_X);
      }
      // else std::cout << "Not found" << '\n';
      // std::cout << "--------" << '\n';

      //find the pointer to cell in list that is -y neighbour of this cell
      neighbour=getNeighbour(*valid_cell, NEG_Y);
      if(neighbour!=all_cells.end())
      {
        valid_cell->addConnection(neighbour, NEG_Y);
        neighbour->addConnection(valid_cell, POS_Y);
      }
      // else std::cout << "Not found" << '\n';
      // std::cout << "--------" << '\n';

      //find the pointer to cell in list that is -x neighbour of this cell
      neighbour=getNeighbour(*valid_cell, NEG_X);
      if(neighbour!=all_cells.end())
      {
        valid_cell->addConnection(neighbour, NEG_X);
        neighbour->addConnection(valid_cell, POS_X);
      }
      // else std::cout << "Not found" << '\n';
      // std::cout << "--------" << '\n';

      //find the pointer to cell in list that is +y neighbour of this cell
      neighbour=getNeighbour(*valid_cell, POS_Y);
      if(neighbour!=all_cells.end())
      {
        valid_cell->addConnection(neighbour, POS_Y);
        neighbour->addConnection(valid_cell, NEG_Y);
      }
      // else std::cout << "Not found" << '\n';
      //
      // std::cout << "------------------" << '\n';
    }

    //clear the CSV files
    std::ofstream csv_file;
    csv_file.open("circumnavigate_points.csv");
    csv_file.close();
    csv_file.open("offline_stc_points.csv");
    csv_file.close();
  }

  void showCells()
  {
    for(auto i:all_cells)
    {
      std::cout << "CURRENT CELL: " << '\n';
      i.display();
      std::cout << "NEIGHBOURS: " << '\n';
      for(auto ii:i.neighbours)
      {
        if(ii!=NULL) ii->display();
      }
    }
  }

  void showSpanningTree()
  {
    for(auto i:spanning_tree_cells)
    {
      std::cout << "CURRENT CELL: " << '\n';
      i->display();
      std::cout << "NEIGHBOURS: " << '\n';
      for(auto ii:i->neighbours)
      {
        if(ii!=NULL) ii->display();
      }
    }
  }

  //returns a pointer to the first cell
  Cell* getAllCellsBegin()
  {
    return &(*all_cells.begin());
  }

  //returns an iterator pointing to the first element of spanning tree cells
  std::list<Cell*>::iterator getSpanningTreeCellsBegin()
  {
    return spanning_tree_cells.begin();
  }

  //Writes the current cell to a CSV file
  void writeCellToCSV(Cell* curr_cell)
  {
    std::ofstream csv_file;
    csv_file.open("offline_stc_points.csv", std::ios::app);

    csv_file<<curr_cell->getX()<<","<<curr_cell->getY()<<std::endl;
  }

  //Writes the current subcell to a CSV file
  void writeSubCellToCSV(SubCell* curr_subcell)
  {
    std::ofstream csv_file;
    csv_file.open("circumnavigate_points.csv", std::ios::app);

    csv_file<<curr_subcell->x<<","<<curr_subcell->y<<std::endl;
  }

  //Traverses the graph using DFS and n iteratorupdates spanning tree cells list
  void DFS(Cell* prev_cell, Cell* curr_cell)
  {
    // curr_cell->display();
    curr_cell->markVisited();
    spanning_tree_cells.push_back(curr_cell);
    writeCellToCSV(curr_cell);

    // std::cout << "-----------" << '\n';

    for(Cell* neighbour:curr_cell->neighbours)
    {
      if(neighbour!=NULL)
        if(!neighbour->isVisited()) DFS(curr_cell, neighbour);
    }

    //Add the current cell to list again to allow backtracking in dead end cases
    spanning_tree_cells.push_back(curr_cell);
    writeCellToCSV(curr_cell);
  }

  //divide all the cells in the grpah into subcells
  void divideIntoSubcells(std::list<Cell*>::iterator spanning_tree_cell)
  {
    if(spanning_tree_cell==spanning_tree_cells.end())
      return;

    // (*spanning_tree_cell)->display();

    //Create a subcellgroup for the current spanning tree cell
    SubCellGroup *temp_scg=new SubCellGroup((*spanning_tree_cell)->getX(),
                              (*spanning_tree_cell)->getY());

    //Set the subcell group in the corresponding spanning tree cell
    (*spanning_tree_cell)->setSubCellGroup(temp_scg);
    divideIntoSubcells(++spanning_tree_cell);
  }

  void showSpanningTreeWithSubCellGroups()
  {
    for(auto i:spanning_tree_cells)
    {
      std::cout << "CURRENT CELL: " << '\n';
      i->display();
      std::cout << "SUBCELLS: " << '\n';
      for(int ii=0; ii<4; ii++)
      {
        i->child_subcellgroup->subcells[ii].display();
      }
    }
  }

  // recursive function to circumnavigate the spanning tree
  void circumnavigate(Cell* curr_cell, Cell* next_cell, SubCell* curr_subcell,
                      std::list<Cell*>::iterator spanning_tree_cell, int count)
  {
    if(spanning_tree_cell==spanning_tree_cells.end())
      return;

    writeSubCellToCSV(curr_subcell);

    std::cout << "CURRENT CELL: " << '\n';
    curr_cell->display();
    std::cout << "CURRENT SUBCELL" << '\n';
    curr_subcell->display();

    SubCell up_subcell;
    up_subcell.x=curr_subcell->x;
    up_subcell.y=curr_subcell->y-SUBUNIT_CELL;

    SubCell right_subcell;
    right_subcell.x=curr_subcell->x+SUBUNIT_CELL;
    right_subcell.y=curr_subcell->y;

    SubCell down_subcell;
    down_subcell.x=curr_subcell->x;
    down_subcell.y=curr_subcell->y+SUBUNIT_CELL;

    SubCell left_subcell;
    left_subcell.x=curr_subcell->x-SUBUNIT_CELL;
    left_subcell.y=curr_subcell->y;

    //if the current subcell is the top right subcell,
    //try going up to next cell, else right to same
    if(*curr_subcell==curr_cell->child_subcellgroup->subcells[TOP_RIGHT])
    {
      //see if going up is valid
      if(next_cell->child_subcellgroup->subcells[BOTTOM_RIGHT]==up_subcell)
      {
        std::cout << "Should go up" << '\n';
        curr_subcell=&next_cell->child_subcellgroup->subcells[BOTTOM_RIGHT];
        curr_cell=next_cell;
        next_cell=*(++spanning_tree_cell);
      }
      else
      {
        std::cout << "Should go ledt" << '\n';
        curr_subcell=&curr_cell->child_subcellgroup->subcells[TOP_LEFT];
      }
    }

    //if top left
    else if(*curr_subcell==curr_cell->child_subcellgroup->subcells[TOP_LEFT])
    {
      //see if going up is valid
      if(next_cell->child_subcellgroup->subcells[TOP_RIGHT]==left_subcell)
      {
        std::cout << "Should go left" << '\n';
        curr_subcell=&next_cell->child_subcellgroup->subcells[TOP_RIGHT];
        curr_cell=next_cell;
        next_cell=*(++spanning_tree_cell);
      }
      else
      {
        std::cout << "Should go down" << '\n';
        curr_subcell=&curr_cell->child_subcellgroup->subcells[BOTTOM_LEFT];
      }
    }

    //if bottom left
    else if(*curr_subcell==curr_cell->child_subcellgroup->subcells[BOTTOM_LEFT])
    {
      //see if going up is valid
      if(next_cell->child_subcellgroup->subcells[TOP_LEFT]==down_subcell)
      {
        std::cout << "Should go down" << '\n';
        curr_subcell=&next_cell->child_subcellgroup->subcells[TOP_LEFT];
        curr_cell=next_cell;
        next_cell=*(++spanning_tree_cell);
      }
      else
      {
        std::cout << "Should go right" << '\n';
        curr_subcell=&curr_cell->child_subcellgroup->subcells[BOTTOM_RIGHT];
      }
    }

    //if bottom right
    else if(*curr_subcell==curr_cell->child_subcellgroup->subcells[BOTTOM_RIGHT])
    {
      //see if going up is valid
      if(next_cell->child_subcellgroup->subcells[BOTTOM_LEFT]==right_subcell)
      {
        std::cout << "Should go right" << '\n';
        curr_subcell=&next_cell->child_subcellgroup->subcells[BOTTOM_LEFT];
        curr_cell=next_cell;
        next_cell=*(++spanning_tree_cell);
      }
      else
      {
        std::cout << "Should go up" << '\n';
        curr_subcell=&curr_cell->child_subcellgroup->subcells[TOP_RIGHT];
      }
    }

    // circumnavigate(next_cell, *(spanning_tree_cell), curr_subcell, spanning_tree_cell, count+1);
    circumnavigate(curr_cell, next_cell, curr_subcell, spanning_tree_cell, count+1);
  }
};

int main()
{
  STC_handler handler(Cell(50, 50));
  // handler.showCells();
  handler.DFS(handler.getAllCellsBegin(), handler.getAllCellsBegin());
  // handler.showSpanningTree();
  handler.divideIntoSubcells(handler.getSpanningTreeCellsBegin());
  // handler.showSpanningTreeWithSubCellGroups();

  auto st_cell_iterator=handler.getSpanningTreeCellsBegin();
  auto curr_cell=*st_cell_iterator;
  SubCell* curr_subcell=&curr_cell->child_subcellgroup->subcells[TOP_RIGHT];
  st_cell_iterator++;
  auto next_cell=*st_cell_iterator;

  handler.circumnavigate(curr_cell, next_cell, curr_subcell, st_cell_iterator, 0);
}
