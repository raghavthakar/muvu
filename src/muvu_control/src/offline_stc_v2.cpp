#include <iostream>
#include <list>

#define MAP_WIDTH 1000
#define MAP_HEIGHT 1000
#define CELL_UNIT 100
#define SUBCELL_UNIT 50

//To simplify accessing neighbours in these directions
enum {POS_X, NEG_Y, NEG_X, POS_Y};

class Cell
{
  int x;
  int y;

  bool visited=false;

public:
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

public:
  //constructor sets up the graph
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
        if(i-SUBCELL_UNIT>=0 && i+SUBCELL_UNIT<=MAP_HEIGHT
          && ii-SUBCELL_UNIT>=0 && ii+SUBCELL_UNIT<=MAP_WIDTH)
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

  //returns an iterator pointing to the first cell
  Cell* getAllCellsBegin()
  {
    return &(*all_cells.begin());
  }

  void DFS(Cell* prev_cell, Cell* curr_cell)
  {
    curr_cell->display();
    curr_cell->markVisited();
    spanning_tree_cells.push_back(curr_cell);

    std::cout << "-----------" << '\n';

    for(Cell* neighbour:curr_cell->neighbours)
    {
      if(neighbour!=NULL)
        if(!neighbour->isVisited()) DFS(curr_cell, neighbour);
    }
  }
};

int main()
{
  STC_handler handler(Cell(50, 50));
  handler.showCells();
  handler.DFS(handler.getAllCellsBegin(), handler.getAllCellsBegin());
  handler.showSpanningTree();
}
