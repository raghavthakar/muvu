# muvu
Coverage using offline STC.

STC stands for Spanning Tree Coverage, and is a method of planning a route in a given work area that allows a robot to cover the entire area, while traversing any specific location only once.

Some application could include surface buffing or coating operations that ideally must occur only once on a particular point.

Research paper describing the algorithm can be found [here](http://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated4/gabriely_spanning.pdf.).

The algorithm works by dividing a work area into a collection of complete squares, each of dimension 2D (compared to the robot of size D). These squares, considered as vertices of a graph are then used to create a Spanning Tree for the graph. Subsequently, each square is divided into 4 squares, each of size D. Starting from a specified sub-square, the robot circumnaviagtes the Spanning Tree, effectively covering the entire work-area.

This implementation of the offline version of STC runs the algorithm in C++, taking the coordinates of any obstacles as input. The spanning tree and the route that circumnavigates it are stored locally as CSV files.



Visualisation of the spanning tree and the route have been outsourced, and are done with Python, using the PyGame package.
