# Generic Implementation of A* Algorithm for Shortest Paths in C++

This implementation is not necessarily the most runtime or memory efficient version of the algorithm, 
but it is lightweight (single-file, header-only) and very generic, so it should be quickly usable.

## Notes

The path finding algorithm is templated regarding the node type (which may also be an ID such as an 
```int``` or a const* for your Node type) and numeric value for the cost function (default is ```float```). 
Please consider the documentation of the ```PathFinding``` class in ```pathfinding.h``` for 
restrictions / requirements regarding the template arguments.
The heuristic and edge cost as well as the function retrieving the neighbors of a given node need to be 
provided as functors to parametrize the algorithm. 

## Usage Examples
You will find two usage examples, one with a graph that is explicitly represented using lists of nodes 
and edges (```example_with_graph.cpp```) and one that is based on an image where the graph is given implicitly 
by the regular grid sructure of the pixels (```example_with_image.cpp```). For the image example, you will
also need ```stb_image_write.h``` which you can find [here](https://github.com/nothings/stb). The result image 
of the image example is also included (```result_path```). 

## Platforms
The code has been compiled and tested on Linux Mint 19 with GCC 7.4 and Windows 7 64-but with Visual Studio 2017.
