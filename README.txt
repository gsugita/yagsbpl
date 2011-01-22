******************************************************************************************
*                                                                                        *
*    Yet Another Graph-Search Based Planning Library (YAGSBPL)                           *
*    A template-based C++ library for graph search and planning                          *
*    Version 1.5                                                                         *
*    ----------------------------------------------------------                          *
*    Copyright (C) 2010  Subhrajit Bhattacharya                                          *
*                                                                                        *
*    This program is free software: you can redistribute it and/or modify                *
*    it under the terms of the GNU General Public License as published by                *
*    the Free Software Foundation, either version 3 of the License, or                   *
*    (at your option) any later version.                                                 *
*                                                                                        *
*    This program is distributed in the hope that it will be useful,                     *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of                      *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                       *
*    GNU General Public License for more details <http://www.gnu.org/licenses/>.         *
*                                                                                        *
*                                                                                        *
*    Contact: subhrajit@gmail.com, http://fling.seas.upenn.edu/~subhrabh/                *
*                                                                                        *
*                                                                                        *
******************************************************************************************

**********************************************************************************************************
*  For the latest documentation, detailed tutorial and download, visit                                   *
*  http://fling.seas.upenn.edu/~subhrabh/cgi-bin/wiki/index.php?n=Projects.ProgrammingLibraries-YAGSBPL  *
**********************************************************************************************************

Yet Another Graph-Search Based Planning Library - YAGSBPL

A template-based C++ library for graph search and planning


INTRODUCTION
============

Graph search algorithms like Dijkstra's, A* and weighted A* has been implemented in several C++ libraries. Some of the well-known libraries like STL and LEMON have quite nice graph search algorithm implementations. Another very important and technical implementation is the SBPL by Maxim Likhachev from CMU (http://www.cs.cmu.edu/~maxim/software.html). While I did use many of these in multiple occasions with great results, I still ended up writing a library of my own because of certain specific needs in my applications. Hopefully you'll find this library useful.

The primary considerations behind writing the library were:
* Dynamic graph construction: One shouldn't need to construct and store a complete graph before starting the search/planning process. The graph is constructed and states are expanded on the fly and as required. (This is an already-existing feature in SBPL)
* Should be easy to use: Specifically defining a graph, even the complex ones, should be very easy to do. This was one of the most important motivating factor behind writing this library.
* Should be fast & efficient: Any graph search without the global knowledge of the graph from before typically needs a hash table for maintaining and creating the graph nodes, as well as needs an efficient heap for ordering the nodes in the open list. These are the components  that tend to slow down the search processes.
* In the graph search we should be able to have multiple start coordinates (seeds), store trajectories intermediately, and have multiple possible goals (or complex ways of defining possible goal nodes).

YAGSBPL supports:
* Directed graphs with complex cost functions.
* Dynamic graph construction.
* Arbitrary graph node type declaration.
* Arbitrary edge cost type declaration.
* Intermediate storage of paths during graph search.
* Multiple goals (or goal manifold) that determine when to stop search.
* Ability to write new planners with much ease. Comes with a weighted A* (that includes Dijkstra's and normal A*) planner by default.

So without any further delay, we dive right into some tutorials on YAGSBPL.


BASIC CONCEPT
=============

YAGSBPL has two primary components: A graph descriptor, and a planner. These two are bound together by the YAGSBPL-base library. By default YAGSBPL comes with a weighted A* (that includes Dijkstra's and normal A*) planner. So from an user perspective the first thing you would probably like to do is create a graph and use the planner to search in it.


Creating a Graph:
-----------------

Creating a graph is a two-step process:
# Define a node type (a C++ class) with the operator== overloaded for it.
# Define functions that tell which nodes are accessible, define the connectivity of the graph, define costs & heuristics, etc.

Thus, in your code, say "mygraphsearch.cpp", you would probably start with something like,

/************* CODE *************/
#include <stdio.h>
// YAGSBPL libraries
#include "yagsbpl_base.h"
#include "planners/A_star.h"

// A node of the graph
class myNode
{
public:
	int x, y; // Profiling observation: integer coordinates, hence operator==,
	          //  makes the search significantly faster (almost 10 folds than double)
	bool operator==(const myNode& n) { return (x==n.x && y==n.y); }; // This must be defined for the node
};
/************* CODE-END *************/

This defines discretization of an X-Y plane with a graph node placed at every integer coordinates. Note that we have overloaded the '==' operator. This operator will tell the planner if two nodes are the same. This is the only thing that the planner needs out of a node. Other than that the member variables of the node (x and y in this case) can be anything arbitrary.

Once a node is defined, it's time to define the connectivity of the graph, which nodes are accessible, etc. For these YAGSBPL provides the following template class meant to describe the structure of a graph:

/************* CODE *************/
template <class NodeType, class CostType>
class GenericSearchGraphDescriptor
{
public:
	// Primary functions
	int (*getHashBin_fp)(NodeType& n); // This MUST be provided.
	bool (*isAccessible_fp)(NodeType& n); // optional
	void (*getSuccessors_fp)(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c); // s: successors, c: transition costs
	void (*getPredecessors_fp)(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c); // May not be used by all planners
	CostType (*getHeuristics_fp)(NodeType& n1, NodeType& n2); // optional
	bool (*storePath_fp)(NodeType& n); // optional
	bool (*stopSearch_fp)(NodeType& n); // optional if "TargetNode" is given.
	
	// Primary variables
	int hashTableSize; // Number of hash bins. This MUST be provided. "getHashBin" must return a value between 0 and hashTableSize
	std::vector<NodeType> SeedNodes; // An initial set of "start" nodes to be put in heap. In general this is a single node.
	NodeType TargetNode; // "Goal" used for computing Heuristics. Not required if "getHeuristics" is not provided.
						 //	   If "stopSearch" is not provided, this is used to determine when to stop.
	
	// If pointers to primary functions cannot be provided, we can alternatively use the virtual functions in the FunctionContainer
	SearchGraphDescriptorFunctionContainer<NodeType,CostType>* func_container; // Don't worry about this now. Will be explained later.
	
	// Other variables and functions - constructor chooses a default
	int hashBinSizeIncreaseStep; // optional. Tells how to increase the size of hash bins. Default is 128.
	
	// other internal use functions ...
	// ...
};
/************* CODE-END *************/

Typically, for creating a graph, in your code you'll create an instance of this class and set the function pointers to functions defined by you, and set the values of "hashTableSize", "SeedNodes" and "TargetNode". The members are quite self-explanatory by their name. A brief explanation of each:
* hashTableSize - As discussed earlier, YAGSBPL maintains a hash table for keeping track of the nodes explored efficiently. This declares the size of the hash table. In other words, the value that (*getHashBin_fp) returns should lie between 0 and hashTableSize-1 (both inclusive). Otherwise a segmentation fault will be thrown.
* getHashBin_fp - Pointer to a function that maps nodes to hast bins in the hash table (i.e the hash function). Ideally this needs to be defined such that the nodes are distributed randomly and with uniform probability among all the bins. Read "Hash Function" in any data-structure/programming book for more details. IMPORTANT: This function should return an integer value between (including) 0 and hashTableSize-1.
* isAccessible_fp - Takes in a node and tells whether or not is accessible.
* getSuccessors_fp - Takes a node 'n', and returns a list of successor nodes in 's', along with the transition costs 'c'. 's' and 'c' should be of equal length. This function defines the connectivity of the directed graph and edge costs.
* getPredecessors_fp - Provision is kept. But not used by weighted A* planner. So you need not worry about this if you are using the default planner.
* getHeuristics_fp - The heuristics function for A*. The heuristics will be set to 0 (i.e. Dijkstra's) if this function is not set. The function must return a value less than or equal to the actual cost between nodes 'n1' and 'n2'.
* storePath_fp - Determined based on the node 'n', whether to store the path up to that node. Graph search will continue as usual. Only the path to 'n' will be stored intermediately.
* stopSearch_fp - Whether or not to stop the search process when the node 'n' has been expanded. This function also stores the path up-to 'n'. Note that the function can be used to define multiple possible stop nodes, whichever is reached earlier (e.g. the body may contain the statement "return (n==n1 || n==n2);".)
* SeedNodes - the nodes to put in the "open" list at the start of the graph search.
* TargetNode - for computing heuristics.

Thus, in our "mygraphsearch.cpp" the following code may be present in the main function:

/************* CODE *************/
// Function declarations
int getHashBin(myNode& n);
bool isAccessible(myNode& n);
void getSuccessors(myNode& n, std::vector<myNode>* s, std::vector<double>* c);
double getHeuristics(myNode& n1, myNode& n2)

// Entry point
int main(int argc, char *argv[])
{
	// Profiling observation: Using int instead of double cost provides marginal improvement (~10%)
	GenericSearchGraphDescriptor<myNode,double> myGraph;
	
	// We describe the graph, cost function, heuristics, and the start & goal in this block
	// ------------------------------------------------------------------------------------
	// Set the functions
	myGraph.getHashBin_fp = &getHashBin;
	myGraph.isAccessible_fp = &isAccessible;
	myGraph.getSuccessors_fp = &getSuccessors;
	myGraph.getHeuristics_fp = &getHeuristics;
	// Set other variables
	myGraph.hashTableSize = 212; // Since in this problem, "getHashBin" can return a max of value 201.
	myGraph.hashBinSizeIncreaseStep = 512; // By default it's 128. For this problem, we choose a higher value.
	
	myNode tempNode;
	tempNode.x = -150; tempNode.y = -150; // Start node
	myGraph.SeedNodes.push_back(tempNode);
	tempNode.x = 150; tempNode.y = 150; // Goal node
	myGraph.TargetNode = tempNode;
	// ------------------------------------------------------------------------------------
	
	// Planning and other activities go here.
	// ...
}
/************* CODE-END *************/

Where the functions getHashBin, isAccessible, getSuccessors and getHeuristics can be functions defined in the global scope as follows:

/************* CODE *************/
// Functions that describe the graph

int getHashBin(myNode& n) // Use the absolute value of x coordinate as hash bin counter. Not a good choice though!
{
	return ((int)fabs(n.x));
}

bool isAccessible(myNode& n)
{
	// A 401x401 sized environment with (0,0) at center, and a circular obstacle of radius 100 centered at (0,0)
	return ( n.x*n.x + n.y*n.y > 10000 
				&& n.x>=-200 && n.x<=200 && n.y>=-200 && n.y<=200 );
}

void getSuccessors(myNode& n, std::vector<myNode>* s, std::vector<double>* c) // Define a 8-nearest-neighbor connected graph
{
	// This function needn't account for obstacles or size of environment. That's done by "isAccessible"
	myNode tn;
	s->clear(); c->clear(); // Planner is supposed to clear these. Still, for safety we clear it again.
	for (int a=-1; a<=1; a++)
		for (int b=-1; b<=1; b++) {
			if (a==0 && b==0) continue;
			tn.x = n.x + a;
			tn.y = n.y + b;
			s->push_back(tn);
			c->push_back(sqrt((double)(a*a+b*b)));
		}
}

double getHeuristics(myNode& n1, myNode& n2)
{
	int dx = abs(n1.x - n2.x);
	int dy = abs(n1.y - n2.y);
	return (sqrt((double)(dx*dx + dy*dy))); // Euclidean distance as heuristics
}

/************* CODE-END *************/


Planning / Graph search:
------------------------

Once the graph is created, using the planner and retrieving the result is straight-forward. While retrieving the results, all the intermediately stored paths (triggered by 'storePath_fp') as well as the the last path (when 'stopSearch_fp' returned true). The last path will not be repeated if both 'storePath_fp' and 'stopSearch_fp' returned true for a node.
For planning, the template class and it's relevant members are as follows:

/************* CODE *************/
template <class NodeType, class CostType>
class A_star_planner
{
	// Initializer and planner
	void init( GenericSearchGraphDescriptor<NodeType,CostType> theEnv , double eps=1.0 );
	void plan(void);
	
	// Planner output access: ( to be called after plan(), and before destruction of planner )
	std::vector< NodeType > getGoalNodes(void); // This is most useful
	std::vector< GraphNode_p > getGoalGraphNodePointers(void); // Retrieves pointers from the hash table
	std::vector< std::vector< NodeType > > getPlannedPaths(void);
	std::vector< CostType > getPlannedPathCosts(void); // This is also useful for getting the path costs.
	
	// Other members not relevant to user...
	// ...
};
/************* CODE-END *************/

The following complete code for the main function explains the use of the planner:

/************* CODE *************/
int main(int argc, char *argv[])
{
	// Profiling observation: Using int instead of double cost provides marginal improvement (~10%)
	GenericSearchGraphDescriptor<myNode,double> myGraph;
	
	// We describe the graph, cost function, heuristics, and the start & goal in this block
	// ------------------------------------------------------------------------------------
	// Set the functions
	myGraph.getHashBin_fp = &getHashBin;
	myGraph.isAccessible_fp = &isAccessible;
	myGraph.getSuccessors_fp = &getSuccessors;
	myGraph.getHeuristics_fp = &getHeuristics;
	// Set other variables
	myGraph.hashTableSize = 212; // Since in this problem, "getHashBin" can return a max of value 201.
	myGraph.hashBinSizeIncreaseStep = 512; // By default it's 128. For this problem, we choose a higher value.
	
	myNode tempNode;
	tempNode.x = -150; tempNode.y = -150; // Start node
	myGraph.SeedNodes.push_back(tempNode);
	tempNode.x = 150; tempNode.y = 150; // Goal node.
	myGraph.TargetNode = tempNode; // Since we did not give a value 'stopSearch_fp', this will be used to determine when to stop.
	// ------------------------------------------------------------------------------------
	
	// Planning
	A_star_planner<myNode,double>  planner;
	planner.init(myGraph);
	planner.plan();
	
	// Retrieve the stored paths and print the coordinates.
	std::vector< std::vector< myNode > > paths = planner.getPlannedPaths();
	printf("Number of paths: %d\nPath coordinates: \n[ ", paths.size());
	for (int a=0; a<paths[0].size(); a++)
		printf("[%d, %d]; ", paths[0][a].x, paths[0][a].y);
	printf(" ]\n\n");
}
/************* CODE-END *************/


USING CLASS MEMBER FUNCTIONS
============================

Say, you want to use pointers to member functions of a class for 'isAccessible_fp', 'getSuccessors_fp', etc. The following will fail since "bool (*)(NodeType& n)" and "bool (myGraphDescription::*)(NodeType& n)" are of incompatable types:

/************* CODE *************/

// Includes and declaration of myNode goes here as usual.
// ....

class localGraphClass
{
public:
	set(int xmin, int xmax, int ymin, int ymax) { Xmin=xmin; Xmax=xmax; Ymin=ymin; Ymax=ymax; };
	
	// Member functions
	bool isAccessible(myNode& n) { return( n.x>=Xmin && n.x<=Xmax && n.y>=Ymin && n.y<=Ymax ); }
	// other members.
	// ...
	
private:
	int Xmin, Xmax, Ymin, Ymax;
};

int main(int argc, char *argv[])
{
	localGraphClass localGraphClassInstance;
	localGraphClassInstance.set( -200, 200, -200, 200 );
	
	GenericSearchGraphDescriptor<myNode,double> myGraph;
	
	// Set the functions that describe the graph
	myGraph.isAccessible_fp = &localGraphClassInstance.isAccessible; // This will FAIL!!
	                                                               // No attempts (like type casting) to make this work will work.
	// etc...
}
/************* CODE-END *************/

In order to be able to use class member functions, one of the following two approaches can be taken:

Using function container for direct pointers to member functions:
-----------------------------------------------------------------

YAGSBPL declares the following template class:

/************* CODE *************/
template <class NodeType, class CostType, class ContainerClass>
class SearchGraphDescriptorFunctionPointerContainer
{
public:
	ContainerClass* p;
	int (ContainerClass::*getHashBin_fp)(NodeType& n);
	bool (ContainerClass::*isAccessible_fp)(NodeType& n);
	void (ContainerClass::*getSuccessors_fp)(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c);
	void (ContainerClass::*getPredecessors_fp)(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c);
	CostType (ContainerClass::*getHeuristics_fp)(NodeType& n1, NodeType& n2);
	bool (ContainerClass::*storePath_fp)(NodeType& n);
	bool (ContainerClass::*stopSearch_fp)(NodeType& n);
	
	// other members
	// ...
};
/************* CODE-END *************/

This, along with the class "SearchGraphDescriptorFunctionContainer" (described later), can be used as follows for using class member functions.

/************* CODE *************/

// Includes and declaration of myNode goes here as usual.
// ....

class localGraphClass
{
public:
	set(int xmin, int xmax, int ymin, int ymax) { Xmin=xmin; Xmax=xmax; Ymin=ymin; Ymax=ymax; };
	
	// Member functions
	bool isAccessible(myNode& n) { return( n.x>=Xmin && n.x<=Xmax && n.y>=Ymin && n.y<=Ymax ); }
	// other members.
	// ...
	
private:
	int Xmin, Xmax, Ymin, Ymax;
};

int main(int argc, char *argv[])
{
	localGraphClass localGraphClassInstance;
	localGraphClassInstance.set( -200, 200, -200, 200 );
	
	// We create an instance of "SearchGraphDescriptorFunctionPointerContainer"
	SearchGraphDescriptorFunctionPointerContainer<myNode,double,localGraphClass>* fun_pointer_container
							= new SearchGraphDescriptorFunctionPointerContainer<myNode,double,localGraphClass>;
	// And set the pointers in it...
	fun_pointer_container->p = &localGraphClassInstance; // This tells which instance to use.
	fun_pointer_container->isAccessible_fp = &localGraphClass::isAccessible;
	// likewise, set the other pointers to member functions...
	// ...
	
	// And in the instance of "GenericSearchGraphDescriptor", 
	//   we set the member "func_container" after type casting, instead of directly setting the function pointers
	GenericSearchGraphDescriptor<myNode,double> myGraph;
	myGraph.func_container = (SearchGraphDescriptorFunctionContainer<myNode,double>*)fun_pointer_container; // This will WORK!!
	                          // NOTE: "SearchGraphDescriptorFunctionContainer" is a class defined in YAGSBPL library
	// etc...
}
/************* CODE-END *************/


Creating a class that inherits virtual functions from "SearchGraphDescriptorFunctionContainer":
----------------------------------------------------------------------------------------------

An alternative (and possibly preferred) method is to derive the "localGraphClass" from YAGSBPL's "SearchGraphDescriptorFunctionContainer" and override the virtual functions declared in there. In YAGSBPL the following template class is declared:

/************* CODE *************/
template <class NodeType, class CostType>
class SearchGraphDescriptorFunctionContainer
{
public:
	virtual int getHashBin(NodeType& n);
	virtual bool isAccessible(NodeType& n);
	virtual void getSuccessors(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c);
	virtual void getPredecessors(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c);
	virtual CostType getHeuristics(NodeType& n1, NodeType& n2);
	virtual bool storePath(NodeType& n);
	virtual bool stopSearch(NodeType& n);
	// other members ...
};
/************* CODE-END *************/

The following example illustrates how this can be used:

/************* CODE *************/
// Includes and declaration of myNode goes here as usual.
// ....

class localGraphClass : public SearchGraphDescriptorFunctionContainer<myNode,double>
{
public:
	set(int xmin, int xmax, int ymin, int ymax) { Xmin=xmin; Xmax=xmax; Ymin=ymin; Ymax=ymax; };
	
	// Virtual member functions derived from "SearchGraphDescriptorFunctionContainer" being overwritten...
	// NOTE: The name of the functions are important here.
	bool isAccessible(myNode& n) { return( n.x>=Xmin && n.x<=Xmax && n.y>=Ymin && n.y<=Ymax ); }
	// other members.
	// ...
	
private:
	int Xmin, Xmax, Ymin, Ymax;
};

int main(int argc, char *argv[])
{
	localGraphClass localGraphClassInstance;
	localGraphClassInstance.set( -200, 200, -200, 200 );
	
	// And in the instance of "GenericSearchGraphDescriptor", 
	//   we set the member "func_container", instead of directly setting the function pointers
	GenericSearchGraphDescriptor<myNode,double> myGraph;
	myGraph.func_container = &localGraphClassInstance; // This will WORK!! Automatic type casting.
	
	// etc...
}
/************* CODE-END *************/


WRITING A PLANNER
=================

The YAGSBPL comes with a default weighted A* planner. Setting the weight (eps) to 1.0 makes it standard A*, and setting the heuristics to 0 makes it Dijkstra. However these are definitely not the only planners that one can do away with. More advanced planners like D*, ARA*, etc. that can plan incrementally needs to be included. Writing a planner is also made quite simple in the YAGSBPL framework. YAGSBPL automatically creates and maintains the hash table and a heap. If you are interested in writing a planner, the best way to go about is to look into the code of the A start planner inside the "planners" folder of YAGSBPL. You'll hopefully find it quite short and precise so as to understand how it works and start writing your own planner in no time! The only point I would like to emphasize here is that it is always prudent to check if successors exist before calling "_getSuccessors", since the later is almost always a lot more expensive.


WRITING A GRAPH ENCAPSULATION
=============================

You may, if needed by your application, write an "encapsulation" on top of the "GenericSearchGraphDescriptor" class (e.g. derived from it). This may make your code much more structured and easy to write. No default encapsulation is provided since that will most likely be specific to your application.


INSTALLATION
============

Since YAGSBPL is a template-based library, it cannot be compiled as an object file. All that needs to be done is include the header files in your C++ code. Thus, say the YAGSBPL library is located in the folder "/path/to/yagsbpl" (which contains the "yagsbpl_base.h" file and the "planners" folder), the following two things need to be done by you:
1. In your code place the following includes:
     #include "yagsbpl_base.h"
     #include "planners/A_star.h"
2. When compiling your code have "/path/to/yagsbpl" in your include folder list. That is, compile with the option -I/path/to/yagsbpl .

That's it!


