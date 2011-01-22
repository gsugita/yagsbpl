/******************************************************************************************
*                                                                                        *
*    Yet Another Graph-Search Based Planning Library (YAGSBPL)                           *
*    A template-based C++ library for graph search and planning                          *
*    Version 1.0                                                                         *
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
******************************************************************************************/
//    For a detailed tutorial and download, visit 
//    http://fling.seas.upenn.edu/~subhrabh/cgi-bin/wiki/index.php?n=Projects.ProgrammingLibraries-YAGSBPL


template <class NodeType, class CostType>
void A_star_planner<NodeType,CostType>::init( GenericSearchGraphDescriptor<NodeType,CostType> theEnv , bool resetHash )
{
	GraphNode_p thisGraphNode;
	
	if (resetHash)
		GenericPlannerInstance.init(theEnv, heapKeyCount);  // This initiates the graph, hash and heap of the generic planner
	
	// Remapping for coding convenience
	GraphDescriptor = GenericPlannerInstance.GraphDescriptor;
	hash = GenericPlannerInstance.hash;
	heap = GenericPlannerInstance.heap;
	
	// Init graph, clear the heap and clear stored paths just in case they not empty due to a previous planning
	GraphDescriptor->init();
	heap->clear();
	bookmarkGraphNodes.clear();

	for (int a=0; a<GraphDescriptor->SeedNodes.size(); a++)
	{
		// Check if node is in hash table - get it if it is, otherwise create it.
		thisGraphNode = hash->getNodeInHash( GraphDescriptor->SeedNodes[a] );
		
		// If node was created, initialize it
		if ( !thisGraphNode->initiated )
		{
			thisGraphNode->f = subopEps * GraphDescriptor->_getHeuristicsToTarget( thisGraphNode->n );
			thisGraphNode->came_from = NULL;
			thisGraphNode->plannerVars.seedLineage = a;
			thisGraphNode->plannerVars.g = 0;
			thisGraphNode->plannerVars.expanded = false;
			
			if ( !GraphDescriptor->_isAccessible( thisGraphNode->n ) )
			{
				printf("ERROR (A_star): One of the seed nodes is not accessible!" );
				exit(1);
			}
			else
				thisGraphNode->plannerVars.accessible = true;
				
			thisGraphNode->initiated = true; // Always set this when other variables have already been set
		}
		
		// Push in heap
		heap->push( thisGraphNode );
	}
}

// -----------------------------

template <class NodeType, class CostType>
void A_star_planner<NodeType,CostType>::clearLastPlanAndInit( GenericSearchGraphDescriptor<NodeType,CostType>* theEnv_p )
{
	// Set every node in hash to not expanded
	if (hash->HashTable)
		for (int a=0; a<hash->hashTableSize; a++)
			for (int b=0; b<hash->HashTable[a].size(); b++)
			{
				hash->HashTable[a][b]->plannerVars.expanded = false;
				hash->HashTable[a][b]->initiated = false;
			}
	
	// Clear the last plan, but not the hash table
	if (theEnv_p)
		init(*theEnv_p, false);
	else
		init(*GraphDescriptor, false);
}

// ==================================================================================

template <class NodeType, class CostType>
void A_star_planner<NodeType,CostType>::plan(void)
{
	GraphNode_p thisGraphNode, thisNeighbourGraphNode;
	CostType this_g_val, thisTransitionCost, test_g_val;
	std::vector< NodeType > thisNeighbours;
	std::vector< CostType > thisTransitionCosts;
	int a;
	
	#if VIEW_PROGRESS_A_STAR
		float timediff = 0.0;
		expandcount = 0;
		startclock = clock();
		startsecond = time(NULL);
	#endif
	while ( !heap->empty() )
	{
		#if VIEW_PROGRESS_A_STAR
			if (expandcount % ProgressShowInterval == 0)
			{
				if (timediff>=0.0)
					timediff = ((float)(clock()-startclock)) / ((float)CLOCKS_PER_SEC);
				printf("Number of states expanded: %d. Heap size: %d. Time elapsed: %f s.\n", 
											expandcount, heap->size(), ((timediff>=0.0) ? timediff : difftime(time(NULL),startsecond)) );
			}
			expandcount++;
		#endif
	
		// Get the node with least f-value
		thisGraphNode = heap->pop();
		thisGraphNode->plannerVars.expanded = true; // Put in closed list
		
		// Check if we need to stop furthur expansion
		if ( GraphDescriptor->_stopSearch( thisGraphNode->n ) )
		{
			bookmarkGraphNodes.push_back(thisGraphNode);
			#if VIEW_PROGRESS_A_STAR
				if (timediff>=0.0)
					timediff = ((float)(clock()-startclock)) / ((float)CLOCKS_PER_SEC);
				printf("Stopping search!! Number of states expanded: %d. Heap size: %d. Time elapsed: %f s.\n", 
											expandcount, heap->size(), ((timediff>=0.0) ? timediff : difftime(time(NULL),startsecond)) );
			#endif
			return;
		}
		// Check if we need to store the path leading to this node
		if ( GraphDescriptor->_storePath( thisGraphNode->n ) )
		{
			bookmarkGraphNodes.push_back(thisGraphNode);
			#if VIEW_PROGRESS_A_STAR
				if (timediff>=0.0)
					timediff = ((float)(clock()-startclock)) / ((float)CLOCKS_PER_SEC);
				printf("Stored a path!! Number of states expanded: %d. Heap size: %d. Time elapsed: %f s.\n", 
											expandcount, heap->size(), ((timediff>=0.0) ? timediff : difftime(time(NULL),startsecond)) );
			#endif
		}
		
		// Generate the neighbours if they are already not generated
		thisNeighbours.clear();
		thisTransitionCosts.clear();
		if ( thisGraphNode->successors.empty() ) // Successors were not generated previously
		{
			GraphDescriptor->_getSuccessors( thisGraphNode->n , &thisNeighbours , &thisTransitionCosts );
			thisGraphNode->successors.init( thisNeighbours.size() );
			for (a=0; a<thisNeighbours.size(); a++)
				thisGraphNode->successors.set(a, hash->getNodeInHash(thisNeighbours[a]), thisTransitionCosts[a]);
		}
		
		// Initiate the neighbours (if required) and update their g & f values
		this_g_val = thisGraphNode->plannerVars.g;
		for (a=0; a<thisGraphNode->successors.size(); a++)
		{
			thisNeighbourGraphNode = thisGraphNode->successors.getLinkSearchGraphNode(a); //thisGraphNode->successors->operator[](a);
			thisTransitionCost = thisGraphNode->successors.getLinkCost(a); //thisGraphNode->successorsTransitionCosts[a];
			
			// An uninitiated neighbour node - definitely g & f values not set either.
			if ( !thisNeighbourGraphNode->initiated )
			{
				thisNeighbourGraphNode->plannerVars.accessible = GraphDescriptor->_isAccessible( thisNeighbourGraphNode->n );
				if ( thisNeighbourGraphNode->plannerVars.accessible )
				{
					thisNeighbourGraphNode->came_from = thisGraphNode;
					thisNeighbourGraphNode->plannerVars.seedLineage = thisGraphNode->plannerVars.seedLineage;
					thisNeighbourGraphNode->plannerVars.g = thisGraphNode->plannerVars.g + thisTransitionCost;
					thisNeighbourGraphNode->f = thisNeighbourGraphNode->plannerVars.g + 
													subopEps * GraphDescriptor->_getHeuristicsToTarget( thisNeighbourGraphNode->n );
					thisNeighbourGraphNode->plannerVars.expanded = false;
					
					// Put in open list and continue to next neighbour
					heap->push( thisNeighbourGraphNode );
				}
				thisNeighbourGraphNode->initiated = true; // Always set this when other variables have already been set
				continue;
			}
			
			// Neighbour that are not accessible or in closed list are to be skipped
			if ( !thisNeighbourGraphNode->plannerVars.accessible || thisNeighbourGraphNode->plannerVars.expanded )
				continue;
			
			// Update came_from, g and f values if better
			test_g_val = thisGraphNode->plannerVars.g + thisTransitionCost;
			if ( test_g_val < thisNeighbourGraphNode->plannerVars.g )
			{
				thisNeighbourGraphNode->plannerVars.g = test_g_val;
				thisNeighbourGraphNode->f = thisNeighbourGraphNode->plannerVars.g + 
													subopEps * GraphDescriptor->_getHeuristicsToTarget( thisNeighbourGraphNode->n );
				thisNeighbourGraphNode->came_from = thisGraphNode;
				thisNeighbourGraphNode->plannerVars.seedLineage = thisGraphNode->plannerVars.seedLineage;
				
				// Since thisNeighbourGraphNode->f is changed, re-arrange it in heap
				heap->remove( thisNeighbourGraphNode );
				heap->push( thisNeighbourGraphNode );
			}
		}
	}
}

// ==================================================================================

template <class NodeType, class CostType>
std::vector< SearchGraphNode< NodeType, CostType, A_star_variables<CostType> >* > 
											A_star_planner<NodeType,CostType>::getGoalGraphNodePointers(void)
{
	return (bookmarkGraphNodes);
}

template <class NodeType, class CostType>
std::vector<NodeType> A_star_planner<NodeType,CostType>::getGoalNodes(void)
{
	std::vector<NodeType> ret;
	for (int a=0; a<bookmarkGraphNodes.size(); a++)
		ret.push_back(bookmarkGraphNodes[a]->n);
	return (ret);
}

template <class NodeType, class CostType>
std::vector<CostType> A_star_planner<NodeType,CostType>::getPlannedPathCosts(void)
{
	std::vector<CostType> costs;
	for (int a=0; a<bookmarkGraphNodes.size(); a++)
		costs.push_back(bookmarkGraphNodes[a]->plannerVars.g);
	return (costs);
}

template <class NodeType, class CostType>
std::vector< std::vector< NodeType > > A_star_planner<NodeType,CostType>::getPlannedPaths(void)
{
	std::vector< std::vector< NodeType > > paths;
	std::vector< NodeType > thisPath;
	for (int a=0; a<bookmarkGraphNodes.size(); a++)
	{
		thisPath.clear();
		// Reconstruct path
		GraphNode_p thisGraphNode = bookmarkGraphNodes[a];
		while (thisGraphNode)
		{
			thisPath.push_back(thisGraphNode->n);
			thisGraphNode = thisGraphNode->came_from;
		}
		
		paths.push_back(thisPath);
	}
	return (paths);
}

template <class NodeType, class CostType>
A_star_variables<CostType> A_star_planner<NodeType,CostType>::getNodeInfo(NodeType n)
{
	GraphNode_p thisGraphNode = hash->getNodeInHash(n);
	return thisGraphNode->plannerVars;
}

