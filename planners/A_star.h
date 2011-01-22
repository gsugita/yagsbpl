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


#ifndef __A_STAR_2F585H2B321R_H_
#define __A_STAR_2F585H2B321R_H_


#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include "yagsbpl_base.h"

#define VIEW_PROGRESS_A_STAR 1


template <class CostType>
class A_star_variables
{
public:
	CostType g;
	bool expanded; // Whether in closed list or not
	bool accessible; // Since the environment is assumed to to change, each node has fixed accessibility
	int seedLineage; // stores which seed the node came from
};

template <class NodeType, class CostType>
class A_star_planner
{
public:
	// typedef's for convenience:
	typedef  A_star_variables<CostType>  PlannerSpecificVariables;
	typedef  SearchGraphNode< NodeType, CostType, PlannerSpecificVariables >*  GraphNode_p;
	
	// Instance of generac planner
	GenericPlanner< NodeType, CostType, PlannerSpecificVariables > GenericPlannerInstance;
	// Re-mapping of generic planner variables for ease of use (coding convenience)
	GenericSearchGraphDescriptor<NodeType,CostType>* GraphDescriptor;
	HashTableContainer<NodeType,CostType,PlannerSpecificVariables>* hash;
	HeapContainer<NodeType,CostType,PlannerSpecificVariables>* heap;
	
	// Member variables
	double subopEps;
	int heapKeyCount;
	int ProgressShowInterval;
	std::vector< GraphNode_p > bookmarkGraphNodes;
	
	// Initializer and planner
	A_star_planner()
		{ subopEps = 1.0; heapKeyCount = 20; ProgressShowInterval = 10000; }
	void setParams( double eps=1.0 , int heapKeyCt=20 , int progressDispInterval=10000 ) // call to this is optional.
		{ subopEps = eps; heapKeyCount = heapKeyCt; ProgressShowInterval = progressDispInterval; }
	void init( GenericSearchGraphDescriptor<NodeType,CostType> theEnv , bool resetHash=true );
	void plan(void);
	// Clear the last plan, but not the hash table.
	void clearLastPlanAndInit( GenericSearchGraphDescriptor<NodeType,CostType>* theEnv_p=NULL );
	
	// Planner output access: ( to be called after plan(), and before destruction of planner )
	std::vector< NodeType > getGoalNodes(void);
	std::vector< GraphNode_p > getGoalGraphNodePointers(void);
	std::vector< std::vector< NodeType > > getPlannedPaths(void);
	std::vector< CostType > getPlannedPathCosts(void);
	A_star_variables<CostType> getNodeInfo(NodeType n);
	
	// Other variables for querying progress of planning process from other functions
	#if VIEW_PROGRESS_A_STAR
		clock_t  startclock;
		time_t startsecond;
		int expandcount;
	#endif
};

// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// Since we use templates, the definitions need to be included in the header file as well.

#include "A_star.cpp"

#endif

