//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

#define USE_MANHATTAN 0	/* 0 for Euclidean, 1 for Manhattan */
#define USE_SMALLER_G 1 /* 0 to prefer larger g, 1 to prefer smaller g */
#define DIAGONAL_COST sqrt(2) /* set to 1 or sqrt(2) */
#define WGT_HEURISTIC 1 /* set to 1, 2, 4, 8 for respective parts */

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

	float getManhattanDist(Util::Point p1, Util::Point p2){
		return abs(p1.x-p2.x)+abs(p1.y-p2.y)+abs(p1.z-p2.z);
	}

	float getEuclideanDist(Util::Point p1, Util::Point p2){
		return sqrt(pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2) + pow(p1.z-p2.z, 2));
	}

	float heuristic_cost_estimate(Util::Point p1, Util::Point p2){
		if (USE_MANHATTAN){
			return getManhattanDist(p1, p2);
		}else{
			return getEuclideanDist(p1, p2);
		}
	}

	float getDist(Util::Point p1, Util::Point p2){
		/* Can modify later */
		return getEuclideanDist(p1, p2);
	}

	void AStarPlanner::getNeighbors(std::vector<SteerLib::AStarPlannerNode>& neighbors, SteerLib::AStarPlannerNode p, Util::Point goal){
		/* return all AStarPlannerNodes */
		std::vector<Util::Point> points;
		std::vector<Util::Point> points_diag;

		/*
			1	2	3
			4	p	6
			7	8	9  -testcase search-1 -ai searchAI
		 */

		int x = p.point.x;
		int y = p.point.y;
		int z = p.point.z;

		points_diag.push_back(Util::Point(x-1, y, z+1));
		points.push_back(Util::Point(x  , y, z+1));
		points_diag.push_back(Util::Point(x+1, y, z+1));
		points.push_back(Util::Point(x-1, y, z  ));
		points.push_back(Util::Point(x+1, y, z  ));
		points_diag.push_back(Util::Point(x-1, y, z-1));
		points.push_back(Util::Point(x  , y, z-1));
		points_diag.push_back(Util::Point(x+1, y, z-1));

		for (int i = 0; i < points.size(); i++){
			if(canBeTraversed(gSpatialDatabase->getCellIndexFromLocation(points[i]))){
				neighbors.push_back(SteerLib::AStarPlannerNode(points[i], p.g+1, p.g+heuristic_cost_estimate(goal, points[i]), p.point.x, p.point.y, p.point.z));
			}
		}
		if(!USE_MANHATTAN) {
		for (int i = 0; i < points_diag.size(); i++){
			if(canBeTraversed(gSpatialDatabase->getCellIndexFromLocation(points_diag[i]))){
				neighbors.push_back(SteerLib::AStarPlannerNode(points_diag[i], p.g+DIAGONAL_COST, p.g+WGT_HEURISTIC*heuristic_cost_estimate(goal, points_diag[i]), p.point.x, p.point.y, p.point.z));
			}
		}
		}
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;
		
		/*  0 if Manhattan
			1 if Euclidean */


		std::vector<SteerLib::AStarPlannerNode> closedset;
		std::vector<SteerLib::AStarPlannerNode> openset;
		
		/* 
		 * This is taken care of in our AStarPlannerNode
		 * g_score := map with default value of infinity
		 * g_score[start] := 0
		 * f_score = map with default value of infinity
		 * f_score[start] := g_score[start] + heuristic_cost_esitmate(goal, start)
		 */

		openset.push_back(SteerLib::AStarPlannerNode(start, 0,0, 0, 0, 0));
		float lowestFScore = 0;
		int lowestFIndex = 0;
		while(!openset.empty()){
			lowestFScore = openset[0].f;
			lowestFIndex = 0;

			for (int i = 1; i< openset.size(); i++){
				if (openset[i].f == lowestFScore){
					if (USE_SMALLER_G){
						/* Prefer bigger G */
						if (openset[i].g > openset[lowestFIndex].g){
							lowestFIndex = i;
							lowestFScore = openset[i].f;
						}
					}else{
						/* Prefer smaller G */
						if (openset[i].g < openset[lowestFIndex].g){
							lowestFIndex = i;
							lowestFScore = openset[i].f;
						}
					}

				}else if (openset[i].f < lowestFScore){
					lowestFIndex = i;
					lowestFScore = openset[i].f;
				}
			}

			SteerLib::AStarPlannerNode current = openset[lowestFIndex];
			if (current.point == goal){

				float copy_g = current.g;
				int copy_size = closedset.size();

				int i = 0;
			
				while (current.point!=start){
					agent_path.push_back(current.point);
					for (int i = 0; i< closedset.size(); i++){
						if (closedset[i].point == Util::Point(current.px, current.py, current.pz)){
							current = closedset[i];
							break;
						}
					}
				}

				agent_path.push_back(start);
				std::reverse(agent_path.begin(), agent_path.end());
				std::cout<<"\nPath length: " << copy_g << std::endl;
				std::cout<<"Number of nodes expanded: " << copy_size << std::endl;

				return true;
			}

			openset.erase(openset.begin()+lowestFIndex);
			closedset.push_back(current);


			std::vector<SteerLib::AStarPlannerNode> neighbors;
			getNeighbors(neighbors, current, goal);

			/* for each neighbour in neighbor_nodes(current) */
			for (int i = 0; i < neighbors.size(); i++){
				/* if neighbour in closed set continue*/
				if (std::find(closedset.begin(), closedset.end(), neighbors.at(i))!=closedset.end()){
					//printf("neighbor in closedset\n");
					continue;
				}else{
					 /*		tentative_g_score := g_score[current + dist_between(current, neighbor)]
					 *
					 *		if tentative_g_score < g_score[neighbor]
					 *			came_from[neighbor] := current
					 *			g_score[neighbor] := tentative_g_score;
					 *			f_score[neighbor] := g_score[neighbor] + heuristic_cost_estimate(neighbor, goal)
					 *
					 *		if neighbor not in openset
					 *			add neighbor to openset
					 */
					float tentGScore = current.g + getDist(current.point, neighbors.at(i).point);
					if ( tentGScore < neighbors.at(i).g ){
						neighbors.at(i).px = current.point.x;
						neighbors.at(i).py = current.point.y;
						neighbors.at(i).pz = current.point.z;

						neighbors.at(i).g = tentGScore;
						neighbors.at(i).f = tentGScore + WGT_HEURISTIC*heuristic_cost_estimate(neighbors[i].point, goal);
					}

					if (std::find(openset.begin(), openset.end(), neighbors.at(i))==openset.end()){
						openset.push_back(neighbors.at(i));					}

				}
			}
		
		}
		std::cout<<"\nCould not find path using A*";
		return false;
	}
}
