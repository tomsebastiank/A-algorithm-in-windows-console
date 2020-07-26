/*
OneLoneCoder.com - PathFinding A*
"No more getting lost..." - @Javidx9

License
~~~~~~~
Copyright (C) 2018  Javidx9
This program comes with ABSOLUTELY NO WARRANTY.
This is free software, and you are welcome to redistribute it
under certain conditions; See license for details.
Original works located at:
https://www.github.com/onelonecoder
https://www.onelonecoder.com
https://www.youtube.com/javidx9

GNU GPLv3
https://github.com/OneLoneCoder/videos/blob/master/LICENSE

From Javidx9 :)
~~~~~~~~~~~~~~~
Hello! Ultimately I don't care what you use this for. It's intended to be
educational, and perhaps to the oddly minded - a little bit of fun.
Please hack this, change it and use it in any way you see fit. You acknowledge
that I am not responsible for anything bad that happens as a result of
your actions. However this code is protected by GNU GPLv3, see the license in the
github repo. This means you must attribute me if you use it. You can view this
license here: https://github.com/OneLoneCoder/videos/blob/master/LICENSE
Cheers!


Background
~~~~~~~~~~
The A* path finding algorithm is a widely used and powerful shortest path
finding node traversal algorithm. A heuristic is used to bias the algorithm
towards success. This code is probably more interesting than the video. :-/


Author
~~~~~~
Twitter: @javidx9
Blog: www.onelonecoder.com

Video:
~~~~~~
https://youtu.be/icZj67PTFhc

Last Updated: 08/10/2017
*/

#include <iostream>
#include <string>
#include <algorithm>
using namespace std;

#include "olcConsoleGameEngine.h"

class OneLoneCoder_PathFinding : public olcConsoleGameEngine
{
public:
	OneLoneCoder_PathFinding()
	{
		m_sAppName = L"Path Finding";
	}

private:

	struct sNode
	{
		bool bObstacle = false;			// Is the node an obstruction?
		bool bVisited = false;			// Have we searched this node before?
		float fGlobalGoal;				// Distance to goal so far
		float fLocalGoal;				// Distance to goal if we took the alternative route
		int x;							// Nodes position in 2D space
		int y;
		vector<sNode*> vecNeighbours;	// Connections to neighbours
		sNode* parent;					// Node connecting to this node that offers shortest parent
	};

	sNode *nodes = nullptr;
	int nMapWidth = 16;
	int nMapHeight = 16;

	sNode*nodeStart = nullptr;
	sNode*nodeEnd = nullptr;



protected:
	virtual bool OnUserCreate()
	{
		
		// Create a 2D array of nodes - this is for convenience of rendering and construction
		// and is not required for the algorithm to work - the nodes could be placed anywhere
		// in any space, in multiple dimensions...
		nodes = new sNode[nMapWidth * nMapHeight];
		for (int x = 0; x < nMapWidth; x++)
		{
			for (int y = 0; y < nMapHeight; y++)
			{
				nodes[y * nMapWidth + x].x = x; // ...because we give each node its own coordinates
				nodes[y * nMapWidth + x].y = y;
				nodes[y * nMapWidth + x].bObstacle = false;
				nodes[y * nMapWidth + x].parent = nullptr;
				nodes[y * nMapWidth + x].bVisited = false;
			}
		}
			
			

		//create connection
		for (int x = 0; x < nMapWidth; x++)
		{
			for (int y = 0; y < nMapHeight; y++)
			{
				if (y > 0)
					nodes[y * nMapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * nMapWidth + x]);
				if (x > 0)
					nodes[y * nMapWidth + x].vecNeighbours.push_back(&nodes[y * nMapWidth + (x - 1)]);
				if (y < nMapHeight-1)
					nodes[y * nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * nMapWidth + x]);
				if (x < nMapWidth -1 )
					nodes[y * nMapWidth + x].vecNeighbours.push_back(&nodes[y * nMapWidth + (x + 1)]);

					
			}
		}
			
		nodeStart = &nodes[3 * nMapWidth + 5];
		nodeEnd = &nodes[5 * nMapWidth + 3];
		return true;
	}

	

	virtual bool OnUserUpdate(float fElapsedTime)
	{
		
		int nNodeSize = 9;
		int nNodeBorder = 2;

		int nSelectNodeX = m_mousePosX / nNodeSize;
		int nSelectNodeY = m_mousePosY / nNodeSize;


		if (m_mouse[0].bReleased)
		{
			if (nSelectNodeX > 0 && nSelectNodeX < nMapWidth)
			{
				if (nSelectNodeY > 0 && nSelectNodeY < nMapHeight)
				{
					if (m_keys[VK_SHIFT].bHeld)
						nodeStart = &nodes[nSelectNodeY*nMapWidth + nSelectNodeX];
					else if (m_keys[VK_CONTROL].bHeld)
						nodeEnd = &nodes[nSelectNodeY*nMapWidth + nSelectNodeX];
					else
						nodes[nSelectNodeY*nMapWidth + nSelectNodeX].bObstacle = !nodes[nSelectNodeY*nMapWidth + nSelectNodeX].bObstacle;
					Solve_Astar();
				}
			}
		}

		// Draw Connections First - lines from this nodes position to its
		// connected neighbour node positions
		Fill(0, 0, ScreenWidth(), ScreenHeight(), L' ');
	

		for (int x = 0; x < nMapWidth; x++)
		{
			for (int y = 0; y < nMapHeight; y++)
			{
				for (auto n : nodes[y * nMapWidth + x].vecNeighbours)
				{
					DrawLine(x*nNodeSize + nNodeSize / 2, y*nNodeSize + nNodeSize / 2, n->x*nNodeSize + nNodeSize / 2,
						n->y*nNodeSize + nNodeSize / 2, PIXEL_SOLID, FG_BLUE);
				}

			}
		}

		// Draw Nodes on top
		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++)
			{

				Fill(x*nNodeSize + nNodeBorder, y*nNodeSize + nNodeBorder, (x + 1)*nNodeSize - nNodeBorder, (y + 1)*nNodeSize - nNodeBorder,
					PIXEL_HALF, nodes[y*nMapWidth + x].bObstacle ? FG_WHITE: FG_BLUE);

				if(nodes[y*nMapWidth + x].bVisited)
					Fill(x*nNodeSize + nNodeBorder, y*nNodeSize + nNodeBorder, (x + 1)*nNodeSize - nNodeBorder, (y + 1)*nNodeSize - nNodeBorder,
						PIXEL_SOLID, FG_BLUE);
				if (&nodes[y*nMapWidth + x]==nodeStart)
					Fill(x*nNodeSize + nNodeBorder, y*nNodeSize + nNodeBorder, (x + 1)*nNodeSize - nNodeBorder, (y + 1)*nNodeSize - nNodeBorder,
						PIXEL_HALF, FG_GREEN);

				if (&nodes[y*nMapWidth + x] == nodeEnd)
					Fill(x*nNodeSize + nNodeBorder, y*nNodeSize + nNodeBorder, (x + 1)*nNodeSize - nNodeBorder, (y + 1)*nNodeSize - nNodeBorder,
						PIXEL_HALF, FG_RED);
			}

		if (nodeEnd != nullptr)
		{
			sNode * p = nodeEnd;
			while (p->parent != nullptr)
			{
				DrawLine(p->x * nNodeSize + nNodeSize/2 , p->y * nNodeSize + nNodeSize / 2,
					p->parent->x * nNodeSize + nNodeSize / 2, p->parent->y * nNodeSize + nNodeSize / 2,
					PIXEL_SOLID, FG_YELLOW);
				p = p->parent;
			}
		}
	

		return true;
	}
	void Solve_Astar()
	{
		for (int x = 0; x < nMapWidth; x++)
		{
			for (int y = 0; y < nMapHeight; y++)
			{
				
				nodes[y * nMapWidth + x].parent = nullptr;
				nodes[y * nMapWidth + x].bVisited = false;
				nodes[y * nMapWidth + x].fLocalGoal = INFINITY;
				nodes[y * nMapWidth + x].fGlobalGoal = INFINITY;
			}
		}

		// lamda funsction
		auto distance = [](sNode* a, sNode* b)
		{
			return sqrtf(((a->x - b->x) * (a->x - b->x)) + ((a->y - b->y) * (a->y - b->y)));
		};

		auto heuristic = [distance](sNode* a, sNode* b)
		{
			return distance(a, b);
		};

		sNode * nodeCurrent = nodeStart;
		nodeStart->fLocalGoal = 0.0f;
		nodeStart->fGlobalGoal = heuristic(nodeStart, nodeEnd);

		list<sNode*> listNotTested;
		listNotTested.push_back(nodeStart);

		while (!listNotTested.empty())
		{
			// Sort Untested nodes by global goal, so lowest is first
			//lamda functions
			listNotTested.sort([](const sNode* lhs, const sNode* rhs) { return lhs->fGlobalGoal < rhs->fGlobalGoal; });

			while (!listNotTested.empty() && listNotTested.front()->bVisited)
				listNotTested.pop_front();
			//This will happen when only one remaining one was removed
			if (listNotTested.empty())
				break;
			nodeCurrent = listNotTested.front();
			nodeCurrent->bVisited = true;

			// Visiting neighbours iteratiely 
			for (auto n : nodeCurrent->vecNeighbours)
			{
				if (!n->bVisited && !n->bObstacle)
					listNotTested.push_back(n);
				float fTemp = nodeCurrent->fLocalGoal + distance(nodeCurrent , n);
				if (fTemp < n->fLocalGoal)
				{
					n->parent = nodeCurrent;
					n->fLocalGoal = fTemp;
					n->fGlobalGoal = n->fLocalGoal + heuristic(n, nodeEnd);
				}
			}
		}

		
	}
	

};

int main()
{
	OneLoneCoder_PathFinding game;
	game.ConstructConsole(160,160, 4, 4);
	//game.ConstructConsole(80, 48, 16, 16);
	game.Start();
	return 0;
}
