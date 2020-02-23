//============================================================================
// Name        : bfs_ucs_astar.cpp
// Author      : Matrix7689 / JP
// Title	   : BFS, UCS and A* Search
//============================================================================
#include <fstream>
#include <sstream>
#include <bits/stdc++.h>

using namespace std;

struct node_bfs
{
	int x,y;
    node_bfs(int x, int y): x(x), y(y)
	{
    }
};

struct node
{
	int x,y;
	long long int pathcost;
    node(int x, int y, long long int pathcost): x(x), y(y), pathcost(pathcost)
    {
    }
};


bool list_compare(node const& p1, node const& p2)
{
	return p1.pathcost < p2.pathcost;
}

struct node_as
{
	int x,y;
	long long int pathcost;
	double heuristic;
    node_as(int x, int y, long long int pathcost, double heuristic): x(x), y(y), pathcost(pathcost), heuristic(heuristic)
	{
    }
};

bool list_compare_as(node_as const& p1, node_as const& p2)
{
	return (p1.pathcost + p1.heuristic) < (p2.pathcost + p2.heuristic);
}


double calc_linedistance(int x1, int y1, int x2, int y2)
{
	return (10 * max (abs(x1-x2),abs(y1-y2)));
}

int mapheight, mapwidth, max_elevation;
std::ofstream writeoutput("output.txt");

void printFullPath(map<pair<int,int>, pair<int,int>> parent, pair<int,int> dest, pair<int,int> actual_dest)
{
    if(dest.first == -1 && dest.second == -1)
        return;
    printFullPath(parent, parent[dest], actual_dest);
    if(dest.first == actual_dest.first && dest.second == actual_dest.second)
    	writeoutput << dest.first << "," << dest.second;
    else
    	writeoutput << dest.first << "," << dest.second << " ";
}

void BFS(vector<vector<int>> terrainz, node_bfs source, node_bfs destination)
{
	list <node_bfs> open;
	list <node_bfs> :: iterator ito;
    map<pair<int,int>, pair<int,int>> parent;
    map<pair<int,int>, pair<int,int>>::iterator itr;
    bool visited_arr[mapheight][mapwidth];
    for(int i=0; i<mapheight; i++)
    {
    	for(int j=0; j<mapwidth; j++)
    	{
    		visited_arr[i][j] = false;
         }
    }
    bool goal_state = false;
    parent[make_pair(source.x,source.y)] = make_pair(-1,-1);
	open.push_back(source);
	while(!open.empty())
	{
		node_bfs currnode = open.front();
		open.pop_front();
		int i = currnode.y;
		int j = currnode.x;

		if(i == destination.y && j == destination.x)
		{
			goal_state = true;
			break;
		}

		list <node_bfs> children;
		if(i == 0 && j == 0)							//Top Left Corner
		{
			node_bfs child_south = 		{j, i+1};
			node_bfs child_east = 		{j+1, i};
			node_bfs child_southeast = 	{j+1, i+1};
			children.push_back(child_south);
			children.push_back(child_east);
			children.push_back(child_southeast);
		}
		else if (i == 0 && j == mapwidth-1)			//Top Right Corner
		{
			node_bfs child_south = 		{j, i+1};
			node_bfs child_southwest = 	{j-1, i+1};
			node_bfs child_west = 		{j-1, i};
			children.push_back(child_south);
			children.push_back(child_southwest);
			children.push_back(child_west);
		}
		else if(i == mapheight-1 && j == mapwidth-1)	//Bottom Right Corner
		{
			node_bfs child_west = 		{j-1, i};
			node_bfs child_northwest = 	{j-1, i-1};
			node_bfs child_north = 		{j, i-1};
			children.push_back(child_west);
			children.push_back(child_northwest);
			children.push_back(child_north);
		}
		else if(i == mapheight-1 && j == 0)				//Bottom Left Corner
		{
			node_bfs child_north = 		{j, i-1};
			node_bfs child_northeast = 	{j+1, i-1};
			node_bfs child_east = 		{j+1, i};
			children.push_back(child_north);
			children.push_back(child_northeast);
			children.push_back(child_east);
		}
		else if(i == 0 && j>0 && j<mapwidth-1)			//Top Boundary
		{
			node_bfs child_south = 		{j, i+1};
			node_bfs child_southwest = 	{j-1, i+1};
			node_bfs child_west = 		{j-1, i};
			node_bfs child_east = 		{j+1, i};
			node_bfs child_southeast = 	{j+1, i+1};
			children.push_back(child_south);
			children.push_back(child_southwest);
			children.push_back(child_west);
			children.push_back(child_east);
			children.push_back(child_southeast);
		}
		else if(i == mapheight-1 && j>0 && j<mapwidth-1) //Bottom Boundary
		{
			node_bfs child_west = 		{j-1, i};
			node_bfs child_northwest = 	{j-1, i-1};
			node_bfs child_north = 		{j, i-1};
			node_bfs child_northeast = 	{j+1, i-1};
			node_bfs child_east = 		{j+1, i};
			children.push_back(child_west);
			children.push_back(child_northwest);
			children.push_back(child_north);
			children.push_back(child_northeast);
			children.push_back(child_east);
		}
		else if(j == 0 && i>0 && i<mapheight-1)			//Left Boundary
		{
			node_bfs child_south = 		{j, i+1};
			node_bfs child_north = 		{j, i-1};
			node_bfs child_northeast = 	{j+1, i-1};
			node_bfs child_east = 		{j+1, i};
			node_bfs child_southeast = 	{j+1, i+1};
			children.push_back(child_south);
			children.push_back(child_north);
			children.push_back(child_northeast);
			children.push_back(child_east);
			children.push_back(child_southeast);
		}
		else if(j == mapwidth-1 && i>0 && i<mapheight-1)  //Right Boundary
		{
			node_bfs child_south = 		{j, i+1};
			node_bfs child_southwest = 	{j-1, i+1};
			node_bfs child_west = 		{j-1, i};
			node_bfs child_northwest = 	{j-1, i-1};
			node_bfs child_north = 		{j, i-1};
			children.push_back(child_south);
			children.push_back(child_southwest);
			children.push_back(child_west);
			children.push_back(child_northwest);
			children.push_back(child_north);
		}
		else											//Center case
		{
			node_bfs child_south = 		{j, i+1};
			node_bfs child_southwest = 	{j-1, i+1};
			node_bfs child_west = 		{j-1, i};
			node_bfs child_northwest = 	{j-1, i-1};
			node_bfs child_north = 		{j, i-1};
			node_bfs child_northeast = 	{j+1, i-1};
			node_bfs child_east = 		{j+1, i};
			node_bfs child_southeast = 	{j+1, i+1};
			children.push_back(child_south);
			children.push_back(child_southwest);
			children.push_back(child_west);
			children.push_back(child_northwest);
			children.push_back(child_north);
			children.push_back(child_northeast);
			children.push_back(child_east);
			children.push_back(child_southeast);
		}
		while(!children.empty())
		{
			node_bfs bornchild = children.front();
			children.pop_front();
			if(abs(terrainz[i][j] - terrainz[bornchild.y][bornchild.x]) <= max_elevation && visited_arr[bornchild.y][bornchild.x] == false)
			{
				visited_arr[bornchild.y][bornchild.x] = true;
				open.push_back(bornchild);
				parent[make_pair(bornchild.x,bornchild.y)] = make_pair(currnode.x,currnode.y);
			}
		}
		visited_arr[i][j] = true;
		children.clear();
	} 														//End of empty queue loop
	pair <int,int> destpair (destination.x,destination.y);
	if(goal_state == true)
		printFullPath(parent, destpair, destpair);
	else
		writeoutput << "FAIL";
}

void UCS(vector<vector<int>> terrainz, node source, node destination)
{
	list <node> open;
	list <node> :: iterator ito;
    map<pair<int,int>, pair<int,int>> parent;
    map<pair<int,int>, pair<int,int>>::iterator itr;
    bool closed_arr[mapheight][mapwidth];
    for(int i=0; i<mapheight; i++)
    {
    	for(int j=0; j<mapwidth; j++)
    	{
    		closed_arr[i][j] = false;
         }
    }
    bool goal_state = false;
    parent[make_pair(source.x,source.y)] = make_pair(-1,-1);
	open.push_back(source);
	while(!open.empty())
	{
		node currnode = open.front();
		open.pop_front();
		int i = currnode.y;
		int j = currnode.x;
		long long int pc = currnode.pathcost;
		if(i == destination.y && j == destination.x)
		{
			goal_state = true;
			break;
		}
		list <node> children;
		if(i == 0 && j == 0)							//Top Left Corner
		{
			node child_south = {j, i+1, pc+10};
			node child_east = {j+1, i, pc+10};
			node child_southeast = {j+1, i+1, pc+14};
			children.push_back(child_south);
			children.push_back(child_east);
			children.push_back(child_southeast);
		}
		else if (i == 0 && j == mapwidth-1)			//Top Right Corner
		{
			node child_south = {j, i+1, pc+10};
			node child_southwest = {j-1, i+1, pc+14};
			node child_west = {j-1, i, pc+10};
			children.push_back(child_south);
			children.push_back(child_southwest);
			children.push_back(child_west);
		}
		else if(i == mapheight-1 && j == mapwidth-1)	//Bottom Right Corner
		{
			node child_west = {j-1, i, pc+10};
			node child_northwest = {j-1, i-1, pc+14};
			node child_north = {j, i-1, pc+10};
			children.push_back(child_west);
			children.push_back(child_northwest);
			children.push_back(child_north);
		}
		else if(i == mapheight-1 && j == 0)				//Bottom Left Corner
		{
			node child_north = {j, i-1, pc+10};
			node child_northeast = {j+1, i-1, pc+14};
			node child_east = {j+1, i, pc+10};
			children.push_back(child_north);
			children.push_back(child_northeast);
			children.push_back(child_east);
		}
		else if(i == 0 && j>0 && j<mapwidth-1)			//Top Boundary
		{
			node child_south = {j, i+1, pc+10};
			node child_southwest = {j-1, i+1, pc+14};
			node child_west = {j-1, i, pc+10};
			node child_east = {j+1, i, pc+10};
			node child_southeast = {j+1, i+1, pc+14};
			children.push_back(child_south);
			children.push_back(child_southwest);
			children.push_back(child_west);
			children.push_back(child_east);
			children.push_back(child_southeast);
		}
		else if(i == mapheight-1 && j>0 && j<mapwidth-1) //Bottom Boundary
		{
			node child_west = {j-1, i, pc+10};
			node child_northwest = {j-1, i-1, pc+14};
			node child_north = {j, i-1, pc+10};
			node child_northeast = {j+1, i-1, pc+14};
			node child_east = {j+1, i, pc+10};
			children.push_back(child_west);
			children.push_back(child_northwest);
			children.push_back(child_north);
			children.push_back(child_northeast);
			children.push_back(child_east);
		}
		else if(j == 0 && i>0 && i<mapheight-1)			//Left Boundary
		{
			node child_south = {j, i+1, pc+10};
			node child_north = {j, i-1, pc+10};
			node child_northeast = {j+1, i-1, pc+14};
			node child_east = {j+1, i, pc+10};
			node child_southeast = {j+1, i+1, pc+14};
			children.push_back(child_south);
			children.push_back(child_north);
			children.push_back(child_northeast);
			children.push_back(child_east);
			children.push_back(child_southeast);
		}
		else if(j == mapwidth-1 && i>0 && i<mapheight-1)  //Right Boundary
		{
			node child_south = {j, i+1, pc+10};
			node child_southwest = {j-1, i+1, pc+14};
			node child_west = {j-1, i, pc+10};
			node child_northwest = {j-1, i-1, pc+14};
			node child_north = {j, i-1, pc+10};
			children.push_back(child_south);
			children.push_back(child_southwest);
			children.push_back(child_west);
			children.push_back(child_northwest);
			children.push_back(child_north);
		}
		else											//Center case
		{
			node child_south = {j, i+1, pc+10};
			node child_southwest = {j-1, i+1, pc+14};
			node child_west = {j-1, i, pc+10};
			node child_northwest = {j-1, i-1, pc+14};
			node child_north = {j, i-1, pc+10};
			node child_northeast = {j+1, i-1, pc+14};
			node child_east = {j+1, i, pc+10};
			node child_southeast = {j+1, i+1, pc+14};
			children.push_back(child_south);
			children.push_back(child_southwest);
			children.push_back(child_west);
			children.push_back(child_northwest);
			children.push_back(child_north);
			children.push_back(child_northeast);
			children.push_back(child_east);
			children.push_back(child_southeast);
		}
		while(!children.empty())
		{
			node bornchild = children.front();
			children.pop_front();
			if(abs(terrainz[i][j] - terrainz[bornchild.y][bornchild.x]) <= max_elevation)
			{
				bool open_check = false;
				for(ito = open.begin(); ito != open.end(); ++ito)
				{
					node opennode = *ito;
					if(opennode.x == bornchild.x && opennode.y == bornchild.y)
					{
						open_check = true;
						if(bornchild.pathcost < opennode.pathcost)
						{
							open.erase(ito);
							open.push_back(bornchild);
							parent[make_pair(bornchild.x,bornchild.y)] = make_pair(currnode.x,currnode.y);
						}
						break;
					}
				}

				if(open_check == false && closed_arr[bornchild.y][bornchild.x] == false)
				{
					open.push_back(bornchild);
					parent[make_pair(bornchild.x,bornchild.y)] = make_pair(currnode.x,currnode.y);
				}
			}
		}
		closed_arr[i][j] = true;
		open.sort(list_compare);
		children.clear();
	} 														//End of empty queue loop
	pair <int,int> destpair (destination.x,destination.y);
	if(goal_state == true)
		printFullPath(parent, destpair, destpair);
	else
		writeoutput << "FAIL";
}

void AStar(vector<vector<int>> terrainz, node_as source, node_as destination)
{
	list <node_as> open;
	list <node_as> :: iterator ito;
	vector<node_as> closed;
    vector<node_as>::iterator it;
    map<pair<int,int>, pair<int,int>> parent;
    map<pair<int,int>, pair<int,int>>::iterator itr;
    bool closed_arr[mapheight][mapwidth];
    for(int i=0; i<mapheight; i++)
    {
    	for(int j=0; j<mapwidth; j++)
    	{
    		closed_arr[i][j] = false;
         }
    }
    bool goal_state = false;
    parent[make_pair(source.x,source.y)] = make_pair(-1,-1);
	open.push_back(source);
	while(!open.empty())
	{
		node_as currnode = open.front();
		open.pop_front();
		int i = currnode.y;
		int j = currnode.x;
		long long int pc = currnode.pathcost;

		if(i == destination.y && j == destination.x)
		{
			goal_state = true;
			break;
		}

		list <node_as> children;
		if(i == 0 && j == 0)							//Top Left Corner
		{
			double hn_south = 		calc_linedistance(j, i+1, destination.x, destination.y);
			int zdiff_south = 		abs(terrainz[i][j] - terrainz[i+1][j]);
			double hn_east = 		calc_linedistance(j+1, i, destination.x, destination.y);
			int zdiff_east =		abs(terrainz[i][j] - terrainz[i][j+1]);
			double hn_southeast = 	calc_linedistance(j+1, i+1, destination.x, destination.y);
			int zdiff_southeast = 	abs(terrainz[i][j] - terrainz[i+1][j+1]);

			node_as child_south = 		{j, i+1, pc+10+zdiff_south, hn_south};
			node_as child_east = 		{j+1, i, pc+10+zdiff_east, hn_east};
			node_as child_southeast = 	{j+1, i+1, pc+14+zdiff_southeast, hn_southeast};
			children.push_back(child_south);
			children.push_back(child_east);
			children.push_back(child_southeast);
		}
		else if (i == 0 && j == mapwidth-1)			//Top Right Corner
		{
			double hn_south = 		calc_linedistance(j, i+1, destination.x, destination.y);
			int zdiff_south = 		abs(terrainz[i][j] - terrainz[i+1][j]);
			double hn_southwest = 	calc_linedistance(j-1, i+1, destination.x, destination.y);
			int zdiff_southwest =	abs(terrainz[i][j] - terrainz[i+1][j-1]);
			double hn_west = 		calc_linedistance(j-1, i, destination.x, destination.y);
			int zdiff_west =		abs(terrainz[i][j] - terrainz[i][j-1]);

			node_as child_south = 		{j, i+1, pc+10+zdiff_south, hn_south};
			node_as child_southwest = 	{j-1, i+1, pc+14+zdiff_southwest, hn_southwest};
			node_as child_west = 		{j-1, i, pc+10+zdiff_west, hn_west};
			children.push_back(child_south);
			children.push_back(child_southwest);
			children.push_back(child_west);
		}
		else if(i == mapheight-1 && j == mapwidth-1)	//Bottom Right Corner
		{
			double hn_west = 		calc_linedistance(j-1, i, destination.x, destination.y);
			int zdiff_west =		abs(terrainz[i][j] - terrainz[i][j-1]);
			double hn_northwest = 	calc_linedistance(j-1, i-1, destination.x, destination.y);
			int zdiff_northwest =	abs(terrainz[i][j] - terrainz[i-1][j-1]);
			double hn_north = 		calc_linedistance(j, i-1, destination.x, destination.y);
			int zdiff_north = 		abs(terrainz[i][j] - terrainz[i-1][j]);

			node_as child_west = 		{j-1, i, pc+10+zdiff_west, hn_west};
			node_as child_northwest = 	{j-1, i-1, pc+14+zdiff_northwest, hn_northwest};
			node_as child_north = 		{j, i-1, pc+10+zdiff_north, hn_north};
			children.push_back(child_west);
			children.push_back(child_northwest);
			children.push_back(child_north);
		}
		else if(i == mapheight-1 && j == 0)				//Bottom Left Corner
		{
			double hn_north = 		calc_linedistance(j, i-1, destination.x, destination.y);
			int zdiff_north = 		abs(terrainz[i][j] - terrainz[i-1][j]);
			double hn_northeast = 	calc_linedistance(j+1, i-1, destination.x, destination.y);
			int zdiff_northeast = 	abs(terrainz[i][j] - terrainz[i-1][j+1]);
			double hn_east = 		calc_linedistance(j+1, i, destination.x, destination.y);
			int zdiff_east =		abs(terrainz[i][j] - terrainz[i][j+1]);

			node_as child_north = 		{j, i-1, pc+10+zdiff_north, hn_north};
			node_as child_northeast = 	{j+1, i-1, pc+14+zdiff_northeast, hn_northeast};
			node_as child_east = 		{j+1, i, pc+10+zdiff_east, hn_east};
			children.push_back(child_north);
			children.push_back(child_northeast);
			children.push_back(child_east);
		}
		else if(i == 0 && j>0 && j<mapwidth-1)			//Top Boundary
		{
			double hn_south = 		calc_linedistance(j, i+1, destination.x, destination.y);
			int zdiff_south = 		abs(terrainz[i][j] - terrainz[i+1][j]);
			double hn_southwest = 	calc_linedistance(j-1, i+1, destination.x, destination.y);
			int zdiff_southwest =	abs(terrainz[i][j] - terrainz[i+1][j-1]);
			double hn_west = 		calc_linedistance(j-1, i, destination.x, destination.y);
			int zdiff_west =		abs(terrainz[i][j] - terrainz[i][j-1]);
			double hn_east = 		calc_linedistance(j+1, i, destination.x, destination.y);
			int zdiff_east =		abs(terrainz[i][j] - terrainz[i][j+1]);
			double hn_southeast = 	calc_linedistance(j+1, i+1, destination.x, destination.y);
			int zdiff_southeast = 	abs(terrainz[i][j] - terrainz[i+1][j+1]);

			node_as child_south = 		{j, i+1, pc+10+zdiff_south, hn_south};
			node_as child_southwest = 	{j-1, i+1, pc+14+zdiff_southwest, hn_southwest};
			node_as child_west = 		{j-1, i, pc+10+zdiff_west, hn_west};
			node_as child_east = 		{j+1, i, pc+10+zdiff_east, hn_east};
			node_as child_southeast = 	{j+1, i+1, pc+14+zdiff_southeast, hn_southeast};
			children.push_back(child_south);
			children.push_back(child_southwest);
			children.push_back(child_west);
			children.push_back(child_east);
			children.push_back(child_southeast);
		}
		else if(i == mapheight-1 && j>0 && j<mapwidth-1) //Bottom Boundary
		{
			double hn_west = 		calc_linedistance(j-1, i, destination.x, destination.y);
			int zdiff_west =		abs(terrainz[i][j] - terrainz[i][j-1]);
			double hn_northwest = 	calc_linedistance(j-1, i-1, destination.x, destination.y);
			int zdiff_northwest =	abs(terrainz[i][j] - terrainz[i-1][j-1]);
			double hn_north = 		calc_linedistance(j, i-1, destination.x, destination.y);
			int zdiff_north = 		abs(terrainz[i][j] - terrainz[i-1][j]);
			double hn_northeast = 	calc_linedistance(j+1, i-1, destination.x, destination.y);
			int zdiff_northeast = 	abs(terrainz[i][j] - terrainz[i-1][j+1]);
			double hn_east = 		calc_linedistance(j+1, i, destination.x, destination.y);
			int zdiff_east =		abs(terrainz[i][j] - terrainz[i][j+1]);

			node_as child_west = 		{j-1, i, pc+10+zdiff_west, hn_west};
			node_as child_northwest = 	{j-1, i-1, pc+14+zdiff_northwest, hn_northwest};
			node_as child_north = 		{j, i-1, pc+10+zdiff_north, hn_north};
			node_as child_northeast = 	{j+1, i-1, pc+14+zdiff_northeast, hn_northeast};
			node_as child_east = 		{j+1, i, pc+10+zdiff_east, hn_east};
			children.push_back(child_west);
			children.push_back(child_northwest);
			children.push_back(child_north);
			children.push_back(child_northeast);
			children.push_back(child_east);
		}
		else if(j == 0 && i>0 && i<mapheight-1)			//Left Boundary
		{
			double hn_south = 		calc_linedistance(j, i+1, destination.x, destination.y);
			int zdiff_south = 		abs(terrainz[i][j] - terrainz[i+1][j]);
			double hn_north = 		calc_linedistance(j, i-1, destination.x, destination.y);
			int zdiff_north = 		abs(terrainz[i][j] - terrainz[i-1][j]);
			double hn_northeast = 	calc_linedistance(j+1, i-1, destination.x, destination.y);
			int zdiff_northeast = 	abs(terrainz[i][j] - terrainz[i-1][j+1]);
			double hn_east = 		calc_linedistance(j+1, i, destination.x, destination.y);
			int zdiff_east =		abs(terrainz[i][j] - terrainz[i][j+1]);
			double hn_southeast = 	calc_linedistance(j+1, i+1, destination.x, destination.y);
			int zdiff_southeast = 	abs(terrainz[i][j] - terrainz[i+1][j+1]);

			node_as child_south = 		{j, i+1, pc+10+zdiff_south, hn_south};
			node_as child_north = 		{j, i-1, pc+10+zdiff_north, hn_north};
			node_as child_northeast = 	{j+1, i-1, pc+14+zdiff_northeast, hn_northeast};
			node_as child_east = 		{j+1, i, pc+10+zdiff_east, hn_east};
			node_as child_southeast = 	{j+1, i+1, pc+14+zdiff_southeast, hn_southeast};
			children.push_back(child_south);
			children.push_back(child_north);
			children.push_back(child_northeast);
			children.push_back(child_east);
			children.push_back(child_southeast);
		}
		else if(j == mapwidth-1 && i>0 && i<mapheight-1)  //Right Boundary
		{
			double hn_south = 		calc_linedistance(j, i+1, destination.x, destination.y);
			int zdiff_south = 		abs(terrainz[i][j] - terrainz[i+1][j]);
			double hn_southwest = 	calc_linedistance(j-1, i+1, destination.x, destination.y);
			int zdiff_southwest =	abs(terrainz[i][j] - terrainz[i+1][j-1]);
			double hn_west = 		calc_linedistance(j-1, i, destination.x, destination.y);
			int zdiff_west =		abs(terrainz[i][j] - terrainz[i][j-1]);
			double hn_northwest = 	calc_linedistance(j-1, i-1, destination.x, destination.y);
			int zdiff_northwest =	abs(terrainz[i][j] - terrainz[i-1][j-1]);
			double hn_north = 		calc_linedistance(j, i-1, destination.x, destination.y);
			int zdiff_north = 		abs(terrainz[i][j] - terrainz[i-1][j]);

			node_as child_south = 		{j, i+1, pc+10+zdiff_south, hn_south};
			node_as child_southwest = 	{j-1, i+1, pc+14+zdiff_southwest, hn_southwest};
			node_as child_west = 		{j-1, i, pc+10+zdiff_west, hn_west};
			node_as child_northwest = 	{j-1, i-1, pc+14+zdiff_northwest, hn_northwest};
			node_as child_north = 		{j, i-1, pc+10+zdiff_north, hn_north};
			children.push_back(child_south);
			children.push_back(child_southwest);
			children.push_back(child_west);
			children.push_back(child_northwest);
			children.push_back(child_north);
		}
		else											//Center case
		{
			double hn_south = 		calc_linedistance(j, i+1, destination.x, destination.y);
			double hn_southwest = 	calc_linedistance(j-1, i+1, destination.x, destination.y);
			double hn_west = 		calc_linedistance(j-1, i, destination.x, destination.y);
			double hn_northwest = 	calc_linedistance(j-1, i-1, destination.x, destination.y);
			double hn_north = 		calc_linedistance(j, i-1, destination.x, destination.y);
			double hn_northeast = 	calc_linedistance(j+1, i-1, destination.x, destination.y);
			double hn_east = 		calc_linedistance(j+1, i, destination.x, destination.y);
			double hn_southeast = 	calc_linedistance(j+1, i+1, destination.x, destination.y);

			int zdiff_south = 		abs(terrainz[i][j] - terrainz[i+1][j]);
			int zdiff_southwest =	abs(terrainz[i][j] - terrainz[i+1][j-1]);
			int zdiff_west =		abs(terrainz[i][j] - terrainz[i][j-1]);
			int zdiff_northwest =	abs(terrainz[i][j] - terrainz[i-1][j-1]);
			int zdiff_north = 		abs(terrainz[i][j] - terrainz[i-1][j]);
			int zdiff_northeast = 	abs(terrainz[i][j] - terrainz[i-1][j+1]);
			int zdiff_east =		abs(terrainz[i][j] - terrainz[i][j+1]);
			int zdiff_southeast = 	abs(terrainz[i][j] - terrainz[i+1][j+1]);

			node_as child_south = 		{j, i+1, pc+10+zdiff_south, hn_south};
			node_as child_southwest = 	{j-1, i+1, pc+14+zdiff_southwest, hn_southwest};
			node_as child_west = 		{j-1, i, pc+10+zdiff_west, hn_west};
			node_as child_northwest = 	{j-1, i-1, pc+14+zdiff_northwest, hn_northwest};
			node_as child_north = 		{j, i-1, pc+10+zdiff_north, hn_north};
			node_as child_northeast = 	{j+1, i-1, pc+14+zdiff_northeast, hn_northeast};
			node_as child_east = 		{j+1, i, pc+10+zdiff_east, hn_east};
			node_as child_southeast = 	{j+1, i+1, pc+14+zdiff_southeast, hn_southeast};
			children.push_back(child_south);
			children.push_back(child_southwest);
			children.push_back(child_west);
			children.push_back(child_northwest);
			children.push_back(child_north);
			children.push_back(child_northeast);
			children.push_back(child_east);
			children.push_back(child_southeast);
		}
		while(!children.empty())
		{
			node_as bornchild = children.front();
			children.pop_front();
			if(abs(terrainz[i][j] - terrainz[bornchild.y][bornchild.x]) <= max_elevation)
			{
				double fn_born = (bornchild.pathcost + bornchild.heuristic);
				bool open_check = false;
				if(bornchild.x == destination.x && bornchild.y == destination.y)
				{
					goal_state = true;
					parent[make_pair(bornchild.x,bornchild.y)] = make_pair(currnode.x,currnode.y);
					goto found_dest;
				}
				for(ito = open.begin(); ito != open.end(); ++ito)
				{
					node_as opennode = *ito;
					if(opennode.x == bornchild.x && opennode.y == bornchild.y)
					{
						open_check = true;
						double fn_open = (opennode.pathcost + opennode.heuristic);
						if(fn_born < fn_open)
						{
							open.erase(ito);
							open.push_back(bornchild);
							parent[make_pair(bornchild.x,bornchild.y)] = make_pair(currnode.x,currnode.y);
						}
						break;
					}
				}
				if(closed_arr[bornchild.y][bornchild.x] == true)
				{
					for(auto it = closed.begin(); it != closed.end(); ++it)
					{
						node_as closednode = *it;
						if(closednode.x == bornchild.x && closednode.y == bornchild.y)
						{
							double fn_closed = (closednode.pathcost + closednode.heuristic);
							if(fn_born < fn_closed)
							{
								closed.erase(it);
								open.push_back(bornchild);
								parent[make_pair(bornchild.x,bornchild.y)] = make_pair(currnode.x,currnode.y);
							}
							break;
						}
					}
				}
				if(open_check == false && closed_arr[bornchild.y][bornchild.x] == false)
				{
					open.push_back(bornchild);
					parent[make_pair(bornchild.x,bornchild.y)] = make_pair(currnode.x,currnode.y);
				}
			}
		}
		closed.push_back(currnode);
		closed_arr[i][j] = true;
		open.sort(list_compare_as);
		children.clear();
	} 														//End of empty queue loop
	found_dest:
	pair <int,int> destpair (destination.x,destination.y);
	if(goal_state == true)
		printFullPath(parent, destpair, destpair);
	else
		writeoutput << "FAIL";
}

int main()
{
	string searchtype,line,str;
	int landx, landy, no_of_targets;
	vector<vector<int>> terrainz_vec;
	std::ifstream readinput("input.txt");
	readinput >> searchtype;
	readinput >> mapwidth >> mapheight;
	readinput >> landx >> landy;
	readinput >> max_elevation;
	readinput >> no_of_targets;
	int targets[no_of_targets][2];
	for(int i=0; i<no_of_targets; i++)
	{
		int tempa,tempb;
		readinput >> tempa >> tempb;
		targets[i][0] = tempa;
		targets[i][1] = tempb;
	}

	getline(readinput, line);
	for(int i=0; i<mapheight; i++)
	{
		getline(readinput, str);
		stringstream ss;
		ss << str;
		string temp;
		int found;
		vector<int> row;
		while (!ss.eof())
		{
			ss >> temp;
			if (stringstream(temp) >> found)
				row.push_back(found);
			temp = "";
	    }
		terrainz_vec.push_back(row);
	}
    if(searchtype == "BFS")
    {
        for(int i=0; i<no_of_targets; i++)
        {
            int targetx = targets[i][0];
            int targety = targets[i][1];
            node_bfs source = {landx, landy};
            node_bfs destination = {targetx, targety};
            BFS(terrainz_vec, source, destination);
            if(i != no_of_targets-1)
				writeoutput << endl;
        }
    }
    else if (searchtype == "UCS")
    {
        for(int i=0; i<no_of_targets; i++)
        {
            int targetx = targets[i][0];
            int targety = targets[i][1];
            node source = {landx, landy, 0};
            node destination = {targetx, targety, 0};
            UCS(terrainz_vec, source, destination);
            if(i != no_of_targets-1)
            	writeoutput << endl;
        }
    }
    else if (searchtype == "A*")
    {
        for(int i=0; i<no_of_targets; i++)
        {
            int targetx = targets[i][0];
            int targety = targets[i][1];
            node_as source = {landx, landy, 0, 0};
            node_as destination = {targetx, targety, 0, 0};
            AStar(terrainz_vec, source, destination);
            if(i != no_of_targets-1)
            	writeoutput << endl;
        }
    }
}
