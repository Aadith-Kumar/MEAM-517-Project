#include <bits/stdc++.h>
#include <vector>

using namespace std;


// TODO: Add ROS
// TODO: ROS waypoint message should contain x,y,theta

typedef pair <int, int> xy;
vector<vector<int>> grid
{
    {0,0,0,0,1,0,0,0},
    {0,1,1,1,1,0,0,0},
    {0,0,0,1,1,0,0,0},
    {0,0,0,0,1,0,0,0},
    {0,0,0,0,1,0,0,0},
    {0,1,1,0,1,0,0,0},
    {0,0,0,1,0,0,0,0},
    {0,0,0,0,0,0,0,0}
}; 
// 0 - Free
// 1 - Occupied

struct waypoint{
    float cost;
    xy location;
    xy parent;
};

struct CompareCost {
    bool operator()(waypoint const& w1, waypoint const& w2)
    {
        return w1.cost > w2.cost;
    }
};


vector<xy> get_neighbors(int x, int y)
{
    vector<xy> neighbors;
    if(x!=0){
        neighbors.push_back(make_pair(x-1, y)); // West
        if(y!=0)
            neighbors.push_back(make_pair(x-1, y-1)); // North west
        if(y!=grid[0].size()-1)
            neighbors.push_back(make_pair(x-1, y+1)); // South west
    }
    if(x!=grid.size()-1){
        neighbors.push_back(make_pair(x+1, y)); // East
        if(y!=0)
            neighbors.push_back(make_pair(x+1, y-1)); // North east
        if(y!=grid[0].size()-1)
            neighbors.push_back(make_pair(x+1, y+1)); // South east
    }
    if(y!=0)
        neighbors.push_back(make_pair(x, y-1)); // North
    if(y!=grid[0].size()-1)
        neighbors.push_back(make_pair(x, y+1)); // South
        
    return neighbors;
}

float get_distance(int x1, int y1, int x2, int y2)
{
    return sqrt(
        pow(x1-x2, 2) + pow(y1-y2, 2)
    );
}

vector<xy> astar(xy start, xy goal, vector<vector<int>> grid)
{
    vector<xy> path;
    priority_queue<waypoint, vector<waypoint>, CompareCost> minheap;
    set<xy> visited;
    vector<vector<float>> dist(grid.size(), vector<float>(grid[0].size(), INFINITY));
    vector<vector<xy>> parent(grid.size(), vector<xy>(grid[0].size()));

    waypoint s = {0, start, make_pair(-1,-1)};
    dist[start.first][start.second] = 0;
    minheap.push(s);
    parent[start.first][start.second] = make_pair(-1, -1);
    
    while(!minheap.empty()){
        waypoint top = minheap.top();
        // cout << top.location.first << ", " << top.location.second << endl;
        minheap.pop();
        // cout << visited.count(top.location) << endl;
        if(visited.count(top.location)==0){
            visited.insert(top.location);
            parent[top.location.first][top.location.second] = top.parent;
            if(top.location == goal){
                // GOAL condition
                xy path_next = top.location;
                while(path_next != make_pair(-1,-1)){
                    path.push_back(path_next);
                    path_next = parent[path_next.first][path_next.second];
                }
                reverse(path.begin(), path.end());
                
                return path;
            }


            for(auto n : get_neighbors(top.location.first, top.location.second)){
                if(grid[n.first][n.second]==0){
                    float f = dist[n.first][n.second];
                    float g = top.cost + 
                            get_distance(top.location.first, top.location.second, n.first, n.second) +
                            get_distance(goal.first, goal.second, n.first, n.second);
                    
                    if(g<f){
                        dist[top.location.first][top.location.second] = g;
                        waypoint neighbor = {g, n, top.location};
                        minheap.push(neighbor);
                    }
                }
            }
        }
    }

}


int main(int argc, const char** argv)
{

    xy start, goal;
    start.first = stoi(argv[1]);
    start.second = stoi(argv[2]);
    goal.first = stoi(argv[3]);
    goal.second = stoi(argv[4]);

    vector<xy> path = astar(start, goal, grid);
    cout << "PATH" << endl;
    for(auto p : path)
        cout << p.first << ", " << p.second << endl;
    cout << endl;

    return 0;
}