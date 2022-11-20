#include <bits/stdc++.h>

using namespace std;


typedef pair <int, int> xy;
vector<vector<int>> grid
{
    {0,0,0,0},
    {0,1,1,0},
    {0,0,0,1}
};

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

vector<xy> astar(xy start, xy goal)
{
    vector<xy> path;
    priority_queue<waypoint, vector<waypoint>, CompareCost> minheap;
    set<xy> visited;

    waypoint s = {0, start};
    minheap.push(s);
    cout << minheap.top().location.first << ", " << minheap.top().location.second << endl;

    return path;
}


int main(int argc, const char** argv)
{

    xy start, goal;
    start.first = stoi(argv[1]);
    start.second = stoi(argv[2]);
    goal.first = stoi(argv[3]);
    goal.second = stoi(argv[4]);

    cout << start.first << ", " << start.second << endl;
    cout << goal.first << ", " << goal.second << endl;

    printf("Func debug\n");
    astar(start, goal);

    return 0;
}