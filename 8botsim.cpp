#include <bits/stdc++.h>
#include "raylib.h"
using namespace std;

const int ROWS = 6;
const int COLS = 6;
const int CELL = 80;
const int ROBOTS = 8;
const int MAX_TIME = 1000;

int grid[ROWS][COLS] = {
    {0,0,0,0,0,0},
    {0,0,0,0,0,0},
    {0,1,1,1,0,0},
    {0,0,0,0,0,0},
    {0,0,0,0,0,0},
    {0,0,0,0,0,0}
};

map<int, set<pair<int,int>>> vertexRes;
map<int, set<pair<pair<int,int>,pair<int,int>>>> edgeRes;

struct Node {
    int x,y,t,g,h;
    Node* parent;
};

struct Cmp {
    bool operator()(Node* a, Node* b) {
        return (a->g + a->h) > (b->g + b->h);
    }
};

bool valid(int x,int y){
    return x>=0 && y>=0 && x<ROWS && y<COLS && grid[x][y]==0;
}

int heuristic(int x1,int y1,int x2,int y2){
    return abs(x1-x2) + abs(y1-y2);
}

vector<pair<int,int>> reconstruct(Node* n){
    vector<pair<int,int>> p;
    while(n){
        p.push_back({n->x,n->y});
        n = n->parent;
    }
    reverse(p.begin(),p.end());
    return p;
}

vector<pair<int,int>> timeExpandedAStar(pair<int,int> s, pair<int,int> g, int startTime){
    priority_queue<Node*,vector<Node*>,Cmp> pq;
    set<tuple<int,int,int>> visited;

    pq.push(new Node{s.first,s.second,startTime,0,
        heuristic(s.first,s.second,g.first,g.second),nullptr});

    while(!pq.empty()){
        Node* cur = pq.top(); pq.pop();

        if(visited.count({cur->x,cur->y,cur->t})) continue;
        visited.insert({cur->x,cur->y,cur->t});

        if(cur->x==g.first && cur->y==g.second){
            return reconstruct(cur);
        }

        if(cur->t >= startTime + MAX_TIME) continue;

        vector<pair<int,int>> moves = {
            {0,0},{1,0},{-1,0},{0,1},{0,-1}
        };

        for(auto [dx,dy] : moves){
            int nx = cur->x + dx;
            int ny = cur->y + dy;
            int nt = cur->t + 1;

            if(!valid(nx,ny)) continue;

            if(vertexRes[nt].count({nx,ny})) continue;

            if(edgeRes[nt].count({{nx,ny},{cur->x,cur->y}})) continue;

            pq.push(new Node{
                nx,ny,nt,
                cur->g + 1,
                heuristic(nx,ny,g.first,g.second),
                cur
            });
        }
    }
    
    return {s};
}

void reservePath(const vector<pair<int,int>>& p, int timeOffset){
    for(int t=0;t<p.size();t++){
        int absTime = timeOffset + t;
        vertexRes[absTime].insert(p[t]);
        if(t>0){
            edgeRes[absTime].insert({p[t-1],p[t]});
            edgeRes[absTime].insert({p[t],p[t-1]});
        }
    }
}

int main(){
    int cycles;
    cin >> cycles;

    vector<pair<int,int>> start = {
        {0,1},{0,2},{0,3},{0,4},
        {1,0},{2,0},{3,0},{4,0}
    };

    vector<pair<int,int>> goal = {
        {5,4},{5,3},{5,2},{5,1},
        {4,5},{3,5},{2,5},{1,5}
    };

    vector<vector<pair<int,int>>> paths(ROBOTS);
    
    int globalTime = 0;

    for(int c=0;c<cycles;c++){
        bool forward = (c % 2 == 0);
        
        vector<vector<pair<int,int>>> cyclePaths(ROBOTS);
        int cycleMaxTime = 0;
        
        for(int r=0;r<ROBOTS;r++){
            auto s = forward ? start[r] : goal[r];
            auto g = forward ? goal[r]  : start[r];
            
            auto p = timeExpandedAStar(s, g, globalTime);
            cyclePaths[r] = p;
            
            reservePath(p, globalTime);
            
            cycleMaxTime = max(cycleMaxTime, (int)p.size());
        }
        
        for(int r=0;r<ROBOTS;r++){
            auto dest = forward ? goal[r] : start[r];
            
            int originalLength = cyclePaths[r].size();
            
            while(cyclePaths[r].size() < cycleMaxTime){
                cyclePaths[r].push_back(dest);
            }
            
            for(int t = originalLength; t < cycleMaxTime; t++){
                int absTime = globalTime + t;
                vertexRes[absTime].insert(dest);
            }
            
            paths[r].insert(paths[r].end(), cyclePaths[r].begin(), cyclePaths[r].end());
        }
        
        globalTime += cycleMaxTime;
    }

    InitWindow(COLS*CELL, ROWS*CELL, "Time-Expanded A* (Correct MAPF)");
    SetTargetFPS(5);

    vector<float> angle(ROBOTS,0);
    vector<float> smoothX(ROBOTS,0);
    vector<float> smoothY(ROBOTS,0);

    Color colors[ROBOTS] = {
        RED, BLUE, GREEN, ORANGE,
        PURPLE, BROWN, DARKGREEN, MAROON
    };

    for(int r=0;r<ROBOTS;r++){
        if(!paths[r].empty()){
            smoothX[r] = paths[r][0].second * CELL + CELL/2.0f;
            smoothY[r] = paths[r][0].first * CELL + CELL/2.0f;
        }
    }

    int step = 0;
    int maxSteps = 0;
    for(auto& p:paths) maxSteps = max(maxSteps,(int)p.size());

    float interpolation = 0.0f;
    const float interpSpeed = 0.25f;

    while(!WindowShouldClose()){
        interpolation += interpSpeed;
        if(interpolation >= 1.0f){
            interpolation = 0.0f;
            if(step < maxSteps-1) step++;
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);

        for(int i=0;i<ROWS;i++){
            for(int j=0;j<COLS;j++){
                if(grid[i][j])
                    DrawRectangle(j*CELL,i*CELL,CELL,CELL,DARKGRAY);
                DrawRectangleLines(j*CELL,i*CELL,CELL,CELL,LIGHTGRAY);
            }
        }

        for(int r=0;r<ROBOTS;r++){
            if(step >= paths[r].size()) continue;

            auto [x,y] = paths[r][step];
            float targetX = y*CELL + CELL/2.0f;
            float targetY = x*CELL + CELL/2.0f;

            if(step < paths[r].size()-1 && interpolation > 0){
                auto [nx,ny] = paths[r][step+1];
                float nextX = ny*CELL + CELL/2.0f;
                float nextY = nx*CELL + CELL/2.0f;
                
                smoothX[r] = targetX + (nextX - targetX) * interpolation;
                smoothY[r] = targetY + (nextY - targetY) * interpolation;
                
                if(nx > x) angle[r] = 90;
                else if(nx < x) angle[r] = 270;
                else if(ny > y) angle[r] = 0;
                else if(ny < y) angle[r] = 180;
            } else {
                smoothX[r] = targetX;
                smoothY[r] = targetY;
            }

            Rectangle body = {
                smoothX[r] - CELL*0.3f,
                smoothY[r] - CELL*0.2f,
                CELL*0.6f,
                CELL*0.4f
            };

            DrawRectanglePro(
                body,
                {body.width/2, body.height/2},
                angle[r],
                colors[r]
            );

            const char* label = TextFormat("R%d",r);
            int fontSize = 20;
            int textWidth = MeasureText(label, fontSize);
            float labelX = smoothX[r] - textWidth/2;
            float labelY = smoothY[r] - fontSize/2;
            
            for(int ox=-1; ox<=1; ox++){
                for(int oy=-1; oy<=1; oy++){
                    if(ox!=0 || oy!=0){
                        DrawText(label, labelX+ox, labelY+oy, fontSize, BLACK);
                    }
                }
            }
            
            DrawText(label, labelX, labelY, fontSize, WHITE);
        }

        DrawText(TextFormat("Step: %d/%d", step, maxSteps-1), 10, 10, 20, BLACK);
        
        int currentCycle = 0;
        for(int c=0; c<cycles; c++){
            if(step < (c+1) * (maxSteps/cycles)){
                currentCycle = c + 1;
                break;
            }
        }
        DrawText(TextFormat("Cycle: %d/%d", min(currentCycle, cycles), cycles), 10, 35, 20, BLACK);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
