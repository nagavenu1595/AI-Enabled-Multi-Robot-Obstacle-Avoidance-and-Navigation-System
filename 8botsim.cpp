#include <bits/stdc++.h>
#include "raylib.h"
using namespace std;

const int ROWS = 6;
const int COLS = 6;
const int CELL = 80;
const int ROBOTS = 8;
const int MAX_TIME = 1000;

map<int, set<pair<int,int>>> vertexRes;
map<int, set<pair<pair<int,int>, pair<int,int>>>> edgeRes;

int grid[ROWS][COLS];

struct Node {
    int x,y,t,g,h;
    Node* parent;
};

struct Cmp {
    bool operator()(Node* a, Node* b) {
        return (a->g + a->h) > (b->g + b->h);
    }
};

bool validCell(int x,int y){
    return x>=0 && y>=0 && x<ROWS && y<COLS && grid[x][y]==0;
}

int manhattan(int x1,int y1,int x2,int y2){
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

static bool goalCanSettle(pair<int,int> cell, int fromT){
    for(auto& [t, cells] : vertexRes){
        if(t >= fromT && cells.count(cell))
            return false;
    }
    return true;
}

vector<pair<int,int>> timeExpandedAStar(pair<int,int> s, pair<int,int> g){
    priority_queue<Node*,vector<Node*>,Cmp> pq;
    set<tuple<int,int,int>> visited;

    pq.push(new Node{s.first,s.second,0,0,
        manhattan(s.first,s.second,g.first,g.second),nullptr});

    while(!pq.empty()){
        Node* cur = pq.top(); pq.pop();

        if(visited.count({cur->x,cur->y,cur->t})) continue;
        visited.insert({cur->x,cur->y,cur->t});

        if(cur->x==g.first && cur->y==g.second){
            if(goalCanSettle({cur->x,cur->y}, cur->t))
                return reconstruct(cur);
        }

        if(cur->t >= MAX_TIME) continue;

        vector<pair<int,int>> moves = {
            {0,0},{1,0},{-1,0},{0,1},{0,-1}
        };

        for(auto [dx,dy] : moves){
            int nx = cur->x + dx;
            int ny = cur->y + dy;
            int nt = cur->t + 1;

            if(!validCell(nx,ny)) continue;
            if(vertexRes[nt].count({nx,ny})) continue;
            if(edgeRes[nt].count({{nx,ny},{cur->x,cur->y}})) continue;

            pq.push(new Node{
                nx,ny,nt,
                cur->g + 1,
                manhattan(nx,ny,g.first,g.second),
                cur
            });
        }
    }
    return {s};
}

void reservePath(const vector<pair<int,int>>& path){
    for(int t=0;t<(int)path.size();t++){
        vertexRes[t].insert(path[t]);
        if(t>0){
            edgeRes[t].insert({path[t-1],path[t]});
            edgeRes[t].insert({path[t],path[t-1]});
        }
    }
    auto goalPos = path.back();
    for(int t=(int)path.size();t<MAX_TIME;t++)
        vertexRes[t].insert(goalPos);
}

void replanAll(
    int fromStep,
    const vector<pair<int,int>>& goals,
    vector<vector<pair<int,int>>>& paths,
    int& maxLen
){
    vertexRes.clear();
    edgeRes.clear();

    vector<pair<int,int>> curPos(ROBOTS);
    for(int r=0;r<ROBOTS;r++){
        int clampedStep = min(fromStep, (int)paths[r].size()-1);
        curPos[r] = paths[r][clampedStep];
    }

    vector<vector<pair<int,int>>> newSegs(ROBOTS);
    for(int r=0;r<ROBOTS;r++){
        newSegs[r] = timeExpandedAStar(curPos[r], goals[r]);
        reservePath(newSegs[r]);
    }

    for(int r=0;r<ROBOTS;r++){
        vector<pair<int,int>> full;
        int histLen = min(fromStep+1, (int)paths[r].size());
        for(int t=0;t<histLen;t++)
            full.push_back(paths[r][t]);
        for(int t=1;t<(int)newSegs[r].size();t++)
            full.push_back(newSegs[r][t]);
        paths[r] = full;
    }

    maxLen = 0;
    for(auto &p: paths) maxLen = max(maxLen,(int)p.size());
    for(auto &p: paths)
        while((int)p.size()<maxLen) p.push_back(p.back());
}

int main(){
    srand(time(0));

    vector<pair<int,int>> start = {
        {0,1},{0,2},{0,3},{0,4},
        {1,0},{2,0},{3,0},{4,0}
    };

    vector<pair<int,int>> goal = {
        {5,4},{5,3},{5,2},{5,1},
        {4,5},{3,5},{2,5},{1,5}
    };

    auto isProtected = [&](int x,int y){
        for(auto &s: start) if(s.first==x && s.second==y) return true;
        for(auto &g: goal)  if(g.first==x && g.second==y) return true;
        return false;
    };

    for(int i=0;i<ROWS;i++)
        for(int j=0;j<COLS;j++)
            grid[i][j]=0;

    int mode,n;
    cin>>mode>>n;

    InitWindow(COLS*CELL, ROWS*CELL, "Time Expanded A*");
    SetTargetFPS(60);

    vector<pair<int,int>> obstacles;

    if(mode==1){
        int placed=0;
        while(placed<n){
            int x=rand()%ROWS, y=rand()%COLS;
            if(grid[x][y]==0 && !isProtected(x,y)){
                grid[x][y]=1;
                obstacles.push_back({x,y});
                placed++;
            }
        }

        vertexRes.clear(); edgeRes.clear();
        vector<vector<pair<int,int>>> paths(ROBOTS);
        for(int i=0;i<ROBOTS;i++){
            paths[i]=timeExpandedAStar(start[i],goal[i]);
            reservePath(paths[i]);
        }
        int maxLen=0;
        for(auto &p:paths) maxLen=max(maxLen,(int)p.size());
        for(auto &p:paths) while((int)p.size()<maxLen) p.push_back(p.back());

        const float TIME_PER_STEP=1.8f;
        double startDelay=5.0, startTime=GetTime();
        int step=0; double lastStep=0; bool started=false;
        bool finished=false;

        vector<float> angle(ROBOTS,0), posX(ROBOTS), posY(ROBOTS);
        for(int i=0;i<ROBOTS;i++){
            posX[i]=start[i].second*CELL+CELL/2;
            posY[i]=start[i].first*CELL+CELL/2;
        }
        Color colors[ROBOTS]={RED,BLUE,GREEN,ORANGE,PURPLE,BROWN,DARKGREEN,MAROON};

        while(!WindowShouldClose()){
            double now=GetTime();

            if(!started && now-startTime>=startDelay){
                started=true;
                lastStep=now;
            }

            if(started && !finished && now-lastStep>=TIME_PER_STEP){
                if(step<maxLen-1){
                    step++;
                    lastStep=now;
                } else {
                    finished=true;
                }
            }

            BeginDrawing(); ClearBackground(RAYWHITE);

            for(int i=0;i<ROWS;i++) for(int j=0;j<COLS;j++){
                if(grid[i][j]) DrawRectangle(j*CELL,i*CELL,CELL,CELL,DARKGRAY);
                DrawRectangleLines(j*CELL,i*CELL,CELL,CELL,LIGHTGRAY);
            }

            for(int r=0;r<ROBOTS;r++){
                pair<int,int> cur, nxt;

                if(!started){
                    cur = start[r];
                    nxt = start[r];
                } else {
                    cur = paths[r][step];
                    nxt = paths[r][min(step+1,maxLen-1)];
                }

                float cx=cur.second*CELL+CELL/2, cy=cur.first*CELL+CELL/2;
                float nx2=nxt.second*CELL+CELL/2, ny2=nxt.first*CELL+CELL/2;

                float alpha=0.0f;
                if(started && !finished)
                    alpha=min(1.0f,float((now-lastStep)/TIME_PER_STEP));

                posX[r]=cx+(nx2-cx)*alpha;
                posY[r]=cy+(ny2-cy)*alpha;

                if(nxt.first>cur.first) angle[r]=90;
                else if(nxt.first<cur.first) angle[r]=270;
                else if(nxt.second>cur.second) angle[r]=0;
                else if(nxt.second<cur.second) angle[r]=180;

                Rectangle body={posX[r]-CELL*0.3f,posY[r]-CELL*0.2f,CELL*0.6f,CELL*0.4f};
                DrawRectanglePro(body,{body.width/2,body.height/2},angle[r],colors[r]);

                string label="R"+to_string(r); int fs=20;
                DrawText(label.c_str(),(int)(posX[r]-MeasureText(label.c_str(),fs)/2.0f),
                         (int)(posY[r]-fs/2.0f),fs,BLACK);
            }

            if(!started){
                int rem=max(0,(int)(startDelay-(now-startTime))+1);
                DrawText(("Starting in "+to_string(rem)+"s...").c_str(),10,10,20,BLACK);
            } else if(finished){
                DrawText("All robots reached their goals!",10,10,20,DARKGREEN);
            }

            EndDrawing();
        }
        CloseWindow(); return 0;
    }

    if(mode==2){

        const float  TIME_PER_STEP  = 1.8f;
        const double START_DELAY    = 5.0;
        const double OBS_INTERVAL   = 5.0;
        const double RELOC_INTERVAL = 8.0;

        double wallStart       = GetTime();

        int    obsPlaced       = 0;
        double lastObsTime     = wallStart;
        bool   allObsPlaced    = false;

        bool   simStarted      = false;
        double lastStepTime    = 0.0;
        int    step            = 0;
        int    maxLen          = 1;
        bool   pathsReady      = false;

        double relocTriggerTime= 0.0;
        int    relocCount      = 0;

        vector<vector<pair<int,int>>> paths(ROBOTS);
        Color colors[ROBOTS]={RED,BLUE,GREEN,ORANGE,PURPLE,BROWN,DARKGREEN,MAROON};
        vector<float> angle(ROBOTS,0), posX(ROBOTS), posY(ROBOTS);
        for(int i=0;i<ROBOTS;i++){
            posX[i]=start[i].second*CELL+CELL/2;
            posY[i]=start[i].first*CELL+CELL/2;
        }

        auto placeOneObs = [&]() -> bool {
            for(int att=0;att<1000;att++){
                int x=rand()%ROWS, y=rand()%COLS;
                if(grid[x][y]==0 && !isProtected(x,y)){
                    grid[x][y]=1;
                    obstacles.push_back({x,y});
                    obsPlaced++;
                    return true;
                }
            }
            return false;
        };

        auto planFromStart = [&](){
            vertexRes.clear(); edgeRes.clear();
            for(int i=0;i<ROBOTS;i++){
                paths[i]=timeExpandedAStar(start[i],goal[i]);
                reservePath(paths[i]);
            }
            maxLen=0;
            for(auto &p:paths) maxLen=max(maxLen,(int)p.size());
            for(auto &p:paths) while((int)p.size()<maxLen) p.push_back(p.back());
            pathsReady=true;
        };

        while(!WindowShouldClose()){
            double now = GetTime();

            if(!allObsPlaced && now - lastObsTime >= OBS_INTERVAL){
                if(placeOneObs()){
                    lastObsTime = now;
                    if(simStarted && pathsReady)
                        replanAll(step, goal, paths, maxLen);
                }
                if(obsPlaced >= n){
                    allObsPlaced     = true;
                    relocTriggerTime = now;
                }
            }

            if(!simStarted && now - wallStart >= START_DELAY){
                simStarted   = true;
                lastStepTime = now;
                planFromStart();
            }

            if(simStarted && pathsReady && now - lastStepTime >= TIME_PER_STEP){
                if(step < maxLen-1) step++;
                lastStepTime = now;
            }

            if(allObsPlaced && simStarted && pathsReady
               && !obstacles.empty()
               && now - relocTriggerTime >= RELOC_INTERVAL)
            {
                int idx = relocCount % (int)obstacles.size();
                auto oldPos = obstacles[idx];

                grid[oldPos.first][oldPos.second] = 0;

                bool moved = false;
                for(int att=0; att<1000; att++){
                    int nx2=rand()%ROWS, ny2=rand()%COLS;
                    if(grid[nx2][ny2]==0 && !isProtected(nx2,ny2)){
                        grid[nx2][ny2] = 1;
                        obstacles[idx] = {nx2, ny2};
                        moved = true;
                        break;
                    }
                }
                if(!moved)
                    grid[oldPos.first][oldPos.second] = 1;

                replanAll(step, goal, paths, maxLen);

                relocCount++;
                relocTriggerTime = now;
            }

            BeginDrawing();
            ClearBackground(RAYWHITE);

            for(int i=0;i<ROWS;i++) for(int j=0;j<COLS;j++){
                if(grid[i][j]) DrawRectangle(j*CELL,i*CELL,CELL,CELL,DARKGRAY);
                DrawRectangleLines(j*CELL,i*CELL,CELL,CELL,LIGHTGRAY);
            }

            for(int r=0;r<ROBOTS;r++){
                int s2 = (simStarted && pathsReady) ? step : 0;
                int ml = (simStarted && pathsReady) ? maxLen : 1;

                auto cur = (simStarted && pathsReady) ? paths[r][s2]             : start[r];
                auto nxt = (simStarted && pathsReady) ? paths[r][min(s2+1,ml-1)] : start[r];

                float cx=cur.second*CELL+CELL/2, cy=cur.first*CELL+CELL/2;
                float nx2=nxt.second*CELL+CELL/2, ny2=nxt.first*CELL+CELL/2;

                float alpha = 0.0f;
                if(simStarted && pathsReady)
                    alpha = min(1.0f, float((now-lastStepTime)/TIME_PER_STEP));

                posX[r]=cx+(nx2-cx)*alpha;
                posY[r]=cy+(ny2-cy)*alpha;

                if(nxt.first>cur.first) angle[r]=90;
                else if(nxt.first<cur.first) angle[r]=270;
                else if(nxt.second>cur.second) angle[r]=0;
                else if(nxt.second<cur.second) angle[r]=180;

                Rectangle body={posX[r]-CELL*0.3f,posY[r]-CELL*0.2f,CELL*0.6f,CELL*0.4f};
                DrawRectanglePro(body,{body.width/2,body.height/2},angle[r],colors[r]);

                string label="R"+to_string(r); int fs=20;
                DrawText(label.c_str(),
                    (int)(posX[r]-MeasureText(label.c_str(),fs)/2.0f),
                    (int)(posY[r]-fs/2.0f), fs, BLACK);
            }

            if(!simStarted){
                int rem=max(0,(int)(START_DELAY-(now-wallStart))+1);
                string msg="Robots start in "+to_string(rem)
                           +"s  |  Obstacles: "+to_string(obsPlaced)+"/"+to_string(n);
                DrawText(msg.c_str(),6,6,18,BLACK);
            } else {
                string msg = allObsPlaced
                    ? ("Obstacles: "+to_string(obsPlaced)+"/"+to_string(n)
                       +"  |  Relocations: "+to_string(relocCount))
                    : ("Placing obstacles: "+to_string(obsPlaced)+"/"+to_string(n));
                DrawText(msg.c_str(),6,6,18,BLACK);
            }

            EndDrawing();
        }

        CloseWindow();
        return 0;
    }

    CloseWindow();
    return 0;
}
