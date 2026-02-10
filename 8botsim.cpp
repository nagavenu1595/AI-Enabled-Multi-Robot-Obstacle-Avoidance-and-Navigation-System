#include "raylib.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

const int GRID_SIZE = 5;
const int CELL = 100;

int grid[GRID_SIZE][GRID_SIZE] = {0}; // 0 free, 1 obstacle

struct Node { int x,y; float f,g,h; };

struct Robot {
    int id;
    std::vector<Vector2> path;
    int index = 0;

    Vector2 gridPos;
    Vector2 pixelPos;
    Vector2 targetPixel;

    float angle = 0;
    float targetAngle = 0;

    Vector2 goal; // store goal for replanning
    Color color;
};

// ------------------------------------------
int Heuristic(int x1,int y1,int x2,int y2){
    return abs(x1-x2)+abs(y1-y2);
}

// ------------------------------------------
// considerRobots == true -> treat other robots as blocked cells
bool IsCellBlocked(int x,int y,const std::vector<Robot>& robots,int selfID,bool considerRobots){
    if(x<0||y<0||x>=GRID_SIZE||y>=GRID_SIZE) return true;
    if(grid[y][x]==1) return true;

    if(considerRobots){
        for(const auto &r:robots){
            if(r.id!=selfID && (int)r.gridPos.x==x && (int)r.gridPos.y==y)
                return true;
        }
    }
    return false;
}

// ------------------------------------------
std::vector<Vector2> AStar(Vector2 start, Vector2 goal, const std::vector<Robot>& robots, bool considerRobots){
    // Special case: start == goal
    if((int)start.x == (int)goal.x && (int)start.y == (int)goal.y) return { start };

    std::vector<Node> open, closed;
    std::vector<std::vector<Vector2>> parent(GRID_SIZE,std::vector<Vector2>(GRID_SIZE,{ -1,-1 }));

    Node startNode={(int)start.x,(int)start.y,0,0,(float)Heuristic((int)start.x,(int)start.y,(int)goal.x,(int)goal.y)};
    startNode.f=startNode.h;
    open.push_back(startNode);

    auto inClosed=[&](int x,int y){ for(auto &n:closed) if(n.x==x&&n.y==y) return true; return false; };
    auto inOpen=[&](int x,int y){ for(auto &n:open) if(n.x==x&&n.y==y) return true; return false; };

    while(!open.empty()){
        std::sort(open.begin(),open.end(),[](Node&a,Node&b){ return a.f < b.f; });
        Node cur = open.front(); open.erase(open.begin()); closed.push_back(cur);

        if(cur.x == (int)goal.x && cur.y == (int)goal.y) break;

        int dirs[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
        for(auto &d : dirs){
            int nx = cur.x + d[0], ny = cur.y + d[1];
            if(IsCellBlocked(nx,ny,robots,-1,considerRobots) || inClosed(nx,ny)) continue;

            float g = cur.g + 1.0f;
            float h = (float)Heuristic(nx,ny,(int)goal.x,(int)goal.y);
            if(!inOpen(nx,ny)){
                open.push_back({nx,ny,g+h,g,h});
                parent[ny][nx] = { (float)cur.x, (float)cur.y };
            }
        }
    }

    // Reconstruct path; if parent is -1 then path not found
    std::vector<Vector2> path;
    Vector2 cur = goal;
    while(!((int)cur.x == (int)start.x && (int)cur.y == (int)start.y)){
        Vector2 p = parent[(int)cur.y][(int)cur.x];
        if((int)p.x == -1) { // fail: no parent -> return empty to indicate failure
            return {};
        }
        path.push_back(cur);
        cur = p;
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

// ------------------------------------------
void StepRobots(std::vector<Robot>& robots){
    for(auto &r:robots){
        if(r.index >= r.path.size()-1) continue;

        Vector2 next = r.path[r.index+1];

        // When moving, consider other robots as obstacles (prevent collisions)
        if(IsCellBlocked((int)next.x,(int)next.y,robots,r.id,true)){
            // Replan considering other robots
            std::vector<Vector2> newpath = AStar(r.gridPos, r.goal, robots, true);
            if(newpath.empty()){
                // can't replan right now; stay and try next tick
                // print a small message (console)
                std::cout << "Robot " << r.id << " replanning failed (blocked). Waiting.\n";
                continue;
            } else {
                r.path = newpath;
                r.index = 0;
                continue;
            }
        }

        Vector2 old = r.gridPos;
        r.index++;
        r.gridPos = next;
        r.targetPixel = { next.x * CELL + CELL / 2, next.y * CELL + CELL / 2 };

        Vector2 diff = { next.x - old.x, next.y - old.y };
        if((int)diff.x == 1) r.targetAngle = 0;
        if((int)diff.x == -1) r.targetAngle = 180;
        if((int)diff.y == 1) r.targetAngle = 90;
        if((int)diff.y == -1) r.targetAngle = 270;
    }
}

// ------------------------------------------
void DrawGrid(){
    for(int y=0;y<GRID_SIZE;y++)
        for(int x=0;x<GRID_SIZE;x++){
            if(grid[y][x]==1)
                DrawRectangle(x*CELL,y*CELL,CELL,CELL,DARKGRAY);
            DrawRectangleLines(x*CELL,y*CELL,CELL,CELL,GRAY);
        }
}

// ------------------------------------------
int main(){
    InitWindow(GRID_SIZE*CELL,GRID_SIZE*CELL,"Dynamic Multi-Robot Navigation");
    SetTargetFPS(60);

    std::cout << "Grid size: " << GRID_SIZE << "x" << GRID_SIZE << " (rows cols 0-" << GRID_SIZE-1 << ")\n";

    // Allow user to input static obstacles
    int obsCount = 0;
    std::cout << "Enter number of static obstacles: ";
    std::cin >> obsCount;
    for(int i=0;i<obsCount;i++){
        int orow, ocol;
        while(true){
            std::cout << "Obstacle " << i << " row col: ";
            std::cin >> orow >> ocol;
            if(orow>=0 && orow<GRID_SIZE && ocol>=0 && ocol<GRID_SIZE){
                grid[orow][ocol] = 1;
                break;
            } else {
                std::cout << "Invalid! Row/col must be 0-" << GRID_SIZE-1 << ".\n";
            }
        }
    }

    std::vector<Robot> robots;
    Color cols[8] = { RED, GREEN, BLUE, ORANGE, YELLOW, PURPLE, PINK, SKYBLUE };

    for(int i=0;i<8;i++){
        int sx,sy,gx,gy;

        while(true){
            std::cout << "Robot " << i << " start row col: ";
            std::cin >> sx >> sy;
            if(sx>=0 && sx<GRID_SIZE && sy>=0 && sy<GRID_SIZE && grid[sx][sy]==0) break;
            std::cout << "Invalid start or occupied by obstacle. Try again.\n";
        }

        while(true){
            std::cout << "Robot " << i << " goal row col: ";
            std::cin >> gx >> gy;
            if(gx>=0 && gx<GRID_SIZE && gy>=0 && gy<GRID_SIZE && grid[gx][gy]==0) break;
            std::cout << "Invalid goal or occupied by obstacle. Try again.\n";
        }

        Vector2 start = { (float)sy, (float)sx }; // col = x, row = y
        Vector2 goal  = { (float)gy, (float)gx };

        Robot r;
        r.id = i; r.color = cols[i];
        r.gridPos = start;
        r.pixelPos = { start.x*CELL + CELL/2, start.y*CELL + CELL/2 };
        r.targetPixel = r.pixelPos;
        r.goal = goal;

        // initial planning: ignore other robots so static obstacles determine reachability
        std::vector<Vector2> initialPath = AStar(start, goal, robots, false);
        if(initialPath.empty()){
            std::cout << "Warning: No path found for Robot " << i << " given current static obstacles. Robot will stay at start.\n";
            r.path = { start };
            r.index = 0;
        } else {
            r.path = initialPath;
            r.index = 0;
        }

        robots.push_back(r);
    }

    float timer = 0;
    const float STEP_INTERVAL = 0.5f;

    while(!WindowShouldClose()){
        float dt = GetFrameTime();
        timer += dt;
        if(timer >= STEP_INTERVAL){ StepRobots(robots); timer = 0; }

        for(auto &r:robots){
            r.pixelPos.x += (r.targetPixel.x - r.pixelPos.x) * 0.1f;
            r.pixelPos.y += (r.targetPixel.y - r.pixelPos.y) * 0.1f;
            r.angle += (r.targetAngle - r.angle) * 0.15f;
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);
        DrawGrid();

        for(auto &r:robots){
            Rectangle rect = { r.pixelPos.x - 20, r.pixelPos.y - 15, 40, 30 };
            DrawRectanglePro(rect, {20,15}, r.angle, r.color);
            std::string label = "R" + std::to_string(r.id);
            DrawText(label.c_str(), (int)r.pixelPos.x - 15, (int)r.pixelPos.y - 5, 18, BLACK);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
