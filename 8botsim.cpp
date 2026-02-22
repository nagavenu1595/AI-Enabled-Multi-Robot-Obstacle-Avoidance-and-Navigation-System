#include <bits/stdc++.h>
#include "raylib.h"
using namespace std;

const int ROWS = 6;
const int COLS = 6;
const int CELL = 80;
const int ROBOTS = 8;
const int MAX_TIME = 250;
const int LOOKAHEAD = 3;

const int OVERSHOOT_PENALTY   = 4;
const int GOAL_HOLD_STEPS     = 15;
const int BACKTRACK_PENALTY   = 8;
const int REVISIT_WINDOW      = 8;
const int MAX_REPLAN_ATTEMPTS = 3;
const int YIELD_WAIT_STEPS    = 4;

map<int, set<pair<int,int>>>                         vertexRes;
map<int, set<pair<pair<int,int>,pair<int,int>>>>     edgeRes;
int grid[ROWS][COLS];

const char* ROBOT_NAMES[ROBOTS] = {
    "R0(RED)","R1(BLUE)","R2(GREEN)","R3(ORANGE)",
    "R4(PURPLE)","R5(BROWN)","R6(DKGRN)","R7(MAROON)"
};

struct Node { int x,y,t,g,h; Node* parent; };
struct Cmp {
    bool operator()(const Node* a,const Node* b) const {
        return (a->g+a->h)>(b->g+b->h);
    }
};

bool validCell(int x,int y){
    return x>=0&&y>=0&&x<ROWS&&y<COLS&&grid[x][y]==0;
}

int heuristic(int nx,int ny,int gx,int gy){
    int base=abs(nx-gx)+abs(ny-gy);
    int pen=0;
    if(nx>gx) pen+=OVERSHOOT_PENALTY*(nx-gx);
    if(nx<gx) pen+=OVERSHOOT_PENALTY*(gx-nx)/2;
    if(ny>gy) pen+=OVERSHOOT_PENALTY*(ny-gy);
    if(ny<gy) pen+=OVERSHOOT_PENALTY*(gy-ny)/2;
    return base+pen;
}

vector<pair<int,int>> reconstruct(Node* n){
    vector<pair<int,int>> p;
    while(n){p.push_back({n->x,n->y});n=n->parent;}
    reverse(p.begin(),p.end());
    return p;
}

bool recentlyVisited(Node* cur,int nx,int ny){
    Node* p=cur;
    for(int i=0;i<REVISIT_WINDOW&&p;i++,p=p->parent)
        if(p->x==nx&&p->y==ny) return true;
    return false;
}

map<pair<int,int>, int> cellLastReserved;

void rebuildCellLastReserved(){
    cellLastReserved.clear();
    for(auto&[t,cells]:vertexRes)
        for(auto& cell:cells){
            auto it=cellLastReserved.find(cell);
            if(it==cellLastReserved.end()||t>it->second)
                cellLastReserved[cell]=t;
        }
}

static bool goalCanSettle(pair<int,int> cell,int fromT){
    auto it=cellLastReserved.find(cell);
    if(it==cellLastReserved.end()) return true;
    return it->second < fromT;
}

string dirLabel(pair<int,int> f,pair<int,int> t){
    int dr=t.first-f.first,dc=t.second-f.second;
    if(dr==0&&dc==0)  return "WAIT ";
    if(dr==1&&dc==0)  return "DOWN ";
    if(dr==-1&&dc==0) return "UP   ";
    if(dr==0&&dc==1)  return "RIGHT";
    if(dr==0&&dc==-1) return "LEFT ";
    return "DIAG_ERR";
}

bool strictMoveOK(pair<int,int> from,pair<int,int> to){
    return (abs(to.first-from.first)+abs(to.second-from.second))<=1;
}

struct CollisionReport {
    bool hasCollision = false;
    vector<int> affectedRobots;
    int timeStep = -1;
    string description;
};

CollisionReport detectCollisions(const vector<vector<pair<int,int>>>& paths,
                                  int fromStep=0, bool verbose=false) {
    CollisionReport report;
    int maxLen = 0;
    for(auto& p : paths) maxLen = max(maxLen, (int)p.size());

    for(int t = fromStep; t < maxLen; t++) {
        map<pair<int,int>, vector<int>> cellOccupancy;
        for(int r = 0; r < ROBOTS; r++) {
            int idx = min(t, (int)paths[r].size()-1);
            cellOccupancy[paths[r][idx]].push_back(r);
        }
        for(auto& [pos, robots] : cellOccupancy) {
            if(robots.size() > 1) {
                report.hasCollision = true;
                report.timeStep = t;
                report.description = "VERTEX @("+to_string(pos.first)+","+to_string(pos.second)+") t="+to_string(t);
                report.affectedRobots = robots;
                if(verbose) printf(" !! Collision: %s\n", report.description.c_str());
                return report;
            }
        }
        if(t > fromStep) {
            for(int r1 = 0; r1 < ROBOTS; r1++) {
                auto p1p = paths[r1][min(t-1,(int)paths[r1].size()-1)];
                auto p1c = paths[r1][min(t,  (int)paths[r1].size()-1)];
                for(int r2 = r1+1; r2 < ROBOTS; r2++) {
                    auto p2p = paths[r2][min(t-1,(int)paths[r2].size()-1)];
                    auto p2c = paths[r2][min(t,  (int)paths[r2].size()-1)];
                    if(p1p==p2c && p1c==p2p && p1p!=p1c) {
                        report.hasCollision = true;
                        report.timeStep = t;
                        report.affectedRobots = {r1, r2};
                        report.description = "EDGE swap t="+to_string(t)+
                            " "+ROBOT_NAMES[r1]+"<->"+ROBOT_NAMES[r2];
                        if(verbose) printf(" !! Collision: %s\n", report.description.c_str());
                        return report;
                    }
                }
            }
        }
    }
    return report;
}

bool validatePath(int r,const vector<pair<int,int>>& path,int fromStep=0, bool verbose=true){
    bool bad=false;
    for(int t=max(1,fromStep);t<(int)path.size();t++){
        int dr=abs(path[t].first-path[t-1].first);
        int dc=abs(path[t].second-path[t-1].second);
        if(dr+dc>1){
            if(verbose)
                printf(" !! ILLEGAL MOVE %s step%d->%d: (%d,%d)->(%d,%d)\n",
                       ROBOT_NAMES[r],t-1,t,
                       path[t-1].first,path[t-1].second,
                       path[t].first,path[t].second);
            bad=true;
        }
    }
    if(!bad && verbose) printf(" %s OK (%d steps)\n",ROBOT_NAMES[r],(int)path.size());
    return !bad;
}

void printSingleStep(int step,
                     const vector<vector<pair<int,int>>>& paths,
                     const vector<pair<int,int>>& goals)
{
    printf("\n[STEP %d]\n",step);
    for(int r=0;r<ROBOTS;r++){
        int ct=min(step,(int)paths[r].size()-1);
        int nt=min(step+1,(int)paths[r].size()-1);
        auto cur=paths[r][ct], nxt=paths[r][nt];
        bool atGoal     =(cur==goals[r]);
        bool illegalMove=!strictMoveOK(cur,nxt);
        bool trulyStuck =((int)paths[r].size()<=1&&!atGoal);
        bool plannedWait=(cur==nxt&&!atGoal&&!trulyStuck);
        printf(" %-14s (%d,%d)->(%d,%d) [%s]%s%s%s%s\n",
               ROBOT_NAMES[r],
               cur.first,cur.second,nxt.first,nxt.second,
               dirLabel(cur,nxt).c_str(),
               atGoal      ?" GOAL!":"",
               trulyStuck  ?" STUCK":"",
               plannedWait ?" wait":"",
               illegalMove ?" ILLEGAL!":"");
    }
}

vector<pair<int,int>> timeExpandedAStar(pair<int,int> s,pair<int,int> g,
                                         bool startOnObs=false)
{
    vector<Node*> pool;
    auto makeNode=[&](int x,int y,int t,int gc,int h,Node* par)->Node*{
        Node* n=new Node{x,y,t,gc,h,par};
        pool.push_back(n);
        return n;
    };

    priority_queue<Node*,vector<Node*>,Cmp> pq;
    map<tuple<int,int,int>,bool> visited;

    pq.push(makeNode(s.first,s.second,0,0,
                     heuristic(s.first,s.second,g.first,g.second),nullptr));

    vector<pair<int,int>> result;

    while(!pq.empty()){
        Node* cur=pq.top(); pq.pop();
        auto key=make_tuple(cur->x,cur->y,cur->t);
        if(visited.count(key)) continue;
        visited[key]=true;

        if(cur->x==g.first&&cur->y==g.second&&
           goalCanSettle({cur->x,cur->y},cur->t)){
            result=reconstruct(cur);
            break;
        }
        if(cur->t>=MAX_TIME) continue;

        for(auto[dx,dy]:vector<pair<int,int>>{{0,0},{1,0},{-1,0},{0,1},{0,-1}}){
            int nx=cur->x+dx, ny=cur->y+dy, nt=cur->t+1;
            bool waitOnBlockedStart=(dx==0&&dy==0&&cur->t==0
                                     &&startOnObs&&nx==s.first&&ny==s.second);
            if(!waitOnBlockedStart&&!validCell(nx,ny)) continue;
            if(vertexRes[nt].count({nx,ny})) continue;
            if(edgeRes[nt].count({{nx,ny},{cur->x,cur->y}})) continue;

            bool revisit=(dx!=0||dy!=0)&&recentlyVisited(cur,nx,ny);
            int  extra_g=revisit?BACKTRACK_PENALTY:0;
            pq.push(makeNode(nx,ny,nt,
                             cur->g+1+extra_g,
                             heuristic(nx,ny,g.first,g.second),
                             cur));
        }
    }

    for(Node* n:pool) delete n;
    if(result.empty()) result={s};

    for(int i=1;i<(int)result.size();i++){
        int dr=abs(result[i].first-result[i-1].first);
        int dc=abs(result[i].second-result[i-1].second);
        if(dr+dc>1){
            printf(" !! [A* SAFETY] Illegal move, truncating\n");
            result.resize(i); break;
        }
    }
    return result;
}

void reservePath(const vector<pair<int,int>>& path){
    for(int t=0;t<(int)path.size();t++){
        vertexRes[t].insert(path[t]);
        if(t>0){
            edgeRes[t].insert({path[t-1],path[t]});
            edgeRes[t].insert({path[t],path[t-1]});
        }
    }
    auto gp=path.back();
    int holdUntil=min((int)path.size()+GOAL_HOLD_STEPS,MAX_TIME);
    for(int t=(int)path.size();t<holdUntil;t++)
        vertexRes[t].insert(gp);
    rebuildCellLastReserved();
}

void erasePathReservations(const vector<pair<int,int>>& path){
    for(int t=0;t<(int)path.size();t++){
        vertexRes[t].erase(path[t]);
        if(t>0){
            edgeRes[t].erase({path[t-1],path[t]});
            edgeRes[t].erase({path[t],path[t-1]});
        }
    }
    auto gp=path.back();
    int holdUntil=min((int)path.size()+GOAL_HOLD_STEPS,MAX_TIME);
    for(int t=(int)path.size();t<holdUntil;t++)
        vertexRes[t].erase(gp);
    rebuildCellLastReserved();
}

vector<int> planningOrder(const vector<pair<int,int>>& pos,
                          const vector<pair<int,int>>& goals,
                          const set<int>& priorityOverride={})
{
    vector<int> overrideList(priorityOverride.begin(), priorityOverride.end());
    vector<tuple<int,int,int>> di;
    for(int r=0;r<ROBOTS;r++){
        if(priorityOverride.count(r)) continue;
        int dist=abs(pos[r].first-goals[r].first)
                +abs(pos[r].second-goals[r].second);
        int grp=(r>=4)?0:1;
        di.push_back({dist,grp,r});
    }
    sort(di.begin(),di.end(),[](auto&a,auto&b){
        if(get<0>(a)!=get<0>(b)) return get<0>(a)>get<0>(b);
        return get<1>(a)<get<1>(b);
    });
    vector<int> o=overrideList;
    for(auto&[d,g,r]:di) o.push_back(r);
    return o;
}

bool allAtGoals(int step,const vector<vector<pair<int,int>>>& paths,
                const vector<pair<int,int>>& goals)
{
    for(int r=0;r<ROBOTS;r++){
        int t=min(step,(int)paths[r].size()-1);
        if(paths[r][t]!=goals[r]) return false;
    }
    return true;
}

void resolveYield(int fromStep,
                  const vector<pair<int,int>>& goals,
                  const vector<pair<int,int>>& curPos,
                  vector<vector<pair<int,int>>>& newSegs,
                  bool verbose)
{
    for(int stuckR=0; stuckR<ROBOTS; stuckR++){
        if(curPos[stuckR]==goals[stuckR]) continue;
        if((int)newSegs[stuckR].size() > 1)  continue;

        if(grid[curPos[stuckR].first][curPos[stuckR].second]==1){
            if(verbose)
                printf(" [YIELD] %s is ON an obstacle at (%d,%d) — waiting for reloc\n",
                       ROBOT_NAMES[stuckR],curPos[stuckR].first,curPos[stuckR].second);
            newSegs[stuckR]={curPos[stuckR]};
            for(int i=0;i<YIELD_WAIT_STEPS+2;i++)
                newSegs[stuckR].push_back(curPos[stuckR]);
            reservePath(newSegs[stuckR]);
            continue;
        }

        if(verbose)
            printf(" [YIELD] %s stuck at (%d,%d) — scanning for goal-blocker...\n",
                   ROBOT_NAMES[stuckR],curPos[stuckR].first,curPos[stuckR].second);

        bool resolved=false;

        for(int blocker=0; blocker<ROBOTS && !resolved; blocker++){
            if(blocker==stuckR) continue;
            if(curPos[blocker]!=goals[blocker]) continue;

            erasePathReservations(newSegs[blocker]);

            auto testPath=timeExpandedAStar(curPos[stuckR],goals[stuckR],false);

            if((int)testPath.size()<=1){
                reservePath(newSegs[blocker]);
                continue;
            }

            pair<int,int> yieldCell={-1,-1};
            for(auto[dx,dy]:vector<pair<int,int>>{{1,0},{-1,0},{0,1},{0,-1}}){
                int nx=goals[blocker].first+dx, ny=goals[blocker].second+dy;
                if(!validCell(nx,ny)) continue;
                if(vertexRes[1].count({nx,ny})) continue;
                yieldCell={nx,ny};
                break;
            }

            if(yieldCell.first==-1){
                if(verbose)
                    printf(" [YIELD]  %s has no free neighbour\n",ROBOT_NAMES[blocker]);
                reservePath(newSegs[blocker]);
                continue;
            }

            if(verbose)
                printf(" [YIELD]  %s steps to (%d,%d) to let %s through\n",
                       ROBOT_NAMES[blocker],yieldCell.first,yieldCell.second,
                       ROBOT_NAMES[stuckR]);

            vector<pair<int,int>> yieldPath;
            yieldPath.push_back(goals[blocker]);
            yieldPath.push_back(yieldCell);
            for(int i=0;i<YIELD_WAIT_STEPS;i++)
                yieldPath.push_back(yieldCell);
            yieldPath.push_back(goals[blocker]);

            newSegs[blocker]=yieldPath;
            reservePath(yieldPath);

            newSegs[stuckR]=timeExpandedAStar(curPos[stuckR],goals[stuckR],false);

            if((int)newSegs[stuckR].size()>1){
                reservePath(newSegs[stuckR]);
                if(verbose)
                    printf(" [YIELD]  %s found path (%d steps)\n",
                           ROBOT_NAMES[stuckR],(int)newSegs[stuckR].size());
                resolved=true;
            } else {
                erasePathReservations(yieldPath);
                newSegs[blocker]={goals[blocker]};
                reservePath(newSegs[blocker]);
                if(verbose)
                    printf(" [YIELD]  Still stuck after %s yielded — reverting\n",
                           ROBOT_NAMES[blocker]);
            }
        }

        if(!resolved){
            if(verbose)
                printf(" [YIELD] %s: no blocker found — waiting in place\n",
                       ROBOT_NAMES[stuckR]);
            newSegs[stuckR]={curPos[stuckR]};
            for(int i=0;i<YIELD_WAIT_STEPS;i++)
                newSegs[stuckR].push_back(curPos[stuckR]);
            reservePath(newSegs[stuckR]);
        }
    }
}

void replanAll(int fromStep,const vector<pair<int,int>>& goals,
               vector<vector<pair<int,int>>>& paths,int& maxLen,
               bool verbose=false)
{
    if(!paths.empty()&&allAtGoals(fromStep,paths,goals)){
        if(verbose) printf(" [REPLAN SKIPPED] All at goals\n");
        return;
    }
    if(verbose){
        printf("\n=== REPLAN at step %d ===\n",fromStep);
        for(int r=0;r<ROBOTS;r++){
            int cs=min(fromStep,(int)paths[r].size()-1);
            auto p=paths[r][cs];
            printf(" %s : (%d,%d)%s\n",ROBOT_NAMES[r],p.first,p.second,
                   grid[p.first][p.second]?" [ON OBSTACLE!]":"");
        }
    }

    vector<pair<int,int>> curPos(ROBOTS);
    for(int r=0;r<ROBOTS;r++){
        int cs=min(fromStep,(int)paths[r].size()-1);
        curPos[r]=paths[r][cs];
    }

    auto assemblePaths=[&](const vector<vector<pair<int,int>>>& segs)
        -> vector<vector<pair<int,int>>> {
        vector<vector<pair<int,int>>> cand(ROBOTS);
        for(int r=0;r<ROBOTS;r++){
            int histLen=min(fromStep+1,(int)paths[r].size());
            for(int t=0;t<histLen;t++) cand[r].push_back(paths[r][t]);
            if(!segs[r].empty()&&segs[r][0]!=curPos[r]&&verbose)
                printf(" [WARNING] %s discontinuity\n",ROBOT_NAMES[r]);
            for(int t=1;t<(int)segs[r].size();t++) cand[r].push_back(segs[r][t]);
        }
        int len=0;
        for(auto&p:cand) len=max(len,(int)p.size());
        for(auto&p:cand) while((int)p.size()<len) p.push_back(p.back());
        return cand;
    };

    set<int> conflictedRobots;
    vector<vector<pair<int,int>>> bestSegs;

    for(int attempt=0; attempt<MAX_REPLAN_ATTEMPTS; attempt++){
        vertexRes.clear(); edgeRes.clear(); cellLastReserved.clear();

        for(int r=0;r<ROBOTS;r++) vertexRes[0].insert(curPos[r]);
        rebuildCellLastReserved();

        vector<int> order;
        if(attempt==0){
            order=planningOrder(curPos,goals);
        } else {
            vector<int> cv(conflictedRobots.begin(),conflictedRobots.end());
            unsigned seed=(unsigned)(time(0)+attempt*999983);
            shuffle(cv.begin(),cv.end(),default_random_engine(seed));
            set<int> cs(conflictedRobots.begin(),conflictedRobots.end());
            vector<int> rest=planningOrder(curPos,goals,cs);
            order=cv;
            for(int r:rest) order.push_back(r);
        }

        if(verbose&&attempt==0){
            printf(" Order: ");
            for(int r:order) printf("%s ",ROBOT_NAMES[r]);
            printf("\n");
        }

        vector<vector<pair<int,int>>> newSegs(ROBOTS);
        for(int r:order){
            bool onObs=(grid[curPos[r].first][curPos[r].second]==1);
            bool atGoal=(curPos[r]==goals[r]);
            if(atGoal){
                newSegs[r]={curPos[r]};
            } else {
                newSegs[r]=timeExpandedAStar(curPos[r],goals[r],onObs);
                if((int)newSegs[r].size()<=1&&!atGoal){
                    if(verbose&&attempt==0)
                        printf(" [STUCK L1] %s at (%d,%d)\n",
                               ROBOT_NAMES[r],curPos[r].first,curPos[r].second);
                    newSegs[r]={curPos[r]};
                    for(int i=0;i<5;i++) newSegs[r].push_back(curPos[r]);
                }
            }
            reservePath(newSegs[r]);
        }

        auto cand=assemblePaths(newSegs);
        auto cr=detectCollisions(cand,fromStep,false);

        if(!cr.hasCollision){
            paths=cand;
            maxLen=(int)cand[0].size();
            if(verbose){
                printf(" Validation:\n");
                for(int r=0;r<ROBOTS;r++) validatePath(r,paths[r],fromStep,verbose);
                printf(" COLLISION-FREE (attempt %d/%d)\n",attempt+1,MAX_REPLAN_ATTEMPTS);
                printf(" maxLen=%d\n=== REPLAN done ===\n\n",maxLen);
            }
            return;
        }

        for(int r:cr.affectedRobots) conflictedRobots.insert(r);
        if(verbose)
            printf(" [attempt %d/%d] %s — retrying...\n",
                   attempt+1,MAX_REPLAN_ATTEMPTS,cr.description.c_str());
        if(bestSegs.empty()) bestSegs=newSegs;
    }

    if(verbose)
        printf(" [YIELD PASS] All %d attempts collided — trying yield...\n",
               MAX_REPLAN_ATTEMPTS);

    vertexRes.clear(); edgeRes.clear(); cellLastReserved.clear();
    for(int r=0;r<ROBOTS;r++) vertexRes[0].insert(curPos[r]);
    rebuildCellLastReserved();

    vector<int> finalOrder=planningOrder(curPos,goals);
    vector<vector<pair<int,int>>> finalSegs(ROBOTS);

    for(int r:finalOrder){
        bool onObs=(grid[curPos[r].first][curPos[r].second]==1);
        bool atGoal=(curPos[r]==goals[r]);
        if(atGoal){
            finalSegs[r]={curPos[r]};
        } else {
            finalSegs[r]=timeExpandedAStar(curPos[r],goals[r],onObs);
            if((int)finalSegs[r].size()<=1&&!atGoal)
                finalSegs[r]={curPos[r]};
        }
        reservePath(finalSegs[r]);
    }

    resolveYield(fromStep,goals,curPos,finalSegs,verbose);

    auto finalCand=assemblePaths(finalSegs);
    auto finalCr=detectCollisions(finalCand,fromStep,false);

    paths=finalCand;
    maxLen=(int)finalCand[0].size();

    if(verbose){
        if(!finalCr.hasCollision){
            printf(" Validation:\n");
            for(int r=0;r<ROBOTS;r++) validatePath(r,paths[r],fromStep,verbose);
            printf(" COLLISION-FREE (after yield pass)\n");
        } else {
            printf(" [WARNING] Residual collision after yield: %s\n",
                   finalCr.description.c_str());
        }
        printf(" maxLen=%d\n=== REPLAN done ===\n\n",maxLen);
    }
}

bool lookaheadBlocked(int step,const vector<vector<pair<int,int>>>& paths,
                      bool verbose=false)
{
    int ml=(int)paths[0].size();
    for(int r=0;r<ROBOTS;r++){
        for(int ahead=0;ahead<=LOOKAHEAD;ahead++){
            int t=min(step+ahead,ml-1);
            auto[x,y]=paths[r][t];
            if(grid[x][y]==1){
                if(verbose) printf("[LOOKAHEAD] %s step+%d will hit obstacle\n",
                                   ROBOT_NAMES[r],ahead);
                return true;
            }
        }
    }
    return false;
}

bool isRobotAt(int x,int y,int step,
               const vector<vector<pair<int,int>>>& paths,bool sim)
{
    if(!sim) return false;
    for(int r=0;r<ROBOTS;r++){
        int t=min(step,(int)paths[r].size()-1);
        if(paths[r][t].first==x&&paths[r][t].second==y) return true;
    }
    return false;
}

void renderRobot(int r,float px,float py,float ang,
                 pair<int,int> cur,pair<int,int> goal,
                 const vector<pair<int,int>>& paths_r,
                 Color col)
{
    bool atGoal    =(cur==goal);
    bool trulyStuck=((int)paths_r.size()<=1&&!atGoal);
    if(trulyStuck)
        DrawRectangleLinesEx({px-CELL*0.38f,py-CELL*0.28f,CELL*0.76f,CELL*0.56f},3,RED);
    DrawRectanglePro({px-CELL*0.3f,py-CELL*0.2f,CELL*0.6f,CELL*0.4f},
                     {CELL*0.3f,CELL*0.2f},ang,col);
    string lbl="R"+to_string(r); int fs=18;
    DrawText(lbl.c_str(),(int)(px-MeasureText(lbl.c_str(),fs)/2.f),(int)(py-fs/2.f),fs,BLACK);
    DrawText(("("+to_string(cur.first)+","+to_string(cur.second)+")").c_str(),
             (int)(px-20),(int)(py+14),12,BLACK);
}

int main(){
    srand(time(0));
    vector<pair<int,int>> start={{0,1},{0,2},{0,3},{0,4},{1,0},{2,0},{3,0},{4,0}};
    vector<pair<int,int>> goal ={{5,1},{5,2},{5,3},{5,4},{1,5},{2,5},{3,5},{4,5}};

    auto isProtectedStatic=[&](int x,int y){
        for(auto&s:start) if(s.first==x&&s.second==y) return true;
        for(auto&g:goal)  if(g.first==x&&g.second==y) return true;
        return false;
    };

    for(int i=0;i<ROWS;i++) for(int j=0;j<COLS;j++) grid[i][j]=0;

    int mode,n;
    printf("Enter mode (1=static, 2=dynamic, 3=random-goals): "); scanf("%d",&mode);
    printf("Enter number of obstacles: ");        scanf("%d",&n);

    InitWindow(COLS*CELL,ROWS*CELL+40,"Multi-Robot Pathfinding (No Collisions)");
    SetTargetFPS(60);
    Color colors[ROBOTS]={RED,BLUE,GREEN,ORANGE,PURPLE,BROWN,DARKGREEN,MAROON};
    vector<pair<int,int>> obstacles;

    auto doInitialPlan=[&](vector<vector<pair<int,int>>>& paths,
                           int& maxLen,
                           const vector<pair<int,int>>& curGoal){
        printf("\n--- Initial planning ---\n");
        vertexRes.clear(); edgeRes.clear(); cellLastReserved.clear();

        for(int r=0;r<ROBOTS;r++) vertexRes[0].insert(start[r]);
        rebuildCellLastReserved();

        auto order=planningOrder(start,curGoal);
        printf(" Order: ");
        for(int r:order) printf("%s ",ROBOT_NAMES[r]);
        printf("\n");

        for(int r:order){
            bool atGoal=(start[r]==curGoal[r]);
            if(atGoal) paths[r]={start[r]};
            else paths[r]=timeExpandedAStar(start[r],curGoal[r],false);
            reservePath(paths[r]);
            printf(" %s: %d steps\n",ROBOT_NAMES[r],(int)paths[r].size());
        }
        maxLen=0;
        for(auto&p:paths) maxLen=max(maxLen,(int)p.size());
        for(auto&p:paths) while((int)p.size()<maxLen) p.push_back(p.back());

        printf(" maxLen=%d\nValidation:\n",maxLen);
        for(int r=0;r<ROBOTS;r++) validatePath(r,paths[r],0);
        auto cr=detectCollisions(paths,0,true);
        if(!cr.hasCollision) printf(" COLLISION-FREE\n");
        else                 printf(" WARNING: collisions detected\n");
    };

    if(mode==1){
        int placed=0;
        while(placed<n){
            int x=rand()%ROWS,y=rand()%COLS;
            if(grid[x][y]==0&&!isProtectedStatic(x,y)){
                grid[x][y]=1; obstacles.push_back({x,y}); placed++;
            }
        }
        printf("\n=== MODE 1 : %d static obstacles ===\n",n);

        vector<vector<pair<int,int>>> paths(ROBOTS);
        int maxLen=0;
        doInitialPlan(paths,maxLen,goal);

        const float TIME_PER_STEP=1.5f; const double START_DELAY=5.0;
        double startTime=GetTime(); int step=0; double lastStep=0;
        bool started=false,finished=false; int lastPrinted=-1;
        vector<float> angle(ROBOTS,0),posX(ROBOTS),posY(ROBOTS);
        for(int i=0;i<ROBOTS;i++){
            posX[i]=start[i].second*CELL+CELL/2.f;
            posY[i]=start[i].first*CELL+CELL/2.f;
        }

        while(!WindowShouldClose()){
            double now=GetTime();
            if(!started&&now-startTime>=START_DELAY){started=true;lastStep=now;}
            if(started&&!finished&&now-lastStep>=TIME_PER_STEP){
                if(step<maxLen-1){step++;lastStep=now;}else finished=true;}
            if(started&&step!=lastPrinted){
                printSingleStep(step,paths,goal); lastPrinted=step;
                if(finished) printf("\nAll robots at goals!\n");}

            BeginDrawing(); ClearBackground(RAYWHITE);
            for(int i=0;i<ROWS;i++) for(int j=0;j<COLS;j++){
                if(grid[i][j]) DrawRectangle(j*CELL,i*CELL,CELL,CELL,DARKGRAY);
                DrawRectangleLines(j*CELL,i*CELL,CELL,CELL,LIGHTGRAY);}
            for(int r=0;r<ROBOTS;r++){
                Color gc=colors[r]; gc.a=60;
                DrawRectangle(goal[r].second*CELL+2,goal[r].first*CELL+2,CELL-4,CELL-4,gc);
                DrawText(("G"+to_string(r)).c_str(),goal[r].second*CELL+4,goal[r].first*CELL+4,14,colors[r]);}
            for(int r=0;r<ROBOTS;r++){
                auto cur=started?paths[r][step]:start[r];
                auto nxt=started?paths[r][min(step+1,maxLen-1)]:start[r];
                float cx=cur.second*CELL+CELL/2.f,cy=cur.first*CELL+CELL/2.f;
                float nx2=nxt.second*CELL+CELL/2.f,ny2=nxt.first*CELL+CELL/2.f;
                float alpha=started&&!finished?min(1.f,float((now-lastStep)/TIME_PER_STEP)):0.f;
                posX[r]=cx+(nx2-cx)*alpha; posY[r]=cy+(ny2-cy)*alpha;
                if(nxt.first>cur.first) angle[r]=90;
                else if(nxt.first<cur.first) angle[r]=270;
                else if(nxt.second>cur.second) angle[r]=0;
                else if(nxt.second<cur.second) angle[r]=180;
                renderRobot(r,posX[r],posY[r],angle[r],cur,goal[r],paths[r],colors[r]);}
            DrawRectangle(0,ROWS*CELL,COLS*CELL,40,Fade(BLACK,0.75f));
            if(!started){
                DrawText(("Mode1|Start in "+to_string(max(0,(int)(START_DELAY-(now-startTime))+1))+"s").c_str(),10,ROWS*CELL+10,18,WHITE);
            }else if(finished){
                DrawText("All at goals!",10,ROWS*CELL+10,18,GREEN);
            }else{
                DrawText(("Step "+to_string(step)+"/"+to_string(maxLen-1)).c_str(),10,ROWS*CELL+10,18,WHITE);
            }
            EndDrawing();
        }
        CloseWindow(); return 0;
    }

    if(mode==2){
        const float TIME_PER_STEP=1.5f; const double START_DELAY=5.0;
        const double OBS_INTERVAL=5.0; const double RELOC_INTERVAL=8.0;
        double wallStart=GetTime();
        int obsPlaced=0; double lastObsTime=wallStart; bool allObsPlaced=false;
        bool simStarted=false; double lastStepTime=0.0;
        int step=0,maxLen=1; bool pathsReady=false;
        double relocTriggerTime=0.0; int relocCount=0;
        vector<vector<pair<int,int>>> paths(ROBOTS);
        vector<float> angle(ROBOTS,0),posX(ROBOTS),posY(ROBOTS);
        for(int i=0;i<ROBOTS;i++){
            posX[i]=start[i].second*CELL+CELL/2.f;
            posY[i]=start[i].first*CELL+CELL/2.f;
        }
        bool replanFlash=false; double replanFlashTime=0.0;
        int lastPrinted=-1,lastReplanStep=-1;

        printf("\n=== MODE 2 : %d dynamic obstacles ===\n",n);

        auto placeOneObs=[&]()->bool{
            for(int att=0;att<1000;att++){
                int x=rand()%ROWS,y=rand()%COLS;
                if(grid[x][y]==0&&!isProtectedStatic(x,y)&&!isRobotAt(x,y,step,paths,simStarted)){
                    grid[x][y]=1; obstacles.push_back({x,y}); obsPlaced++;
                    printf("[OBS #%d] placed at (%d,%d)\n",obsPlaced,x,y); return true;}}
            return false;};

        while(!WindowShouldClose()){
            double now=GetTime();

            if(!allObsPlaced&&now-lastObsTime>=OBS_INTERVAL){
                if(placeOneObs()){
                    lastObsTime=now;
                    if(simStarted&&pathsReady&&lastReplanStep!=step){
                        replanAll(step,goal,paths,maxLen,true);
                        lastReplanStep=step; replanFlash=true; replanFlashTime=now;}}
                if(obsPlaced>=n){allObsPlaced=true;relocTriggerTime=now;}}

            if(!simStarted&&now-wallStart>=START_DELAY){
                simStarted=true;lastStepTime=now;
                doInitialPlan(paths,maxLen,goal);
                pathsReady=true;}

            if(simStarted&&pathsReady&&lastReplanStep!=step){
                if(lookaheadBlocked(step,paths,true)){
                    replanAll(step,goal,paths,maxLen,true);
                    lastReplanStep=step; replanFlash=true; replanFlashTime=now;}}

            if(simStarted&&pathsReady&&now-lastStepTime>=TIME_PER_STEP){
                if(step<maxLen-1){step++;lastStepTime=now;}}

            if(simStarted&&pathsReady&&step!=lastPrinted){
                printSingleStep(step,paths,goal); lastPrinted=step;}

            if(allObsPlaced&&simStarted&&pathsReady&&!obstacles.empty()
               &&now-relocTriggerTime>=RELOC_INTERVAL){
                if(allAtGoals(step,paths,goal)){
                    relocTriggerTime=now;
                }else{
                    int idx=relocCount%(int)obstacles.size();
                    auto oldPos=obstacles[idx];
                    printf("\n[RELOC #%d] (%d,%d)->",relocCount,oldPos.first,oldPos.second);
                    grid[oldPos.first][oldPos.second]=0;
                    bool moved=false;
                    for(int att=0;att<1000;att++){
                        int nx2=rand()%ROWS,ny2=rand()%COLS;
                        if(grid[nx2][ny2]==0&&!isProtectedStatic(nx2,ny2)&&!isRobotAt(nx2,ny2,step,paths,simStarted)){
                            grid[nx2][ny2]=1; obstacles[idx]={nx2,ny2};
                            printf("(%d,%d)\n",nx2,ny2); moved=true; break;}}
                    if(!moved){grid[oldPos.first][oldPos.second]=1;printf("kept\n");}
                    if(lastReplanStep!=step){
                        replanAll(step,goal,paths,maxLen,true);
                        lastReplanStep=step; replanFlash=true; replanFlashTime=now;}
                    relocCount++; relocTriggerTime=now;}}

            BeginDrawing(); ClearBackground(RAYWHITE);
            for(int i=0;i<ROWS;i++) for(int j=0;j<COLS;j++){
                if(grid[i][j]) DrawRectangle(j*CELL,i*CELL,CELL,CELL,DARKGRAY);
                DrawRectangleLines(j*CELL,i*CELL,CELL,CELL,LIGHTGRAY);}
            for(int r=0;r<ROBOTS;r++){
                Color gc=colors[r]; gc.a=60;
                DrawRectangle(goal[r].second*CELL+2,goal[r].first*CELL+2,CELL-4,CELL-4,gc);
                DrawText(("G"+to_string(r)).c_str(),goal[r].second*CELL+4,goal[r].first*CELL+4,14,colors[r]);}
            for(int r=0;r<ROBOTS;r++){
                int  s2=(simStarted&&pathsReady)?step:0;
                int  ml=(simStarted&&pathsReady)?maxLen:1;
                auto cur=(simStarted&&pathsReady)?paths[r][s2]:start[r];
                auto nxt=(simStarted&&pathsReady)?paths[r][min(s2+1,ml-1)]:start[r];
                float cx=cur.second*CELL+CELL/2.f,cy=cur.first*CELL+CELL/2.f;
                float nx2=nxt.second*CELL+CELL/2.f,ny2=nxt.first*CELL+CELL/2.f;
                float alpha=(simStarted&&pathsReady)?min(1.f,float((now-lastStepTime)/TIME_PER_STEP)):0.f;
                posX[r]=cx+(nx2-cx)*alpha; posY[r]=cy+(ny2-cy)*alpha;
                if(nxt.first>cur.first) angle[r]=90;
                else if(nxt.first<cur.first) angle[r]=270;
                else if(nxt.second>cur.second) angle[r]=0;
                else if(nxt.second<cur.second) angle[r]=180;
                renderRobot(r,posX[r],posY[r],angle[r],cur,goal[r],paths[r],colors[r]);}
            DrawRectangle(0,ROWS*CELL,COLS*CELL,40,Fade(BLACK,0.75f));
            if(!simStarted){
                DrawText(("Mode2|Start in "+to_string(max(0,(int)(START_DELAY-(now-wallStart))+1))+"s").c_str(),10,ROWS*CELL+10,18,WHITE);
            }else{
                bool allDone=simStarted&&pathsReady&&allAtGoals(step,paths,goal);
                if(allDone){
                    DrawText("All at goals!",10,ROWS*CELL+10,18,GREEN);
                }else{
                    string msg=allObsPlaced
                        ?("Step "+to_string(step)+"/"+to_string(maxLen-1)+" | Relocs:"+to_string(relocCount))
                        :("Placing obs "+to_string(obsPlaced)+"/"+to_string(n));
                    DrawText(msg.c_str(),10,ROWS*CELL+10,18,WHITE);
                    if(replanFlash&&now-replanFlashTime<1.0){
                        DrawText("REPLAN",400,ROWS*CELL+10,18,YELLOW);
                    }else replanFlash=false;
                }}
            EndDrawing();
        }
        CloseWindow(); return 0;
    }

    if(mode==3){
        vector<pair<int,int>> rowGoalPool={{5,1},{5,2},{5,3},{5,4}};
        vector<pair<int,int>> colGoalPool={{1,5},{2,5},{3,5},{4,5}};

        std::random_device rd;
        std::mt19937 rng(rd());
        shuffle(rowGoalPool.begin(),rowGoalPool.end(),rng);
        shuffle(colGoalPool.begin(),colGoalPool.end(),rng);

        goal.clear();
        for(int i=0;i<4;i++) goal.push_back(rowGoalPool[i]);
        for(int i=0;i<4;i++) goal.push_back(colGoalPool[i]);

        printf("\n=== MODE 3 : Random Goals + Dynamic ===\n");
        printf("Goals:\n");
        for(int r=0;r<ROBOTS;r++)
            printf(" %s: (%d,%d)->(%d,%d)\n",ROBOT_NAMES[r],
                   start[r].first,start[r].second,goal[r].first,goal[r].second);

        const float TIME_PER_STEP=1.5f; const double START_DELAY=5.0;
        const double OBS_INTERVAL=5.0; const double RELOC_INTERVAL=8.0;
        double wallStart=GetTime();
        int obsPlaced=0; double lastObsTime=wallStart; bool allObsPlaced=false;
        bool simStarted=false; double lastStepTime=0.0;
        int step=0,maxLen=1; bool pathsReady=false;
        double relocTriggerTime=0.0; int relocCount=0;
        vector<vector<pair<int,int>>> paths(ROBOTS);
        vector<float> angle(ROBOTS,0),posX(ROBOTS),posY(ROBOTS);
        for(int i=0;i<ROBOTS;i++){
            posX[i]=start[i].second*CELL+CELL/2.f;
            posY[i]=start[i].first*CELL+CELL/2.f;
        }
        bool replanFlash=false; double replanFlashTime=0.0;
        int lastPrinted=-1,lastReplanStep=-1;

        auto placeOneObs=[&]()->bool{
            for(int att=0;att<1000;att++){
                int x=rand()%ROWS,y=rand()%COLS;
                if(grid[x][y]==0&&!isProtectedStatic(x,y)&&!isRobotAt(x,y,step,paths,simStarted)){
                    grid[x][y]=1; obstacles.push_back({x,y}); obsPlaced++;
                    printf("[OBS #%d] at (%d,%d)\n",obsPlaced,x,y); return true;}}
            return false;};

        while(!WindowShouldClose()){
            double now=GetTime();

            if(!allObsPlaced&&now-lastObsTime>=OBS_INTERVAL){
                if(placeOneObs()){
                    lastObsTime=now;
                    if(simStarted&&pathsReady&&lastReplanStep!=step){
                        replanAll(step,goal,paths,maxLen,true);
                        lastReplanStep=step; replanFlash=true; replanFlashTime=now;}}
                if(obsPlaced>=n){allObsPlaced=true;relocTriggerTime=now;}}

            if(!simStarted&&now-wallStart>=START_DELAY){
                simStarted=true;lastStepTime=now;
                doInitialPlan(paths,maxLen,goal);
                pathsReady=true;}

            if(simStarted&&pathsReady&&lastReplanStep!=step){
                if(lookaheadBlocked(step,paths,true)){
                    replanAll(step,goal,paths,maxLen,true);
                    lastReplanStep=step; replanFlash=true; replanFlashTime=now;}}

            if(simStarted&&pathsReady&&now-lastStepTime>=TIME_PER_STEP){
                if(step<maxLen-1){step++;lastStepTime=now;}}

            if(simStarted&&pathsReady&&step!=lastPrinted){
                printSingleStep(step,paths,goal); lastPrinted=step;}

            if(allObsPlaced&&simStarted&&pathsReady&&!obstacles.empty()
               &&now-relocTriggerTime>=RELOC_INTERVAL){
                if(allAtGoals(step,paths,goal)){
                    relocTriggerTime=now;
                }else{
                    int idx=relocCount%(int)obstacles.size();
                    auto oldPos=obstacles[idx];
                    printf("\n[RELOC #%d] (%d,%d)->",relocCount,oldPos.first,oldPos.second);
                    grid[oldPos.first][oldPos.second]=0;
                    bool moved=false;
                    for(int att=0;att<1000;att++){
                        int nx2=rand()%ROWS,ny2=rand()%COLS;
                        if(grid[nx2][ny2]==0&&!isProtectedStatic(nx2,ny2)&&!isRobotAt(nx2,ny2,step,paths,simStarted)){
                            grid[nx2][ny2]=1; obstacles[idx]={nx2,ny2};
                            printf("(%d,%d)\n",nx2,ny2); moved=true; break;}}
                    if(!moved){grid[oldPos.first][oldPos.second]=1;printf("kept\n");}
                    if(lastReplanStep!=step){
                        replanAll(step,goal,paths,maxLen,true);
                        lastReplanStep=step; replanFlash=true; replanFlashTime=now;}
                    relocCount++; relocTriggerTime=now;}}

            BeginDrawing(); ClearBackground(RAYWHITE);
            for(int i=0;i<ROWS;i++) for(int j=0;j<COLS;j++){
                if(grid[i][j]) DrawRectangle(j*CELL,i*CELL,CELL,CELL,DARKGRAY);
                DrawRectangleLines(j*CELL,i*CELL,CELL,CELL,LIGHTGRAY);}
            for(int r=0;r<ROBOTS;r++){
                Color gc=colors[r]; gc.a=60;
                DrawRectangle(goal[r].second*CELL+2,goal[r].first*CELL+2,CELL-4,CELL-4,gc);
                DrawText(("G"+to_string(r)).c_str(),goal[r].second*CELL+4,goal[r].first*CELL+4,14,colors[r]);}
            for(int r=0;r<ROBOTS;r++){
                int  s2=(simStarted&&pathsReady)?step:0;
                int  ml=(simStarted&&pathsReady)?maxLen:1;
                auto cur=(simStarted&&pathsReady)?paths[r][s2]:start[r];
                auto nxt=(simStarted&&pathsReady)?paths[r][min(s2+1,ml-1)]:start[r];
                float cx=cur.second*CELL+CELL/2.f,cy=cur.first*CELL+CELL/2.f;
                float nx2=nxt.second*CELL+CELL/2.f,ny2=nxt.first*CELL+CELL/2.f;
                float alpha=(simStarted&&pathsReady)?min(1.f,float((now-lastStepTime)/TIME_PER_STEP)):0.f;
                posX[r]=cx+(nx2-cx)*alpha; posY[r]=cy+(ny2-cy)*alpha;
                if(nxt.first>cur.first) angle[r]=90;
                else if(nxt.first<cur.first) angle[r]=270;
                else if(nxt.second>cur.second) angle[r]=0;
                else if(nxt.second<cur.second) angle[r]=180;
                renderRobot(r,posX[r],posY[r],angle[r],cur,goal[r],paths[r],colors[r]);}
            DrawRectangle(0,ROWS*CELL,COLS*CELL,40,Fade(BLACK,0.75f));
            if(!simStarted){
                DrawText(("Mode3|Start in "+to_string(max(0,(int)(START_DELAY-(now-wallStart))+1))+"s").c_str(),10,ROWS*CELL+10,18,WHITE);
            }else{
                bool allDone=simStarted&&pathsReady&&allAtGoals(step,paths,goal);
                if(allDone){
                    DrawText("All at goals!",10,ROWS*CELL+10,18,GREEN);
                }else{
                    string msg=allObsPlaced
                        ?("Step "+to_string(step)+"/"+to_string(maxLen-1)+" | Relocs:"+to_string(relocCount))
                        :("Placing obs "+to_string(obsPlaced)+"/"+to_string(n));
                    DrawText(msg.c_str(),10,ROWS*CELL+10,18,WHITE);
                    if(replanFlash&&now-replanFlashTime<1.0){
                        DrawText("REPLAN",400,ROWS*CELL+10,18,YELLOW);
                    }else replanFlash=false;
                }}
            EndDrawing();
        }
        CloseWindow(); return 0;
    }

    CloseWindow(); return 0;
}
