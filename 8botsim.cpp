#include <bits/stdc++.h>
#include "raylib.h"
using namespace std;

/* ================================================================ CONFIG */
static const int ROWS=6, COLS=6, CELL=80, ROBOTS=8;
static const int MAX_TIME=400, LOOKAHEAD=4;
static const int GOAL_HOLD_STEPS=2;
static const int BACKTRACK_PENALTY=2;
static const int REVISIT_WINDOW=3;
static const int MAX_REPLAN_ATTEMPTS=5, YIELD_WAIT_STEPS=3;
static const int WAIT_PENALTY=0;

static const float SPEED_MIN_GLOBAL=0.30f;
static const float SPEED_MAX_GLOBAL=1.50f;
static const float SPEED_BASE=1.00f;
static const int   VIC_MIN_GLOBAL=1;
static const int   VIC_MAX_GLOBAL=4;
static const float COLL_PROB_HIGH=0.70f;
static const float COLL_PROB_MED=0.40f;
static const float EMA_ALPHA=0.65f;

static const double SIM_START_DELAY = 4.0;   // wall-clock seconds before sim
static const double TPS              = 1.4;   // seconds per logical step
// Obstacle timing: spread all n obstacles so the last one fires at
// OBS_END_FRAC * (EST_SIM_STEPS * TPS) seconds into the simulation.
// With TPS=1.4 and EST_SIM_STEPS=14 => ~19.6 s total; 65% => ~12.7 s window.
static const double OBS_END_FRAC   = 0.65;
static const int    EST_SIM_STEPS  = 14;

/* ================================================================ GLOBALS */
static map<int,set<pair<int,int>>>                     vertexRes;
static map<int,set<pair<pair<int,int>,pair<int,int>>>> edgeRes;
static int grid[ROWS][COLS];
static map<pair<int,int>,int> cellLastReserved;

static const char* ROBOT_NAMES[ROBOTS]={
    "R0(RED)","R1(BLUE)","R2(GREEN)","R3(ORANGE)",
    "R4(PURPLE)","R5(BROWN)","R6(DKGRN)","R7(MAROON)"
};

/* ================================================================ HELPERS */
static bool validCell(int x,int y){
    return x>=0&&y>=0&&x<ROWS&&y<COLS&&grid[x][y]==0;
}
static void rebuildCellLastReserved(){
    cellLastReserved.clear();
    for(auto&[t,cells]:vertexRes)
        for(auto&c:cells){
            auto it=cellLastReserved.find(c);
            if(it==cellLastReserved.end()||t>it->second) cellLastReserved[c]=t;
        }
}
static bool goalCanSettle(pair<int,int> c,int fromT){
    auto it=cellLastReserved.find(c);
    return it==cellLastReserved.end()||it->second<fromT;
}

/* ================================================================ A* */
struct ANode{int x,y,t,g,h; ANode*parent;};
struct ACmp{bool operator()(const ANode*a,const ANode*b)const{
    return(a->g+a->h)>(b->g+b->h);}};

static int heuristic(int nx,int ny,int gx,int gy){
    return abs(nx-gx)+abs(ny-gy);
}
static vector<pair<int,int>> reconstruct(ANode*n){
    vector<pair<int,int>> p;
    while(n){p.push_back({n->x,n->y});n=n->parent;}
    reverse(p.begin(),p.end());
    return p;
}
static bool recentlyVisited(ANode*cur,int nx,int ny){
    ANode*p=cur;
    for(int i=0;i<REVISIT_WINDOW&&p;i++,p=p->parent)
        if(p->x==nx&&p->y==ny) return true;
    return false;
}
static vector<pair<int,int>> timeExpandedAStar(
        pair<int,int>s,pair<int,int>g,
        bool /*startOnObs*/=false,int localMax=MAX_TIME){
    vector<ANode*> pool;
    auto mk=[&](int x,int y,int t,int gc,int h,ANode*par)->ANode*{
        auto*n=new ANode{x,y,t,gc,h,par};
        pool.push_back(n); return n;
    };
    struct Guard{vector<ANode*>&p;~Guard(){for(auto*n:p)delete n;}} guard{pool};
    priority_queue<ANode*,vector<ANode*>,ACmp> pq;
    map<tuple<int,int,int>,int> minG;
    pq.push(mk(s.first,s.second,0,0,heuristic(s.first,s.second,g.first,g.second),nullptr));
    vector<pair<int,int>> result;
    while(!pq.empty()){
        ANode*cur=pq.top(); pq.pop();
        auto key=make_tuple(cur->x,cur->y,cur->t);
        auto it=minG.find(key);
        if(it!=minG.end()&&cur->g>=it->second) continue;
        minG[key]=cur->g;
        if(cur->x==g.first&&cur->y==g.second&&goalCanSettle({cur->x,cur->y},cur->t)){
            result=reconstruct(cur); break;
        }
        if(cur->t>=localMax) continue;
        for(auto[dx,dy]:vector<pair<int,int>>{{0,0},{1,0},{-1,0},{0,1},{0,-1}}){
            int nx=cur->x+dx,ny=cur->y+dy,nt=cur->t+1;
            if(nx<0||ny<0||nx>=ROWS||ny>=COLS) continue;
            if(grid[nx][ny]!=0) continue;
            if(vertexRes[nt].count({nx,ny})) continue;
            bool isW=(dx==0&&dy==0);
            if(!isW&&edgeRes[nt].count({{nx,ny},{cur->x,cur->y}})) continue;
            bool rev=(!isW)&&recentlyVisited(cur,nx,ny);
            int ng=cur->g+1+(isW?WAIT_PENALTY:(rev?BACKTRACK_PENALTY:0));
            auto nkey=make_tuple(nx,ny,nt);
            auto nit=minG.find(nkey);
            if(nit!=minG.end()&&ng>=nit->second) continue;
            pq.push(mk(nx,ny,nt,ng,heuristic(nx,ny,g.first,g.second),cur));
        }
    }
    if(result.empty()) result={s};
    for(int i=1;i<(int)result.size();i++){
        if(abs(result[i].first-result[i-1].first)+
           abs(result[i].second-result[i-1].second)>1){
            result.resize(i); break;
        }
    }
    return result;
}

/* ================================================================ RESERVATIONS */
static void reservePath(const vector<pair<int,int>>&p){
    for(int t=0;t<(int)p.size();t++){
        vertexRes[t].insert(p[t]);
        if(t>0){edgeRes[t].insert({p[t-1],p[t]}); edgeRes[t].insert({p[t],p[t-1]});}
    }
    auto gp=p.back();
    int h=min((int)p.size()+GOAL_HOLD_STEPS,MAX_TIME);
    for(int t=(int)p.size();t<h;t++) vertexRes[t].insert(gp);
    rebuildCellLastReserved();
}
static void erasePathReservations(const vector<pair<int,int>>&p){
    for(int t=0;t<(int)p.size();t++){
        vertexRes[t].erase(p[t]);
        if(t>0){edgeRes[t].erase({p[t-1],p[t]}); edgeRes[t].erase({p[t],p[t-1]});}
    }
    auto gp=p.back();
    int h=min((int)p.size()+GOAL_HOLD_STEPS,MAX_TIME);
    for(int t=(int)p.size();t<h;t++) vertexRes[t].erase(gp);
    rebuildCellLastReserved();
}

/* ================================================================ COLLISION */
struct CollisionReport{
    bool hasCollision=false;
    vector<int> affectedRobots;
    int timeStep=-1;
    string description;
};
static CollisionReport detectCollisions(
        const vector<vector<pair<int,int>>>&paths,
        int from=0,int toStep=-1,bool verbose=false){
    CollisionReport rep;
    int ml=0;
    for(auto&p:paths) ml=max(ml,(int)p.size());
    if(toStep>=0) ml=min(ml,toStep);
    for(int t=from;t<ml;t++){
        map<pair<int,int>,vector<int>> occ;
        for(int r=0;r<ROBOTS;r++)
            occ[paths[r][min(t,(int)paths[r].size()-1)]].push_back(r);
        for(auto&[pos,rs]:occ){
            if(rs.size()>1){
                rep.hasCollision=true; rep.timeStep=t;
                rep.description="VERTEX@("+to_string(pos.first)+","+
                                to_string(pos.second)+")t="+to_string(t);
                rep.affectedRobots=rs;
                if(verbose) printf(" !! %s\n",rep.description.c_str());
                return rep;
            }
        }
        if(t>from){
            for(int r1=0;r1<ROBOTS;r1++){
                auto p1p=paths[r1][min(t-1,(int)paths[r1].size()-1)];
                auto p1c=paths[r1][min(t,  (int)paths[r1].size()-1)];
                for(int r2=r1+1;r2<ROBOTS;r2++){
                    auto p2p=paths[r2][min(t-1,(int)paths[r2].size()-1)];
                    auto p2c=paths[r2][min(t,  (int)paths[r2].size()-1)];
                    if(p1p==p2c&&p1c==p2p&&p1p!=p1c){
                        rep.hasCollision=true; rep.timeStep=t;
                        rep.affectedRobots={r1,r2};
                        rep.description="EDGE t="+to_string(t)+" "+
                                       ROBOT_NAMES[r1]+"<->"+ROBOT_NAMES[r2];
                        if(verbose) printf(" !! %s\n",rep.description.c_str());
                        return rep;
                    }
                }
            }
        }
    }
    return rep;
}

/* ================================================================ DECISIONS */
enum class Decision{PROCEED,SLOW,WAIT,DEVIATE,STEPBACK,YIELD_TO_OTHER};
static const char* decisionName(Decision d){
    switch(d){
        case Decision::PROCEED:        return "PROCEED";
        case Decision::SLOW:           return "SLOW";
        case Decision::WAIT:           return "WAIT";
        case Decision::DEVIATE:        return "DEVIATE";
        case Decision::STEPBACK:       return "STEPBACK";
        case Decision::YIELD_TO_OTHER: return "YIELD";
        default: return "?";
    }
}

/* ================================================================ RobotBehavior */
struct RobotBehavior{
    int id;
    pair<int,int> start,goal;
    float speedFactor,baseSpeed,smoothedSpeed;
    int vicMin,vicMax,nearestDist;
    float nearestSpeed,collisionProb;
    Decision lastDecision;
    int waitCountdown;
    bool stepBackActive;
    int waitCount,behavWaitCount,deviationCells,originalPathLen;
    vector<pair<int,int>> originalPath;
    float px,py,angle;

    RobotBehavior()=default;
    RobotBehavior(int _id,pair<int,int> s,pair<int,int> g){
        id=_id; start=s; goal=g;
        px=s.second*CELL+CELL/2.f;
        py=s.first *CELL+CELL/2.f;
        angle=0.f;
        baseSpeed=SPEED_BASE+((float)(rand()%21)-10)/100.f;
        speedFactor=smoothedSpeed=baseSpeed;
        vicMin=VIC_MIN_GLOBAL; vicMax=VIC_MAX_GLOBAL+(rand()%2);
        nearestDist=vicMax; nearestSpeed=SPEED_BASE;
        collisionProb=0.f; lastDecision=Decision::PROCEED;
        waitCountdown=0; stepBackActive=false;
        waitCount=behavWaitCount=deviationCells=originalPathLen=0;
    }
    void setOriginalPath(const vector<pair<int,int>>&p){
        originalPath=p; originalPathLen=(int)p.size();
    }
    void trackStep(int step,pair<int,int> pos){
        if(pos==goal) return;
        if(step<originalPathLen){if(pos!=originalPath[step]) deviationCells++;}
        else deviationCells++;
    }
    int computeVicinity(int step,const vector<vector<pair<int,int>>>&paths,
                        const vector<RobotBehavior>&robots){
        int best=ROWS+COLS+1;
        int myIdx=min(step,(int)paths[id].size()-1);
        auto myPos=paths[id][myIdx];
        nearestSpeed=SPEED_BASE;
        for(int r=0;r<ROBOTS;r++){
            if(r==id) continue;
            int rIdx=min(step,(int)paths[r].size()-1);
            auto rPos=paths[r][rIdx];
            int d=abs(myPos.first-rPos.first)+abs(myPos.second-rPos.second);
            if(d<best){best=d; nearestSpeed=robots[r].speedFactor;}
        }
        return best;
    }
    void adjustSpeed(int dist,float neighborSpeed){
        nearestDist=dist;
        float zoneFloor,raw;
        if(dist<=vicMin){zoneFloor=SPEED_MIN_GLOBAL; raw=SPEED_MIN_GLOBAL+0.05f;}
        else if(dist==vicMin+1){
            zoneFloor=0.35f;
            float t=(float)(dist-vicMin)/(float)(vicMax-vicMin);
            raw=0.35f+t*0.25f;
        } else if(dist<=vicMax/2){
            zoneFloor=0.55f;
            float t=(float)(dist-vicMin)/(float)(vicMax-vicMin);
            raw=0.55f+t*0.35f;
        } else if(dist<=vicMax){
            zoneFloor=baseSpeed*0.8f;
            float t=(float)(dist-vicMax/2)/(float)max(1,vicMax-vicMax/2);
            raw=baseSpeed+t*0.20f;
        } else {
            zoneFloor=baseSpeed;
            raw=min(SPEED_MAX_GLOBAL,baseSpeed+0.30f);
        }
        if(dist<=vicMax&&neighborSpeed<raw){
            float ff=1.0f-(float)(vicMax-dist)/(float)vicMax;
            raw=raw*(1.f-ff)+neighborSpeed*ff;
        }
        if(dist>vicMax&&neighborSpeed>raw) raw=min(raw+0.05f,SPEED_MAX_GLOBAL);
        raw=max(zoneFloor,min(SPEED_MAX_GLOBAL,raw));
        smoothedSpeed=EMA_ALPHA*raw+(1.f-EMA_ALPHA)*smoothedSpeed;
        smoothedSpeed=max(smoothedSpeed,zoneFloor);
        speedFactor=max(SPEED_MIN_GLOBAL,min(SPEED_MAX_GLOBAL,smoothedSpeed));
    }
    float estimateCollisionProbability(int step,
            const vector<vector<pair<int,int>>>&paths)const{
        float score=0.f;
        for(int ahead=1;ahead<=LOOKAHEAD;ahead++){
            int t =min(step+ahead,(int)paths[id].size()-1);
            auto mp=paths[id][t];
            int t1=(ahead>1)?min(step+ahead-1,(int)paths[id].size()-1):-1;
            auto mp1=(t1>=0)?paths[id][t1]:mp;
            for(int r=0;r<ROBOTS;r++){
                if(r==id) continue;
                int rt =min(step+ahead,(int)paths[r].size()-1);
                auto rp=paths[r][rt];
                int rt1=(ahead>1)?min(step+ahead-1,(int)paths[r].size()-1):-1;
                auto rp1=(rt1>=0)?paths[r][rt1]:rp;
                int d=abs(mp.first-rp.first)+abs(mp.second-rp.second);
                if(d==0)      score+=0.80f;
                else if(d==1) score+=0.30f;
                else if(d==2) score+=0.10f;
                if(t1>=0&&rt1>=0&&mp1==rp&&rp1==mp&&mp1!=mp) score+=0.60f;
            }
        }
        return min(1.0f,score/((ROBOTS-1)*LOOKAHEAD*0.80f));
    }
    float computePriority(int step,
                          const vector<vector<pair<int,int>>>&paths)const{
        int idx=min(step,(int)paths[id].size()-1);
        auto pos=paths[id][idx];
        int dist=abs(pos.first-goal.first)+abs(pos.second-goal.second);
        return -(float)dist+speedFactor*0.5f+id*0.01f;
    }
    Decision decideAction(int step,const vector<vector<pair<int,int>>>&paths,
                          const vector<RobotBehavior>&robots,bool verbose){
        int dist=computeVicinity(step,paths,robots);
        adjustSpeed(dist,nearestSpeed);
        collisionProb=estimateCollisionProbability(step,paths);
        if(waitCountdown>0){
            --waitCountdown; waitCount++; behavWaitCount++;
            lastDecision=Decision::WAIT; return Decision::WAIT;
        }
        int idx=min(step,(int)paths[id].size()-1);
        if(paths[id][idx]==goal){lastDecision=Decision::PROCEED; return Decision::PROCEED;}
        if(collisionProb>=COLL_PROB_HIGH){
            float myPri=computePriority(step,paths);
            bool iAmTop=true;
            for(int r=0;r<ROBOTS;r++){
                if(r==id) continue;
                int rIdx=min(step,(int)paths[r].size()-1);
                int d=abs(paths[id][idx].first -paths[r][rIdx].first)
                     +abs(paths[id][idx].second-paths[r][rIdx].second);
                if(d<=vicMax&&robots[r].computePriority(step,paths)<myPri){iAmTop=false;break;}
            }
            if(!iAmTop){
                if(idx>0){
                    auto prev=paths[id][idx-1];
                    if(validCell(prev.first,prev.second)&&prev!=paths[id][idx]){
                        stepBackActive=true;
                        lastDecision=Decision::STEPBACK; return Decision::STEPBACK;
                    }
                }
                waitCountdown=2; speedFactor=SPEED_MIN_GLOBAL;
                waitCount++; behavWaitCount++;
                lastDecision=Decision::WAIT; return Decision::WAIT;
            }
            lastDecision=Decision::PROCEED; return Decision::PROCEED;
        } else if(collisionProb>=COLL_PROB_MED){
            lastDecision=Decision::DEVIATE; return Decision::DEVIATE;
        } else if(dist<=vicMin){
            lastDecision=Decision::SLOW; return Decision::SLOW;
        }
        lastDecision=Decision::PROCEED; return Decision::PROCEED;
    }
    bool applyDecision(Decision dec,int step,vector<pair<int,int>>&myPath){
        int posIdx=min(step,(int)myPath.size()-1);
        auto cur=myPath[posIdx];
        auto ins=[&](pair<int,int> c){
            if(posIdx+1<=(int)myPath.size())
                myPath.insert(myPath.begin()+posIdx+1,c);
            else myPath.push_back(c);
        };
        if(dec==Decision::WAIT){ins(cur); return true;}
        if(dec==Decision::STEPBACK){
            if(posIdx>0){
                auto prev=myPath[posIdx-1];
                ins(validCell(prev.first,prev.second)?prev:cur);
                stepBackActive=false;
            } else {ins(cur);}
            return true;
        }
        return false;
    }
    Decision tick(int step,vector<vector<pair<int,int>>>&paths,
                  const vector<RobotBehavior>&robots,bool verbose){
        Decision dec=decideAction(step,paths,robots,verbose);
        applyDecision(dec,step,paths[id]);
        return dec;
    }
    // [BUG-SMOOTH-JUMP FIX] visF is clamped cosmetic lead/lag, never overshoots 1.0
    void updateSmooth(int step,const vector<pair<int,int>>&path,
                      float stepFrac,float dt){
        int curIdx=min(step,  (int)path.size()-1);
        int nxtIdx=min(step+1,(int)path.size()-1);
        float lead=(speedFactor-SPEED_BASE)/SPEED_MAX_GLOBAL*0.15f;
        float visF=max(0.f,min(1.f,stepFrac+lead));
        float cx=path[curIdx].second*CELL+CELL/2.f;
        float cy=path[curIdx].first *CELL+CELL/2.f;
        float nx2=path[nxtIdx].second*CELL+CELL/2.f;
        float ny2=path[nxtIdx].first *CELL+CELL/2.f;
        float ease=visF*visF*(3.f-2.f*visF);
        px=cx+(nx2-cx)*ease;
        py=cy+(ny2-cy)*ease;
        float dx2=nx2-cx, dy2=ny2-cy;
        if(fabsf(dx2)+fabsf(dy2)>0.5f){
            float desired=(fabsf(dy2)>fabsf(dx2))?((dy2>0)?90.f:270.f):((dx2>0)?0.f:180.f);
            float diff=desired-angle;
            while(diff>180.f)diff-=360.f; while(diff<-180.f)diff+=360.f;
            angle+=diff*min(1.f,dt*14.f);
        }
    }
    void printStats()const{
        printf("  %-14s  base=%.2f vic=%d/%d  waits=%3d(behav=%3d)  deviation=%3d\n",
               ROBOT_NAMES[id],baseSpeed,vicMin,vicMax,
               waitCount,behavWaitCount,deviationCells);
    }
};

/* ================================================================ FORWARDS */
static vector<int> planningOrder(const vector<pair<int,int>>&,
                                 const vector<pair<int,int>>&,
                                 const set<int>&);
static bool allAtGoals(int,const vector<vector<pair<int,int>>>&,
                       const vector<pair<int,int>>&);
static void replanAll(int,const vector<pair<int,int>>&,
                      vector<vector<pair<int,int>>>&,int&,bool,bool);
static bool coopYield(int,bool,vector<vector<pair<int,int>>>&,
                      const vector<pair<int,int>>&,
                      const vector<pair<int,int>>&,bool);
static void resolveYield(int,const vector<pair<int,int>>&,
                         const vector<pair<int,int>>&,
                         vector<vector<pair<int,int>>>&,bool);

/* ================================================================ resolveConflicts */
static void resolveConflicts(
        int step,vector<vector<pair<int,int>>>&paths,
        const vector<pair<int,int>>&goals,
        vector<RobotBehavior>&robots,int&maxLen,bool verbose){
    bool needsGlobalReplan=false;
    vector<Decision> dec(ROBOTS,Decision::PROCEED);
    for(int r=0;r<ROBOTS;r++){
        int idx=min(step,(int)paths[r].size()-1);
        if(paths[r][idx]==goals[r]) dec[r]=Decision::PROCEED;
        else dec[r]=robots[r].tick(step,paths,robots,verbose);
    }
    for(int r=0;r<ROBOTS;r++){
        if(dec[r]!=Decision::DEVIATE) continue;
        int posIdx=min(step,(int)paths[r].size()-1);
        vector<pair<int,int>> oldFull=paths[r];
        erasePathReservations(oldFull);
        auto newSeg=timeExpandedAStar(paths[r][posIdx],goals[r],false);
        if((int)newSeg.size()>1){
            paths[r].resize(posIdx);
            for(auto&c:newSeg) paths[r].push_back(c);
            reservePath(paths[r]); needsGlobalReplan=true;
        } else {paths[r]=oldFull; reservePath(paths[r]);}
    }
    int lookWindow=min((int)paths[0].size(),step+LOOKAHEAD+2);
    CollisionReport cr=detectCollisions(paths,step+1,lookWindow,false);
    if(cr.hasCollision){
        vector<pair<float,int>> ranked;
        for(int r:cr.affectedRobots)
            ranked.push_back({robots[r].computePriority(step,paths),r});
        sort(ranked.begin(),ranked.end());
        for(size_t i=1;i<ranked.size();i++){
            int loser=ranked[i].second;
            int posIdx=min(step,(int)paths[loser].size()-1);
            auto cur2=paths[loser][posIdx];
            vector<pair<int,int>> oldPath=paths[loser];
            bool sb=false;
            if(posIdx>0){
                auto prev=paths[loser][posIdx-1];
                if(validCell(prev.first,prev.second)&&prev!=cur2&&
                   !vertexRes[step+1].count(prev)){
                    if(posIdx+1<=(int)paths[loser].size())
                        paths[loser].insert(paths[loser].begin()+posIdx+1,prev);
                    else paths[loser].push_back(prev);
                    sb=true;
                }
            }
            if(!sb){
                if(posIdx+1<=(int)paths[loser].size())
                    paths[loser].insert(paths[loser].begin()+posIdx+1,cur2);
                else paths[loser].push_back(cur2);
                robots[loser].waitCountdown=1; robots[loser].waitCount++;
            }
            erasePathReservations(oldPath); reservePath(paths[loser]);
        }
    }
    if(needsGlobalReplan) replanAll(step,goals,paths,maxLen,verbose,false);
    for(auto&p:paths) maxLen=max(maxLen,(int)p.size());
}

/* ================================================================ PLANNER */
static vector<int> planningOrder(const vector<pair<int,int>>&pos,
        const vector<pair<int,int>>&goals,const set<int>&overrides={}){
    vector<int> ol(overrides.begin(),overrides.end());
    vector<tuple<int,int,int>> di;
    for(int r=0;r<ROBOTS;r++){
        if(overrides.count(r)) continue;
        int d=abs(pos[r].first-goals[r].first)+abs(pos[r].second-goals[r].second);
        di.push_back({d,(r>=4)?0:1,r});
    }
    sort(di.begin(),di.end(),[](auto&a,auto&b){
        if(get<0>(a)!=get<0>(b)) return get<0>(a)>get<0>(b);
        if(get<1>(a)!=get<1>(b)) return get<1>(a)<get<1>(b);
        return get<2>(a)<get<2>(b);
    });
    for(auto&[d,g,r]:di) ol.push_back(r);
    return ol;
}
static bool allAtGoals(int step,const vector<vector<pair<int,int>>>&paths,
                       const vector<pair<int,int>>&goals){
    for(int r=0;r<ROBOTS;r++){
        int t=min(step,(int)paths[r].size()-1);
        if(paths[r][t]!=goals[r]) return false;
    }
    return true;
}

// [BUG-ASSEMBLY + BUG-FULLREPLAN FIX]
// Pads short paths by waiting at goal (not oscillating).
// Clears and re-reserves ALL robots so at-goal reservations are correct.
static vector<vector<pair<int,int>>> assembleAndReserve(
        int fromStep,
        const vector<vector<pair<int,int>>>&oldPaths,
        const vector<vector<pair<int,int>>>&segs,
        const vector<pair<int,int>>&goals){
    vector<vector<pair<int,int>>> cand(ROBOTS);
    for(int r=0;r<ROBOTS;r++){
        int hl=min(fromStep+1,(int)oldPaths[r].size());
        for(int t=0;t<hl;t++) cand[r].push_back(oldPaths[r][t]);
        for(int t=1;t<(int)segs[r].size();t++) cand[r].push_back(segs[r][t]);
    }
    int maxLen=0;
    for(auto&p:cand) maxLen=max(maxLen,(int)p.size());
    // [BUG-ASSEMBLY FIX] pad by waiting at goal only
    for(int r=0;r<ROBOTS;r++){
        auto padCell=goals[r];
        while((int)cand[r].size()<maxLen) cand[r].push_back(padCell);
    }
    // [BUG-FULLREPLAN FIX] full clear + rebuild
    vertexRes.clear(); edgeRes.clear(); cellLastReserved.clear();
    for(int r=0;r<ROBOTS;r++) reservePath(cand[r]);
    return cand;
}

// [BUG-REPLAN-SKIP FIX] forceAll=true replans every non-goal robot
static void replanAll(int fromStep,const vector<pair<int,int>>&goals,
                      vector<vector<pair<int,int>>>&paths,
                      int&maxLen,bool verbose,bool forceAll){
    if(!paths.empty()&&allAtGoals(fromStep,paths,goals)){
        if(verbose) printf(" [REPLAN SKIP] all at goals\n"); return;
    }
    vector<vector<pair<int,int>>> origPaths=paths;
    vector<pair<int,int>> cp(ROBOTS);
    for(int r=0;r<ROBOTS;r++) cp[r]=paths[r][min(fromStep,(int)paths[r].size()-1)];

    vector<bool> nr(ROBOTS,false); int aff=0;
    for(int r=0;r<ROBOTS;r++){
        if(cp[r]==goals[r]) continue;
        if(forceAll){nr[r]=true; aff++;}
        else{
            for(int t=fromStep;t<(int)paths[r].size();t++){
                auto c=paths[r][t];
                if(grid[c.first][c.second]!=0){nr[r]=true;aff++;break;}
            }
        }
    }
    if(aff==0){if(verbose) printf(" [REPLAN SKIP] no robots affected\n"); return;}

    if(verbose){
        printf("\n=== REPLAN @ step %d forceAll=%d ===\n",fromStep,(int)forceAll);
        for(int r=0;r<ROBOTS;r++){
            const char*tag=(cp[r]==goals[r])?"[GOAL]":nr[r]?"[REPLAN]":"[KEEP]";
            printf("  %s@(%d,%d) %s\n",ROBOT_NAMES[r],cp[r].first,cp[r].second,tag);
        }
    }

    set<int> conf;
    vector<vector<pair<int,int>>> best;

    for(int att=0;att<MAX_REPLAN_ATTEMPTS;att++){
        vertexRes.clear(); edgeRes.clear(); cellLastReserved.clear();
        for(int r=0;r<ROBOTS;r++) vertexRes[0].insert(cp[r]);
        rebuildCellLastReserved();
        vector<int> order;
        if(att==0){
            set<int> aff2; for(int r=0;r<ROBOTS;r++) if(nr[r]) aff2.insert(r);
            order=planningOrder(cp,goals,aff2);
        } else {
            set<int> rs; for(int r:conf) if(nr[r]) rs.insert(r);
            vector<int> cv(rs.begin(),rs.end());
            shuffle(cv.begin(),cv.end(),default_random_engine((unsigned)(time(0)+(unsigned)att*999983u)));
            auto rest=planningOrder(cp,goals,rs);
            order=cv; for(int r:rest) order.push_back(r);
        }
        vector<vector<pair<int,int>>> ns(ROBOTS);
        for(int r:order){
            if(cp[r]==goals[r]||!nr[r]) continue;
            bool onObs=(grid[cp[r].first][cp[r].second]==1);
            ns[r]=timeExpandedAStar(cp[r],goals[r],onObs);
            if((int)ns[r].size()<=1){
                if(!coopYield(r,onObs,ns,cp,goals,verbose&&att==0)){
                    ns[r]={cp[r]}; for(int i=0;i<3;i++) ns[r].push_back(cp[r]);
                }
            }
            reservePath(ns[r]);
        }
        for(int r=0;r<ROBOTS;r++){
            if(cp[r]==goals[r]){
                ns[r]={cp[r]}; for(int i=0;i<GOAL_HOLD_STEPS;i++) ns[r].push_back(cp[r]);
                reservePath(ns[r]); continue;
            }
            if(nr[r]) continue;
            ns[r].clear();
            for(int t=fromStep;t<(int)paths[r].size();t++) ns[r].push_back(paths[r][t]);
            reservePath(ns[r]);
        }
        auto cand=assembleAndReserve(fromStep,paths,ns,goals);
        auto cr=detectCollisions(cand,fromStep,-1,false);
        if(!cr.hasCollision){
            paths=cand; maxLen=(int)cand[0].size();
            if(verbose) printf(" CF(att%d) maxLen=%d\n=== REPLAN done ===\n\n",att+1,maxLen);
            return;
        }
        for(int r:cr.affectedRobots){conf.insert(r); nr[r]=true;}
        if(best.empty()) best=ns;
    }

    // Yield pass
    vertexRes.clear(); edgeRes.clear(); cellLastReserved.clear();
    for(int r=0;r<ROBOTS;r++) vertexRes[0].insert(cp[r]);
    rebuildCellLastReserved();
    vector<vector<pair<int,int>>> fs(ROBOTS);
    auto fo=planningOrder(cp,goals);
    for(int r:fo){
        if(cp[r]==goals[r]||!nr[r]) continue;
        bool onObs=(grid[cp[r].first][cp[r].second]==1);
        fs[r]=timeExpandedAStar(cp[r],goals[r],onObs);
        if((int)fs[r].size()<=1){
            if(!coopYield(r,onObs,fs,cp,goals,verbose)){
                fs[r]={cp[r]}; for(int i=0;i<3;i++) fs[r].push_back(cp[r]);
            }
        }
        reservePath(fs[r]);
    }
    for(int r:fo){
        if(cp[r]==goals[r]){
            fs[r]={cp[r]}; for(int i=0;i<GOAL_HOLD_STEPS;i++) fs[r].push_back(cp[r]);
            reservePath(fs[r]); continue;
        }
        if(nr[r]) continue;
        fs[r].clear(); for(int t=fromStep;t<(int)paths[r].size();t++) fs[r].push_back(paths[r][t]);
        reservePath(fs[r]);
    }
    resolveYield(fromStep,goals,cp,fs,verbose);
    auto fc=assembleAndReserve(fromStep,paths,fs,goals);
    if(!detectCollisions(fc,fromStep,-1,false).hasCollision){
        paths=fc; maxLen=(int)fc[0].size();
        if(verbose) printf(" CF(yield) maxLen=%d\n=== REPLAN done ===\n\n",maxLen);
        return;
    }

    // Extended search
    int ext=min(MAX_TIME*2,MAX_TIME+120);
    vertexRes.clear(); edgeRes.clear(); cellLastReserved.clear();
    for(int r=0;r<ROBOTS;r++) vertexRes[0].insert(cp[r]);
    rebuildCellLastReserved();
    vector<vector<pair<int,int>>> xs(ROBOTS);
    for(int r=0;r<ROBOTS;r++){
        bool onObs=(grid[cp[r].first][cp[r].second]==1);
        xs[r]=timeExpandedAStar(cp[r],goals[r],onObs,ext);
        if((int)xs[r].size()<=1){xs[r]={cp[r]}; for(int i=0;i<3;i++) xs[r].push_back(cp[r]);}
        reservePath(xs[r]);
    }
    auto xc=assembleAndReserve(fromStep,paths,xs,goals);
    if(!detectCollisions(xc,fromStep,-1,false).hasCollision){
        paths=xc; maxLen=(int)xc[0].size();
        if(verbose) printf(" CF(ext) maxLen=%d\n=== REPLAN done ===\n\n",maxLen);
        return;
    }
    if(!best.empty()){
        auto bc=assembleAndReserve(fromStep,paths,best,goals);
        if(!detectCollisions(bc,fromStep,-1,false).hasCollision){
            paths=bc; maxLen=(int)bc[0].size(); return;
        }
    }
    if(!detectCollisions(origPaths,fromStep,-1,false).hasCollision){
        paths=origPaths; maxLen=(int)origPaths[0].size(); return;
    }
    if(verbose) printf(" [REPLAN FAILED]\n");
}

static void resolveYield(int,const vector<pair<int,int>>&goals,
                         const vector<pair<int,int>>&cur,
                         vector<vector<pair<int,int>>>&ns,bool verbose){
    for(int pass=0;pass<ROBOTS;pass++){
        bool anyStuck=false;
        for(int s=0;s<ROBOTS;s++){
            if(cur[s]==goals[s]||(int)ns[s].size()>1) continue;
            anyStuck=true;
            if(grid[cur[s].first][cur[s].second]==1){
                ns[s]={cur[s]};
                pair<int,int> ss=cur[s];
                for(auto[dx,dy]:vector<pair<int,int>>{{1,0},{-1,0},{0,1},{0,-1}}){
                    int nx=ss.first+dx,ny=ss.second+dy;
                    if(nx>=0&&ny>=0&&nx<ROWS&&ny<COLS&&validCell(nx,ny)&&!vertexRes[1].count({nx,ny})){
                        ss={nx,ny}; break;
                    }
                }
                for(int i=0;i<YIELD_WAIT_STEPS;i++) ns[s].push_back(ss);
                reservePath(ns[s]); continue;
            }
            bool res=false;
            for(int b=0;b<ROBOTS&&!res;b++){
                if(b==s||cur[b]!=goals[b]) continue;
                erasePathReservations(ns[b]);
                auto tp=timeExpandedAStar(cur[s],goals[s],false);
                if((int)tp.size()<=1){
                    ns[b]={goals[b]}; for(int i=0;i<GOAL_HOLD_STEPS;i++) ns[b].push_back(goals[b]);
                    reservePath(ns[b]); continue;
                }
                pair<int,int> yc={-1,-1};
                for(auto[dx,dy]:vector<pair<int,int>>{{1,0},{-1,0},{0,1},{0,-1}}){
                    int nx=goals[b].first+dx,ny=goals[b].second+dy;
                    if(!validCell(nx,ny)||vertexRes[1].count({nx,ny})) continue;
                    yc={nx,ny}; break;
                }
                if(yc.first==-1){
                    ns[b]={goals[b]}; for(int i=0;i<GOAL_HOLD_STEPS;i++) ns[b].push_back(goals[b]);
                    reservePath(ns[b]); continue;
                }
                vector<pair<int,int>> yp={goals[b],yc};
                for(int i=0;i<YIELD_WAIT_STEPS;i++) yp.push_back(yc);
                yp.push_back(goals[b]);
                ns[b]=yp; reservePath(yp);
                ns[s]=timeExpandedAStar(cur[s],goals[s],false);
                if((int)ns[s].size()>1){reservePath(ns[s]); res=true;}
                else{erasePathReservations(yp); ns[b]={goals[b]}; reservePath(ns[b]);}
            }
            if(!res){
                ns[s]={cur[s]}; for(int i=0;i<YIELD_WAIT_STEPS;i++) ns[s].push_back(cur[s]);
                reservePath(ns[s]);
            }
        }
        if(!anyStuck) break;
    }
}

static bool coopYield(int r,bool onObs,vector<vector<pair<int,int>>>&segs,
                      const vector<pair<int,int>>&cp,
                      const vector<pair<int,int>>&goals,bool verbose){
    for(int b=0;b<ROBOTS;b++){
        if(b==r||segs[b].empty()||cp[b]!=goals[b]) continue;
        erasePathReservations(segs[b]);
        auto tp=timeExpandedAStar(cp[r],goals[r],onObs);
        if((int)tp.size()<=1){
            segs[b]={goals[b]}; for(int i=0;i<GOAL_HOLD_STEPS;i++) segs[b].push_back(goals[b]);
            reservePath(segs[b]); continue;
        }
        pair<int,int> yc={-1,-1};
        for(auto[dx,dy]:vector<pair<int,int>>{{1,0},{-1,0},{0,1},{0,-1}}){
            int nx=goals[b].first+dx,ny=goals[b].second+dy;
            if(!validCell(nx,ny)||vertexRes[1].count({nx,ny})) continue;
            yc={nx,ny}; break;
        }
        if(yc.first==-1){
            segs[b]={goals[b]}; for(int i=0;i<GOAL_HOLD_STEPS;i++) segs[b].push_back(goals[b]);
            reservePath(segs[b]); continue;
        }
        int wd=abs(cp[r].first-goals[b].first)+abs(cp[r].second-goals[b].second);
        int w=max(1,min(3,wd));
        vector<pair<int,int>> yp={goals[b]}; for(int i=0;i<w;i++) yp.push_back(yc);
        yp.push_back(goals[b]);
        segs[b]=yp; reservePath(yp);
        segs[r]=timeExpandedAStar(cp[r],goals[r],onObs);
        if((int)segs[r].size()>1){
            if(verbose) printf(" [COOP-YIELD] %s->%s\n",ROBOT_NAMES[b],ROBOT_NAMES[r]);
            return true;
        }
        erasePathReservations(yp); segs[b]={goals[b]}; reservePath(segs[b]); segs[r]={cp[r]};
    }
    return false;
}

static bool lookaheadBlocked(int step,const vector<vector<pair<int,int>>>&paths){
    int ml=(int)paths[0].size();
    for(int r=0;r<ROBOTS;r++)
        for(int a=0;a<=LOOKAHEAD;a++){
            int t=min(step+a,ml-1);
            auto[x,y]=paths[r][t];
            if(grid[x][y]==1) return true;
        }
    return false;
}
static bool isRobotAt(int x,int y,int step,const vector<vector<pair<int,int>>>&paths){
    for(int r=0;r<ROBOTS;r++){
        int t=min(step,(int)paths[r].size()-1);
        if(paths[r][t].first==x&&paths[r][t].second==y) return true;
    }
    return false;
}

/* ================================================================ RENDER */
static void renderRobot(int r,float px,float py,float ang,
                        pair<int,int> cur,pair<int,int> goal,
                        const vector<pair<int,int>>&pr,
                        const RobotBehavior&rb,Color col){
    bool stuck=((int)pr.size()<=1&&cur!=goal);
    if(stuck) DrawRectangleLinesEx({px-CELL*0.38f,py-CELL*0.28f,CELL*0.76f,CELL*0.56f},3,RED);
    DrawRectanglePro({px-CELL*0.3f,py-CELL*0.2f,CELL*0.6f,CELL*0.4f},{CELL*0.3f,CELL*0.2f},ang,col);
    string lbl="R"+to_string(r);
    int fs=16;
    DrawText(lbl.c_str(),(int)(px-MeasureText(lbl.c_str(),fs)/2.f),(int)(py-fs/2.f-2),fs,BLACK);
    DrawText(("("+to_string(cur.first)+","+to_string(cur.second)+")").c_str(),(int)(px-22),(int)(py+11),10,BLACK);
    string info=string(decisionName(rb.lastDecision))+" W:"+to_string(rb.waitCount)+" D:"+to_string(rb.deviationCells);
    DrawText(info.c_str(),(int)(px-MeasureText(info.c_str(),8)/2.f),(int)(py+22),8,DARKBLUE);
}

static void printRethink(const vector<RobotBehavior>&robots){
    printf("\n+----------------------------------------------+\n");
    printf("|   FINAL STATS (all bugs fixed edition)      |\n");
    printf("+----------------------------------------------+\n");
    for(auto&rb:robots) rb.printStats();
    int tw=0,tbw=0,td=0;
    for(auto&rb:robots){tw+=rb.waitCount;tbw+=rb.behavWaitCount;td+=rb.deviationCells;}
    printf("  TOTALS  waits=%d(behav=%d)  deviation=%d\n",tw,tbw,td);
    printf("+----------------------------------------------+\n\n");
}

/* ================================================================ OBS HELPERS */
// [BUG-OBS-TIMING FIX] always calls replanAll(forceAll=true) — no guard
static bool tryPlaceObs(int&opc,vector<pair<int,int>>&obs,
        const function<bool(int,int)>&prot,const function<bool(int,int)>&rob,
        int step,vector<vector<pair<int,int>>>&paths,int&ml,
        const vector<pair<int,int>>&goals,bool verbose){
    for(int a=0;a<3000;a++){
        int x=rand()%ROWS,y=rand()%COLS;
        if(grid[x][y]==0&&!prot(x,y)&&!rob(x,y)){
            grid[x][y]=1; obs.push_back({x,y}); opc++;
            printf("[OBS #%d] at (%d,%d) step=%d\n",opc,x,y,step);
            replanAll(step,goals,paths,ml,verbose,true);
            return true;
        }
    }
    printf("[OBS] no free cell\n"); return false;
}
static void tryRelocObs(int&rc,vector<pair<int,int>>&obs,
        const function<bool(int,int)>&prot,const function<bool(int,int)>&rob,
        int step,vector<vector<pair<int,int>>>&paths,int&ml,
        const vector<pair<int,int>>&goals,bool verbose){
    int idx=rc%(int)obs.size();
    auto old=obs[idx];
    printf("\n[RELOC #%d] (%d,%d)->",rc,old.first,old.second);
    grid[old.first][old.second]=0;
    bool moved=false;
    for(int a=0;a<3000;a++){
        int nx=rand()%ROWS,ny=rand()%COLS;
        if((nx!=old.first||ny!=old.second)&&grid[nx][ny]==0&&!prot(nx,ny)&&!rob(nx,ny)){
            grid[nx][ny]=1; obs[idx]={nx,ny};
            printf("(%d,%d)\n",nx,ny); moved=true; break;
        }
    }
    if(!moved){grid[old.first][old.second]=1; printf("kept\n");}
    replanAll(step,goals,paths,ml,verbose,true);
    rc++;
}

/* ================================================================ MAIN */
int main(){
    srand((unsigned)time(nullptr));
    vector<pair<int,int>> startPos={
        {0,1},{0,2},{0,3},{0,4},
        {1,0},{2,0},{3,0},{4,0}
    };
    vector<pair<int,int>> goalPos={
        {5,1},{5,2},{5,3},{5,4},
        {1,5},{2,5},{3,5},{4,5}
    };
    auto isProtSt=[&](int x,int y){
        for(auto&s:startPos) if(s.first==x&&s.second==y) return true;
        for(auto&g:goalPos)  if(g.first==x&&g.second==y) return true;
        if((x==0&&y==0)||(x==0&&y==COLS-1)||
           (x==ROWS-1&&y==0)||(x==ROWS-1&&y==COLS-1)) return true;
        return false;
    };
    for(int i=0;i<ROWS;i++) for(int j=0;j<COLS;j++) grid[i][j]=0;

    int modeInput,n;
    printf("Enter mode (1=dynamic obstacles+relocate, 2=random-goals+dynamic): ");
    scanf("%d",&modeInput);
    printf("Enter number of obstacles: ");
    scanf("%d",&n);
    n=max(0,n);

    InitWindow(COLS*CELL,ROWS*CELL+60,"Multi-Robot Pathfinding (Fixed)");
    SetTargetFPS(60);
    Color colors[ROBOTS]={RED,BLUE,GREEN,ORANGE,PURPLE,BROWN,DARKGREEN,MAROON};
    vector<pair<int,int>> obstacles;

    auto runSim=[&](vector<pair<int,int>>&cg,int modeLabel){
        // [BUG-OBS FIX] Compute interval so ALL n obstacles fire inside
        // the first OBS_END_FRAC fraction of the estimated simulation window.
        double simWindow=EST_SIM_STEPS*TPS*OBS_END_FRAC; // seconds
        double obsInterval=(n>0)?(simWindow/n):999999.0;
        obsInterval=max(2.5,obsInterval); // never faster than one every 2.5 s
        // Obstacle i fires when simElapsed >= (i+1)*obsInterval  (i=0..n-1)
        int obsPlaced=0;
        double nextRelocAt=simWindow+5.0;
        int rc=0;

        double wallStart=GetTime(), simStart=0.0;
        bool started=false;
        int step=0, maxLen=1;
        bool ready=false;
        int lbh=-1, lp=-1;
        bool finalPrinted=false, origFrozen=false;
        int lastObsReplanStep=-1;
        bool replanFlash=false; double replanFlashT=0.0;

        double prevFrameT=GetTime();
        float stepFrac=0.f;

        vector<vector<pair<int,int>>> paths(ROBOTS);
        vector<RobotBehavior> robots;
        for(int r=0;r<ROBOTS;r++) robots.emplace_back(r,startPos[r],cg[r]);

        auto isRH=[&](int x,int y)->bool{return isRobotAt(x,y,step,paths);};
        auto isPr=[&](int x,int y){return isProtSt(x,y);};

        printf("\n=== MODE %d : %d obstacles ===\n",modeLabel,n);
        if(n>0) printf("    obsInterval=%.2fs  window=%.2fs\n",obsInterval,simWindow);

        while(!WindowShouldClose()){
            double now=GetTime();
            float dt=(float)(now-prevFrameT);
            prevFrameT=now;
            dt=min(dt,0.1f);

            if(!started&&now-wallStart>=SIM_START_DELAY){
                started=true; simStart=now;
                vertexRes.clear(); edgeRes.clear(); cellLastReserved.clear();
                for(int r=0;r<ROBOTS;r++) vertexRes[0].insert(startPos[r]);
                rebuildCellLastReserved();
                for(int r=0;r<ROBOTS;r++){
                    paths[r]=timeExpandedAStar(startPos[r],cg[r],false);
                    reservePath(paths[r]);
                }
                maxLen=0;
                for(auto&p:paths) maxLen=max(maxLen,(int)p.size());
                for(auto&p:paths) while((int)p.size()<maxLen) p.push_back(p.back());
                vertexRes.clear(); edgeRes.clear(); cellLastReserved.clear();
                for(int r=0;r<ROBOTS;r++) reservePath(paths[r]);
                ready=true; stepFrac=0.f;
            }

            if(started&&ready){
                double simElapsed=now-simStart;

                // [BUG-OBS FIX] fire obstacle i when elapsed >= (i+1)*interval
                if(obsPlaced<n && simElapsed>=(obsPlaced+1)*obsInterval){
                    tryPlaceObs(obsPlaced,obstacles,isPr,isRH,
                                step,paths,maxLen,cg,false);
                    lastObsReplanStep=step; // suppress redundant lookahead this step
                    replanFlash=true; replanFlashT=now;
                }
                // Relocate after all placed
                if(obsPlaced>=n&&n>0&&!obstacles.empty()&&simElapsed>=nextRelocAt){
                    tryRelocObs(rc,obstacles,isPr,isRH,step,paths,maxLen,cg,false);
                    nextRelocAt=simElapsed+12.0;
                    lastObsReplanStep=step;
                    replanFlash=true; replanFlashT=now;
                }
                // [BUG-OBS-TIMING FIX] lookahead fires only when no placement this step
                if(lastObsReplanStep!=step&&lookaheadBlocked(step,paths)){
                    replanAll(step,cg,paths,maxLen,false,true);
                    lastObsReplanStep=step;
                    replanFlash=true; replanFlashT=now;
                }
                // Behavior once per logical step
                if(step!=lbh){
                    resolveConflicts(step,paths,cg,robots,maxLen,false);
                    lbh=step;
                    if(!origFrozen){
                        for(int r=0;r<ROBOTS;r++) robots[r].setOriginalPath(paths[r]);
                        origFrozen=true;
                    }
                }
                // Console log
                if(step!=lp){
                    printf("Step %d:",step);
                    for(int r=0;r<ROBOTS;r++){
                        int t=min(step,(int)paths[r].size()-1);
                        printf("  R%d@(%d,%d)%s",r,paths[r][t].first,paths[r][t].second,
                               (paths[r][t]==cg[r])?"[G]":"");
                    }
                    printf("\n"); lp=step;
                }
                // Advance step fraction
                stepFrac+=dt*(1.f/(float)TPS);
                if(stepFrac>=1.f&&step<maxLen-1){
                    step++; stepFrac=0.f;
                    for(int r=0;r<ROBOTS;r++){
                        int t=min(step,(int)paths[r].size()-1);
                        robots[r].trackStep(step,paths[r][t]);
                    }
                }
                if(stepFrac>1.f) stepFrac=1.f;
                if(!finalPrinted&&allAtGoals(step,paths,cg)){
                    printRethink(robots); finalPrinted=true;
                }
            }

            // ==================== RENDER ====================
            BeginDrawing();
            ClearBackground(RAYWHITE);
            for(int i=0;i<ROWS;i++) for(int j=0;j<COLS;j++){
                if(grid[i][j]) DrawRectangle(j*CELL,i*CELL,CELL,CELL,DARKGRAY);
                DrawRectangleLines(j*CELL,i*CELL,CELL,CELL,LIGHTGRAY);
            }
            for(int r=0;r<ROBOTS;r++){
                Color gc=colors[r]; gc.a=60;
                DrawRectangle(cg[r].second*CELL+2,cg[r].first*CELL+2,CELL-4,CELL-4,gc);
                DrawText(("G"+to_string(r)).c_str(),cg[r].second*CELL+4,cg[r].first*CELL+4,13,colors[r]);
            }
            for(int r=0;r<ROBOTS;r++){
                if(!started||!ready){
                    robots[r].px=(float)(startPos[r].second*CELL+CELL/2);
                    robots[r].py=(float)(startPos[r].first *CELL+CELL/2);
                    robots[r].angle=0.f;
                    renderRobot(r,robots[r].px,robots[r].py,robots[r].angle,
                                startPos[r],cg[r],{startPos[r]},robots[r],colors[r]);
                    continue;
                }
                robots[r].updateSmooth(step,paths[r],stepFrac,dt);
                int curIdx=min(step,(int)paths[r].size()-1);
                renderRobot(r,robots[r].px,robots[r].py,robots[r].angle,
                            paths[r][curIdx],cg[r],paths[r],robots[r],colors[r]);
            }
            DrawRectangle(0,ROWS*CELL,COLS*CELL,60,Fade(BLACK,0.80f));
            if(!started){
                int cd=(int)(SIM_START_DELAY-(now-wallStart))+1;
                DrawText(("Mode "+to_string(modeLabel)+" | Obs="+to_string(n)+
                    " | Start in "+to_string(max(0,cd))+"s").c_str(),
                    6,ROWS*CELL+6,15,WHITE);
            } else {
                bool done=allAtGoals(step,paths,cg);
                if(done) DrawText("ALL AT GOALS — see console",6,ROWS*CELL+6,15,GREEN);
                else{
                    string msg="Step "+to_string(step)+"/"+to_string(maxLen-1)
                              +" | Obs:"+to_string(obsPlaced)+"/"+to_string(n)
                              +" | Relocs:"+to_string(rc);
                    DrawText(msg.c_str(),6,ROWS*CELL+6,15,WHITE);
                    if(replanFlash&&now-replanFlashT<1.0) DrawText("REPLAN",340,ROWS*CELL+6,15,YELLOW);
                    else replanFlash=false;
                }
                for(int r=0;r<ROBOTS;r++){
                    float sx=4.f+r*(COLS*CELL/(float)ROBOTS);
                    DrawText((to_string(r)+":"+string(decisionName(robots[r].lastDecision))
                             +" W"+to_string(robots[r].waitCount)
                             +" D"+to_string(robots[r].deviationCells)).c_str(),
                             (int)sx,ROWS*CELL+34,10,colors[r]);
                }
            }
            EndDrawing();
        }
    }; // end runSim

    if(modeInput==1){
        runSim(goalPos,1);
    } else {
        vector<pair<int,int>> rp={{5,1},{5,2},{5,3},{5,4}};
        vector<pair<int,int>> cp2={{1,5},{2,5},{3,5},{4,5}};
        mt19937 rng((unsigned)time(nullptr));
        shuffle(rp.begin(),rp.end(),rng);
        shuffle(cp2.begin(),cp2.end(),rng);
        vector<pair<int,int>> rgoal;
        for(int i=0;i<4;i++) rgoal.push_back(rp[i]);
        for(int i=0;i<4;i++) rgoal.push_back(cp2[i]);
        {
            set<pair<int,int>> ug(rgoal.begin(),rgoal.end());
            assert((int)ug.size()==ROBOTS&&"Mode 2: duplicate goal");
        }
        printf("\n=== MODE 2 : Random Goals ===\n");
        for(int r=0;r<ROBOTS;r++)
            printf(" %s: (%d,%d)->(%d,%d)\n",ROBOT_NAMES[r],
                   startPos[r].first,startPos[r].second,
                   rgoal[r].first,rgoal[r].second);
        runSim(rgoal,2);
    }
    CloseWindow();
    return 0;
}
