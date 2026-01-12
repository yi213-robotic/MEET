/*************************************************
 * Meet
 * **********************************************/
#ifndef MEET_H
#define MEET_H
#include "optimalSearchDefinition.h"
#include "PancakePuzzle.h"
using namespace std;
#define PHI 1.07967
#define DigCost 1.41421356237
#define HALF_PI 1.57079632679
#define uniteCost 1

struct heapMin{ //using for Open list
   	bool operator()(State const& s1, State const& s2) {
        //cerr << "debuging here " << v1.l.x << " " << v1.l.y << ", " << v2.l.x << "  " << v2.l.y << " " << v1.f << " " << v2.f << endl; 
        
        if(s1.cf.f > s2.cf.f)
            return true;
        else if (s1.cf.f  < s2.cf.f)
            return false;
        else
        {
             if(s1.cf.g < s2.cf.g)
                return true;
             else if(s1.cf.g > s2.cf.g) 
                return false;
             else
                return s1.cf.h > s2.cf.h;         
        }
            //return s1.cf.g < s2.cf.g;// selects larger g-value for tire-breaking
    }
};

struct heapMinB{ //using for Open list
   	bool operator()(State const& s1, State const& s2) {
        //cerr << "debuging here " << v1.l.x << " " << v1.l.y << ", " << v2.l.x << "  " << v2.l.y << " " << v1.f << " " << v2.f << endl; 
        
        if(s1.cb.f > s2.cb.f)
            return true;
        else if (s1.cb.f < s2.cb.f)
            return false;
        else
        {
             if(s1.cb.g < s2.cb.g)
                return true;
             else if(s1.cb.g > s2.cb.g) 
                return false;
             else
                return s1.cb.h > s2.cb.h;  
        }    
    }
};

class meet {
public:
    meet();
    ~meet();
    bool TC1();
    bool TC6(int Gcur, int Hcur);
    

    void meetSEARCH(double &time, int &forwardExpandedN_, int &backwardExpandedN_,vector<int> start, vector<int> goal, int &pathLength, int &OSME_, int &OPM_, int &t1, int &t2, int &t3, int &t4, int &t5, int &t6);

    void ExpansionInForward(State cur_, vector<State> suc);
    
    void ExpansionInBackward(State cur_, vector<State> suc);
    bool TC5(int hCur_);

    void forwardSearch();

    bool stateExpansion();

    bool TC3(int gCur_, int hCur_); 
    bool TC4(int gCur_, int hCur_); 

    void minValueOps(int sf_, bool spi_);

    bool TC2(bool fpi_, bool bpi_, bool fmt_, bool  bmt_, int fg, int bg);
    void stateFormat(vector <int> st);
    unsigned long long generateKey(vector <int > ordering) ;
    
    int gapHeuristic(const State& state, int X_) ;
    std::vector<State> successors(const State& state);
   
    std::vector<State> predecessors(const State& state) ;

    void translateIn(vector<int> &s, vector<int > target);
    void backwardSearch();
    void getGFminF(int &gminF, int &fminF, int minf);
    void getGFminB(int &gminB, int &fminB, int minf);
    void generatingSuccessor();
    
    bool Eq(int gCur, int hCur);
    bool hEq(int gCur, int hCur);
    bool Terminate_(int sg_, int sh_);
    
    bool hElevation(int ex_g, int c_g);
    
    bool betterSolution(int sf_, int sg_, bool cpi_);

    bool TerminationCondition(int Gcur, int Hcur, bool pi_, bool PreExpansion);
    
    int CalculateH(vector<int> s, vector<int > target, int X_); 
   
    void flipOrdering(std::vector<State>& succs, std::vector< int> ordering, int loc) ;
    
    void getMinMetricValue_(int sf_, int sg_, int sh_, bool digm_, bool Ms_ , int Gcur, int Hcur, int &lminF_, NGcost_ &val_, int mt_);
 
    void UpdateVertex(EsCost &sucC, int g_value, unsigned long long keyV, State &Parent, State cur_, bool &update, State & suc, bool &hasP, bool &dig_);

    void leastCostFound(int leastCost_, State suc, int f_ ,int fg_, int bg_, int hc_, bool digm_, bool &firstS_, bool &lmet, int prtf_);

    void show_path();/********** Return vertices on an optimal anyangle path ****************/
private:
    
    vector<bool> grid_;
    vector<bool> blocks_;
    vector<vertex> vertex_;
    State fdir_,bdir_,Ib_,CurMS_;
    EsCost forw_,backw_;
    priority_queue<State, vector<State >, heapMin> OpenF;
    priority_queue<State, vector<State>, heapMinB> OpenB;
    State Startstate,Goalstate;
    NGcost_ sval_, IMV_;
    unordered_map<State, pair<EsCost,EsCost>,HashState> STATE;
    unordered_map<State, State,HashState> parentF;
    unordered_map<State, State,HashState> parentB;
    unordered_map<unsigned long long, bool> hasF;
    unordered_map<unsigned long long, bool> hasB;
    int fmin_, C_curr, Sfmin_, prtIbG_, prtIbH_, prtIpi_ ,IGMin_, Gcur_,  Hcur_, f_dMin, hmin_, f_min_, b_min_, _min_;
    bool skip_, fExpd_, find_solution, findASolution_, findMs_, IbDir_, hasChild_, hasSucc_,Spt_, Ih_, bSoluF_, bSoluB_, locSol_, pi_, p_i;
    bool TC1_,TC2_,TC3_,TC4_,TC5_,TC6_, locHep_,lSpt_, acrt_, arch_, sIh_, arh_, bpi_,prti_, ari_, dirEq_, dirSol_,preDf_,preDb_, c_bt;
    pair<pair<int,int>, pair<int,bool> > MS_, PMS_;
    int OSME, OPM, curL_, lnh_;
    unsigned long long start_, end_;
    int map_x, map_y, start_x, start_y, end_x, end_y, fCounter_, bCounter_, _ID,START,GOAL,PgV_,PhV_;
};
#endif
