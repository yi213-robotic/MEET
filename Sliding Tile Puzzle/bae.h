/*************************************************
 * Meet in middle in C++
 * **********************************************/
 //Author: Yi Wang
#ifndef BAE_H
#define BAE_H
#include "optimalSearchDefinition.h"
#include "stp.h"
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

class bae {
public:
    bae();
    ~bae();
    void baeSEARCH(double &time, int &forwardExpandedN_, int &backwardExpandedN_, array<uint8_t,16> start_, array<uint8_t,16> startr_, array<uint8_t,16> startc_, array<uint8_t,16> goalr_ , array<uint8_t,16> goalc_, array<uint8_t,16> goal_, int &pathLength, int &OSME_, int &OPM_);

    moveTo inverseOf(moveTo a);
    void ExpansionInForward(State cur_, vector<State> suc);
    
    void ExpansionInBackward(State cur_, vector<State> suc);

    bool PropertyForward();

    void forwardSearch();

    void stateFormat(array<uint8_t, 16> tiles);
    unsigned long long generateKey(const array <uint8_t, 16>& tiles);
    
    int blankPos(const array<uint8_t, 16> & tiles);

    int count_inversions_small(const uint8_t seq[4], int n);   
   
    std::vector<State> successors(const State& state, moveTo pdir_);
   
    void addSuccessor(const State &state, vector<State> &suc_, moveTo a, int z, int nz);

    void getTargetPos(const array<uint8_t, 16> tar_, array<uint8_t, 16> &row_ , array<uint8_t, 16>  &col_);
  
    int MD(const array<uint8_t, 16> &tiles, const array<uint8_t, 16>& srow_ , const array<uint8_t, 16>&  scol_);

    int MD_LC(const std::array<uint8_t,16>& tiles, const std::array<uint8_t,16>& srow_, const std::array<uint8_t,16>& scol_);
    
    void conflictValue(int &h_val, const std::array<uint8_t,16>& tiles, const std::array<uint8_t,16>& srow_, const std::array<uint8_t,16>& scol_, bool row);
   
    void backwardSearch();
    /*******************Constraining, Modifying Propagating range***********/
    void generatingSuccessor();
    
   
    void Geting_Min_Successor(int sf_, int sg_, int sh_, bool digm_, bool Ms_ , int Gcur, int Hcur, int &lminF_, NGcost_ &val_, bool mt_, int &lminh_);
    /*********** Constraining Propagating range from surrounding neighors ***************/
    void UpdateVertex(EsCost &sucC, int g_value, unsigned long long keyV, State &Parent, State cur_, bool &update, State &suc, bool & hasp, array<uint8_t, 16> tiles);

    void leastCostFound(int leastCost_, State suc);

    void show_path();/********** Return vertices on an optimal anyangle path ****************/
private:
    
    vector<bool> grid_;
    vector<bool> blocks_;
    vector<vertex> vertex_;
    State forward_,backward_,intersect_,CurMS_;
    EsCost forw_,backw_;
    priority_queue<State, vector<State >, heapMin> OpenF;
    priority_queue<State, vector<State>, heapMinB> OpenB;
    State Startstate,Goalstate;
    NGcost_ sval_, nval_;
    std::unordered_map<unsigned long long, int> OpenListF_;
    std::unordered_map<unsigned long long, int> OpenListB_;
    unordered_map<State, pair<EsCost,EsCost>,HashState> STATE;
    unordered_map<State, State,HashState> parentF;
    unordered_map<State, State,HashState> parentB;
    unordered_map<unsigned long long, bool> hasF;
    unordered_map<unsigned long long, bool> hasB;
    bool skip_, forwardExpanded_, find_solution, curSolution_, findMs_, curMSDir_, potImprove_, hasChild_, hasSucc_,adf_, DI_, Ms_, MSt_;
    pair<pair<int,int>, pair<int,bool> > MS_, PMS_;
    double maxValue_, epsilon, infinity;
    array<uint8_t, 16> Ini_, Tar_, startR_, startC_, goalR_, goalC_;
    int U, minG_, lsb_, lnb_, OSME, OPM, lfmin_, ProVal_, curL_, lnh_, fminimal_, lminh_, maxG_;
    double L, C;
    unsigned long long start_, end_;
    int map_x, map_y, start_x, start_y, end_x, end_y, ForwardCounter_, BackwardCounter_, _ID,START,GOAL;
};
#endif
