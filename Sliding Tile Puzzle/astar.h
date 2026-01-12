/*************************************************
 * A star
 * **********************************************/
#ifndef astar_H
#define astar_H
#include "optimalSearchDefinition.h"
#include "stp.h"
using namespace std;
#define PHI 1.07967
#define MaxRavDist 1.41421356237
#define HALF_PI 1.57079632679

class astar {
public:
    astar();
    ~astar();
    void astar_(double &time, int &forwardExpanded_, int &backwardExpanded_, array<uint8_t,16> start_, array<uint8_t,16> startR_, array<uint8_t,16> startC_, array<uint8_t,16> goalR_ , array<uint8_t,16> goalC_, array<uint8_t,16> goal_, int &pathLength);
    
/************Find an optimal path********************/
void astar_();

void generatingSuccessor(State cur_); 

moveTo inverseOf(moveTo a);

std::vector<State> successors(const State& state);
 
/***************************************15-STP**********************************************/

int blankPos(const array<uint8_t, 16> & tiles);

int count_inversions_small(const uint8_t seq[4], int n);

unsigned long long generateKey(const array <uint8_t, 16>& tiles);

void stateForm(array<uint8_t,16> tiles_);

void addSuccessor(const State &state, vector<State> &suc_, moveTo a, int z, int nz);

void getTargetPos(const array<uint8_t, 16> tar_, array<uint8_t, 16> &row_ , array<uint8_t, 16>  &col_);
int MD(const array<uint8_t, 16> &tiles, const array<uint8_t, 16>& srow_ , const array<uint8_t, 16>&  scol_);

int MD_LC(const std::array<uint8_t,16>& tiles, const std::array<uint8_t,16>& srow_, const std::array<uint8_t,16>& scol_);

void conflictValue(int &h_val, const std::array<uint8_t,16>& tiles, const std::array<uint8_t,16>& srow_, const std::array<uint8_t,16>& scol_, bool row);

void show_path();
 
 private:
    double infinity,epsilon;
    unordered_map<unsigned long long, int> gCostF;
    unordered_map<unsigned long long, int> gCostB;
    unordered_map<unsigned long long, bool> pr;
    priority_queue<State, vector<State>,MinHeap> Open;
    unordered_map<unsigned long long, bool> ClosedF;
    unordered_map<unsigned long long, bool> ClosedB;
    unsigned long long start_, end_;
    array<uint8_t, 16> Ini_, Tar_, goalR_, goalC_;
    State Startstate,Goalstate;
    unordered_map<State, EsCost,HashState> STATE;
    unordered_map<State, State,HashState> parent;
    int map_x, map_y, start_x, start_y, end_x, end_y, expanded_, _ID, EbestN, START, GOAL;
};
#endif
