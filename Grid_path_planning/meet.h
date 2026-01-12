

/*************************************************
 * MEET authors: Yi, Bingxian, Eyal, Oren
 * **********************************************/
#ifndef MEET_H
#define MEET_H
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>
#include <stack>
#include <algorithm> 
#include <cmath>
#include <limits>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <chrono>
#include <list>
#include <cstdio>
#include <fstream>
#include <cstring>
#include "optimalSearchDefinition.h"
using namespace std;
#define PHI 1.07967
#define MaxRavDist 1.41421356237
#define HALF_PI 1.57079632679

class meet{
public:
    meet();
    ~meet();
    meet(vector<bool > grid_, int width, int height, int sx, int sy, int gx, int gy);
    bool TC1();
    bool TC6();
    void get_Max();
    /********** Euclidean distance between two vertices  **************/
    bool isCSRstate();
    void parsing_map();
    
    int ID(int x, int y);
    
    void forwardSearch();
    
    bool hasBetterSolu();

    void backwardSearch();
     
    void printStatement(); 

    bool stateExpansion();
    
    bool lSolu(double sf_);
    bool leqSolu(double sf_);
    bool TC5(double Gcur, double Hcur);
    /******* Insert start into Open ***********/
    
    bool les(double v1, double v2);    
    
    bool gre(double v1, double v2);
   
    bool leq(double v1, double v2);
    
    bool geq(double v1, double v2);
    
    bool hEq(double gCur, double hCur);
    
    bool minEdge(double v1, double v2);

    bool Eq(double gCur, double hCur);

    void Initialize_And_Insert_start();


    bool CSRS(double Gcur, double Hcur);
    bool TC3(double gCur_, double hCur_);  
    
    double OctileDistance(int dx, int dy);
    
    void minValueOps(double sf_, bool spi_);
    bool Terminate_(double sg_, double sh_);

    double EuclideanDistance(int dx, int dy);
    
    vertex Initialize_(int x, int y, int ID);
      
    bool hElevation(double curG, double curH);
    
    bool checkingCsrs(double Gcur, double Hcur);
 
    bool TC4(double gCur_, double hCur_, bool pi_);
    
    void leastCostFound(double leasCost_, vertex suc);
    
    bool NotConsideredState(double Gcur, double Hcur);
    
    bool refineState(double sf_, double sg_, double sh_, bool vPI_);
    
    bool TerminationSearch(double sg_, double sh_, bool potentialT);
    
    void TerminatingSearch(bool search_meet, vertex fcur_, vertex bcur_);
    
    bool TC2(bool fpi_, bool bpi_, bool fmt_, bool  bmt_, double fg, double bg);
    
    /*******************Constraining, Modifying Propagating range***********/
    
    bool TerminationCondition(double Gcur, double Hcur, bool Ibest_, bool PreExpansion);
    
    
    void generatingSuccessor(vertex cur_, double Gcur, double Hcur, bool Ibest_, bool curDm, bool opt, bool curSpt_);
    
    double meetSEARCH(double &runtime, int &FE, int &BE, int &OSME, int &chi_, int &T1, int &T2, int &T3, int &T4, int &T5, int &T6); 
    
    void GetNeighbors(int x, int y, vertex &cur_, vertex &suc, bool diagNeighbor, bool &update, double Gcur, double Hcur);  
    
    void getMinMetricValue_(double sf_, double sg_, double sh_, bool digm_, bool Ms_ , double Gcur, double Hcur, double &lminF_, NGcost_ &val_, bool mt_, bool pI_, int ID_, bool uiCost);
    
    /*********** Constraining Propagating range from surrounding neighors ***************/
    void valueUpdating(int x, int y, bool diagNeighbor, Cost &suc_, double cur_g, int &prt, int tx, int ty, bool oprDr, double rg, bool &digma_, bool &bmeet, double rf, bool curD_, bool &child_);
    
    
    void updateMetricValues(int lx, int ly, bool diagNeighbor, Cost &vc_, double cg, double ch ,int &curId_, int sId_ , int ox, int oy, bool oprD_, double bg_, bool &sDm_, bool &ms_, double bf, bool curDm, bool &child_, bool vis_);

    void show_path();/********** Return vertices on an optimal anyangle path ****************/
private:
    vector<bool> grid_;
    vector<bool> blocks_;
    vector<vertex> vertex_;
    vertex fdir_; // forward search direction
    vertex bdir_; // backward search direction
    vertex Ib_;// the current best solution
    priority_queue<vertex, vector<vertex>,smallerForward> OpenF;
    priority_queue<vertex, vector<vertex>,smallerBackward> OpenB;
    
    bool Ih_;// check whether Ib has a greater g_value in the generated direction 
    NGcost_ sval_;
    NGcost_ IMV_, ExIMV_;
    bool TC1_,TC2_,TC3_,TC4_,TC5_,TC6_,cPi_,lSpt_, acrt_, preDf_, preDb_,bpi_;
    bool fExpd_, find_solution, skip_, localFind_, culling_, findMS_,IbDir_, prtI_, locSol_, c_bt, bFs_, bIh_, pSolF_, pSolB_,locS_;
    bool findASolution_,Spi_,Spt_, bSoluF_, bSoluB_, hasChild_, firstS_,Dhill_,locHep_, bSolu_;
    double Sfmin_, C_curr, prtIbG_, prtIbH_, epsilon, infinity, fmin_, f_dMin ,curCost_, lnh_,f_min_,b_min_, IGMin_,gI_, hI_, IgI_,sGmin_, curG_, curH_, maxG_;
    int map_x, map_y, start_x, start_y, end_x, end_y, fCounter_, bCounter_, _ID,START,GOAL, minID_, OSME_;
};
#endif
