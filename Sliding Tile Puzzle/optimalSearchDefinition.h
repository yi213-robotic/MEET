/*************************************************
 * common set up for BEAS, MM, and A*, in C++
 * **********************************************/
 //Author: Yi Wang
#ifndef OptimalSearchDefinition_H
#define OptimalSearchDefinition_H
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
using namespace std;
#define PHI 1.07967
#define DigCost 1.41421356237
#define HALF_PI 1.57079632679
#define uniteCost 1
struct loc { // this is for a vertex's location or a line
    int x,y,Id;// (x = start.x-end.x, y = start.y-end.y) for a line
    loc():x(-1),y(-1),Id(-1){}
    loc(int x_, int y_, int Id_):x(x_),y(y_),Id(Id_){}
};

struct NGcost_{
  int g,h,f,_f;
  bool pi_;
  NGcost_() : f(0), g(0), h(0), pi_(false), _f(0){}
  NGcost_(int f_, int g_, int h_ , bool pi, int t_) :f(f_), g(g_), h(h_), pi_ (pi), _f(t_) {}
};

struct Cost { // Estimated cost for a vertex
   unsigned long long prt;
   bool pr,vis,digm_;
   int g, h, f, tg, prC; // g: cost to come, h: cost to go, f: total estimated path cost
   Cost() : g(0), h(0), f(0), tg(0), prC(0), prt(-1), pr(false), vis(false), digm_{false} {}
};


struct vertex{ //a vertex
    loc l;
    Cost cstf,cstb;
    int f, tg;
    bool src,gol,vis,meet,ad,op;
    vertex():src(false),gol(false),meet(false),ad(false),op(false),vis(0),tg(0),f(0){}
};

struct smallerForward{ //using for Open list
   	bool operator()(vertex const& v1, vertex const& v2) {
        //return v1.f > v2.f;
        if(v1.cstf.f > v2.cstf.f)
            return true;
        else if (v1.cstf.f < v2.cstf.f)
            return false;
        else
            return v1.cstf.g < v2.cstf.g;// appa selects larger g-value for tire-breaking
    }
};

struct smallerBackward{ //using for Open list
   	bool operator()(vertex const& v1, vertex const& v2) {
        //return v1.f > v2.f;
        if(v1.cstb.f > v2.cstb.f)
            return true;
        else if (v1.cstb.f < v2.cstb.f)
            return false;
        else
            return v1.cstb.g < v2.cstb.g;// appa selects larger g-value for tire-breaking
    }
};

struct smallerForwardpr{ //using for Open list
   	bool operator()(vertex const& v1, vertex const& v2) {
        //return v1.f > v2.f;
        if(v1.cstf.prC > v2.cstf.prC )
            return true;
        else if (v1.cstf.prC  < v2.cstf.prC)
            return false;
        else
            return v1.cstf.g < v2.cstf.g;// appa selects larger g-value for tire-breaking
    }
};

struct smallerBackwardpr{ //using for Open list
   	bool operator()(vertex const& v1, vertex const& v2) {
        //return v1.f > v2.f;
        if(v1.cstb.prC > v2.cstb.prC )
            return true;
        else if (v1.cstb.prC  < v2.cstb.prC)
            return false;
        else
            return v1.cstb.g < v2.cstb.g;// appa selects larger g-value for tire-breaking
    }
};
#endif
