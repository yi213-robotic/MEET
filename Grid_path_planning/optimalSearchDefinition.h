/*************************************************
 * common set up in grid map for BEAS, MM, and A*, in C++
 * **********************************************/
//Author: Yi Wang, Bingxian Mu, Oren Salzman
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
  double g,h,f;
  bool pi_,mt,pI,val_;
  NGcost_() : f(0), g(0), h(0), pi_(false), mt(false), pI(false), val_(false){}
  NGcost_(double f_, double g_, double h_ , bool pi, bool mt_, bool pI_, bool val) :f(f_), g(g_), h(h_), pi_ (pi), mt(mt_), pI(pI_), val_(val) {}
};


struct Cost { // Estimated cost for a vertex
   int prt;
   bool pr, vis_, pi_, Spt_;
   double g, h, f, tg, prC, nf; // g: cost to come, h: cost to go, f: total estimated path cost
   Cost() : g(0), h(0), f(0), tg(0), prC(0), prt(-1), nf(0) , pr(false), vis_(false), pi_(false), Spt_{false}{}
};

struct locV {
   bool digm,pt;
   double g;
   int Id;
};

struct vertex{ //a vertex
    loc l;
    Cost cf,cb;
    double f, tg;
    bool src,gol,mt,vis;
    vertex():src(false),gol(false),mt(false),vis(false),tg(0),f(0){}
};


struct smallerForward{ //using for Open list
   	bool operator()(vertex const& v1, vertex const& v2) {
        //return v1.f > v2.f;
        if(v1.cf.f > v2.cf.f + 0.000001)
            return true;
        else if (v1.cf.f + 0.000001 < v2.cf.f)
            return false;
        else
        {
          if(v1.cf.g +0.000001 < v2.cf.g)
          {
              return true;
          }
          else if(v1.cf.g > v2.cf.g + 0.000001)
          {
              return false;
          }
          else
          {
              return v1.cf.h > v2.cf.h;// appa selects larger g-value for tire-breaking
          }
        }
            
    }
};

struct smallerBackward{ //using for Open list
   	bool operator()(vertex const& v1, vertex const& v2) {
        //return v1.f > v2.f;
        if(v1.cb.f > v2.cb.f + 0.000001)
            return true;
        else if (v1.cb.f + 0.000001 < v2.cb.f)
            return false;
        else
        {
          if(v1.cb.g +0.000001 < v2.cb.g)
          {
              return true;
          }
          else if(v1.cb.g > v2.cb.g + 0.000001)
          {
              return false;
          }
          else
          {
              return v1.cb.h > v2.cb.h;// appa selects larger g-value for tire-breaking
          } 
        }
            //return v1.cstb.g < v2.cstb.g;// appa selects larger g-value for tire-breaking
    }
};

struct smallerForwardpr{ //using for Open list
   	bool operator()(vertex const& v1, vertex const& v2) {
        //return v1.f > v2.f;
        if(v1.cf.prC > v2.cf.prC + 0.000001)
            return true;
        else if (v1.cf.prC + 0.000001 < v2.cf.prC)
            return false;
        else
            return v1.cf.g < v2.cf.g;// appa selects larger g-value for tire-breaking
    }
};

struct smallerBackwardpr{ //using for Open list
   	bool operator()(vertex const& v1, vertex const& v2) {
        //return v1.f > v2.f;
        if(v1.cb.prC > v2.cb.prC + 0.000001)
            return true;
        else if (v1.cb.prC + 0.000001 < v2.cb.prC)
            return false;
        else
            return v1.cb.g < v2.cb.g;// appa selects larger g-value for tire-breaking
    }
};
#endif
