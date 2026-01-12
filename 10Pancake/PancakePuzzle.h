#pragma once
#include <limits>
#include <ostream>
#include <sstream>
#include <fstream>
#include <string>
#include <queue>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <bitset>
#include <unordered_map>

#include <bitset>

#define COST_MAX std::numeric_limits<int>::max();


#ifndef PancakePuzzle_H
#define PancakePuzzle_H
using namespace std;
    
    struct EsCost {
       unsigned long long prt = -1;      
       bool pre = false;
       bool bpre = false;
       bool visited = false, meet = false, pi_ = false, Spt_ = false;
       int pri, g, h, f, nf, prtf_;
       EsCost() { pri = g = nf = h =f = 0;}
    };
    
    struct State{
         vector<int > stackOrdering;
         unsigned long long key = -1;
         int label = 0;
         EsCost cf,cb;
         bool meet;
         State(){key = -1;}
         State(vector<int> b, int l, unsigned long long ky): stackOrdering(b),label(l), key(ky) {};
         //friend std::ostream& operator<<(std::ostream& stream, const State& state);
         bool operator==(const State &other) const {
           return key == other.key;  // Assuming key uniquely identifies a State
         }    
    };
    
    struct HashState {
        std::size_t operator()(const State& s) const { return s.key; }
    };
      struct MinHeap{ //using for Open list
   	bool operator()(State const& s1, State const& s2) {
        if(s1.cf.f > s2.cf.f)// return a smaller f-value
            return true;
        else if (s1.cf.f < s2.cf.f)
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
    
#endif    

