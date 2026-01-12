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
#include <cstdint>
#include <unordered_map>

#include <bitset>
#include <array>

#define COST_MAX std::numeric_limits<int>::max();


#ifndef SlidingTilePuzzle_H
#define SlidingTilePuzzle_H
using namespace std;
    enum moveTo:uint8_t { Up_ = 0, Dw_ = 1, Lt_ = 2, Rt_ = 3, Nan_ = 4 };
    struct EsCost {
       unsigned long long prt = -1;      
       bool pre = false;
       bool bpre = false;
       bool visited = false, meet = false, pi_ = false, Spt_ = false;
       moveTo dir_ = Nan_;
       int pri, g, h, f, nf, prtf_;
       EsCost() { bpre= pre = pi_ = pri = g = nf = h =f = 0;}
    };
    
    struct State{
         array<uint8_t, 16> tiles{};
         unsigned long long key = -1;
         moveTo dir_ = Nan_;
         EsCost cf,cb;
         bool meet;
         State(){key = -1;}
         State(array<uint8_t,16> tis, moveTo dir, unsigned long long ky): tiles(tis),dir_(dir), key(ky) {};
         //friend std::ostream& operator<<(std::ostream& stream, const State& state);
         bool operator==(const State &other) const {
           return key == other.key;  // Assuming key uniquely identifies a State
         }    
    };
    

    
    struct HashState {
        std::size_t operator()(const State& s) const noexcept{ return std::hash<unsigned long long>{} (s.key); }
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

