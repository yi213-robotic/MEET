#include "astar.h"
/***** Construct astar *****/
astar::astar(){
    epsilon = 1e-6;//numeric_limits<double>:: epsilon()
    infinity = numeric_limits<double>:: infinity();
    expanded_ = 0;
}
/***** Deconstructastar */
astar::~astar(){}

void astar::stateForm(array<uint8_t,16> tiles_)
{
            for(int i = 0; i < 16;i++)
            { cerr << static_cast<unsigned>(tiles_[i]) << " ";  } 
}
/************Find an optimal anyangle path********************/
void astar::astar_(double &time, int &forwardExpanded_, int &backwardExpanded_, array<uint8_t,16> start_, array<uint8_t,16> startR_, array<uint8_t,16> startC_, array<uint8_t,16> goalr_ , array<uint8_t,16> goalc_, array<uint8_t,16> goal_, int &pathLength) {
    gCostF.clear();
    gCostB.clear();
    pr.clear();
    ClosedF.clear();
    ClosedB.clear();
    STATE.clear();
    parent.clear();
    Open = priority_queue<State, vector<State>,MinHeap>();
    
    Ini_ = start_;
    Tar_ = goal_;
    goalR_ = goalr_;
    goalC_ = goalc_;
    Startstate = State(start_,Nan_, generateKey(start_));
    
    State oG = State(goal_,Nan_, generateKey(goal_));
    
    Goalstate = oG;
    end_ = generateKey(goal_);
    Open.push(Startstate); 
    double startTime = clock();
    int ty = 0, tx = 0;
    vector<pair<int,int> > forwardL;
    while(!Open.empty()){//OpenList is not empty
         State cur_ = Open.top();
                Open.pop();
         if(cur_.key == end_)
         { // find the goal
              //cerr << "find the goal " <<endl;
               break;
         }
         if(!STATE[cur_].visited)
         {  
            expanded_++;
            //forwardL.push_back(make_pair(cur_.l.x,cur_.l.y));
            generatingSuccessor(cur_);
            
         }
         //if(expanded_ == 100)exit(-1);
    }
    double endTime = clock();
    double rtime = (double) (endTime - startTime) / CLOCKS_PER_SEC;
    cerr << "astar takes: "<<rtime*1000 <<" (ms)";
    time = rtime;
    forwardExpanded_ = expanded_;
    pathLength = STATE[oG].g;
    /*for(int i = 0 ; i < forwardL.size();i++)
    {
        cerr << "\\node[draw,circle,inner sep=5pt,fill=cyan] at(" << forwardL[i].first<< "," << forwardL[i].second<< ") {};" << endl;
        
    }*/
  //return pathLength; 
    if(pathLength != 0)
    {
        cerr << " to find the optimal path length: " << setprecision(15) <<pathLength << " and expanded vertices: " << expanded_ << endl;
        //show_path();
    }
    else
    {
        cerr << " to find no path." << endl;
    }  /**/
    //cerr << endl;
}
/*************************************************************************************************************************************/
unsigned long long astar::generateKey(const array <uint8_t, 16>& tiles) {
            unsigned long long key = 0; 
            for (int i=0; i<16; i++) {
                 key = (key << 4) | (unsigned long long)(tiles[i] & 0xF);
            }
            return key;
}
    
void astar::getTargetPos(const array<uint8_t, 16> tar_, array<uint8_t, 16> &row_ , array<uint8_t, 16>  &col_) {
       for(int i =0; i <16; i++)
       {
            row_[tar_[i]] = static_cast<uint8_t>(i / 4);
            col_[tar_[i]] = static_cast<uint8_t>(i % 4);
       } 
}
 
    
int astar::MD(const array<uint8_t, 16> &tiles, const array<uint8_t, 16>& srow_ , const array<uint8_t, 16>&  scol_) {
        
        int h_val = 0;
        for(int i=0; i <16;i++)
        {
             if(tiles[i] == 0) continue; // skip blank
             uint8_t t = tiles[i];
             
             int row_ = i/4, col_ = i%4;
             
             h_val += abs(row_- int(srow_[t])) + abs(col_-int(scol_[t]));
             
        }
        return h_val;
}
   
int astar::count_inversions_small(const uint8_t seq[4], int n)
{
        /*const int a = seq[0], b = seq[1], c = seq[2], d = seq[3];
        switch (n) 
        {
           case 4: return (a>b)+(a>c)+(a>d) + (b>c)+(b>d) + (c>d);
           case 3: return (a>b)+(a>c) + (b>c);
           case 2: return (a>b);
           default: return 0;
        }*/
        
         int inv = 0;
    for (int i = 0; i < n; ++i)
    {    for (int j = i+1; j < n; ++j)
         {   
         inv += (seq[i] > seq[j]);
         }  
    }        
    return inv;   
}
   
   
void astar::conflictValue(int &h_val, const std::array<uint8_t,16>& tiles, const std::array<uint8_t,16>& srow_, const std::array<uint8_t,16>& scol_, bool row)
{
                   for (int row_ = 0; row_ < 4; ++row_) 
                   {
                         uint8_t seq[4] = {}; 
                         int i =0; 
                         int base =0;
                         if(row)
                           base = row_ * 4;
                         for (int col_ = 0; col_ < 4; ++col_) 
                         {   
                               uint8_t t = tiles[row ? (base + col_) : (col_*4 + row_)];
                               if (t == 0) 
                                   continue;                 // skip blank
                               if (srow_[t] == row_) 
                                   seq[i++] = scol_[t]; // tile belongs in this row; use goal column
                         }
                         h_val += 2 * count_inversions_small(seq,i); // each LC pair adds +2
                   } 
}
   
int astar::MD_LC(const std::array<uint8_t,16>& tiles, const std::array<uint8_t,16>& srow_, const std::array<uint8_t,16>& scol_) {
                   
                   int h_val = MD(tiles,srow_,scol_);
                   // linear confict in rows
                   conflictValue(h_val,tiles,srow_,scol_,true);
         
                   // Linear conflict in cols
                   conflictValue(h_val,tiles,scol_,srow_,false);
                   return h_val;          
}
   
int astar::blankPos(const array<uint8_t, 16> & tiles) {
            for(int i = 0; i < 16; i++)
            {
               if(tiles[i] == 0)
                 return i;  
            }
            return -1;    
}
   
moveTo astar::inverseOf(moveTo a)
{
        switch (a) 
        {
               case Up_: return Dw_;
               case Dw_: return Up_;
               case Lt_: return Rt_;
               case Rt_: return Lt_;
               default:  return Nan_;
        }
}
   
void astar::addSuccessor(const State &state, vector<State> &suc_, moveTo a, int z, int nz)
{
       State child_ = state;
       std::swap(child_.tiles[z],child_.tiles[nz]);
       child_.key = generateKey(child_.tiles);
       child_.dir_ = a;
       suc_.push_back(child_);
}
   
std::vector<State> astar::successors(const State& state) 
{
         
         vector<State> suc_;
         suc_.reserve(4);
         
         int pos_ = blankPos(state.tiles);
         
         if(pos_ < 0 )
           return suc_;
         
         int row_ = pos_/4, col_ = pos_ % 4;
         
         moveTo forbid = inverseOf(state.dir_);
         
         if(row_ > 0 && forbid != Up_)
             addSuccessor(state,suc_,Up_,pos_,pos_-4);
         if(row_ < 3 && forbid != Dw_) 
             addSuccessor(state,suc_,Dw_,pos_,pos_+4);
         if(col_ > 0 && forbid != Lt_)
             addSuccessor(state,suc_,Lt_,pos_,pos_-1);
         if(col_ < 3 && forbid != Rt_)
             addSuccessor(state,suc_,Rt_,pos_,pos_+1);             
         return suc_;
   }


void astar::generatingSuccessor(State cur_) {//
       
           vector <State> suc =  successors(cur_);
           //cerr << "the expanding state is: ";
           //stateForm(cur_.tiles); 
           //cerr << " ---> g: " << cur_.cf.g << " " << cur_.cf.f <<endl;
           cur_.cf.visited = true;
           STATE[cur_] = cur_.cf;
           for(State child: suc)
           {
                   EsCost chi = STATE[child];
                   if(!chi.pre)
                   {
                       //chi.h = MD(child.tiles,goalR_,goalC_);
                       
                       chi.h = MD_LC(child.tiles,goalR_,goalC_);
                       chi.g =  COST_MAX;
                   } 
                   if(!chi.visited)
                   {
                      int pCost = cur_.cf.g + 1; 
                      if(chi.g > pCost )
                      {
                         chi.pre = true;
                         chi.g  = pCost;
                         chi.prt = cur_.key;
                         chi.f  = pCost + chi.h;
                         child.cf = chi;
                         STATE[child] = chi;
                         parent[child] = cur_;
                         Open.push(child);
                         //cerr << "\t generating a new state: {";
                         //stateForm(child.tiles);
                         //cerr << "} with g value: " << chi.g << ", h value: " << chi.h << ", and f value: " << chi.f << ", and move direction: " << child.dir_ <<endl;  
                                
                      }
                   }
            }
}


void astar::show_path() 
{
      stack< State > pt;
      State cur_ = Goalstate;
      pt.push(cur_);
      while(STATE[cur_].pre) {
         cur_ = parent[cur_];
         pt.push(cur_);
     }
     cerr << pt.size() <<"The optimal path: " << endl;
     while(!pt.empty()) {
        State optimal = pt.top();
        for (int r = 0; r < 16; r++) {
            cerr << std::setw(3) << static_cast<unsigned> (optimal.tiles[r]) << " ";
        }
        cerr << endl;
        pt.pop();
     }
}
