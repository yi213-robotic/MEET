#include "mm.h"
/***** Construct mm *****/
mm::mm(){
    epsilon = 1e-9;//numeric_limits<double>:: epsilon()
    infinity = numeric_limits<double>:: infinity();
    ForwardCounter_ = BackwardCounter_ = 0;
}
/***** Deconstructmm */
mm::~mm(){}

void mm::getGFminF(int &gminF, int &fminF, int minf)
{
     priority_queue<State, vector<State>, heapPriMin> OpenFr;
     State cur_ = OpenF.top();
     OpenFr = OpenF; 
     while(!OpenFr.empty())
     {
        cur_ = OpenFr.top();
        if(cur_.cf.f < fminF)
        {
             fminF = cur_.cf.f;
        }
        if(cur_.cf.g < gminF)
        {
             gminF = cur_.cf.g;
                //forward_ = cur_;
        }
        //if(cur_.l.x == 9 && cur_.l.y == 21)
        OpenFr.pop();
     }

}



void mm::getGFminB(int &gminB, int &fminB, int minf)
{
     priority_queue<State, vector<State>,heapPriMinB> OpenBr;
     State cur_ = OpenB.top();
     OpenBr = OpenB; 
     while(!OpenBr.empty())
     {
        cur_ = OpenBr.top();
        if(cur_.cb.f < fminB)
        {
              fminB = cur_.cb.f;
        }
        if(cur_.cb.g < gminB)
        {
              gminB = cur_.cb.g;
        }
        OpenBr.pop();
     }
     //cerr << OpenBr.size() << endl;
}

bool mm::PropertyForward()
{
   forwardExpanded_= skip_ = false; 
   int C = COST_MAX;
   int fmin_f = COST_MAX;
   int fmin_b = COST_MAX;
   int gmin_f = COST_MAX;
   int gmin_b = COST_MAX;
   forward_ = OpenF.top();
   if(STATE[forward_].first.visited)
   {
      OpenF.pop();
      skip_ = true; 
      return false;
   }
   backward_ = OpenB.top();
   if(STATE[backward_].second.visited)
   {
      OpenB.pop();
      skip_ = true; 
      return false;
   }
   getGFminF(gmin_f,fmin_f,forward_.cf.pri);
   //cerr << " minimal priority " << forward_.cst.pri << " " << backward_.cstb.pri << endl;
   getGFminB(gmin_b,fmin_b,backward_.cb.pri);
   C = std::min(forward_.cf.pri,backward_.cb.pri);
   maxValue_ = std::max(C,max(fmin_b,max(fmin_f,gmin_b+gmin_f+1)));
   //cerr <<  "\t checking the max........  " << U << " " << maxValue_ << " " << C << " " << fmin_b << " " << fmin_f << " " << gmin_b << " " << gmin_f << endl;
   if(U <= maxValue_)
   {
     find_solution = true;
   }
   /*if(forward_.cst.f < backward_.cstb.f)
   {
             
             return true;     
   }
   else if(backward_.cstb.f < forward_.cst.f)
   {  
             
             return false;
   }
   else
   {
           return forward_.cst.g < backward_.cstb.g;
   }*/
   
   if(forward_.cf.pri < backward_.cb.pri)
   {
             
             return true;     
   }
   else if(backward_.cb.pri < forward_.cf.pri)
   {  
             
             return false;
   }
   else
   {
            //cerr << "hello " << forward_.cst.h << " " << backward_.cstb.h << endl;
           if(forward_.cf.g < backward_.cb.g)
               return true;
           else if(forward_.cf.g > backward_.cb.g)
               return false;
           else         
           return true;
   }
   return false;
}


void mm::stateFormat(array<uint8_t, 16> tiles)
{           
            for(int i = 0; i < 16;i++)
            { cerr << static_cast<unsigned>(tiles[i]) << " ";  }
}

/************Find an optimal anyangle path********************/
void mm::MMSEARCH(double &time, int &forwardExpandedN_, int &backwardExpandedN_,array<uint8_t,16> start_, array<uint8_t,16> startr_, array<uint8_t,16> startc_, array<uint8_t,16> goalr_ , array<uint8_t,16> goalc_, array<uint8_t,16> goal_, int &pathLength, int &OSME, int &OPM) {

    OpenF = priority_queue<State, vector<State >, heapPriMin>();
    OpenB = priority_queue<State, vector<State>, heapPriMinB>();
    STATE.clear();
    parentF.clear();
    parentB.clear();
    hasF.clear();
    hasB.clear();


    Startstate = State(start_,Nan_, generateKey(start_));
    Goalstate = State(goal_,Nan_, generateKey(goal_));
    
    startR_ = startr_;
    startC_ = startc_;
    goalR_ = goalr_;
    goalC_ = goalc_;
    
    U = COST_MAX;
    ForwardCounter_ = BackwardCounter_ = 0;
    find_solution = false;
    end_ = generateKey(goal_);
    starT_ = generateKey(start_);
    OpenF.push(Startstate); 
    OpenB.push(Goalstate);
    double startTime = clock();
    bool ProrityC = false;
    int ty = 0, tx = 0;
    int debug =0;
    //vector<pair<int,int> > forwardL, backwardL;
    while(!OpenF.empty() && !OpenB.empty()){//OpenList is not empty
          forwardExpanded_ = PropertyForward();
          if(find_solution)
             break;
          if(!skip_)
          {     
              generatingSuccessor(); 
          }

    }
    double endTime = clock();
    double rtime = (double) (endTime - startTime) / CLOCKS_PER_SEC;
    //cerr << "mm takes: "<<rtime*1000 <<" (ms)";
    time = rtime;
    pathLength = U;
    forwardExpandedN_ = ForwardCounter_;
    backwardExpandedN_ = BackwardCounter_;
    OSME = OSME_;
    OPM  = OPM_;
    /*for(int i = 0 ; i < forwardL.size();i++)
    {
        cerr << "\\node[draw,circle,inner sep=5pt,fill=purple] at (" << forwardL[i].first<< "," << forwardL[i].second<< ") {};" << " "  << endl;
        
    }
    for(int i = 0 ; i < backwardL.size();i++)
    {
        cerr << "\\node[draw,circle,inner sep=5pt,fill=pink] at (" << backwardL[i].first<< "," << backwardL[i].second<< ") {};" << endl;
        
    }*/
    /*if(pathLength != 0)
    {
       time = rtime;
       cerr << " to find the optimal path length: " << setprecision(15) <<pathLength << " and expanded forward vertices: " << ForwardCounter_ << ", backward vertices: " << BackwardCounter_ << " " << (!OpenF.empty() && !OpenB.empty()) << endl;
       show_path();
    }
    else
    {
        cerr << " to find no path." << endl;
    }*/
}

/*******************Constraining, Modifying Propagating range***********/
void mm::generatingSuccessor() {//eight immediate neighbors, where (0, 0) is the bottom left corner

         if (forwardExpanded_)
         {         
                if(!STATE[forward_].first.visited)
                {
                      OpenF.pop();
                     ForwardCounter_++;  
                     vector <State> suc =  successors(forward_,forward_.cf.dir_); 
                     ExpansionInForward(forward_, suc);
                }

         }
         else
         {
                if(!STATE[backward_].second.visited)
                {
                     OpenB.pop();
                    BackwardCounter_++;
                    vector <State> suc =  successors(backward_,backward_.cb.dir_); 
                    ExpansionInBackward(backward_, suc);
                }
         }       
}


void mm::ExpansionInForward(State cur_, vector<State> Suc)
{
        
        //stateFormat(cur_.tiles);
        //cerr << endl;cerr << "expanded in forward direction ---- > " << cur_.cf.g << " " << cur_.cf.f << "  " << ForwardCounter_ << endl;
        cur_.cf.visited = true;
        STATE[cur_].first = cur_.cf;
        
        for(State child: Suc)
        {
            bool update = false;
            child.cf = STATE[child].first;
            UpdateVertex(child.cf, cur_.cf.g, cur_.key , parentF[child], cur_,update,child,hasF[child.key],child.tiles);
            if(update)
            {
               STATE[child].first = child.cf;
               EsCost backward = STATE[child].second;
               if((backward.pre && !backward.visited) || child.key == end_)
               {
                    int leastC = child.cf.g + backward.g;
                    leastCostFound(leastC,child);
               }
               OpenF.push(child);
               //cerr << " sucessor is: ";            
               //stateFormat(child.tiles);
               //cerr << child.cf.g << " : " << child.cf.h <<"  "  << child.cf.pri <<endl;
            }
        }
}

void mm::leastCostFound(int leasCost_, State suc)
{
        if(leasCost_  < U)
        {
            U = leasCost_;
            intersect_ = suc;
            OSME_ = OPM_ = ForwardCounter_ + BackwardCounter_;
            //cerr << " found a solution here ****************************************************************************************** " << U << endl;
        }
}

void mm::ExpansionInBackward(State cur_, vector<State> Suc)
{
        //stateFormat(cur_.tiles);
        //cerr << endl;
        cur_.cb.visited = true;
        STATE[cur_].second = cur_.cb;
        //cerr << "  expanded in backward direction ---- > " << cur_.cb.g << " " << cur_.cb.f << endl;

        //cerr << BackwardCounter_ << endl;
        
        for(State child: Suc)
        {   
            bool update = false;
            child.cb = STATE[child].second; 
            UpdateVertex(child.cb, cur_.cb.g, cur_.key , parentB[child], cur_,update,child,hasB[child.key],child.tiles);
            if(update)
            {
               STATE[child].second = child.cb;
               EsCost forward = STATE[child].first;
               if((forward.pre && !forward.visited) || child.key == starT_)
               {
                    int leastC = child.cb.g + forward.g;
                    leastCostFound(leastC,child);
               }
               OpenB.push(child);
               //cerr << " sucessor is: ";            
               //stateFormat(child.tiles);
               //cerr << child.cb.g << " : " << child.cb.h << "  "<< child.cb.pri <<endl;
            }
        }
}




void mm::UpdateVertex(EsCost &sucC, int g_value, unsigned long long keyV, State &Parent, State cur_, bool &update, State & suc, bool &hasP, array<uint8_t, 16> tiles)
{
         if(!sucC.pre)
         {
                //sucC.h = forwardExpanded_ ? MD(tiles,goalR_,goalC_): MD(tiles,startR_,startC_);
                //cerr <<  "\t\t\t heuristic " << sucC.h << endl;
                sucC.h = forwardExpanded_ ? MD_LC(tiles,goalR_,goalC_): MD_LC(tiles,startR_,startC_);
                sucC.g = COST_MAX; 
         }

         if(!sucC.visited)
         {
               int edgeCost = 1;
               int pCost = g_value + 1;
               if(sucC.g > pCost)
               {
                  //cerr << "\t generate state here " << endl;
                  update = true;
                  hasP = true;
                  sucC.pre = true;
                  sucC.g = pCost;
                  sucC.prt = keyV;
                  sucC.f = pCost + sucC.h;////max(pCost + sucC.h,pCost + pCost)
                  sucC.pri = max(sucC.f,pCost + pCost);
                  Parent = cur_;
               }
                      
         }

}




/*********** STP ***************/

    unsigned long long mm::generateKey(const array <uint8_t, 16>& tiles) {
            unsigned long long key = 0; 
            for (int i=0; i<16; i++) {
                 key = (key << 4) | (unsigned long long)(tiles[i] & 0xF);
            }
            return key;
    }
    
    void mm::getTargetPos(const array<uint8_t, 16> tar_, array<uint8_t, 16> &row_ , array<uint8_t, 16>  &col_) {
       for(int i =0; i <16; i++)
       {
            row_[tar_[i]] = static_cast<uint8_t>(i / 4);
            col_[tar_[i]] = static_cast<uint8_t>(i % 4);
       } 
    }
 
    
   int mm::MD(const array<uint8_t, 16> &tiles, const array<uint8_t, 16>& srow_ , const array<uint8_t, 16>&  scol_) {
        
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
   
   int mm::count_inversions_small(const uint8_t seq[4], int n)
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
   
   
   void mm::conflictValue(int &h_val, const std::array<uint8_t,16>& tiles, const std::array<uint8_t,16>& srow_, const std::array<uint8_t,16>& scol_, bool row)
   {
                   for (int row_ = 0; row_ < 4; ++row_) 
                   {
                         uint8_t seq[4]; 
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
   
   int mm::MD_LC(const std::array<uint8_t,16>& tiles, const std::array<uint8_t,16>& srow_, const std::array<uint8_t,16>& scol_) {
                   
                   int h_val = MD(tiles,srow_,scol_);
                   // linear confict in rows
                   conflictValue(h_val,tiles,srow_,scol_,true);
         
                   // Linear conflict in cols
                   conflictValue(h_val,tiles,scol_,srow_,false);
                   return h_val;          
   }
   
   int mm::blankPos(const array<uint8_t, 16> & tiles) {
            for(int i = 0; i < 16; i++)
            {
               if(tiles[i] == 0)
                 return i;  
            }
            return -1;    
   }
   
   moveTo mm::inverseOf(moveTo a)
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
   
   void mm::addSuccessor(const State &state, vector<State> &suc_, moveTo a, int z, int nz)
   {
       State child_ = state;
       std::swap(child_.tiles[z],child_.tiles[nz]);
       child_.key = generateKey(child_.tiles);
       forwardExpanded_ ? child_.cf.dir_ = a : child_.cb.dir_ = a;
       suc_.push_back(child_);
   }
   
   std::vector<State> mm::successors(const State& state, moveTo pdir_) {
         
         vector<State> suc_;
         suc_.reserve(4);
         
         int pos_ = blankPos(state.tiles);
         
         if(pos_ < 0 )
           return suc_;
         
         int row_ = pos_/4, col_ = pos_ % 4;
         
         moveTo forbid = inverseOf(pdir_);
         
         if(row_ > 0 && forbid != Up_)
             addSuccessor(state,suc_, Up_,pos_,pos_-4);
         if(row_ < 3 && forbid != Dw_) 
             addSuccessor(state,suc_, Dw_,pos_,pos_+4);
         if(col_ > 0 && forbid != Lt_)
             addSuccessor(state,suc_, Lt_,pos_,pos_-1);
         if(col_ < 3 && forbid != Rt_)
             addSuccessor(state,suc_, Rt_,pos_,pos_+1);               
         return suc_;
   }
   



/********** Return vertices on an optimal anyangle path ****************/
void mm::show_path() {
      stack<State > pt;
      State cur_ = intersect_;
      pt.push(cur_);
      while(STATE[cur_].first.pre) {
         cur_ = parentF[cur_];
         pt.push(cur_);
     }
     //cerr << "The optimal path: " << endl;
     while(!pt.empty()) {
        State optimal = pt.top();
        for (int r = 0; r < 16; r++) {
            cerr << std::setw(3) << optimal.tiles[r] << " ";
        }
        cerr << endl;
        pt.pop();
     }
     cur_ = intersect_;
     while(STATE[cur_].second.prt != -1)
     {
         cur_ = parentB[cur_];
         for (int r = 0; r < 16; r++) {
            cerr << std::setw(3) << cur_.tiles[r] << " ";
         }
         cerr <<endl;
     }
     cerr << endl;
     //cerr << "the meeting point is " << intersect_.l.x << " " << intersect_.l.y << endl;
}


















