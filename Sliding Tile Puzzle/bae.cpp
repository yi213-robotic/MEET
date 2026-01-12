#include "bae.h"
/***** Construct mm *****/
bae::bae(){
    epsilon = 1e-9;//numeric_limits<double>:: epsilon()
    infinity = numeric_limits<double>:: infinity();
    ForwardCounter_ = BackwardCounter_ = 0;
}
/***** Deconstructmm */
bae::~bae(){}


bool bae::PropertyForward()
{
   skip_ = false; 
   forward_ = OpenF.top();
   backward_ = OpenB.top();
   //int C = min(forward_.cst.f,backward_.cstb.f);
   if(forward_.key == start_)
   {
      return (forwardExpanded_ = true);
   }
   if(forwardExpanded_)
   {
       EsCost backwar_ =  STATE[forward_].second;
       if(backwar_.pre && !backwar_.visited)
       {
             //cerr << backwar_.pre << " f " << backwar_.visited << endl;
             leastCostFound(forward_.cf.g+backwar_.g, forward_);
       }
   }
   else
   {
       EsCost forwar_ = STATE[backward_].first;
       if(forwar_.pre && !forwar_.visited)
       {
             //cerr << forwar_.pre << " " << forwar_.visited << endl;
             leastCostFound(backward_.cb.g+forwar_.g,backward_);
       }  
   }
  
   L = (double) (forward_.cf.f + backward_.cb.f)/2;
   //cerr << " minimal priority " << forward_.cst.f << " " << backward_.cstb.f << "***************************************************" << endl;
   if(L + 0.0000001 >= C)
   {
     //cerr << " ???? " << L << "  " <<  C<< endl;
     find_solution = true;
   }
   return false;
}


void bae::stateFormat(array<uint8_t, 16> tiles)
{
         
}

/************Find an optimal anyangle path********************/
void bae::baeSEARCH(double &time, int &forwardExpandedN_, int &backwardExpandedN_, array<uint8_t,16> starT_, array<uint8_t,16> startr_, array<uint8_t,16> startc_, array<uint8_t,16> goalr_ , array<uint8_t,16> goalc_, array<uint8_t,16> goal_, int &pathLength, int &OSME_, int &OPM_) 
{

    OpenF = priority_queue<State, vector<State >, heapMin>();
    OpenB = priority_queue<State, vector<State >, heapMinB>();
    STATE.clear();
    parentF.clear();
    parentB.clear();
    OpenListF_.clear();
    OpenListB_.clear();
    Startstate = State(starT_,Nan_, generateKey(starT_));
    Goalstate = State(goal_,Nan_, generateKey(goal_));
    U = COST_MAX;
    C = infinity;
    OSME = OPM = ForwardCounter_ = BackwardCounter_ = 0;
    Ms_ = DI_ = adf_ = potImprove_ = curMSDir_ = findMs_ = curSolution_ = find_solution = false;
    
    startR_ = startr_;
    startC_ = startc_;
    goalR_ = goalr_;
    goalC_ = goalc_;
    
    end_ = generateKey(goal_);
    start_ = generateKey(starT_);
    OpenF.push(Startstate); 
    OpenB.push(Goalstate);
    NGcost_ sval(0,0,0,false,false);
    NGcost_ nval(0,0,0,false,false);
    double startTime = clock();
    bool ProrityC = false;
    int ty = 0, tx = 0, tmx=999999 , tmy=999999;
    int debug =0;
    //vector<pair<int,int> > forwardL, backwardL;
    while(!OpenF.empty() && !OpenB.empty()){//OpenList is not empty
          PropertyForward();
          if(find_solution)
             break;
          if(!skip_)
          {     
              generatingSuccessor(); 
          }

    }
    double endTime = clock();
    double rtime = (double) (endTime - startTime) / CLOCKS_PER_SEC;
    cerr << "bae takes: "<<rtime*1000 <<" (ms)";
    time = rtime;
    pathLength = C;
    forwardExpandedN_ = ForwardCounter_;
    backwardExpandedN_ = BackwardCounter_;
    OSME_ = OSME;
    OPM_  = OPM;
    cerr << endl;
    //show_path();
}

/*******************Constraining, Modifying Propagating range***********/
void bae::generatingSuccessor() {//eight immediate neighbors, where (0, 0) is the bottom left corner
         MSt_ = hasSucc_ = hasChild_ = findMs_ = false;
         lminh_= lsb_ = lnb_ = lfmin_ = COST_MAX;
         ProVal_ = curL_;
         int Gcur =0,  Hcur = 0;
         //cerr << ProVal_<< "   " << curL_ << " " << lnh_ << endl;
         if(curSolution_)//EC_ = C;
         { CurMS_ = intersect_; }
         if (forwardExpanded_)
         {      
                OpenF.pop(); 
                if(!STATE[forward_].first.visited)
                {
                     ForwardCounter_++;
                     forw_ = STATE[forward_].first;
                     Ms_ =forw_.meet;      
                     vector <State> suc =  successors(forward_,forward_.cf.dir_); 
                     OpenListF_.erase(forward_.key);
                     Gcur = forward_.cf.g; Hcur = forward_.cf.h;
                     ExpansionInForward(forward_, suc);
                }
         }
         else
         {
                OpenB.pop();
                if(!STATE[backward_].second.visited)
                {
                    BackwardCounter_++;
                    backw_ = STATE[backward_].second;
                    Ms_ =backw_.meet;   
                    vector <State> suc =  successors(backward_,backward_.cb.dir_); 
                    Gcur = backward_.cb.g; Hcur = backward_.cb.h;
                    OpenListB_.erase(backward_.key);
                    ExpansionInBackward(backward_, suc);
                }
         } 
         
         if(OpenListF_.size() <= OpenListB_.size())
         {
             forwardExpanded_ = true;
         }
         else
         {
             forwardExpanded_ = false;
         }    
}


void bae::ExpansionInForward(State cur_, vector<State> Suc)
{
        //stateFormat(cur_.tiles);
        //cerr << " is expanding in forward search with g:" << cur_.cf.g << " " << cur_.cf.f << " " << ForwardCounter_ << endl;
        cur_.cf.visited = true;
        STATE[cur_].first = cur_.cf;
        bool nPath_ = false;
        for(State child: Suc)
        {
            bool update = false;
            child.cf = STATE[child].first;
            UpdateVertex(child.cf, cur_.cf.g, cur_.key , parentF[child],cur_,update,child, child.cf.pre,child.tiles);
            if(update)
            {
               STATE[child].first = child.cf;
               EsCost backward = STATE[child].second;
               bool localMeet = false;
               OpenListF_[child.key] = child.cf.f;
               OpenF.push(child);
               //stateFormat(child.tiles);
               //cerr << "\t\t in forward->  cost : g: " << child.cf.g << " h: " << child.cf.h << "  f:"  << child.cf.f << endl;
            }
        }
}

void bae::leastCostFound(int leastCost_, State cur)
{       
        if(leastCost_  < C )
        {
            C = leastCost_;
            intersect_ = cur;
        }
}



void bae::ExpansionInBackward(State cur_, vector<State> Suc)
{
        //stateFormat(cur_.tiles);
        //cerr << " is expanding in backward search with g:" << cur_.cb.g << " " << cur_.cb.f << " " << BackwardCounter_ << endl;
        cur_.cb.visited = true;
        STATE[cur_].second = cur_.cb;
        bool nPath_ = false, localMeet = false;
        for(State child: Suc)
        {   bool update = false;
            child.cb = STATE[child].second;
            UpdateVertex(child.cb, cur_.cb.g, cur_.key , parentB[child], cur_,update,child, child.cb.pre,child.tiles);
            if(update)
            {
               STATE[child].second = child.cb;
               EsCost forward = STATE[child].first;
               OpenListB_[child.key] = child.cb.f;
               OpenB.push(child);
               //stateFormat(child.tiles);
               //cerr << "\t\t in backward-----> cost : g: " << child.cb.g << " h: " << child.cb.h << "  f:"  << child.cb.f << endl;
            }
        }
}


void bae::UpdateVertex(EsCost &sucC, int g_value, unsigned long long keyV, State &Parent, State cur_, bool &update, State &suc, bool & hasP, array<uint8_t, 16> tiles)
{

         if(!sucC.visited)
         {
              int rh = 0;
              
              
                   /*if(forwardExpanded_)
                   { 
                      if(!sucC.pre)
                      {sucC.h = MD(tiles,goalR_,goalC_);} 
                      
                      rh = MD(tiles,startR_,startC_);
                   } 
                   else
                   {
                      if(!sucC.pre)
                      sucC.h= MD(tiles,startR_,startC_); 
                      
                      rh = MD(tiles,goalR_,goalC_);
                   }*/
                   
                   if(forwardExpanded_)
                   { 
                      if(!sucC.pre) 
                        sucC.h = MD_LC(tiles,goalR_,goalC_);
                        rh = MD_LC(tiles,startR_,startC_);
                   } 
                   else
                   {
                      if(!sucC.pre)
                      sucC.h= MD_LC(tiles,startR_,startC_); 
                      rh = MD_LC(tiles,goalR_,goalC_);
                   }
                   //sucC.h = forwardExpanded_ ? MD_LC(tiles,goalR_,goalC_): MD_LC(tiles,startR_,startC_);
                if(!sucC.pre)
                sucC.g = COST_MAX; 
               
               int edgeCost = 1;
               int pCost = g_value + 1;
               if(sucC.g > pCost)
               {
                  hasChild_ = update = true;
                  hasP = true;
                  sucC.pre = true;
                  sucC.g = pCost;
                  sucC.prt = keyV;
                  sucC.nf = sucC.f = pCost + pCost + sucC.h -rh; 
                  Parent = cur_;
               }                      
         }
}

/*********** Constraining Propagating range from surrounding neighors ***************/

    unsigned long long bae::generateKey(const array <uint8_t, 16>& tiles) {
            unsigned long long key = 0; 
            for (int i=0; i<16; i++) {
                 key = (key << 4) | (unsigned long long)(tiles[i] & 0xF);
            }
            return key;
    }
    
    void bae::getTargetPos(const array<uint8_t, 16> tar_, array<uint8_t, 16> &row_ , array<uint8_t, 16>  &col_) {
       for(int i =0; i <16; i++)
       {
            row_[tar_[i]] = static_cast<uint8_t>(i / 4);
            col_[tar_[i]] = static_cast<uint8_t>(i % 4);
       } 
    }
 
    
   int bae::MD(const array<uint8_t, 16> &tiles, const array<uint8_t, 16>& srow_ , const array<uint8_t, 16>&  scol_) {
        
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
   
   int bae::count_inversions_small(const uint8_t seq[4], int n)
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
   
   
   void bae::conflictValue(int &h_val, const std::array<uint8_t,16>& tiles, const std::array<uint8_t,16>& srow_, const std::array<uint8_t,16>& scol_, bool row)
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
   
   int bae::MD_LC(const std::array<uint8_t,16>& tiles, const std::array<uint8_t,16>& srow_, const std::array<uint8_t,16>& scol_) {
                   
                   int h_val = MD(tiles,srow_,scol_);
                   // linear confict in rows
                   conflictValue(h_val,tiles,srow_,scol_,true);
         
                   // Linear conflict in cols
                   conflictValue(h_val,tiles,scol_,srow_,false);
                   return h_val;          
   }
   
   int bae::blankPos(const array<uint8_t, 16> & tiles) {
            for(int i = 0; i < 16; i++)
            {
               if(tiles[i] == 0)
                 return i;  
            }
            return -1;    
   }
   
   moveTo bae::inverseOf(moveTo a)
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
   
   void bae::addSuccessor(const State &state, vector<State> &suc_, moveTo a, int z, int nz)
   {
       State child_ = state;
       std::swap(child_.tiles[z],child_.tiles[nz]);
       child_.key = generateKey(child_.tiles);
       forwardExpanded_ ? child_.cf.dir_ = a : child_.cb.dir_ = a;
       suc_.push_back(child_);
   }
   
   std::vector<State> bae::successors(const State& state, moveTo pdir_) {
         
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
void bae::show_path() {
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

