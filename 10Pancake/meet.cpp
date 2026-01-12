#include "meet.h"
/***** Construct mm *****/
meet::meet()
{
    fCounter_ = bCounter_ = 0;
    arch_ = acrt_ = TC1_ = TC2_ = TC3_ = TC4_ = TC5_ = TC6_ = false;
}
/***** Deconstructmm */
meet::~meet(){}

int meet::CalculateH(vector<int> s, vector<int > target, int X_) // tranlate the goal once it is unsorted
{      
       vector<int > cTar(11);
       for (int i = 0; i < target.size(); i++)
       {      
              if(i != 10)      
              cTar[target[i]] = i+1;
       }
       
       int h = 0;
       for(int i = 0; i<s.size(); i++)
       {
                 if(i ==s.size() -1)
		     continue;
		  int x = s[i];
		  int y = s[i+1];   
		  if(cTar[x] < X_ || cTar[y] < X_)
		     continue;  
		  int dif = cTar[x] - cTar[y];
                  //if( X_ == 0 || (x>X_ && y > X_)) // GPA AND GAP-X
	          {
			    if (dif > 1 || dif < -1){
			           ++h; 
			    }
	          }
       }
       return h;    
}


bool meet::stateExpansion()
{
   fExpd_= skip_ = false; 
   
   fdir_ = OpenF.top();
   bdir_ = OpenB.top();
   
   if(STATE[fdir_].first.visited)
   {
      skip_ = true;
      OpenF.pop();
      return false;
   }
   
   if(STATE[bdir_].second.visited)
   {
      skip_ = true;
      OpenB.pop();
      return false;
   }
   //cerr << " minimal f-values: " <<fdir_.cf.f << " Fdir: " << fdir_.cf.Spt_ << "  "<< fdir_.cf.g << " " << fdir_.cf.h  << " Bdir: " << bdir_.cb.f << "   " << bdir_.cb.g << "  " << bdir_.cb.h << " NSB " << bdir_.cb.Spt_ << endl;
   fmin_ = min(fdir_.cf.f,bdir_.cb.f);
   bool fm = (fdir_.cf.g < fdir_.cf.h), bm = (bdir_.cb.g < bdir_.cb.h);
   if(findASolution_&& (find_solution = TerminationCondition(0,0,false, true)))
   {
       return true;
   }
   if(fdir_.cf.f < bdir_.cb.f)
   {
             minValueOps(bdir_.cb.f,bdir_.cb.pi_); 
             return true;     
   }
   else if(bdir_.cb.f < fdir_.cf.f)
   {  
             minValueOps(fdir_.cf.f,fdir_.cf.pi_); 
             return false;
   }
   else
   {
            //cerr << "hello " << fdir_.cf.h << " " << bdir_.cb.h << endl;
           if(fdir_.cf.g < bdir_.cb.g)
           { 
              minValueOps(bdir_.cb.f,bdir_.cb.pi_);       
              return true; 
           }
           else if(fdir_.cf.g > bdir_.cb.g)
           {    
               minValueOps(fdir_.cf.f,fdir_.cf.pi_);   
               return false;
           }
           else
           { 
             minValueOps(bdir_.cb.f,bdir_.cb.pi_);      
             return true;
           }
   }
   return false;
}

void meet::minValueOps(int sf_, bool spi_)
{
     f_dMin =  sf_;
     bpi_ = spi_; 
}
void meet::stateFormat(vector <int> child)
{           
          for(int i=0; i < child.size(); i++ )
          {
                  cerr << child[i] << ","; 
          }
}

/************Find an optimal anyangle path********************/
void meet::meetSEARCH(double &time, int &forwardExpandedN_, int &backwardExpandedN_,vector<int> start, vector<int> goal, int &pathLength, int &OSME_, int &OPM_, int &t1, int &t2, int &t3, int &t4, int &t5, int &t6) 
{

    OpenF = priority_queue<State, vector<State >, heapMin>();
    OpenB = priority_queue<State, vector<State >, heapMinB>();
    STATE.clear();
    parentF.clear();
    parentB.clear();

    Startstate = State(start,0, generateKey(start));
    Goalstate = State(goal,0, generateKey(goal));
    C_curr = COST_MAX;
    OSME = OPM = fCounter_ = bCounter_ = 0;
    findASolution_ = find_solution = false;
    end_ = generateKey(goal);
    start_ = generateKey(start);
    OpenF.push(Startstate); 
    OpenB.push(Goalstate);
    NGcost_ sval(0,0,0,false,false);
    double startTime = clock();
    bool ProrityC = false;
    int ty = 0, tx = 0, tmx=999999 , tmy=999999;
    int debug =0;
    //vector<pair<int,int> > forwardL, backwardL;
    while(!OpenF.empty() && !OpenB.empty()){//OpenList is not empty
          fExpd_ = stateExpansion();
          if(find_solution)
             break;
          if(!skip_)
          {     
              generatingSuccessor(); 
          }
          if(find_solution)
             break;
    }
    double endTime = clock();
    double rtime = (double) (endTime - startTime) / CLOCKS_PER_SEC;
    cerr << "meet takes: "<<rtime*1000 <<" (ms)";
    time = rtime;
    pathLength = C_curr;
    forwardExpandedN_ = fCounter_;
    backwardExpandedN_ = bCounter_;
    OSME_ = OSME;
    OPM_  = OPM;
    cerr << endl;
    cerr << " TC occurs when "<<  TC1_ << "  " << TC2_<< " " <<  TC3_ << "  " <<  TC4_ << "  "<<  TC5_ << "  " <<  TC6_ << "  "; 
    stateFormat(Startstate.stackOrdering); 
    cerr<< " accurate? " << Ih_ << "  " << OPM<< endl;
    if(TC1_)
        t1++;
    else if(TC2_)
        t2++;
    else if(TC3_)
        t3++;
    else if(TC4_)
        t4++;        
    else if(TC5_)
        t5++;
    else
        t6++;          
    //show_path();
}


void meet::generatingSuccessor() {//eight immediate neighbors, where (0, 0) is the bottom left corner
         hasSucc_ = hasChild_ = findMs_ = false;
         _min_ = hmin_ = Sfmin_ = COST_MAX;
         Gcur_ =0,  Hcur_ = 0;
         pi_ = false;
         bool prdf_ = false, prdb_ = false;
         if (fExpd_)
         {      
                OpenF.pop(); 
                if(!STATE[fdir_].first.visited)
                {
                     fCounter_++;
                     forw_ = STATE[fdir_].first;
                     pi_ = fdir_.cf.pi_; 
                     Spt_= fdir_.cf.Spt_;  
                     vector <State> suc =  successors(fdir_); 
                     Gcur_ = fdir_.cf.g; Hcur_ = fdir_.cf.h;
                     prdf_ = Gcur_ < Hcur_;
                     if(findASolution_)
                     {
                        locHep_ = hElevation(Gcur_,Hcur_);
                     }
                     ExpansionInForward(fdir_, suc);
                }
         }
         else
         {
                OpenB.pop();
                if(!STATE[bdir_].second.visited)
                {
                    bCounter_++;
                    backw_ = STATE[bdir_].second;
                    pi_ = bdir_.cb.pi_;  
                    Spt_= bdir_.cb.Spt_;  
                    vector <State> suc =  predecessors(bdir_); 
                    Gcur_ = bdir_.cb.g; Hcur_ = bdir_.cb.h;
                    prdb_ = Gcur_ < Hcur_;
                    if(findASolution_)
                    {
                        locHep_ = hElevation(Gcur_,Hcur_);
                    }
                    ExpansionInBackward(bdir_, suc);
                }
         }
         if(findMs_)
         {
            findASolution_ = true;
            prtIbG_ = Gcur_ ; prtIbH_ = Hcur_;
            IbDir_ = fExpd_; prti_ = pi_;
            lSpt_ = acrt_ ? true: false;
            locSol_ = bSoluF_ =  bSoluB_ = false; //fExpd_ ? f_min_ >= C_curr : b_min_ >= C_curr
            
            c_bt = fExpd_ ? f_min_ >= C_curr : b_min_ >= C_curr;
         }
         else
         {
               if(findASolution_ && !find_solution) //&& llmin_ + epsilon < C_curr
	       { 
	                Sfmin_ = sval_.f;
	                if(Spt_ && f_dMin < C_curr &&  !lSpt_ && (sval_.f < C_curr))
                        {
                            lSpt_ = true;
                        }
                        find_solution = TerminationCondition(Gcur_,Hcur_,pi_,false);
	                preDf_ = prdf_, preDb_ = prdb_;
	       }
	       fExpd_ ? f_min_ = _min_: b_min_ = _min_; 
         }        
}

bool meet::TC1()
{  
   return TC1_= (fmin_ >= C_curr || (f_dMin >= C_curr && !bpi_));     
}

bool meet::TC2(bool fpi_, bool bpi_, bool fmt_, bool  bmt_, int fg, int bg)//&& fdir_.cf.h < lnh_ && bdir_.cb.h < lnh_
{
     return TC2_= ((!fpi_ && !bpi_) && !fmt_ && !bmt_  && fg+bg+1 > C_curr);
}

bool meet::TC3(int gCur_, int hCur_)
{
    //cerr << Ih_ << " " << (gCur_ >= prtIbG_) << "  "<< prti_ << " " << pi_ << "  " << hCur_ << "  " << prtIbH_ << " " << locSol_ << "  "<< arch_ << endl;   
    /*if(gCur_ > prtIbG_)
    return sval_.g > IMV_.g;*/
    //return TC3_= (Ih_ &&  ( prti_ && pi_ ? hEq(gCur_, hCur_) : (gCur_ > prtIbG_ || ( gCur_ == prtIbG_  && !pi_ && (!locSol_ || arch_))) &&  (sval_.g > IMV_.g || (hCur_ > prtIbH_) || (arch_&& hElevation(gCur_,hCur_)))));    
    return TC3_ = (Ih_ && (pi_? prti_ && hEq(gCur_, hCur_) : (!locSol_ ? hEq(gCur_, hCur_) : arch_ && hElevation(gCur_, hCur_))));
}
bool meet::TC4(int gCur_, int hCur_)
{
     
     //cerr << (!bSoluF_ || !bSoluB_) << "  " << sval_.pi_ << "  " << (Sfmin_ >= C_curr) << " " << arch_ << "  " << sval_._f << endl;
     bool sDirSol_ = !Ih_ && ((!bSoluF_ &&fExpd_) || (!bSoluB_ && !fExpd_));
     bool sDir_ = false;
     //return false;
     if(!sDirSol_ || ((sDir_ = (fExpd_== IbDir_)) && (sval_._f < C_curr && (f_dMin < C_curr && Eq(gCur_, hCur_))))) 
     {return TC4_ = false;}  
     
     if(!sDir_)
     {
        return (TC4_ = (hElevation(gCur_,hCur_) || Sfmin_ >= C_curr));
     }
     else if(!pi_)
     {
        //cerr << gCur_ << "  " <<Hcur_ << "  " <<fExpd_ << "  " << preDb_ << "  " << preDf_ << endl;  
        if(!Eq(gCur_, hCur_) || hCur_ >= lnh_)
        return (TC4_ = (gCur_ == hCur_ ? (fExpd_ ? preDb_ : preDf_): true) && ((!sval_.pi_ && Sfmin_ >= C_curr) || hElevation(gCur_,hCur_) ));//
     }
     else
     {
        return (TC4_ = (hEq( gCur_, hCur_) || Sfmin_ >= C_curr) );
     }
     return TC4_ = false;
}
bool meet::TC5(int hCur_)
{
     //&& (!sval_.pi_&& (prti_ ? pi_ : true ))
     //return false;
     bool sDirSol_ = ((!bSoluF_ &&fExpd_) || (!bSoluB_ && !fExpd_));
     bool El = (fExpd_== IbDir_) && (Gcur_ == prtIbG_ && hCur_ == prtIbH_);    
       
     return TC5_= sDirSol_ && ((!Ih_ && !El  && !arh_ && c_bt )  || fExpd_!= IbDir_ || (!sval_.pi_ && !El  && !arh_ && c_bt)) && (sval_.g+sval_.h >= C_curr);
}

bool meet::TC6(int gCur_, int hCur_)
{
    return TC6_= (Spt_ && !lSpt_ && !Ih_ && !sval_.pi_ && Sfmin_ >= C_curr && gCur_ < hCur_ );
}

bool meet::hElevation(int gCur, int hCur)
{
    return(gCur >= prtIbG_ && hCur >= prtIbH_);
}

bool meet::hEq(int gCur, int hCur)
{
    return(gCur > prtIbG_ && hCur >= prtIbH_) || (gCur >= prtIbG_ && hCur > prtIbH_);
}

bool meet::Eq(int gCur, int hCur)
{
    return(gCur == prtIbG_ && hCur == prtIbH_);
}

bool meet::betterSolution(int sf_, int sg_, bool cpi_)
{
     if(Ih_)
     {
        if(prti_ && fExpd_ == IbDir_ && pi_)
        { return (sf_ < C_curr); }
        return (cpi_? !arch_ && sg_ <= IMV_.g : sf_ < C_curr);  
     }
     else
     { 
            //cerr << IMV_.h << " " << lnh_ << " " << pi_  << " " << !arh_ << endl;
      return ((IMV_.h >= lnh_  || (pi_ &&  Gcur_ != prtIbG_ && Hcur_ != prtIbH_  )) ? sf_ <= C_curr: sf_ < C_curr); 
     }  
}
void meet::ExpansionInForward(State cur_, vector<State> Suc)
{
        //stateFormat(cur_.stackOrdering);
        //cerr << " is expanding in forward search with g:" << cur_.cf.g << " " << cur_.cf.h <<" " << cur_.cf.f << " " << fCounter_ << endl;
        cur_.cf.visited = true;
        STATE[cur_].first = cur_.cf;
        bool update = false, nPath_ = false;
        for(State child: Suc)
        {
            UpdateVertex(child.cf, cur_.cf.g, cur_.key , parentF[child],cur_,update,child, child.cb.pre,child.cf.pi_);
            if(update)
            {
               child.cf.prtf_ = fmin_;
               STATE[child].first = child.cf;
               EsCost backward = STATE[child].second;
               bool localMeet = false;
               if((backward.pre && !backward.visited) || child.key == end_)
               {
                    int leastC = child.cf.g + backward.g;
                    leastCostFound(leastC,child,child.cf.f,child.cf.g,backward.g,child.cf.h,child.cf.pi_,nPath_,localMeet,backward.prtf_);
               }
               _min_ = min(child.cf.f,_min_); 
               if(findASolution_)
               {
                 if(localMeet)
                 { 
                     STATE[child].first.meet = !(STATE[Ib_].first.meet = false);
                     EsCost _prtb = STATE[parentB[child]].second; 
                     arch_ = Hcur_ > child.cf.h;
                     arh_ = pi_ && !_prtb.pi_;
                     ari_ = pi_ && _prtb.pi_ && (_prtb.h <= child.cf.h);
                     //cerr << _prtb.g << " " << _prtb.h << endl;
                     acrt_ = (child.cf.h >= backward.g && !arch_ && _prtb.h <= backward.h); 
                 }
                 
                 child.cf.Spt_ = true;
                 if(Ih_ && !pi_ && locHep_)
                 { locSol_ = !locSol_ ? (child.cf.g + child.cf.h < C_curr) : locSol_;}
                 
                 //
                 if((!bSoluF_ || !bSoluB_)  &&  betterSolution(child.cf.f, child.cf.g, child.cf.pi_))  
                 {  fExpd_ ? bSoluF_ = true : bSoluB_ = true; /*find a refine state**/}
                 
               }
               getMinMetricValue_((localMeet ? curL_: child.cf.f),child.cf.g,child.cf.h,child.cf.pi_,false, cur_.cf.g, cur_.cf.h, Sfmin_,sval_,child.cf.nf);
               if(child.cf.f <= C_curr)
                   OpenF.push(child);
               //stateFormat(child.stackOrdering);
               //cerr << "\t\t in forward->  cost : g: " << child.cf.g << " h: " << child.cf.h << "  f:"  << child.cf.f << endl;
            }
        }
}

void meet::leastCostFound(int leastCost_, State suc, int f_ ,int fg_, int bg_, int hc_, bool pi_, bool &firstS_, bool &lmet, int prtf)
{       
        bool newState_ = false;
        if(leastCost_  < C_curr || (newState_ = (findASolution_ && !firstS_ && C_curr == leastCost_ && min(fg_,bg_) < IGMin_)))
        {
            C_curr = leastCost_;
            Ib_ = suc;
            if(!findASolution_)
            {
                findASolution_ = true;
            }
            if(!firstS_)
            {
               if(!newState_)
               {
                  firstS_ = true;
                  OPM =  fCounter_ + bCounter_;
               }
               Ih_ = (fg_ > bg_);
               curL_ = !pi_ ? f_: (fg_+fg_);
               lnh_ = bg_;
               lmet = findMs_ = true;
               OSME = fCounter_ + bCounter_;
               IMV_ = NGcost_(f_,fg_,hc_,pi_,false);
            }
            IGMin_ = min(fg_,bg_);
               //if(newState_)
            cerr << "---------------------------------------------------------------> found a solution here " << leastCost_ << " " << fg_ << "  " << bg_ << " " << IGMin_ << " " << hc_ << " " << curL_ << " --->" << pi_ << " " << Ih_ << " " << (f_dMin>=C_curr) << endl;
        }
}

void meet::getMinMetricValue_(int sf_, int sg_, int sh_, bool pi_, bool Mms_ , int Gcur, int Hcur, int &lminF_, NGcost_ &val_, int mt_)
{
       if(findASolution_)
       {
            if(sf_ < lminF_ || (sf_ == lminF_ && (!Ih_ && sf_ == C_curr && !pi_ && sh_<= lnh_) ))
            {
               lminF_ = sf_;
               val_ = NGcost_(sf_,sg_,sh_,pi_,mt_);
            }
            if(prti_)
            {
                 hmin_= min(hmin_,sh_); 
            }     
       }   
}

void meet::ExpansionInBackward(State cur_, vector<State> Suc)
{
        //stateFormat(cur_.stackOrdering);
        //cerr << " is expanding in backward search with g:" << cur_.cb.g << " "<< cur_.cb.h<< "  " << cur_.cb.f << " " << bCounter_ << endl;
        cur_.cb.visited = true;
        STATE[cur_].second = cur_.cb;
        bool update = false, nPath_ = false;
        for(State child: Suc)
        {
            UpdateVertex(child.cb, cur_.cb.g, cur_.key , parentB[child], cur_,update,child, child.cb.pre ,child.cb.pi_);
            if(update)
            {
               child.cb.prtf_ = fmin_;
               STATE[child].second = child.cb;
               EsCost forward = STATE[child].first;
               bool localMeet = false;
               if((forward.pre && !forward.visited) || child.key == start_)
               {
                    int leastC = child.cb.g + forward.g;
                    leastCostFound(leastC,child,child.cb.f,child.cb.g,forward.g,child.cb.h,child.cb.pi_,nPath_,localMeet,forward.prtf_);
               }
               _min_ = min(child.cb.f,_min_); 
               if(findASolution_)
               {
                 if(localMeet)
                 {  
                     STATE[child].second.meet = !(STATE[Ib_].second.meet = false); 
                     EsCost _prt = STATE[parentF[child]].first;
                     arch_ = Hcur_ > child.cb.h;
                     arh_ = pi_ && !_prt.pi_;
                     ari_ = pi_ && _prt.pi_ && _prt.h <= forward.h;
                     //cerr << _prt.g << " " << _prt.h << endl;
                     acrt_ = (child.cb.h >= forward.g && !arch_ && _prt.h <= forward.h);
                 }
                 child.cb.Spt_ = true;
                 if(Ih_ && !pi_&& locHep_)
                 { locSol_ = !locSol_ ? (child.cb.g + child.cb.h < C_curr): locSol_;}
                  
                 if((!bSoluF_ || !bSoluB_)  &&betterSolution(child.cb.f, child.cb.g, child.cb.pi_))  
                 {  fExpd_ ? bSoluF_ = true : bSoluB_ = true; /*find a refine state**/}
               }
               getMinMetricValue_((localMeet ? curL_: child.cb.f),child.cb.g,child.cb.h,child.cb.pi_,false, cur_.cb.g, cur_.cb.h,Sfmin_,sval_,child.cb.nf);
               if(child.cb.f <= C_curr)
                  OpenB.push(child);
               //stateFormat(child.stackOrdering);
               //cerr << "\t\t in backward-----> cost : g: " << child.cb.g << " h: " << child.cb.h << "  f:"  << child.cb.f << endl;
            }
        }
}

bool meet::TerminationCondition(int Gcur, int Hcur, bool pi_, bool PreExpansion)
{ 
         if(PreExpansion)
         {
                 if(TC1() || TC2(fdir_.cf.pi_,bdir_.cb.pi_,fdir_.meet, bdir_.meet,fdir_.cf.g,bdir_.cb.g))
                 {
                   return true;
                 }
         }
         else
         {
                 if(TC3(Gcur, Hcur) || TC4(Gcur, Hcur) || TC5(Hcur) || TC6(Gcur, Hcur))
                 {
                    return true;
                 }  
         }
         
         return false;
}

void meet::UpdateVertex(EsCost &sucC, int g_value, unsigned long long keyV, State &Parent, State cur_, bool &update, State & suc, bool &hasP, bool &dig_)
{

         if(!sucC.visited)
         {
              if(!sucC.pre)
              {
                sucC.h = gapHeuristic(suc, 1);
                //cerr <<  "\t\t\t heuristic " << sucC.h << endl;
                sucC.g = COST_MAX;
               }
               int edgeCost = 1;
               int pCost = g_value + 1;
               if(sucC.g > pCost)
               {
                  hasChild_ = update = true;
                  hasP = true;
                  sucC.pre = true;
                  sucC.g = pCost;
                  sucC.prt = keyV;
                  sucC.nf = sucC.f = pCost + sucC.h;
                  if(pCost > sucC.h)
                  {
                     sucC.f = pCost + pCost;
                     dig_ = true;
                  } 
                  else if(dig_)
                  {
                    dig_ = false;
                  }  
                  Parent = cur_;
               }                      
         }
}


    unsigned long long meet::generateKey(vector <int > ordering) {
            /*
                    FNV-1a
            */
            unsigned long long offset_basis = 0xCBF29CE484222325;
            unsigned long long fnv_prime = 0x100000001B3;
            for (unsigned int value : ordering) {
                offset_basis ^= value;
                offset_basis *= fnv_prime;
            }
            return offset_basis;
    }
    
    int meet::gapHeuristic(const State& state, int X_) {
		// Using gap heuristic from - Landmark Heuristics for the Pancake Problem
		// Where add 1 to heuristic if the adjacent sizes of the pancakes differs more than 1
		// The GAP-X heuristics exclude gaps that include pancakes among the smallest X in size, X = 0 denotes the regular gap heuristic
                int sum = 0;
                if(!fExpd_)
                {
                   sum =  CalculateH(state.stackOrdering,Startstate.stackOrdering,X_); 
                }
                else
                {
                   sum = CalculateH(state.stackOrdering,Goalstate.stackOrdering,X_); 
                } 
		return sum;
   }
   
   void meet::flipOrdering(std::vector<State>& succs,
            std::vector< int> ordering,
            int loc) {
        int start = 0;
        int end = loc;
        while (start < end) {
            std::swap(ordering[start++], ordering[end--]);// one can also use std::reverse(pancakes.begin(), pancakes.begin() + loc + 1);
        }
        State st = State(ordering, loc, generateKey(ordering));
        if(fExpd_ ? !STATE[st].first.visited :  !STATE[st].second.visited)
        {
              succs.push_back(st);
        }
           
   }
   
   std::vector<State> meet::successors(const State& state) {
		std::vector<State> successors;
               for (int i = 10 - 1; i > 0; --i) {
			// Don't allow inverse actions, to cut down on branching factor
			if (state.label == i) continue; 

			flipOrdering(successors, state.stackOrdering, i);
               }
		return successors;
   }
   


   std::vector<State> meet::predecessors(const State& state) {
		std::vector<State> predecessors;
        for (int i = 10 - 1; i > 0; --i){
			flipOrdering(predecessors, state.stackOrdering, i);
        }
		return predecessors;
	}



/********** Return vertices on an optimal anyangle path ****************/
void meet::show_path() {
      stack<State > pt;
      State cur_ = Ib_;
      pt.push(cur_);
      while(STATE[cur_].first.pre) {
         cur_ = parentF[cur_];
         pt.push(cur_);
     }
     //cerr << "The optimal path: " << endl;
     while(!pt.empty()) {
        State optimal = pt.top();
        for (int r = 0; r < optimal.stackOrdering.size(); r++) {
            cerr << std::setw(3) << optimal.stackOrdering[r] << " ";
        }
        cerr << endl;
        pt.pop();
     }
     cur_ = Ib_;
     while(STATE[cur_].second.prt != -1)
     {
         cur_ = parentB[cur_];
         for (int r = 0; r < cur_.stackOrdering.size(); r++) {
            cerr << std::setw(3) << cur_.stackOrdering[r] << " ";
         }
         cerr <<endl;
     }
     cerr << endl;
     //cerr << "the meeting point is " << Ib_.l.x << " " << Ib_.l.y << endl;
}
