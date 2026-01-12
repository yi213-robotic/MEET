#include "meet.h"
/***** Construct meet *****/
meet::meet(){
    epsilon = 1e-9;//numeric_limits<double>:: epsilon()
    C_curr = infinity = numeric_limits<double>:: infinity();
    fCounter_ = bCounter_ = 0;
    firstS_ = find_solution = findASolution_  = false;
    TC1_ = TC2_ = TC3_ = TC4_ = TC5_ = TC6_ = false;
    
}
/***** Deconstructbeas */
meet::~meet(){}
/********** Initialize location *************/
vertex meet::Initialize_(int x, int y, int ID) {
  vertex init;
    init.l.x =x;  init.l.y = y; init.l.Id = ID;
  return init;
}

int meet::ID(int x, int y)
{ return (x<<16|y); }

/********** Euclidean distance between two vertices  **************/
double meet::EuclideanDistance(int dx, int dy)
  {return sqrt((dx)*(dx) + (dy)*(dy));}

double meet::OctileDistance(int dx, int dy)
{
   double delta_x = fabs(dx);
   double delta_y = fabs(dy);
   double largest = max(delta_x,delta_y);
   double smallest = min(delta_x,delta_y);
   return DigCost*smallest + (largest - smallest);
}

/********** parsing a map and get the corner verrtices ************/
meet::meet(vector<bool > grid_, int width, int height, int sx, int sy, int gx, int gy) {//
    epsilon = 1e-9;//numeric_limits<double>:: epsilon()
    fmin_ = C_curr = infinity = numeric_limits<double>:: infinity();
    OSME_ = fCounter_ = bCounter_ = 0;
    firstS_ = find_solution = findASolution_ = false;
    map_x = width;map_y = height;
    int p = ((width+2)<<16)|(height+2);
    blocks_.resize(p);
    start_x = sx; start_y = sy;
    end_x = gx;  end_y=gy;
    cerr << "Path finding from ("<<start_x << "," << start_y << ") to  ("<<end_x << "," << end_y << ") on the map with the weith: " << map_x << ", and height: " << map_y << "." << endl;
    vertex_.resize((map_x+2)<<16|(map_y+2));  
    for (int x = 0; x <= map_x+1; x++) {
        for (int y = 0; y <= map_y+1; y++){
             int VID = x<<16|y;
                 vertex_[VID] = Initialize_(x,y,VID);
                 blocks_[VID] = true;//Set all cells to be blocked.
        }
    }
    for (int x = 0; x < map_x; x++) {
        for (int y = 0; y < map_y; y++) {
            if (!grid_[x<<16|y]) {
               blocks_[(x+1)<<16|(y+1)] = false;  
            }
        }
    }
    START = ID(start_x,start_y);
    GOAL = ID(end_x,end_y);
    Initialize_And_Insert_start(); 
}

/******* Insert start into Open ***********/
void meet::Initialize_And_Insert_start() 
{
    vertex_[GOAL].gol = true;
    vertex_[START].src = true;
    OpenB.push(vertex_[GOAL]); 
    OpenF.push(vertex_[START]);
}


bool meet::stateExpansion()
{
   
   fExpd_ = false;  
   fdir_ = OpenF.top();
   bdir_ = OpenB.top();   
   fdir_ = vertex_[fdir_.l.Id];
   bdir_ = vertex_[bdir_.l.Id];   
     
   if(fdir_.vis)
   {
       OpenF.pop();
       return false;
   }
   if(bdir_.vis)
   {
       OpenB.pop();
       return false;
   }
   
   fmin_ = std::min(fdir_.cf.f,bdir_.cb.f);
   
   /* evaluate whether need to terminate search**/
   if(findASolution_ && (find_solution = TerminationCondition(0, 0, false, true)))
   { return true; }
   
   /* select the minimum priority for expansion**/
   if(fdir_.cf.f + epsilon < bdir_.cb.f)
   {  
       minValueOps(bdir_.cb.f,bdir_.cb.pi_); 
       fExpd_ = true; 
   }
   else if(fdir_.cf.f > bdir_.cb.f + epsilon)
   { 
         fExpd_ = false;
         minValueOps(fdir_.cf.f,fdir_.cf.pi_); 
   }
   else// tie breaking
   {
        if(fdir_.cf.g + epsilon < bdir_.cb.g)
        {  
            fExpd_ = true; 
            minValueOps(bdir_.cb.f,bdir_.cb.pi_); 
        }
        else if(fdir_.cf.g > bdir_.cb.g + epsilon)
        {  
             fExpd_ = false; 
             minValueOps(fdir_.cf.f,fdir_.cf.pi_);  
        }
        else
        {  
            fExpd_ = true;
            minValueOps(bdir_.cb.f,bdir_.cb.pi_);  
        }
   }
   return true;
}


void meet::forwardSearch()
{
      //cerr << "expanding in forward direction: (" << fdir_.l.x << "," << fdir_.l.y<< " ), f: " << fdir_.cf.f << " g: " << fdir_.cf.g << " h: "<< fdir_.cf.h << " " << fCounter_ << " "  << fdir_.cb.pr << "  " <<  fdir_.mt << endl;
      vertex_[fdir_.l.Id].vis = true;  
      generatingSuccessor(fdir_, curG_ = fdir_.cf.g, curH_ =  fdir_.cf.h, fdir_.mt, cPi_ = fdir_.cf.pi_, fdir_.cb.pr, fdir_.cf.Spt_);
}

void meet::backwardSearch()
{

      //cerr << "expanding in backward direction: (" << bdir_.l.x << "," << bdir_.l.y<< "), f: " << bdir_.cb.f << " g: " << bdir_.cb.g << " h: " << bdir_.cb.h  <<"  " << bCounter_ << " " << bdir_.cf.pr << " " << bdir_.mt << endl;         
      vertex_[bdir_.l.Id].vis = true;
      generatingSuccessor(bdir_, curG_ = bdir_.cb.g, curH_ =  bdir_.cb.h,bdir_.mt, cPi_ = bdir_.cb.pi_,bdir_.cf.pr, bdir_.cb.Spt_);
}

void meet::printStatement()
{
      //if(findASolution_ )
      {
           cerr << "********************* MEET finds a solution C_curr: " << C_curr << ", forward g:  " << fdir_.cf.g << " forward h: " << fdir_.cf.h << ", forward f: " <<  fdir_.cf.f << "; backward g: " <<  bdir_.cb.g << ", backward h: " << bdir_.cb.h; 
           cerr << ", backward f: "  << bdir_.cb.f << "; forward vertex [ "<< fdir_.l.x << " " << fdir_.l.y << "], backward vertex [ " << bdir_.l.x << " " << bdir_.l.y;
           cerr << "]; meeting state is:[ " << Ib_.l.x << " " << Ib_.l.y << " ]."  << endl;
      }
}

/************Find an optimal path********************/
double meet::meetSEARCH(double &rt, int &forwardEx_, int &backwardEx_, int &OSME, int &chi_, int &T1, int &T2, int &T3, int &T4, int &T5, int &T6) {
    double startTime = clock();
    bool ProrityC = false;
    int ty = 0, tx = 0, tmx=999999 , tmy=999999;
    vector<pair<int,int> > forwardL, backwardL;
    findASolution_ = false;
    while(!OpenF.empty() && !OpenB.empty()){//OpenList is not empty

          if(!stateExpansion())
             continue;
          if(find_solution)
             break;   
          if(fExpd_)
          {               
                OpenF.pop();
                if(fdir_.vis)
                  continue;
                //forwardL.push_back(make_pair(fdir_.l.x,fdir_.l.y));
                //printStatement();
                fCounter_++;
                /*if(fCounter_ == 552)
                 exit(-1);*/
                forwardSearch();
          } 
          else
          {
                
                OpenB.pop();
                if(bdir_.vis)
                  continue;
                //backwardL.push_back(make_pair(bdir_.l.x,bdir_.l.y));
                //printStatement(); 
                bCounter_++;

                backwardSearch();
          }
          if(find_solution)
             break;
    }
    double endTime = clock();
    double rtime = (double) (endTime - startTime) / CLOCKS_PER_SEC;
    cerr << "meet takes: "<<rtime*1000 <<" (ms)";
    double pathLength = C_curr;
    rt = rtime;
    forwardEx_ = fCounter_;
    backwardEx_= bCounter_;
    OSME = OSME_;
        if(TC1_)
        T1++;
    else if(TC2_)
        T2++;
    else if(TC3_)
        T3++;
    else if(TC4_)
        T4++;        
    else if(TC5_)
        T5++;
    else
        T6++; 
    if(pathLength != 0)
    {
       cerr << " to find the optimal path length: " << setprecision(15) <<pathLength << " and expanded forward vertices: " << fCounter_ << ", backward vertices: " << bCounter_ << endl;
       cerr << " TC occurs when "<<  TC1_ << "  " << TC2_<< " " <<  TC3_ << "  " <<  TC4_ << "  "<<  TC5_ << "  " <<  TC6_ << endl;
       //show_path();
    }
    else
    {
        cerr << " to find no path." << endl;
    }
    return C_curr;
}


bool meet::lSolu(double sf_)
{  return sf_ + epsilon < C_curr; }

bool meet::leqSolu(double sf_)
{ return sf_ < C_curr+ epsilon ;  }

/*******************Constraining, Modifying Propagating range***********/
void meet::generatingSuccessor(vertex cur_, double Gcur, double Hcur, bool Ibest_, bool pi_, bool opt, bool curSpt_) {//eight ibeasediate neighbors, where (0, 0) is the bottom left corner
      int x = cur_.l.x, y = cur_.l.y;
      int Ax = x+1,Ay = y+1,Dx = x-1,Dy = y-1;
      locS_ = hasChild_ = firstS_ = findMS_ = false;
      bool prdf_ = false, prdb_ = false;
      fExpd_ ? prdf_ = Gcur + epsilon < Hcur : prdb_ = Gcur + epsilon < Hcur;
      Spt_ = curSpt_;
      sGmin_ =  Sfmin_ = infinity;
      sval_ = NGcost_(0,0,0,false,false,false,false);
      int locNE=ID(Ax,Ay),locSE=ID(Ax,y),loc=ID(x,y),locNW=ID(x,Ay),locS=ID(x,Dy),locW=ID(Dx,y),locES=ID(Ax,Dy),locWS=ID(Dx,Dy),locWN=ID(Dx,Ay);//
      //bool NE =!vertex_[Ax][Ay].block, SE = !vertex_[Ax][y].block, SW = !vertex_[x][y].block, NW = !vertex_[x][Ay].block, constrainAngles = false;
      bool NE = !blocks_[locNE], SE = !blocks_[locSE], SW = !blocks_[loc], NW = !blocks_[locNW], dr = false, update = false, diagVertex = false;
      if((NW || NE) && !vertex_[locNW].vis){//north = northWest or northEast locNW
          GetNeighbors(x,y+1,cur_,vertex_[locNW],false,update,Gcur,Hcur);// cerr << "\t north neighbor(" << x << "," << y+1 << ") is visible " << endl;
      }
      if((SE || NE)&& (!vertex_[locSE].vis)){//east = SouthEast or NorthEast locSE
          GetNeighbors(x+1,y,cur_,vertex_[locSE],false,update,Gcur,Hcur);// cerr << "\t east neighbor(" << x+1 << "," << y << ") is visible " << endl;
      }
      if((SW || SE)&& (!vertex_[locS].vis)){//south = southWest or southEast locS
          GetNeighbors(x,y-1,cur_,vertex_[locS],false,update,Gcur,Hcur);// cerr << "\t south neighbor(" << x << "," << y-1 << ") is visible " << endl;
      }
      if((SW || NW)&& (!vertex_[locW].vis)){//west = southWest or northWest locW
          GetNeighbors(x-1,y,cur_,vertex_[locW],false,update,Gcur,Hcur);//cerr << "\t west neighbor(" << x-1 << "," << y << ") is visible " << endl;
      }
      if(NE && (!vertex_[locNE].vis)){//northEast locNE
          GetNeighbors(x+1,y+1,cur_,vertex_[locNE],true,update,Gcur,Hcur);// cerr << "\t northeast neighbor(" << x+1 << "," << y+1 << ") is visible " << endl;
      }
      if(SE && (!vertex_[locES].vis)){//southEast locES
          GetNeighbors(x+1,y-1,cur_,vertex_[locES],true,update,Gcur,Hcur);//cerr << "\t southeast neighbor(" << x+1 << "," << y-1 << ") is visible " << endl;
      }
      if(SW && (!vertex_[locWS].vis)){//southWest locWS
          GetNeighbors(x-1,y-1,cur_,vertex_[locWS],true,update,Gcur,Hcur);//cerr << "\t southwest neighbor(" << x-1 << "," << y-1 << ") is visible " << endl;
      }
      if(NW && (!vertex_[locWN].vis)){//northWest locWN
          GetNeighbors(x-1,y+1,cur_,vertex_[locWN],true,update,Gcur,Hcur);//cerr << "\t northwest neighbor(" << x-1 << "," << y+1 << ") is visible " << endl;
      }
      
      if(findMS_)
      {      
             /** This part is only for grid-path planning domain since it is not uniform cost edge domain*/
             
             if(pi_ && !firstS_ && ((fExpd_ ?!pSolF_ : !pSolB_) ? hElevation(Gcur, Hcur) : (bIh_ && sval_.g > maxG_)))
             { 
               find_solution = true;
               if(Ih_)
               { TC3_ = true;}
               else
               {  TC4_ = true;}
             }
             else
             { pSolF_= pSolB_ = true; }
             
             findASolution_ = true;
             IbDir_ = fExpd_;
             prtI_ = pi_;
             bFs_ = firstS_ ;
             bSolu_ = locSol_ = bSoluF_ =  bSoluB_ = false;//(firstS_ || hElevation(IMV_.g,ExIMV_.g,IMV_.h,ExIMV_.h))
             gI_ = Gcur + 1;
             bIh_ = gI_ + gI_ > C_curr;
             maxG_ = IMV_.g;
             if(firstS_)
             { 
                pSolF_ = pSolB_ = false;
                IgI_ = gI_; hI_ = sval_.h;
             }
             prtIbG_ = Gcur ; prtIbH_ = Hcur;
             lSpt_ = acrt_ ? true: false;
             c_bt = fExpd_ ? f_min_ + epsilon> C_curr : b_min_ + epsilon > C_curr;
             OSME_ = fCounter_ + bCounter_;
      }
      else
      {
	        if(findASolution_ && !find_solution && sval_.val_) //&& llmin_ + epsilon < C_curr
	        { 
	            //Sfmin_ = sval_.f;
	            Sfmin_ = locS_ ? sGmin_ : sval_.f;  
	            
	            /** this is for resetting the evaluation of best solution */
	            if(Spt_ && bSoluF_ &&  bSoluB_ && lSpt_ && Sfmin_ + epsilon  > C_curr)
	            { lSpt_ = bSoluF_ =  bSoluB_ = false;}
	            
	            if(Spt_ && f_dMin + epsilon < C_curr &&  !lSpt_ && (Sfmin_ + epsilon  < C_curr))
                    { lSpt_ = true; } /**this is for the TC5 or TC6*/
                                        
                    bSolu_ =  hasBetterSolu();
                    find_solution = TerminationCondition(Gcur,Hcur,pi_,false);
                    preDf_ = prdf_, preDb_ = prdb_;
		}
		fExpd_ ? f_min_ = Sfmin_: b_min_ = Sfmin_; 
      }
}

bool meet::hasBetterSolu()
{
     if(Ih_ && (fExpd_ ? !bSoluF_ : !bSoluB_))
     {
        if(sval_.f + epsilon > C_curr)
        {   return true;}
        else if (bSoluF_ || bSoluB_)
        {   return false;}
     }
     return ((!bSoluF_ &&fExpd_) || (!bSoluB_ && !fExpd_)); 
}

void meet::minValueOps(double sf_, bool spi_)
{
     f_dMin =  sf_;
     bpi_ = spi_; 
}

bool meet::TC1()
{  
   return TC1_= (fmin_ + epsilon > C_curr || (!bpi_ && (f_dMin + epsilon > C_curr)));     
}

bool meet::TC2(bool fpi_, bool bpi_, bool fmt_, bool  bmt_, double fg, double bg)//&& fdir_.cf.h < lnh_ && bdir_.cb.h < lnh_
{
     return TC2_= ((!fpi_ && !bpi_) && !fmt_ && !bmt_  && gre(fg+bg+1,C_curr));
}

bool meet::TC3(double gCur_, double hCur_)
{
     /** This is for TC3, where the g -cost of a meeting state from its generating side is higher than from the other side, and nothing ahead cannot exceed it */
     
     return TC3_= (Ih_ && !Spt_ &&bSolu_ && (cPi_? prtI_ && ((hEq(gCur_,hCur_)) || (fExpd_!= IbDir_ && Sfmin_ > C_curr+ epsilon)) : (!locSol_ ? (!sval_.pi_ && !lSolu(Sfmin_)) ||  hEq(gCur_, hCur_) : Dhill_ && hElevation(gCur_, hCur_))));
}
bool meet::TC4(double gCur_, double hCur_, bool pi_)
{
     /** This is for TC4, where the g -cost of a meeting state from its generating side is lower, but no further state could improve the incumbent solution*/
     bool sDirSol_ = !Ih_ && bSolu_;
     bool sDir_ = false;
     //return false;
     if(!sDirSol_ || ((sDir_ = (fExpd_== IbDir_)) && Eq(gCur_, hCur_) &&(sval_.g + sval_.h  + epsilon < C_curr && (f_dMin + epsilon < C_curr)))) 
     {return TC4_ = false;}  
     
     if(!sDir_)
     {
        return (TC4_ = (hElevation(gCur_,hCur_) || (bSolu_ && (!sval_.pi_ || pi_) && Sfmin_ + epsilon > C_curr)));
     }
     else if(!pi_)
     {
        
        if(!Eq(gCur_, hCur_) || hCur_ >= lnh_)
        return (TC4_ = (gCur_ == hCur_ ? (fExpd_ ? preDb_ : preDf_): true) && ((!sval_.pi_ && Sfmin_ >= C_curr) || hElevation(gCur_,hCur_)));//
     }
     else
     {
        return (TC4_ = hEq( gCur_, hCur_) || Sfmin_ >= C_curr);
     }
     return TC4_ =false;
}
bool meet::TC5(double Gcur, double Hcur)
{
     bool El = (fExpd_== IbDir_) && Eq(Gcur,Hcur);    
     return TC5_ = bSolu_ && ((!Ih_ && !El  && !prtI_ && c_bt )  || (fExpd_!= IbDir_) || (!sval_.pi_ && !El  && !prtI_ && c_bt)) && IMV_.g + IMV_.h + epsilon < C_curr && (sval_.g+sval_.h  > C_curr+ epsilon);
}

bool meet::TC6()
{
    return TC6_= (Spt_ && !lSpt_ && (!Ih_ || bFs_ )  && !sval_.pi_ && Sfmin_ + epsilon> C_curr);
}

bool meet::hElevation(double gCur, double hCur)
{ return (gCur + epsilon > prtIbG_  && hCur + epsilon > prtIbH_); }

bool meet::hEq(double gCur, double hCur)
{ return(gCur > prtIbG_ + epsilon && hCur + epsilon > prtIbH_) || (gCur + epsilon > prtIbG_ && hCur > prtIbH_+ epsilon); }

bool meet::Eq(double gCur, double hCur)
{ return(gCur + epsilon > prtIbG_ && gCur < prtIbG_ + epsilon  && hCur + epsilon > prtIbH_ && hCur < prtIbH_+ epsilon ); }

bool meet::TerminationCondition(double Gcur, double Hcur, bool pi_, bool PreExpansion)
{ 
         if(PreExpansion)
         {
                 if(TC1() || TC2(fdir_.cf.pi_,bdir_.cb.pi_,fdir_.mt, bdir_.mt,fdir_.cf.g,bdir_.cb.g))
                 {
                   return true;
                 }
         }
         else
         {
                 if(TC3(Gcur, Hcur) || TC4(Gcur, Hcur, pi_) || TC5(Gcur, Hcur) || TC6())
                 {
                    return true;
                 }  
         }
         
         return false;
}


bool meet::refineState(double sf_, double sg_, double sh_, bool vPI_)
{
        if(Ih_)// generating side has larger g-value
        {
               if(vPI_) // if the successor is the privot state
               { /*if gI > C_curr and Ib is corresponds to the first discovery of C_curr, Meet considers sg_ < gI_. Otherwise sg <= gI_**/ 
                 return  (bIh_ && bFs_) ? les(sg_,gI_) :  leq(sg_,gI_); 
               }
               else
               {/* if gI > C_curr, Meet considers sf_ <= C_curr. Otherwise sf_ < C_curr **/
                 return bIh_ ? leqSolu(sf_) : lSolu(sf_); 
               }
        }
        return (sg_ + epsilon < IMV_.g || IMV_.g + IMV_.h + epsilon > C_curr) && lSolu(sf_);
}

void meet::updateMetricValues(int lx, int ly, bool diagNeighbor, Cost &vc_, double cg, double ch ,int &curId_, int sId_ , int ox, int oy, bool oprD_, double bg_, bool &sDm_, bool &sms_, double bf, bool curDm, bool &child_, bool vis_)
{
       bool pIs_ = false, cPI_ = true; //deadend_ = true;
       valueUpdating(lx,ly,diagNeighbor,vc_,cg,curId_,ox,oy,oprD_,bg_,sDm_,sms_,bf,curDm,child_);
       if(child_ || (minEdge(vc_.g,cg)))
       {
             if(findASolution_)
             {
                 vc_.Spt_ = child_;
                 
                 if((child_ && !diagNeighbor))
                 { sGmin_ = min(vc_.f,sGmin_);locS_ = true;}//cPi_ && prtI_
                 
                 if((!pSolF_ || !pSolB_) && vc_.pi_ && (vc_.g + epsilon < IgI_ || curH_ + epsilon < prtIbH_ || vc_.h + epsilon < hI_))
                 {  fExpd_ ? pSolF_ = true : pSolB_ = true;  }
                 
                 if(Ih_ && !cPi_&& locHep_)
                 { locSol_ = !locSol_ ? (vc_.g + vc_.h < C_curr): locSol_;}
                 
                 if((!bSoluF_ || !bSoluB_)  && refineState(vc_.f,vc_.g,vc_.h,vc_.pi_))  
                 {  fExpd_ ? bSoluF_ = true : bSoluB_ = true; /*find a refine state**/}
             }
             getMinMetricValue_((findMS_ ? curCost_ : vc_.f),vc_.g,vc_.h,vc_.pi_, false, cg, ch,Sfmin_,sval_,sms_,pIs_,sId_,!diagNeighbor);   
       }
}

void meet::getMinMetricValue_(double sf_, double sg_, double sh_, bool digm_, bool Ms_ , double Gcur, double Hcur, double &lminF_, NGcost_ &val_, bool smt_, bool pI_, int ID_, bool uiCost)
{
          if(sf_ + epsilon < lminF_ )
          {
             lminF_ = sf_;
             val_ = NGcost_(sf_,sg_,sh_,digm_,smt_,pI_,uiCost); 
          }
}

/*********** Constraining Propagating range from surrounding neighors ***************/
void meet::valueUpdating(int x, int y, bool diagNeighbor, Cost &suc_, double cur_g, int &prt, int tx, int ty, bool oprDr, double rg, bool &pi_, bool &smt_, double rf, bool curD_, bool &child_)
{    
        if(!suc_.pr)
        {
            //suc_.h = EuclideanDistance(x-tx,y-ty);
            suc_.h = OctileDistance(x-tx,y-ty);
            suc_.g = infinity;
        }
        double edge_Cost =1;
        if(diagNeighbor)
        { edge_Cost = DigCost;}
        double gCost = cur_g + edge_Cost;
        double leastC = 0;
        if(suc_.g > gCost + epsilon)
        {
              suc_.prt = prt;
              suc_.g = gCost;
              suc_.nf = suc_.f = suc_.g + suc_.h; 
              suc_.pr = hasChild_ = child_ = true;
              pi_ = (suc_.g > suc_.h + epsilon);                  
              if(!oprDr)
              {
                  suc_.f = pi_ ? suc_.g + suc_.g : suc_.f; //updating heuristic value
                  culling_ = (findASolution_  && suc_.f+epsilon  >  C_curr);
              }  
              else
              {
                  leastC = suc_.g + rg; 
                  bool Ibest_= false;
                  //cerr << " the reverse tree's g-value is " << rg << " " << firstS_ << endl;
                  if(leastC + epsilon < C_curr || (Ibest_ = ( !(leastC > epsilon + C_curr) && (!firstS_ && min(suc_.g,rg) + epsilon < IGMin_))))
                  {
                           curCost_ = (!pi_ ? suc_.f: (suc_.g+suc_.g)); 
		            suc_.f = Ih_ ? leastC : curCost_;   //updating heuristic
		            Ih_ = (suc_.g > rg + epsilon);
		            if(findASolution_)
		            { vertex_[Ib_.l.Id].mt = false; }
		            smt_ = localFind_ = findMS_ = true;
		            lnh_ = rg; ExIMV_ = IMV_;
		            IMV_ = NGcost_(suc_.f,suc_.g,suc_.h,pi_,smt_,false,true); // recording the current meeting state
		            C_curr = leastC;
		            IGMin_ = min(suc_.g,rg);
		            acrt_ = (suc_.h + epsilon> rg && !cPi_); 
		            OSME_ = fCounter_ + bCounter_;
		            if(!Ibest_)
		            { firstS_ = true; //cerr << "\t here we found a new path **********************************************************!"<<endl;
		            }
		            else
		            { //cerr << "\t we found a better meeting state *****************************************************!"<< endl;  
		            }
		            
                  }
                  else
                  {   
                            suc_.f = !pi_ ? suc_.f: suc_.g+suc_.g;  //updating heuristic value
                            culling_ = true;
                  }
              }
              
              /*if(!culling_)
              { cerr << " \t\t ("<< x<< "," << y << ")," <<" g:" << suc_.g << ", h: " << suc_.h << ", f: " << suc_.f << ", origin (" << tx << "," << ty << "), is on reverse tree " << endl; }
              else
              { cerr << " \t\t Pruning: ("<< x<< "," << y << ") : " <<" g:" << suc_.g << ", h: " << suc_.h << ", f: " << suc_.f << ", origin (" << tx << "," << ty << "), is on reverse tree "<< endl; }*/
        }
}


bool meet::minEdge(double v1, double v2)
{ // sg >= cg +1 && sg <= cg+1
    return geq(v1,v2+1) && leq(v1,v2+1);
}

bool meet::leq(double v1, double v2)
{
      return (v1 < v2 + epsilon);
}

bool meet::geq(double v1, double v2)
{
      return (v1 + epsilon > v2);
}

bool meet::les(double v1, double v2)
{
      return (v1 + epsilon < v2);
}

bool meet::gre(double v1, double v2)
{
       return (v1 > v2 + epsilon);
}

void meet::GetNeighbors(int x, int y, vertex &cur_, vertex &suc, bool diagNeighbor, bool &update, double Gcur, double Hcur)
{
           localFind_ = culling_ = false;
           bool childHas_ = false, pI_ =  false, checking_ = false;
           double h_ =0;
           if(findASolution_)
           {  locHep_ = hElevation(Gcur, Hcur);  }
           
           if(fExpd_)
           {
               updateMetricValues(x,y,diagNeighbor,suc.cf,cur_.cf.g,cur_.cf.h,cur_.l.Id,suc.l.Id,end_x,end_y,(suc.cb.pr) || suc.gol,suc.cb.g,suc.cf.pi_,suc.mt,suc.cb.f,cur_.cf.pi_,childHas_, suc.vis);
               h_ = suc.cf.h; 
               if(childHas_ && !culling_)
               {  OpenF.push(suc); }   
               //cerr << "\t forward neighbor is------->: " << x << " " << y << ", f: " << suc.cf.f << " g: " << suc.cf.g << " h: " << suc.cf.h << " bf " << suc.cb.pr << " " << suc.cf.nf << "  " << (suc.cb.g + suc.cf.g) << "  " << suc.cb.g << " -- " << suc.cb.h   << endl;
           }
           else
           {
               updateMetricValues(x,y,diagNeighbor,suc.cb,cur_.cb.g,cur_.cb.h,cur_.l.Id,suc.l.Id,start_x,start_y,(suc.cf.pr) || suc.src,suc.cf.g,suc.cb.pi_,suc.mt,suc.cf.f,cur_.cb.pi_,childHas_, suc.vis);
               h_ = suc.cb.h; 
               if(childHas_ && !culling_)
               {  OpenB.push(suc); } 
               //cerr << "\t backward neighbor is------->: " << x << " --- " << y << ", f: " << suc.cb.f << " g: " << suc.cb.g << " h: " << suc.cb.h << " ff " << suc.cf.pr <<" " << suc.cb.nf << "  " << (suc.cb.g + suc.cf.g) << " "  << suc.cf.g << "  " << suc.cf.h << endl;
           }
           if(localFind_)
           {  Ib_ = suc; findMS_ = true; Dhill_ = Hcur > h_ + epsilon; }           
}

/********** Return vertices on an optimal anyangle path ****************/
void meet::show_path() {
      stack<vertex > pt;
      vertex cur_ = Ib_;
      pt.push(cur_);
      while(cur_.cf.pr) {
         cur_ = vertex_[cur_.cf.prt];
         pt.push(cur_);
     }
     cerr << "The optimal path: " << endl;
     while(!pt.empty()) {
        vertex optimal = pt.top();
        cerr << "("<<optimal.l.x << ","<< optimal.l.y << ")," << endl;
        pt.pop();
     }
     cur_ = Ib_;
     while(cur_.cb.pr)
     {
         cur_ = vertex_[cur_.cb.prt];
         cerr << "("<<cur_.l.x << ","<< cur_.l.y << ")," << endl; 
     }
     cerr << "the meeting point is " << Ib_.l.x << " " << Ib_.l.y << endl;
}
