#include "meet.h"
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <numeric>
#include <algorithm>
#include <ostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
using namespace std;

struct TestPair{	
   int sx, sy, gx, gy;
   TestPair(int stx, int sty, int gox, int goy){
	   sx = stx; sy = sty; gx = gox; gy = goy;
   }
};

void loading_map(string map, vector<bool> &block, int &map_x, int &map_y) {
    FILE *f;
    char tmap2[1024];
     strcpy(tmap2,map.c_str());
	f = fopen(tmap2, "r");
	int xs=0,ys=0;
	char chr= '\0';
	int src = 0, t = 0;
	fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &ys, &xs);
	map_x = xs;
	map_y = ys;
	block.resize(xs<<16|ys);
    for(int i = ys-1; i >= 0;i--) { //row y
		for(int j = 0; j<xs;j++) { // col x		
			char c;
				do {
					fscanf(f, "%c", &c);
					//cerr << c;
				} while (isspace(c));
			if(c == 'T' || c == '@') {	//c=='#'
				//cerr << i << " " << j << endl; 
				block[j<<16|i] = true;
			}
		}
		//cerr << endl;
	} 
}

bool Scen(string path, vector<TestPair > &TestP, string &map){
		  string posflix = path.substr(path.size()-3,path.size()-1);
		  if(posflix == "map")
		  return false;
		  float dist =0;
          ifstream file;
          file.open(path);
          string line;
          string mapScen;
          int num,sizeX,sizeY,xs,ys,xg,yg;
          while(getline(file,line)){
		     stringstream strr(line);//num>> ste>>
		     if(line[0] == 'v') {
			     continue;
			 }
		     strr >> num >> mapScen >> sizeX >> sizeY >> xs >> ys >> xg >> yg  >> dist;
		     TestP.emplace_back(xs,sizeY-ys,xg,sizeY-yg);//
		     //s.push_back(make_pair(xs,sizeY-ys));
		     //t.push_back(make_pair(xg,sizeY-yg));
          }
          int pos = path.find("map");
		  map = path.substr(0,pos+3);
 return true;
}


int main(int argc, char ** argv){
	 string scenario= argv[1];
	 vector<TestPair > TestP;
	 vector<bool > block;
	 string map = "",mapTxt = "";
	 int map_x = 0, map_y=0;
	 
     if(Scen(scenario,TestP,map)) { 
		 ofstream outdata;
		 mapTxt = map+".txt";
		 outdata.open(mapTxt);
		 outdata <<"mapNUm \t mapName \t width \t height \t StartX \t StartY  \t GoalX \t GoalY \t optimalCost \t ForwardverticesNum \t BackwardverticesNum \t times \t OSME" << endl;
		 //cerr << TestP.size() << endl;
		 loading_map(map,block,map_x,map_y);
		 int pos = map.find("Path/");
		 string mapC = map.substr(pos+5,map.size()-1);
		     pos = mapC.find("/");
		 string mapE = mapC.substr(pos+1,mapC.size()-1);
		 
                
                for(int i= 0; i< TestP.size(); i++) 
                {    
                          //if(i!=350)
                          //continue;
                          double runtime = 0;int t1_=0,t2_=0,t3_=0,t4_=0,t5_=0,t6_=0;
			   int forwardExpanded_= 0, backwardExpanded_ =0, OSME_ = 0, chi_ = 0;
		           meet MEET(block,map_x,map_y,TestP[i].sx,TestP[i].sy,TestP[i].gx,TestP[i].gy);
		           double gStar = MEET.meetSEARCH(runtime,forwardExpanded_,backwardExpanded_,OSME_, chi_, t1_,t2_,t3_,t4_,t5_,t6_);
		       outdata << i+1 <<"\t"<< mapE <<"\t" << map_x << "\t" << map_y << "\t" << TestP[i].sx << "\t" << TestP[i].sy << "\t"  << TestP[i].gx << "\t" << TestP[i].gy << "\t" <<setprecision(15) << gStar << "\t " << forwardExpanded_ << "\t " << backwardExpanded_ << " " << runtime << "  " << OSME_ << "  " << chi_<< "  " << t1_ << " " << t2_ << " " << t3_ << " " << t4_ << " " << t5_ << " " << t6_<< endl;  
		       //if(i==204)
                      //break;     
                }
                //cerr<< t1_ << " " << t2_ << " " << t3_ << " " << t4_ << " " << (t1_+t2_+t3_+t4_) << " " << TestP.size() << endl;
	   	outdata.close();  
	 } else {
	    cerr <<  "this file is not what we want. " <<endl;
	 }
    return 0;
}
