#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <dirent.h>
#include <numeric>
#include <ctype.h>
#include <algorithm>
#include <ostream>
#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <sstream>
#include <cmath>
#include <limits>
using namespace std;
vector<pair<string,vector<double> > > ASTAR;
unordered_map<string,vector<double> > ALGO;
vector<double >optimal_Astar; 
 vector<double >optimal_ALGO;
double epsilon = numeric_limits<double>::epsilon();
void ExtractAstar(string astar) {//
   int xs, ys, xg, yg; 
   double dist =0;
   ifstream file;
   file.open(astar);
   string line,mn;
   int num = 0;
     
      while(getline(file,line)){
	      if(isdigit(line[0])) {
		      int count = 0;
		      string mapNum, mapNa, width, height, startX,startY,goalX,goalY;
		      double time;
		      int forward_, backward_, OSME, OPM, pathCost, t1,t2,t3,t4;	  
		      stringstream strr(line);//>> num >> mn
              strr>> time >> forward_ >> backward_ >> OSME >>OPM >> pathCost >> t1 >> t2 >> t3 >> t4; 
              optimal_Astar.push_back(pathCost); 
              //cerr << pathCost << endl;
              //exit(-1);
		  }
   }

  file.close();
  //cerr << "TestingANYA -> " << mapName << " " << optimal.size() << endl;
  //ASTAR.push_back(make_pair(mapName,optimal)) ;	
}
void ExtractAlgo(string algo) {//
   double dist =0;
   ifstream file;
   file.open(algo);
   string line,mn;
   int num = 0;
  
   while(getline(file,line)){
	      if(isdigit(line[0])) {
		      int count = 0;
		      string mapNum, mapNa, width, height, startX,startY,goalX,goalY;
		     
		      double time;
		      int forward_, backward_, OSME, OPM, pathCost, t1,t2,t3,t4;	  
		      stringstream strr(line);//>> num >> mn
              strr >> time >> forward_ >> backward_ >> OSME >>OPM >> pathCost >> t1 >> t2 >> t3 >> t4; 
              optimal_ALGO.push_back(pathCost); 
              //cerr << pathCost << endl;
              //exit(-1);
		  }
   }
  //cerr << "TestingAPPA-> " << mapName << " " << optimal.size() << endl;
  //ALGO[mapName]=optimal;	
  file.close();
}

void Compare(){
	int sameV = 0, diff = 0;
	double meanT = 0; double ndiff = 0;
    for(int i =0; i < optimal_Astar.size();i++) {
	               //cerr << "testing here:  " << it->first << endl;
				  //cerr << "---> " << ANYAD[i].second[k] << " " << APPAD[it->first][k] << " " << (fabs(ANYAD[i].second[k] - APPAD[it->first][k]) < 0.00001) << endl;
			       if(fabs(optimal_Astar[i] - optimal_ALGO[i]) < 0.000001){
				       //cerr << "Testing here for comparison: " << it->first << " instance number: " << k << "The value is: " <<ANYAD[i].second[k] << " " << APPAD[it->first][k] << endl;
				       sameV++; 
				       //exit(-1);
				   } else {
				       diff++;
                       cerr << i << endl;
				   }
	}
	int totalInstance = sameV+diff;
	cerr <<"MAP's num: " << optimal_ALGO.size() <<"Same solutions: " << sameV << " Different soltuions " << diff << " Total Instance is: " << sameV+diff << " sucessful rate is: " << (double) sameV/totalInstance << " total mean: " << (double) meanT/totalInstance << " not find path: " << ndiff << " notfind rate: " << (double) ndiff/diff << "  AUC: "<< (double) (sameV)/(sameV+diff-ndiff) << endl; 
}
int main(int argc, char ** argv) {
    string astar = argv[1], algo = argv[2];//
    ExtractAstar(astar);
	ExtractAlgo(algo);
    Compare();
    return 0;
}


