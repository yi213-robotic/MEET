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
#include <unordered_set>
#include <iostream>
#include <sstream>
#include <cmath>
#include <limits>
using namespace std;
vector<pair<string,vector<double> > > ASTAR;
unordered_map<string,vector<double> > ALGO;
unordered_set<int > differentE;
vector<int >T1;
vector<int >T2;
vector<int >T3;
vector<int >T4;
vector<int >T5;
vector<int >T6;
vector<double >runtime;
vector<double >Fruntime;
vector<int >forwardE;
vector<int > backwardE;  
vector<int > backwardBE; 
vector<int > OME;
vector<int > OPE;  
vector<double >optimal_Astar; 
 vector<double >optimal_ALGO;
double epsilon = numeric_limits<double>::epsilon();
void ExtractAstar(string astar) {//
   double dist =0;
   ifstream file;
   file.open(astar);
   string line,mn;
   int num = 0;
  
   while(getline(file,line)){
	      if(isdigit(line[0])) {
		      int count = 0, forward_, backward_, path , OSME_, OPM_;
		      string mapNum, mapNa, width, height, startX,startY,goalX,goalY;
		      double pathCost,times;	  
		      stringstream strr(line);//>> num >> mn
		      //strr >> times >> forward_ >> backward_ >> count;
              strr >> times >> forward_ >> backward_ >>OSME_>>OPM_>> path; 
              //optimal_ALGO.push_back(pathCost); 
              
              backwardE.push_back(forward_ + backward_);
              Fruntime.push_back(times);
              //OME.push_back(OSME_);
              OPE.push_back(OPM_);
              //cerr << pathCost << endl;
              //exit(-1);
		  }
   }

  file.close();
  //cerr << "TestingANYA -> " << mapName << " " << optimal.size() << endl;
  //ASTAR.push_back(make_pair(mapName,optimal)) ;	
}


void ExtractRAstar(string astar) {//
   int xs, ys, xg, yg; 
   double dist =0;
   ifstream file;
   file.open(astar);
   string line,mn;
   int num = 0;
     
      while(getline(file,line)){
	      if(isdigit(line[0])) {
		      int count = 0, forward_, backward_;
		      string mapNum, mapNa, width, height, startX,startY,goalX,goalY;
		      double pathCost,times;	  
		      stringstream strr(line);//>> num >> mn
              strr >> mapNum >> mapNa>> width>> height>>startX >> startY >> goalX >> goalY >> pathCost>> forward_ >> backward_>> times; 
              backwardBE.push_back(forward_); 
              
              //optimal_Astar.push_back(pathCost); 
              //cerr << pathCost << endl;
              //exit(-1);
		  }
   }

  file.close();
  //cerr << "TestingANYA -> " << mapName << " " << optimal.size() << endl;
  //ASTAR.push_back(make_pair(mapName,optimal)) ;	
}

void averageCost()
{
   int sum = 0;
   double totalNum = 0, toalME = 0, toalPM = 0;
   vector<int > BE;
   for(int i =0; i < forwardE.size(); i++)
   {
       BE.push_back(forwardE[i]);
       totalNum += forwardE[i];
       toalME += OME[i];
       //cerr << OME[i] << endl;
       toalPM += OPE[i];
   }
   double averForward = (double) totalNum/forwardE.size();
   double averME = (double) toalME/forwardE.size();
   double averOP = (double) toalPM/forwardE.size();
   cerr << averForward << " " << forwardE.size() << " " << averME << " " << averOP<< endl;
   totalNum = 0;
   /*int countttt = 0;
   double mean1 = 0, mean2 =0;
   for(int i =0; i < backwardE.size(); i++)
   {
       //cerr << backwardE[i] << "  " << BE[i] << endl;
       if(backwardE[i] >= BE[i])
       {
        // cerr << "func " << endl;
          countttt++;
               mean1 += backwardE[i];
               mean2 += BE[i];
       }
       else
       {
            differentE.insert(i);
       }   
       totalNum += (backwardE[i] + backwardBE[i])/2;
   }
   averForward = (double) totalNum/backwardE.size();
   cout << averForward << " "<< countttt << " " << (mean1/(countttt)) << " " << (mean2/(countttt))  << endl;
   vector<pair<int,int > > RE;
   int cR = 0;
    mean1 = 0; mean2 =0;
   for(int i =0; i < backwardBE.size(); i++)
   {
       if(differentE.find(i) != differentE.end())
       {
           if(backwardBE[i] > BE[i])
           {
               cR++;
               RE.push_back(make_pair(backwardBE[i],BE[i]));
           }
           else
           {
               mean1 += backwardE[i];
               mean2 += BE[i];
           }
       }
       
   }*/
   //cerr << differentE.size()-cR << " " << (mean1/(differentE.size() - cR)) << " " << (mean2/(differentE.size() - cR)) << endl;
   //mean1 = 0; mean2 =0;
   /*for(int i =0; i < RE.size(); i++)
   {
            mean1 += RE[i].first;
            mean2 += RE[i].second;
   }
   cerr << RE.size() << " " << (mean1/(RE.size())) << " " << (mean2/(RE.size())) << endl;*/
   /*totalNum = 0;
   int expdNum_= 0;
   toalPM = 0;
   for(int i=0; i < forwardE.size(); i++)
   {
      totalNum += Fruntime[i];
      //cout << Fruntime[i] << endl;
      expdNum_ += forwardE[i];
      toalPM += OPE[i];
   } 
   averForward = (double) totalNum/Fruntime.size();
   
   cerr <<" aver time " << averForward << " mean expanded number " << (double) expdNum_/Fruntime.size() << " minimi generation: " << (double) toalPM/Fruntime.size() <<endl;*/
}


void ExtractAlgo(string algo) {//
   double dist =0;
   ifstream file;
   file.open(algo);
   string line,mn;
   int num = 0;
  
   while(getline(file,line)){
	      if(isdigit(line[0])) {
		      int count = 0, forward_, backward_, path , OSME_, OPM_, t1,t2,t3,t4,t5,t6;
		      string mapNum, mapNa, width, height, startX,startY,goalX,goalY;
		      double pathCost,times;	  
		      stringstream strr(line);//>> num >> mn
		      //strr >> times >> forward_ >> backward_ >> count;
              strr >> times >> forward_ >> backward_>> path >>OSME_>>OPM_>>t1>>t2>>t3>>t4>>t5>>t6; 
              //optimal_ALGO.push_back(pathCost); 
              
              forwardE.push_back(forward_ + backward_);
              runtime.push_back(times);
              OME.push_back(OSME_);
              OPE.push_back(OPM_);
              
              T1.push_back(t1);
              T2.push_back(t2);
              T3.push_back(t3);
              T4.push_back(t4);
              T5.push_back(t5);
              T6.push_back(t6);
              
              //cerr << pathCost << endl;
              //exit(-1);
		  }
   }
  double avg1 = 0, avg2 = 0 , avg3 = 0, avrtime_ =0, avg4 =0, avg5 = 0, avg6 = 0;
  double avgt1 = 0, avgt2 = 0 , avgt3 = 0, avgt4 =0, avgt5 = 0, avgt6 = 0, averExp_ = 0;
  int ct1_ = 0, ct2_ = 0, ct3_ = 0, ct4_ = 0, ct5_ = 0, ct6_ = 0;
  cerr << T1.size() << " " << T2.size() << " " << T3.size() << " " << T4.size() << " " << T5.size() << " " << T6.size() <<endl;
  for(int i=0; i < forwardE.size();i++)
  {      
       if(runtime[i]<= 0)
         continue;  
       avrtime_ += runtime[i];
       averExp_ += forwardE[i]; 
       //cout << forwardE[i] <<"  " <<runtime[i] <<endl;
       cout <<runtime[i] <<endl;   
       /*if(T1[i] == 1)
       {
           ct1_++;
           avg1 += (double) (backwardE[i] - forwardE[i])/backwardE[i];
           avgt1 += (double) (Fruntime[i] - runtime[i])/Fruntime[i];
       }  
       else if(T2[i] == 1)
       {
           ct2_++;
           avg2 += (double) (backwardE[i] - forwardE[i])/backwardE[i];
           avgt2 += (double) (Fruntime[i] - runtime[i])/Fruntime[i];
       }
       else if (T3[i] == 1)
       { 
           ct3_++;
           avg3 += (double) (backwardE[i] - forwardE[i])/backwardE[i];
           avgt3 += (double) (Fruntime[i] - runtime[i])/Fruntime[i];
       }
       else if (T4[i] == 1)
       {
           ct4_++;
           avg4 += (double) (backwardE[i] - forwardE[i])/backwardE[i];
           avgt4 += (double) (Fruntime[i] - runtime[i])/Fruntime[i];
       }
       else if (T5[i] == 1)
       {
           avg5 += (double) (backwardE[i] - forwardE[i])/backwardE[i];
           avgt5 += (double) (Fruntime[i] - runtime[i])/Fruntime[i];
           ct5_++;
       }
       else if (T6[i] == 1)
       {
           avg6 += (double) (backwardE[i] - forwardE[i])/backwardE[i];
           avgt6 += (double) (Fruntime[i] - runtime[i])/Fruntime[i];
           ct6_++;
       }*/
       
       //cerr << "?? " << endl;
       
  }
  double avruntime_ = static_cast<double>(avrtime_) / forwardE.size();
  double avExp_ = static_cast<double>(averExp_) / forwardE.size();
  cerr << avExp_ <<" rumtime-> " << avruntime_  << "  " << ct1_ << " " << ct2_ << " " << ct3_ << " " << ct4_ << " " << ct5_ << " " << ct6_ << endl;
  avg1 = static_cast<double> (avg1)/ct1_;
  avg2 = static_cast<double> (avg2)/ct2_;
  avg3 = static_cast<double> (avg3)/ct3_;
  avg4 = static_cast<double> (avg4)/ct4_;
  avg5 = static_cast<double> (avg5)/ct5_;
  avg6 = static_cast<double> (avg6)/ct6_;
  
  avgt1 = static_cast<double> (avgt1)/ct1_;
  avgt2 = static_cast<double> (avgt2)/ct2_;
  avgt3 = static_cast<double> (avgt3)/ct3_;
  avgt4 = static_cast<double> (avgt4)/ct4_;
  avgt5 = static_cast<double> (avgt5)/ct5_;
  avgt6 = static_cast<double> (avgt6)/ct6_;
  
  cerr << avg1 << " " << avg2 << " " << avg3 << " " << avg4 << " "<< avg5 << " " << avg6  << endl;
  cerr << avgt1 << " " << avgt2 << " " << avgt3 << " " << avgt4 << " "<< avgt5 << " " << avgt6  << endl;
  file.close();
  //exit(-1);
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
    string  algo = argv[1];//, ral = argv[3], astar = argv[1],
    //ExtractAstar(astar);
    ExtractAlgo(algo);
    //ExtractRAstar(ral);
    averageCost();	 
    //Compare();
    return 0;
}


