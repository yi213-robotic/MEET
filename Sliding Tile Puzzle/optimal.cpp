//#include "mm.h"
//#include "astar.h"
#include "bae.h"
//#include "meet.h"
#include <iostream>
#include <fstream>
#include <string>
#include <regex>
#include <dirent.h>
#include <vector>
#include <utility> // For std::pair


using namespace std;

using Instance = std::array<uint8_t, 16>;

vector<Instance> sgpair;

int instances = 0;

void STPPuzzle(std::istream& input)
{
        std::string line;
        size_t lineno = 0;
        // Skip the label for starting positions
        while(getline(input,line))
        {
             lineno++;  
             if (line.empty() || line[0] == '#') 
               continue;
             std::stringstream ss(line);
             Instance inst{};
             for (int i = 0; i < 16; ++i) 
             {
                    int v;
                    if (!(ss >> v)) 
                    {
                      throw std::runtime_error("Line " + std::to_string(lineno) + ": expected 16 integers");
                    }
                    if (v < 0 || v > 15) 
                    {
                        throw std::runtime_error("Line " + std::to_string(lineno) + ": value out of range 0..15");
                    }
                    inst[i] = static_cast<uint8_t>(v);
                    //cerr << static_cast<unsigned>(inst[i]) << " "; 
             }
             //cerr << endl;
             int extra;
             if (ss >> extra) {
                 throw std::runtime_error("Line " + std::to_string(lineno) +
                                     ": too many numbers");
             }
             sgpair.push_back(inst);
        }
        
}


void processSTPFile(const std::string& filePath) {
    std::ifstream file(filePath);
    if (file.is_open()) {
        STPPuzzle(file);
        //std::cout << "Processed file: " << filePath << std::endl;
    } else {
        std::cerr << "Unable to open file: " << filePath << std::endl;
    }
}


int main(int argc, char *argv[])
{
    //srand (0);// Random seed
    //cerr << "going here " << endl;
    //astar A_star;
    //mm mm_;
    
    const std::string path = "/home/yiwang/Downloads/15STP"; // Replace with your folder path
    DIR* dir;
    struct dirent* ent;
    
    if ((dir = opendir(path.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            std::string fileName = ent->d_name;
            if (fileName.size() > 4 && fileName.substr(fileName.size() - 4) == ".pan") {
                processSTPFile(path + "/" + fileName);
                //cerr << fileName << " " << instances << endl;
                instances++;
                /*if(instances == 60)// instances for experiments
                {
                      break;
                }*/    
            }
        }
        closedir(dir);
    } else {
        std::cerr << "Could not open directory" << std::endl;
        return EXIT_FAILURE;
    }
    //cerr << sgpair.size() <<endl;
    // goal tables
    static constexpr std::array<uint8_t,16> goalR_ = { 3,0,0,0, 0,1,1,1, 1,2,2,2, 2,3,3,3 };
    static constexpr std::array<uint8_t,16> goalC_ = { 3,0,1,2, 3,0,1,2, 3,0,1,2, 3,0,1,2 };
    static constexpr std::array<uint8_t,16> goal_ = {1,2,3,4, 5,6,7,8, 9,10,11,12, 13,14,15,0};
    
    for(size_t i = 0; i<sgpair.size(); i++)
    {
        /*if(i != 44)
           continue;*/
        //array<uint8_t,16> start = {1,6,0,5,13,9,3,2,10,11,4,8,7,14,15,12};   
        //2,3,5,4, 1,9,11,6, 13,14,0,8, 15,10,7,12 //-24
        //2,7,3,4, 1,0,5,12, 9,14,6,11, 13,15,8,10 //-27
        //0,5,2,3, 10,6,1,4, 13,11,15,7, 9,14,12,8 //-34   
        //2,6,7,3, 1,0,9,4, 5,8,11,14, 13,10,15,12 //-35
        //9,6,5,1, 13,7,2,3, 0,10,4,8, 14,12,11,15 //-37
        //2,3,13,4, 1,6,15,8, 5,10,0,12, 7,9,11,14 //-40
        //1,2,3,4, 13,5,9,0, 10,14,6,7, 15,11,12,8 //-52
        //5,1,0,4, 9,2,3,8, 11,10,6,14, 13,7,12,15 //-54
        //5,3,8,4, 2,7,15,11, 1,9,10,12, 13,6,14,0//-57
        //5,3,0,4, 9,2,1,7, 10,14,6,8, 13,11,15,12 //-63
        //2,3,8,6, 1,5,7,4, 9,12,0,11, 10,13,14,15//-68
        //1,13,2,7, 10,0,6,3, 5,9,4,8, 14,11,15,12 //-70
        //2,5,8,3, 1,6,14,4, 10,7,0,11, 9,13,15,12 //-3 
        //5,1,4,8, 9,2,3,7, 11,14,6,12, 10,0,13,15 //-4 
        array< uint8_t, 16> start = sgpair[i];
        std::array<uint8_t,16> startR_{}, startC_{};
        for (int idx = 0; idx < 16; ++idx) 
        {
              uint8_t v = start[idx];
              startR_[v] = static_cast<uint8_t>(idx/4);
              startC_[v] = static_cast<uint8_t>(idx%4);
              //cerr << static_cast<unsigned>(v) << " ";
        }
        //cerr << endl;
        //astar A_star;
        //mm mm_;
        //meet MEET;
        double time =0;
        int expanded_ = 0, backwardExpanded_ =0, pathLength =0, OSME_ = 0, OPM_ = 0;
        int t1_ = 0, t2_ = 0, t3_ = 0, t4_ = 0, t5_ = 0, t6_ = 0;
        //MEET.meetSEARCH(time,expanded_, backwardExpanded_, start ,startR_, startC_, goalR_, goalC_, goal_,pathLength,OSME_,OPM_,t1_,t2_,t3_,t4_,t5_,t6_);
        //A_star.astar_(time,expanded_, backwardExpanded_, start ,startR_, startC_, goalR_, goalC_, goal_,pathLength);
        //mm_.MMSEARCH(time,expanded_, backwardExpanded_, start ,startR_, startC_, goalR_, goalC_, goal_,pathLength,OSME_,OPM_);// finding path 
        
        bae bae_;
        bae_.baeSEARCH(time,expanded_, backwardExpanded_, start ,startR_, startC_, goalR_, goalC_, goal_,pathLength,OSME_,OPM_); 
        cout << time << " " << expanded_ << " " << backwardExpanded_ << " " << OSME_ << " " << OPM_ << " " << pathLength << " " << t1_  << "  " << t2_ << " " << t3_ << " " << t4_ << " " << t5_ << " " << t6_  <<endl;
        //exit(-1);
    }
    
    return 0;
}
