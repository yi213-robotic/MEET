#include "meet.h"
#include <iostream>
#include <fstream>
#include <string>
#include <regex>
#include <dirent.h>
#include <vector>
#include <utility> // For std::pair


using namespace std;

vector<pair<vector<int>, vector<int >>> sgpair;

int instances = 0;

void PancakePuzzle(std::istream& input)
{
        std::string line;
        getline(input, line);  // Get the size of pancake stack
        std::stringstream ss(line);
        int size = 0;
        ss >> size;

        // Skip the label for starting positions
        getline(input, line);

        std::vector<int> startOrdering(size);
        for (int i = 0; i < size; ++i) {
            getline(input, line);
            startOrdering[i] = std::stoi(line);
        }

        // Skip the label for ending positions
        getline(input, line);

        std::vector<int> endOrdering(size);
        for (int i = 0; i < size; ++i) {
            getline(input, line);
            endOrdering[i] = std::stoi(line);
        }
        
        sgpair.push_back(make_pair(startOrdering,endOrdering));
}


void processPancakeFile(const std::string& filePath) {
    std::ifstream file(filePath);
    if (file.is_open()) {
        PancakePuzzle(file);
        //std::cout << "Processed file: " << filePath << std::endl;
    } else {
        std::cerr << "Unable to open file: " << filePath << std::endl;
    }
}



int main(int argc, char *argv[])
{
    //srand (0);// Random seed
    //cerr << "going here " << endl;
    const std::string path = "/home/yiwang/Downloads/PancakePuzzleResuts/10PanTestDomain"; // 
    DIR* dir;
    struct dirent* ent;
    
    if ((dir = opendir(path.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            std::string fileName = ent->d_name;
            if (fileName.size() > 4 && fileName.substr(fileName.size() - 4) == ".pan") {
                processPancakeFile(path + "/" + fileName);
                cerr << fileName << " " << instances << endl;
                instances++;
                if(instances == 30)// instances for experiments
                {
                      break;
                }    
            }
        }
        closedir(dir);
    } else {
        std::cerr << "Could not open directory" << std::endl;
        return EXIT_FAILURE;
    }
    
    for(int i = 0; i<sgpair.size(); i++)
    {int t1 =0, t2=0, t3=0, t4 =0, t5 = 0, t6 =0;
        vector< int > start = sgpair[i].first;
        vector< int > goal  = sgpair[i].second;
        // 3,9,10,6,4,8,2,1,7,5//94-10
        // 10,5,3,1,4,6,8,2,7,9 // 97-10
        //2,10,9,4,3,5,8,1,7,6 // 98-10
        //10,9,2,1,3,4,6,8,5,7  //69-10
        //10,6,3,4,7,5,9,1,8,2  //90-10
        //3,7,1,8,5,10,2,9,4,6 //21-10
        // 9,6,2,10,3,1,7,5,8,4// 18-10
        //9,8,1,6,2,3,5,10,4,7 // 88-10
        //10,6,8,4,9,5,7,1,3,2 //35-10
        //2,10,9,6,7,8,3,4,5,1 // 8-10
        //10,3,6,5,1,2,8,7,9,4 //83-10
        //6,3,7,1,5,8,10,9,2,4// 44-10
        //4,9,2,8,10,1,3,6,5,7 // 30-10
        //9,4,2,7,3,1,10,8,5,6 //85-10
        // 1,5,3,10,8,2,7,6,4,9 //93-10
        // 8,9,3,4,1,7,5,6,10,2 // 54-10
        //5,1,6,2,8,7,9,3,10,4 //46-10
        //3,2,5,10,4,8,1,9,6,7 // 17-10
        //5,3,2,7,9,10,1,6,4,8 // 84-10
        //5,10,7,4,3,9,2,8,1,6 // 43-10
        //8,1,3,9,5,4,7,2,6,10 // 56-10
        //7,5,3,8,2,6,1,4,10,9 //57-10
        //3,7,1,8,5,10,2,9,4,6 //21-10
        //4,7,6,9,3,5,1,2,10,8 //39-10
        //8,1,7,3,6,9,10,5,4,2 // 58-10
        // 3,9,1,4,10,2,8,7,6,5 // 47-10
        // 10,1,2,4,7,8,5,9,6,3 // 1-10
    
        //vector< int > start = {10,6,3,4,7,5,9,1,8,2};
        //vector< int > goal =  {1,2,3,4,5,6,7,8,9,10};
        
        meet MEET;
        double time =0;
        int expanded_ = 0;
        int backwardExpanded_ =0;
        int pathLength =0;
        int OSME_ = 0, OPM_ = 0;
        MEET.meetSEARCH(time,expanded_, backwardExpanded_, start,goal,pathLength,OSME_,OPM_,t1, t2, t3, t4, t5, t6);
        cout << time << " " << expanded_ << " " << backwardExpanded_ << " " << OSME_ << " " << OPM_ << " " << pathLength << " " << t1 << " " << t2 << " " << t3 << " " << t4 << " "<< t5 << " " << t6 << endl;
        //exit(-1); 
    }
    //cerr << "[" << t1 << "," << t2 << "," << t3 << "," << t4 << ","<< t5 << "," << t6 << "]" << endl;
    return 0;
}
