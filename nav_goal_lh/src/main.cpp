//
//  main.cpp
//  ExtendedKalmanFilter
//
//  Created by Karan on 4/6/18.
//  Copyright © 2018 Karan. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "fusion.hpp"
using namespace std;
int main(int argc, const char * argv[])
{
    std::ifstream ip("../src/data.csv");
    if(!ip.is_open())
    {
        std::cerr << "Failed to open the data file";
        std::exit(EXIT_FAILURE);
    }
    
    std::string timestamp;
    std::string ax;
    std::string yaw_rate;
    std::string yaw;
    std::string course;
    std::string lat;
    std::string lon;
    
    std::string str;
    std::getline(ip, str); // Skip the first line
    
    DataPoint data_point;
    Fusion fusion(0.5,0.5,0.1,0.05,0.001,0.05,0.1,0.0,0.0,false);
    while(std::getline(ip, str))
    {
        std::istringstream iss(str);
        std::string token;
        vector<double> data_vector;
        while (std::getline(iss, token, ','))
        {
	  double get_data=atof(token.c_str());
            // process each token
	  data_vector.push_back(get_data);
//            std::cout << token.size() << " ";
        }
        VectorXd raw_data(6);
        long long timestamp_get=data_vector[1];
	raw_data<<data_vector[14],data_vector[15],data_vector[12],
	sqrt(data_vector[6]*data_vector[6]+data_vector[7]*data_vector[7]+data_vector[8]*data_vector[8]),
	sqrt(data_vector[3]*data_vector[3]+data_vector[4]*data_vector[4]+data_vector[5]*data_vector[5]),
	data_vector[16];
	data_point.set(timestamp_get,DataPointType::GPS,raw_data);
	fusion.process(data_point);
	

 //       std::cout << std::endl;
    }
    
    ip.close();
    
    return 0;
}
