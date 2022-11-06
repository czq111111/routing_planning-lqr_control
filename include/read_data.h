#pragma once

#include<stdio.h>	
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include <vector>

using namespace std;

class readDate{
public:
    readDate() = default;
    ~readDate() = default;


    bool readTxt(std::vector<std::pair<double, double>>& xy_points);
    
};