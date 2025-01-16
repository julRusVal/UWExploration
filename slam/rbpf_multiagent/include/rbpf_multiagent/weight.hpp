#ifndef WEIGHT_H
#define WEIGHT_H

#include <string>

class Weight {
private:
    

public:
    double value; //weight value
    int self_index; //particle index of self particle cloud
    int neighbour_index; // particle index of neighbour particle cloud
    std::string neighbour_location; //left or right
};

#endif // WEIGHT_H