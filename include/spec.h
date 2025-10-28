#ifndef _SPEC_H_
#define _SPEC_H_

#include "util.h"

struct Spec 
{
    int problemType;    // 0: required optimal solution, 1: bounding area minimization
    float targetWidth;  // the fixed width of the floorplan
    float targetHeight; // the fixed height of the floorplan

    Spec(std::string specFile) 
    {
        std::ifstream infile(specFile);
        infile >> problemType >> targetWidth >> targetHeight;
    }
    Spec() : problemType(0), targetWidth(0), targetHeight(0) {}
};

#endif