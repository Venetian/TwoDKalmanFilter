//
//  main.cpp
//  TwoDKalmanFilterTest
//
//  Created by Andrew Robertson on 10/03/2015.
//  Copyright (c) 2015 Andrew Robertson. All rights reserved.
//

#include <iostream>
#include "TwoDKalmanTempoFilter.h"

int main(int argc, const char * argv[]) {
    // insert code here...
    std::cout << "SIMPLE EXAMPLE FOR TEMPO AND PHASE PROCESSING WITH KALMAN FILTER!\n";
    
    TwoDKalmanTempoFilter kbf;
    kbf.simpleExample();
    return 0;
}
