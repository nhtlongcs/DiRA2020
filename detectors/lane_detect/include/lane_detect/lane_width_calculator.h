#pragma once

#include "common/slidingwindow.h"

class LaneWidthCalculator : SlidingWindow<int>
{
public:
    LaneWidthCalculator(int maxHistory);
    int getValue() const;
};