#include "lane_detect/lane_width_calculator.h"

LaneWidthCalculator::LaneWidthCalculator(int maxHistory)
    : SlidingWindow<int>{maxHistory}
{
}

int LaneWidthCalculator::getValue() const
{
    float mean = 0;
    for (auto& val : history)
    {
        mean += val;
    }
    mean /= history.size();
}