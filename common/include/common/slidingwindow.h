#pragma once
#include <list>

template <typename T>
class SlidingWindow
{
public:
    SlidingWindow(int maxHistory) : maxHistory{maxHistory}
    {
    }

    void update(const T& value)
    {
        if (history.size() > maxHistory)
        {
            history.pop_front();
        }
        history.push_back(value);
    }

    int getMaxHistory() const
    {
        return maxHistory;
    }

    void reset()
    {
        history.clear();
    }

protected:
    std::list<int> history;
    int maxHistory;
};