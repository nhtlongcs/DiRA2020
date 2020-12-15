#pragma once
#include <list>
#include <map>

template <typename T>
class SlidingCounter
{
public:
    SlidingCounter(int maxHistory) : maxHistory{maxHistory}
    {
    }

    void update(const T& value)
    {
        if (history.size() > maxHistory)
        {
            T val = history.front();
            history.pop_front();
            counter[val]--;
        }
        history.push_back(value);
        if (counter.find(value) != counter.end())
        {
            counter[value]++;
        }
        else
        {
            counter[value] = 1;
        }
    }

    bool getMaxCount(T& obj, int& count) const
    {
        int maxCount = 0;
        const T* objPtr = nullptr;
        for (const auto& pair : counter)
        {
            if (pair.second > maxCount)
            {
                maxCount = pair.second;
                objPtr = &(pair.first);
            }
        }
        if (maxCount > 0)
        {
            obj = *objPtr;
            count = maxCount;
            return true;
        }
        return false;
    }

    void reset()
    {
        history.clear();
        counter.clear();
    }

    std::list<T> getHistory() const
    {
        return history;
    }

    std::map<T, int> getCounter() const
    {
        return counter;
    }

protected:
    std::map<T, int> counter;
    std::list<T> history;
    int maxHistory;
};