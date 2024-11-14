#ifndef __MAXI_TRIGGER_HPP__
#define __MAXI_TRIGGER_HPP__

#include <cmath>

template<typename T>
class maxiTrigger
{
public:
    maxiTrigger() {};
    /*! Generate a trigger when a signal changes beyond a certain amount
    \param input A signal
    \param tolerance The amount of chance allowed before a trigger is generated*/
    double onChanged(T input, T tolerance)
    {
        T changed = 0;
        if (std::abs(input - previousValue) > tolerance)
        {
            changed = 1;
            previousValue = input;
        }
        return changed;
    }

private:
    T previousValue = 1;
    bool firstTrigger = 1;
};

#endif  // __MAXI_TRIGGER_HPP__
