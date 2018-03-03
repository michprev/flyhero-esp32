//
// Created by michal on 28.2.18.
//

#pragma once

#include <map>
#include <vector>
#include <algorithm>

template<typename T>
class Counting_Median_Finder
{
private:
    std::map<T, int> map;
    std::vector<T> keys;

public:
    void Push_Value(T value);

    T Get_Median();

};

template<typename T>
void Counting_Median_Finder<T>::Push_Value(T value)
{
    if (this->map.find(value) != this->map.end())
        this->map[value]++;
    else
    {
        this->keys.push_back(value);
        this->map[value] = 1;
    }
}

template<typename T>
T Counting_Median_Finder<T>::Get_Median()
{
    std::sort(this->keys.begin(), this->keys.end());

    auto begin = this->keys.begin();
    auto end = this->keys.end() - 1;

    if (begin == end)
        return *begin;

    int sum = this->map[*begin] - this->map[*end];

    while (begin + 1 != end)
    {
        if (sum <= 0)
        {
            begin++;
            sum += this->map[*begin];
        } else if (sum > 0)
        {
            end--;
            sum -= this->map[*end];
        }
    }

    if (sum >= 0)
        return *begin;
    else
        return *end;
}