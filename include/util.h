#ifndef _UTIL_H_
#define _UTIL_H_

#include <vector>
#include <string>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <functional>
#include <utility>
#include <cmath>
#include <limits> 
#include <memory>
#include <fstream>
#include <list>   
#include <iostream>
#include <filesystem>
#include <type_traits>

class Point 
{
    float x_, y_;
public:
    Point() : x_(0), y_(0) {}
    Point(float x, float y) : x_(x), y_(y) {}
    Point operator+(const Point &p) const { return Point(x_ + p.x_, y_ + p.y_); }
    Point operator-(const Point &p) const { return Point(x_ - p.x_, y_ - p.y_); }
    Point operator*(float k) const { return Point(x_ * k, y_ * k); }
    Point operator/(float k) const { return Point(x_ / k, y_ / k); }
    float dot(const Point &p) const { return x_ * p.x_ + y_ * p.y_; }
    float cross(const Point &p) const { return x_ * p.y_ - y_ * p.x_; }
    float norm2() const { return x_ * x_ + y_ * y_; }
    double norm() const { return std::sqrt((double)norm2()); }
    Point unit() const { double n = norm(); return Point(x_ / n, y_ / n); } 
    bool operator<(const Point &p) const { return x_ != p.x_ ? x_ < p.x_ : y_ < p.y_; } 
    bool operator==(const Point &p) const { return x_ == p.x_ && y_ == p.y_; }
    float x() const { return x_; }
    float y() const { return y_; }
};

#endif