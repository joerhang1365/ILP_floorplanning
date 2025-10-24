#ifndef _MODULE_H_
#define _MODULE_H_

#include <util.h>

class Module 
{
public:
    Module() {}
    Module(int id, float width, float height) : id_(id), width_(width), height_(height), rotated(false), position(Point(0, 0)) {}
    virtual ~Module() {}
    virtual void setPosition(const Point &pos) { position = pos; }
    virtual void setRotate(bool r) { rotated = r; }
    virtual void setWidth(int w) { width_ = w; }
    virtual void setHeight(int h) { height_ = h; }
    void setID(int id) { id_ = id; }

    int getRotatedWidth() const { return rotated ? height_ : width_; }
    int getRotatedHeight() const { return rotated ? width_ : height_; }
    int getOrgWidth() const { return width_; }
    int getOrgHeight() const { return height_; }

    Point getPosition() const { return position; }
    Point getCenter() const { 
        return Point(position.x() + getRotatedWidth() / 2.0, position.y() + getRotatedHeight() / 2.0); 
    }
    
    bool getRotate() const { return rotated; }
    bool isRotated() const { return rotated; }
    virtual void rotate() { rotated = !rotated; }
    int getId() const { return id_; }

private:
    Point position; // left-down position
    int width_, height_;
    int id_;
    bool rotated;   // the module is rotated if this->rotated == 1
};

#endif