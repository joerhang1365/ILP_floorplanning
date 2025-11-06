#include "cluster.h"
#include <limits>

Cluster::Cluster(std::vector<Module *> M) : Module() 
{
    leaf = M;
    setPosition(Point(0, 0));
    setRotate(0);
}

Cluster::Cluster(std::vector<std::unique_ptr<Module>> &M) : Module() 
{
    leaf.reserve(M.size());
    for (auto &m : M) 
    {
        leaf.push_back(m.get());
    }
    setPosition(Point(0, 0));
    setRotate(0);
}

void Cluster::setRotate(bool val) 
{
    if (val != this->getRotate())
    {
        rotate();
    }
}

void Cluster::setPosition(const Point &pos) 
{
    Point delta = pos - getPosition();
    for (auto m : leaf) 
    {
        m->setPosition(m->getPosition() + delta);
    }
    Module::setPosition(pos);
}

void Cluster::collectAllLeaves(std::vector<Module *> &out) const 
{
    std::unordered_set<const void *> visited;
    collectAllLeaves(out, visited);
}

void Cluster::collectAllClusters(std::vector<Cluster *> &out, bool include_self) const 
{
    std::unordered_set<const void *> visited;
    collectAllClusters(out, include_self, visited);
}

void Cluster::collectAll(std::vector<Module *> &leaves, std::vector<Cluster *> &clusters, bool include_self) const 
{
    std::unordered_set<const void *> visitedL, visitedC;
    collectAllClusters(clusters, include_self, visitedC);
    collectAllLeaves(leaves, visitedL);
}


void Cluster::collectAllLeaves(std::vector<Module *> &out, std::unordered_set<const void *> &visited) const 
{
    if (!visited.insert(this).second)
    {
        return;
    }

    for (Module * m : leaf) 
    {
        if (!m) 
        {
            continue;
        }
        if (auto * c = dynamic_cast<Cluster *>(m)) 
        {
            c->collectAllLeaves(out, visited);
        } 
        else 
        {
            if (visited.insert(m).second)
            {
                out.push_back(m);
            }
        }
    }
}

void Cluster::collectAllClusters(std::vector<Cluster *> &out, bool include_self, std::unordered_set<const void *> &visited) const 
{
    if (include_self) 
    {
        if (visited.insert(this).second)
        {
            out.push_back(const_cast<Cluster *>(this));
        }
    } 
    else 
    {
        visited.insert(this);
    }

    for (Module * m : leaf) 
    {
        if (!m) 
        {
            continue;
        }
        if (auto * c = dynamic_cast<Cluster *>(m)) 
        {
            if (visited.insert(c).second)
            {
                out.push_back(c);
            }
            c->collectAllClusters(out, false, visited);
        }
    }
}

bool isCluster(Module * m) 
{
    return dynamic_cast<Cluster *>(m) != nullptr;
}

void Cluster::rotate()
{
    std::vector<Module *> modules;
    std::vector<Cluster *> clusters;

    collectAll(modules, clusters, true);

    if (modules.empty())
    {
        return;
    }

    Point clusterCenter = this->getCenter();

    // rotate each module individually relative to the cluster center
    for (auto m : modules)
    {
        int moduleWidth = m->getRotatedWidth();
        int moduleHeight = m->getRotatedHeight();
        Point moduleCenter = m->getCenter();

        Point relativePos = moduleCenter - clusterCenter;
        Point rotatedPos = Point(-relativePos.y(), relativePos.x());
        Point newModuleCenter = clusterCenter + rotatedPos;

        Point newModulePos = Point(newModuleCenter.x() - round(moduleHeight / 2.0), newModuleCenter.y() - round(moduleWidth / 2.0));
        
        // update the rotation flag and set new position
        m->rotate();
        m->setPosition(newModulePos);
    }

    // update clusters rotation flag
    // it is derived from the Module class
    Module::rotate();
}

double Cluster::getRotatedWidth()
{
    std::vector<Module *> modules;

    collectAllLeaves(modules);

    if (modules.empty())
    {
        return 0.0;
    }

    double minX = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();

    // find largest and smallest X position in cluster
    for (auto m : modules)
    {
        double moduleX = m->getPosition().x();
        double moduleWidth = m->getRotatedWidth();
        
        if (moduleX + moduleWidth > maxX)
        {
            maxX = moduleX + moduleWidth;
        }
        else if (moduleX < minX)
        {
            minX = moduleX;
        }
    }

    return maxX - minX;
}

double Cluster::getRotatedHeight()
{
    std::vector<Module *> modules;

    collectAllLeaves(modules);

    if (modules.empty())
    {
        return 0.0;
    }

    double minY = std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();

    // find largest and smallest Y position in cluster
    for (auto m : modules)
    {
        double moduleY = m->getPosition().y();
        double moduleHeight = m->getRotatedHeight();

        if (moduleY < minY) 
        {
            minY = moduleY;
        }
        else if (moduleY + moduleHeight > maxY) 
        {
            maxY = moduleY + moduleHeight;
        }
    }

    return maxY - minY;
}

Point Cluster::getCenter()
{
    std::vector<Module *> modules;

    collectAllLeaves(modules);

    if (modules.empty())
    {
        return getPosition();
    }

    double minX = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();

    for (auto m : modules)
    {
        double moduleX = m->getPosition().x();
        double moduleWidth = m->getRotatedWidth();
        double moduleY = m->getPosition().y();
        double moduleHeight = m->getRotatedHeight();

        if (moduleX + moduleWidth > maxX)
        {
            maxX = moduleX + moduleWidth;
        }
        else if (moduleX < minX)
        {
            minX = moduleX;
        }

        if (moduleY < minY) 
        {
            minY = moduleY;
        }
        else if (moduleY + moduleHeight > maxY) 
        {
            maxY = moduleY + moduleHeight;
        }
    }

    // average pos
    double centerX = (minX + maxX) / 2.0;
    double centerY = (minY + maxY) / 2.0;

    return Point(centerX, centerY);
}