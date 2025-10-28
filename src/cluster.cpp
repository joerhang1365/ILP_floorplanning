#include "cluster.h"

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

    Point center = this->getCenter();

    for (auto m : modules)
    {
        Point module_center = m->getCenter();
        Point relative_pos = module_center - center;
        Point rotated_pos = Point(-relative_pos.y(), relative_pos.x());
        Point new_center = center + rotated_pos;
        Point new_position = Point(new_center.x() - m->getRotatedWidth() / 2.0, new_center.y() - m->getRotatedHeight() / 2.0);
        
        m->rotate();
        m->setPosition(new_position);
    }

    Module::rotate();
}
