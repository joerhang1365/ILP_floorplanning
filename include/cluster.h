#ifndef _CLUSTER_H_
#define _CLUSTER_H_

#include "module.h"

class Cluster : public Module
{
public:
    Cluster():Module() {}
    ~Cluster() {}
    Cluster(std::vector<Module *> M);
    Cluster(std::vector<std::unique_ptr<Module>> &M);

    std::vector<Module *>& getSubModules() { return leaf; }

    void setPosition(const Point &pos);     
    void rotate();
    void setRotate(bool);

private:
    void collectAllLeaves(std::vector<Module *> &out) const;
    void collectAllClusters(std::vector<Cluster *> &out, bool include_self=true) const;
    void collectAllLeaves(std::vector<Module *> &out, std::unordered_set<const void *> &visited) const;
    void collectAllClusters(std::vector<Cluster *> &out, bool include_self, std::unordered_set<const void *> &visited) const;
    void collectAll(std::vector<Module *> &leaves, std::vector<Cluster *> &clusters, bool include_self=true) const;

    std::vector<Module *> leaf;
    Cluster * parent;
    bool solved = false;
};

bool isCluster(Module * m);

#endif