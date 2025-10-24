#include "floorplanner.h"

void Floorplanner::initialize(std::string inputFile) 
{
    std::cout << "Reading input file: " << inputFile << std::endl;
    std::string word;
    size_t moduleSize;
    std::ifstream infile(inputFile);

    infile >> word >> moduleSize;
    getline(infile, word); // read garbage
    getline(infile, word); // read garbage

    modules.reserve(moduleSize);
    for (size_t i = 0; i < moduleSize; i++) 
    {
        int id;
        int width, height;
        infile >> id>> width >> height;
        modules.push_back(std::unique_ptr<Module>(new Module(id, width, height)));
    }
}

void Floorplanner::writeOutput(std::string outputFile) 
{
    std::cout << "Writing output file: " << outputFile << std::endl;
    std::ofstream outfile(outputFile);
    for (size_t i = 0; i < modules.size(); i++) 
    {
        const Module * m = modules[i].get();
        outfile << m->getId() << "\t" << m->getPosition().x() << "\t" << m->getPosition().y() << "\t" << (m->isRotated() ? 1 : 0) << std::endl;
    }
}

bool Floorplanner::validityCheck()
{
    for (int i = 0 ; i < modules.size(); i++)
    {
        for (int j = i + 1; j < modules.size(); j++)
        {
            Module * m1 = modules[i].get();
            Module * m2 = modules[j].get();
            if( !(  (m1->getPosition().x() + m1->getRotatedWidth() <= m2->getPosition().x()) ||
                    (m2->getPosition().x() + m2->getRotatedWidth() <= m1->getPosition().x()) ||
                    (m1->getPosition().y() + m1->getRotatedHeight() <= m2->getPosition().y()) ||
                    (m2->getPosition().y() + m2->getRotatedHeight() <= m1->getPosition().y()) ) )
            {
                std::cout << "Module " << m1->getId() << " and Module " << m2->getId() << " overlap!" << std::endl;
                return false;
            }
        }
    }

    for (int i = 0; i < modules.size(); i++)
    {
        Module * m = modules[i].get();
        if( (m->getPosition().x() < -0.1) || (m->getPosition().y() < -0.1) ||   // if solver used int as var. it's possible to get negative value
            (m->getPosition().x() + m->getRotatedWidth() > spec.targetWidth+0.1) ||
            (m->getPosition().y() + m->getRotatedHeight() > spec.targetHeight+0.1))
        {
            std::cout << "Module " << m->getId() << " exceed the boundary!" << std::endl;
            std::cout << "Position: (" << m->getPosition().x() << "," << m->getPosition().y() << ")" << std::endl;
            std::cout<<"Rotated WH: (" << m->getRotatedWidth() << "," << m->getRotatedHeight() << ")" << std::endl;
            return false;
        }
    }

    int bound_y = 0;
    int bound_x = 0;
    int area = 0;

    for (int i = 0; i < modules.size(); i++)
    {
        if( modules[i]->getPosition().y() + modules[i]->getRotatedHeight() > bound_y )
        {
            bound_y = modules[i]->getPosition().y() + modules[i]->getRotatedHeight();
        }
        if( modules[i]->getPosition().x() + modules[i]->getRotatedWidth() > bound_x )
        {
            bound_x = modules[i]->getPosition().x() + modules[i]->getRotatedWidth();
        }
        area += modules[i]->getOrgWidth() * modules[i]->getOrgHeight();
    }

    std::cout << spec.targetWidth << " " << spec.targetHeight << std::endl;
    std::cout << "The height of the floorplan is " << bound_y << std::endl;
    std::cout << "The width of the floorplan is " << bound_x << std::endl;
    std::cout << "intization: " << float(area) / float(bound_y*bound_x) << std::endl;
    return true;
}