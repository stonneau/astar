#ifndef  _BVHEXPORTER_H_
#define  _BVHEXPORTER_H_


#include "prmpath/Export/BVHFileHandler.h"

#include <string>
#include <vector>

namespace planner
{
class Node;
class Robot;
}

/**
*  \class BVHExporter
*  \brief Export A robot to a readable BVH file format
*/
namespace bvh
{
class  BVHExporter
{
public:
     BVHExporter(planner::Robot* /*skeleton*/);
    ~BVHExporter();

public:
    void PushFrame(planner::Node* /*rootNode*/, bool tpose=false);
    bool Save(const std::string& /*filename*/);

private:
    void PushStructure(planner::Robot*/*skeleton*/);
private:
    BVHExporter(const BVHExporter&);
    BVHExporter& operator=(const BVHExporter&);

private:
    BVHFileHandler f_;
    std::vector< std::string > frames_;
};
} //namespace bvh
#endif // _BVHEXPORTER_H_
