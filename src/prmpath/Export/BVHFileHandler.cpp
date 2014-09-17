#include "Export/BVHFileHandler.h"

using namespace bvh;
using namespace std;

BVHFileHandler::BVHFileHandler()
    : bvhStr_()
    , tabs_("")
    , depth_(0)
{
    // NOTHING
}

BVHFileHandler::~BVHFileHandler()
{
    // NOTHING
}

bool BVHFileHandler::Save(const string & filename)
{
    ofstream myfile;
    myfile.open (filename.c_str());
    if (myfile.is_open())
    {
        myfile << bvhStr_.rdbuf();
        myfile.close();
        return true;
    }
    return false;
}

const std::string BVHFileHandler::AddTab(bool newLine)
{
    tabs_ += "\t";
    ++depth_;
    if (newLine) bvhStr_ << nl();
    return tabs_;
}

BVHFileHandler& BVHFileHandler::operator<< (const std::string& val)
{
    bvhStr_ << val;
    return *this;
}

BVHFileHandler& BVHFileHandler::operator<< (const double& val)
{
    bvhStr_ << val;
    return *this;
}

const std::string BVHFileHandler::nl() const
{
    return std::string( "\n" + tabs_);
}

const std::string BVHFileHandler::RemoveTab(bool newLine)
{
    if(depth_>0)
    {
        depth_--;
        tabs_.clear();
        for(int i=0; i<depth_; ++i)
        {
            tabs_ += "\t";
        }
    }
    if (newLine) bvhStr_ << nl();
    return tabs_;
}
