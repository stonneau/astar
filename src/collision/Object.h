
#ifndef _CLASS_OBJECT
#define _CLASS_OBJECT

#include <vector>
#include <memory>

#include "PQP/PQP.h"
#include <Eigen/Dense>

namespace planner
{

class Object
{
public:
    typedef std::vector<Object*> T_Object;

public:
     Object(PQP_Model *model);
    ~Object();

     bool IsColliding(Object* object); // can not be const because of pqp but it is...
     bool IsColliding(T_Object& objects); // can not be const because of pqp but it is...

     void SetOrientation(const Eigen::Matrix3d& orientation);
     void SetPosition(const Eigen::Vector3d& position);

     const Eigen::Matrix3d& GetOrientation();
     const Eigen::Vector3d& GetPosition();
     const PQP_Model* GetModel();

private:
    PQP_Model* model_;
    Eigen::Vector3d position_;
    Eigen::Matrix3d orientation_;
    PQP_REAL pqpOrientation_ [3][3];
    PQP_REAL pqpPosition_ [3];
};
}//namespace planner;
#endif //_CLASS_COLLIDER
