/**
* \file Object.h
* \brief Representation of a collidable 3D object. Contains
* definitions for class Object.
* \author Steve T.
* \version 0.1
* \date 16/07/2014
*
*/
#ifndef _CLASS_OBJECT
#define _CLASS_OBJECT

#include <vector>
#include <memory>

#include "PQP/PQP.h"
#include <Eigen/Dense>

namespace planner
{

/// \class Object
/// \brief Description of a collidable 3d Object.
/// Internal implementation is done using PQP.
class Object
{
public:
    typedef std::vector<Object*> T_Object;
    typedef std::vector<const Object*> CT_Object;

public:
    ///  \brief Constructor
    ///  \param model a PQP_Model upon which Object will be built.
     Object(PQP_Model *model);


     ///  \brief Constructor
     ///  \param parent a Object upon which Object will be built.
      Object(const Object& parent);
     ///  \brief Destructor
    ~Object();

     ///  \brief returns whether the current Object collide with another.
     ///  \param object the object on which to test the collision
     ///  \param return : true if a collision exists, false otherwise
     bool IsColliding(Object* object); // can not be const because of pqp but it is...

     ///  \brief returns whether the current Object collide with a list of Object.
     ///  \param objects the objects on which to test the collision
     ///  \param return : true if a collision exists, false otherwise
     bool IsColliding(T_Object& objects); // can not be const because of pqp but it is...


     ///  \brief returns whether the current Object collide with a list of Object described
     ///  by beginning and endind iterators
     ///  \param from iterator to the first element of a set of Objects
     ///  \param to iterator to the last element of a set of Objects
     ///  \param return : true if a collision exists, false otherwise
     bool IsColliding(const T_Object::iterator& from,  const T_Object::iterator& to); // can not be const because of pqp but it is...

     ///  \brief Sets the orientation of the object in space
     ///  \param orientation the rotation matrix assigned to the Object
     void SetOrientation(const Eigen::Matrix3d& orientation);

     ///  \brief Sets the position of the object in space
     ///  \param orientation the position Vector3d assigned to the Object
     void SetPosition(const Eigen::Vector3d& position);

     const Eigen::Matrix3d& GetOrientation() const;
     const Eigen::Vector3d& GetPosition() const;
     const PQP_Model* GetModel() const;

private:
    PQP_Model* model_;
    Eigen::Vector3d position_;
    Eigen::Matrix3d orientation_;
    PQP_REAL pqpOrientation_ [3][3];
    PQP_REAL pqpPosition_ [3];
};
}//namespace planner;
#endif //_CLASS_COLLIDER
