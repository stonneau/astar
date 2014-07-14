
#ifndef _CLASS_COLLIDER
#define _CLASS_COLLIDER

#include "Object.h"

#include <vector>

namespace planner
{


class Collider
{
public:
     Collider(const Object::T_Object& objects);
    ~Collider();

     bool IsColliding(Object* object);
     bool IsColliding();


private:
    Object::T_Object objects_;
};
}//namespace planner;
#endif //_CLASS_COLLIDER
