
#ifndef _CLASS_COLLIDER
#define _CLASS_COLLIDER

#include <vector>

namespace planner
{

class Object;

typedef std::vector<Object*> T_Object;

class Collider
{
public:
     Collider(const T_Object& objects);
    ~Collider();

     bool IsColliding(const Object* object) const;


private:
    T_Object objects_;
};
}//namespace planner;
#endif //_CLASS_COLLIDER
