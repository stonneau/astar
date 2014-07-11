#include "Collider.h"
#include "Object.h"

using namespace planner;

Collider::Collider(const T_Object& objects)
    : objects_(objects)
{
    // NOTHING
}

Collider::~Collider()
{
    for(T_Object::iterator it = objects_.begin();
        it != objects_.end();
        ++it)
    {
        delete (*it);
    }
}

bool Collider::IsColliding(const Object* model) const
{

}
