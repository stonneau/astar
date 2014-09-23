#include "Collider.h"
#include "Object.h"

using namespace planner;

Collider::Collider(const Object::T_Object& objects)
    : objects_(objects)
{
    // NOTHING
}

Collider::~Collider()
{
    // NOTHING
}

bool Collider::IsColliding(Object *model)
{
    return model->IsColliding(objects_);
}

std::vector<size_t> Collider::IsColliding(Object::T_Object& objects)
{
    size_t id = 0;
    std::vector<size_t> res;
    for(Object::T_Object::iterator it = objects.begin();
        it != objects.end(); ++it, ++id)
    {
        if(IsColliding(*it))
            res.push_back(id);
    }
    return res;
}

bool Collider::IsColliding()
{
    for(Object::T_Object::iterator it = objects_.begin();
        it != objects_.end();
        ++it)
    {
        Object::T_Object::iterator it2 = it; ++it2;
        for(; it2 != objects_.end(); ++it2)
        {
            if((*it)->IsColliding(it2, objects_.end()))
                return true;
        }
    }
    return false;
}
