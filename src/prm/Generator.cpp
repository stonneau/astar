#include "Generator.h"

#include <time.h>

namespace
{
    static bool generatorInit = false;
}

using namespace planner;

Generator::Generator(Object::T_Object &objects, const Object &model)
    : model_(model)
    , collider_(objects)
{
    if(! ::generatorInit)
    {
        ::generatorInit = true;
        srand((unsigned int)(time(0))); //Init Random generation
    }
}

Generator::~Generator()
{
    // NOTHING
}

Object* Generator::operator()()
{
    // en v0, juste les positions
    int limit = 1000;
    int min = -10;
    int max = 10;
    Object * res = new Object(model_);
    Eigen::Vector3d pos;
    while(limit > 0)
    {
        for(int i =0; i<3; ++i)
        {
            pos(i) = (rand() % (max-min + 1) + min);
        }
        res->SetPosition(pos);
        if(!collider_.IsColliding(res))
        {
            return res;
        }
    }
    return 0;
}
