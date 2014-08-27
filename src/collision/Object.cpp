#include "Object.h"

#include "MatrixDefsInternal.h"


namespace
{
    void EigenToDoubleVector(const Eigen::Vector3d& from, PQP_REAL* to)
    {
        for(int i=0; i<3; ++i)
        {
            to[i] = from(i);
        }
    }

    void EigenToDoubleMatrix(const Eigen::Matrix3d& from, PQP_REAL to [3][3])
    {
        for(int i=0; i<3; ++i)
        {
            for(int j=0; j<3; ++j)
            {
                to[i][j] = from(i,j);
            }
        }
    }
}

using namespace planner;

Object::Object(PQP_Model* model)
    : model_(model)
    , position_(Eigen::Vector3d::Zero())
    , orientation_(Eigen::Matrix3d::Identity())
{
    EigenToDoubleMatrix(orientation_, pqpOrientation_);
    EigenToDoubleVector(position_, pqpPosition_);
}

Object::Object(PQP_Model* model, const T_Vector3& normals)
    : model_(model)
    , position_(Eigen::Vector3d::Zero())
    , orientation_(Eigen::Matrix3d::Identity())
	, normals_(normals)
{
    EigenToDoubleMatrix(orientation_, pqpOrientation_);
    EigenToDoubleVector(position_, pqpPosition_);
}

Object::Object(const Object& parent)
    : position_(Eigen::Vector3d::Zero())
    , orientation_(Eigen::Matrix3d::Identity())
{
    model_ = new PQP_Model;
    model_->BeginModel();
    for (int i =0; i< parent.model_->num_tris; ++i)
    {
         // TODO CHECK MEME ?
        model_->AddTri(parent.model_->tris[i].p1, parent.model_->tris[i].p2, parent.model_->tris[i].p3, parent.model_->tris[i].id);
    }
    model_->EndModel();
    SetPosition(parent.GetPosition());
    SetOrientation(parent.GetOrientation());
    EigenToDoubleMatrix(orientation_, pqpOrientation_);
    EigenToDoubleVector(position_, pqpPosition_);
}

Object::~Object()
{
    delete model_;
    //delete pqpOrientation_;
    //delete pqpPosition_;
}

bool Object::IsColliding(Object* object)
{
    PQP_CollideResult cres;
    PQP_Collide(&cres, pqpOrientation_, pqpPosition_, model_,
                object->pqpOrientation_, object->pqpPosition_, object->model_, PQP_FIRST_CONTACT);
    return cres.Colliding();
}

bool Object::IsColliding(T_Object& objects)
{
    PQP_CollideResult cres;
    for(T_Object::iterator it = objects.begin();
        it != objects.end(); ++it)
    {
        PQP_Collide(&cres, pqpOrientation_, pqpPosition_, model_,
                    (*it)->pqpOrientation_, (*it)->pqpPosition_, (*it)->model_, PQP_FIRST_CONTACT);
        if (cres.Colliding())
        {
            return true;
        }
    }
    return false;
}

bool Object::IsColliding(const T_Object::iterator& from,  const T_Object::iterator& to)
{
    T_Object::iterator it = from;
    PQP_CollideResult cres;
    for(;it != to; ++it)
    {
        PQP_Collide(&cres, pqpOrientation_, pqpPosition_, model_,
                    (*it)->pqpOrientation_, (*it)->pqpPosition_, (*it)->model_, PQP_FIRST_CONTACT);
        if (cres.Colliding())
        {
            return true;
        }
    }
    return false;
}

void Object::SetOrientation(const Eigen::Matrix3d& orientation)
{
    orientation_ = orientation;
    EigenToDoubleMatrix(orientation_, pqpOrientation_);
}

void Object::SetPosition(const Eigen::Vector3d& position)
{
    position_ = position;
    EigenToDoubleVector(position_, pqpPosition_);
}

const Eigen::Matrix3d& Object::GetOrientation() const
{
    return orientation_;
}

const Eigen::Vector3d& Object::GetPosition() const
{
    return position_;
}


const PQP_Model* Object::GetModel() const
{
    return model_;
}
