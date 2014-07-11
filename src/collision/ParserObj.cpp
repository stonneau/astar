#include "ParserObj.h"
#include "Object.h"
#include "MatrixDefsInternal.h"
#include "PQP/PQP.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>

using namespace std;
using namespace matrices;

namespace
{
typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > T_Vector3;
typedef std::vector<PQP_Model*> T_PQPModel;

string doubleSlash(const string& s)
{
    //Remplace "//" par "/1/".
    string s1="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(i<s.size()-1&&s[i]=='/'&&s[i+1]=='/')
        {
            s1+="/1/";
            i++;
        }
        else
            s1+=s[i];
    }
    return s1;
}

string remplacerSlash(const string& s)
{
    //Remplace les '/' par des espaces.
    string ret="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(s[i]=='/')
            ret+=' ';
        else
            ret+=s[i];
    }
    return ret;
}

vector<string> splitSpace(const string& s)
{
    //Eclate une chaîne au niveau de ses espaces.
    vector<string> ret;
    string s1="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(s[i]==' '||i==s.size()-1)
        {
            if(i==s.size()-1)
                s1+=s[i];
            ret.push_back(s1);
            s1="";
        }
        else
            s1+=s[i];
    }
    return ret;
}
}

namespace
{
	double isLeft( const VectorX& P0, const VectorX& P1, const Vector3& P2 )
	{
		return ( (P1.x() - P0.x()) * (P2.y() - P0.y())
				- (P2.x() - P0.x()) * (P1.y() - P0.y()) );
	}

	const Vector3& LeftMost(const T_Vector3& points)
	{
		unsigned int i = 0;
		for(unsigned int j = 1; j < points.size(); ++j)
		{
			if(points[j].x() < points[i].x())
			{
				i = j;
			}
		}
		return points[i];
	}

	T_Vector3 ConvexHull(const T_Vector3& points)
	{
		T_Vector3 res;
		Vector3 pointOnHull = LeftMost(points);	
		Vector3 endPoint;
		int i = 0;
		do
		{
			++i;
			VectorX pi = pointOnHull;
			endPoint = points[0];
			for(unsigned int j = 1; j < points.size(); ++j)
			{
				if((endPoint == pointOnHull) || (isLeft(pi, endPoint, points[j]) > 0))
				{
					endPoint = points[j];
				}
				
				if( i > 10000 )
				{
                    std::cout << " WTF " << std::endl << points[j] << std::endl;
				}
			}
			res.push_back(pi);
			pointOnHull = endPoint;
		} while(endPoint != res[0]);
		res.push_back(endPoint);
		return res;
    }

    void Vector3toArray(const Vector3& vect, double* arr)
    {
        for(int i=0; i<3; ++i)
        {
            arr[i] = vect[i];
        }
    }
}


namespace planner
{
	struct ParserPImpl
	{

        ParserPImpl()
            : modelOn_(false)
            , currentModel_(0)
            , currentIndex_(-1)
        {
            // NOTHING
        }

    void CreatePQPObstacle(const std::string& line)
    {
        string ligne = doubleSlash(line);
        ligne=remplacerSlash(ligne); //On remplace les '/' par des espaces, ex : pour "f 1/2/3 4/5/6 7/8/9" on obtiendra "f 1 2 3 4 5 6 7 8 9"
        vector<string> termes=splitSpace(ligne.substr(2)); //On éclate la chaîne en ses espaces (le substr permet d'enlever "f ")
        int ndonnees=(int)termes.size()/3;
        // start just with triangles
        if(ndonnees >= 3 && ndonnees <=4) // only triangles or quad
        {
            PQP_REAL points [ndonnees][3];
            for(int i=0; i< ndonnees; ++i)
            {
                long int idx = (strtol(termes[i*3].c_str(),NULL, 10)) - 1;
                Vector3toArray(points_[(int)idx], points[i]);
            }
            currentModel_->AddTri(points[0], points[1], points[2], ++currentIndex_);
            if(ndonnees == 4)
            {
                currentModel_->AddTri(points[0], points[2], points[3], ++currentIndex_);
            }
        }
    }

	void CreateObstacle(const std::vector<std::string>& lines)
	{
		vector<long int> indices;
		T_Vector3 points;
		for(int i =0; i<2; ++i)
		{
			string ligne = doubleSlash(lines[i]);
			ligne=remplacerSlash(ligne); //On remplace les '/' par des espaces, ex : pour "f 1/2/3 4/5/6 7/8/9" on obtiendra "f 1 2 3 4 5 6 7 8 9"
			vector<string> termes=splitSpace(ligne.substr(2)); //On éclate la chaîne en ses espaces (le substr permet d'enlever "f ")
			int ndonnees=(int)termes.size()/3;
			for(int i=0; i <ndonnees;i++) //On aurait très bien pu mettre i<ndonnees mais je veux vraiment limiter à 3 ou 4
			{
				long int idx = (strtol(termes[i*3].c_str(),NULL, 10)) - 1;
				std::vector<long int>::iterator it = indices.begin();
				it = find (indices.begin(), indices.end(), idx);
				if(it == indices.end())
				{
					indices.push_back(idx);
					points.push_back(points_[(int)idx]);
				}
			}
		}
     
		Vector3 p1(points[0]);
		Vector3 p2(points[1]);
		Vector3 p3(points[2]);
		Vector3 p4(points[3]);
		Vector3 u_(p3-p4);
		Vector3 v_(p1-p4);
		if(abs( u_.dot(v_)) > 0.001) v_ = p2 - p4;
		double a_, b_, c_, d_, norm_, normsquare_;
		//we need convex hull of this crap
		Vector3 normal (u_.cross(v_));
		a_ = (float)(normal.x());
		b_ = (float)(normal.y());
		c_ = (float)(normal.z());
		//if (c_ < 0) c_ = -c_;
		norm_ = (float)(normal.norm());
		normsquare_ = norm_ * norm_;
		d_ = (float)(-(a_ * p1.x() + b_ * p1.y() + c_ * p1.z()));

		Matrix4 basis_ = Matrix4::Zero();
		Vector3 x = u_; x.normalize();
		Vector3 y = v_; y.normalize();
		normal.normalize();
		basis_.block(0,0,3,1) = x;
		basis_.block(0,1,3,1) = y;
		basis_.block(0,2,3,1) = normal;
		basis_.block(0,3,3,1) = p4;
		basis_(3,3) = 1;
		Matrix4 basisInverse_ = basis_.inverse();

		T_Vector3 transformedPoints;
		for(int i=0; i<4; ++i)
		{
			transformedPoints.push_back(matrices::matrix4TimesVect3(basisInverse_, points[i]));
		}

		points = ConvexHull(transformedPoints);
		transformedPoints.clear();
		for(int i=0; i<4; ++i)
		{
			transformedPoints.push_back(matrices::matrix4TimesVect3(basis_, points[i]));
		}

		// make sure normal in the good sense :
		u_ = transformedPoints[2] - transformedPoints[3];
		v_ = transformedPoints[0] - transformedPoints[3];

		normal = u_.cross(v_);
		normals_[(int)indices[0]];
		/*if(isGround)
		{
			if(normal.dot(normals_[(int)indices[0]]) > 0 )
			{
				manager_.AddGround(transformedPoints[0], transformedPoints[1], transformedPoints[2], transformedPoints[3]);
			}
			else
			{
				manager_.AddGround(transformedPoints[2], transformedPoints[1], transformedPoints[0], transformedPoints[3]);
			}
		}
		else
		{
			if(normal.dot(normals_[(int)indices[0]]) > 0 )
			{
				manager_.AddObstacle(transformedPoints[0], transformedPoints[1], transformedPoints[2], transformedPoints[3]);
			}
			else
			{
				manager_.AddObstacle(transformedPoints[2], transformedPoints[1], transformedPoints[0], transformedPoints[3]);
			}
		}*/
	}

		T_Vector3 points_;
		T_Vector3 normals_;
        T_PQPModel models_;
        bool modelOn_;
        PQP_Model* currentModel_;
        long int currentIndex_;
	};
}

using namespace planner;

ParserObj::ParserObj()
	: pImpl_(new ParserPImpl())
{
	// NOTHING
}

ParserObj::~ParserObj()
{
	// NOTHING
}


Object::T_Object ParserObj::CreateWorld(const std::string& filename)
{
    Object::T_Object objects;
	string line;
	ifstream myfile (filename);
	std::vector<std::string> lines;
	if (myfile.is_open())
	{
		while ( myfile.good() )
		{
			getline (myfile, line);
            // new model
            if(line.find("o ") == 0 || line.find("g ") == 0)
            {
                PQP_Model* m = new PQP_Model;
                if(pImpl_->modelOn_)
                {
                    // TODO ADD MATRIX !
                    pImpl_->currentModel_->EndModel();
                }
                else
                {
                    pImpl_->modelOn_ = true;
                }
                pImpl_->currentIndex_ = -1;
                pImpl_->models_.push_back(m);
                pImpl_->currentModel_ = m;
                pImpl_->currentModel_->BeginModel();
            }
			if(line.find("v ") == 0)
			{
				char x[255],y[255],z[255];
				sscanf(line.c_str(),"v %s %s %s",x,z,y);
				pImpl_->points_.push_back(Vector3(-strtod (x, NULL), strtod(y, NULL), strtod(z, NULL)));
			}
			if(line.find("vn ") == 0)
			{
				char x[255],y[255],z[255];
				sscanf(line.c_str(),"vn %s %s %s",x,z,y);
				pImpl_->normals_.push_back(Vector3(-strtod (x, NULL), strtod(y, NULL), strtod(z, NULL)));
			}
			if(line.find("f ") == 0)
            {
                pImpl_->CreatePQPObstacle(line);
                //for now just accept triangles
                /*lines.push_back(line);
                if(lines.size() == 2)
				{
					pImpl_->CreateObstacle(lines);
					lines.clear();
                }*/
			}
		}
        myfile.close();
        if(pImpl_->modelOn_)
        {
            pImpl_->currentModel_->EndModel();
            pImpl_->modelOn_ = false;
        }
        // now create Objects
        for(T_PQPModel::iterator it = pImpl_->models_.begin();
            it != pImpl_->models_.end();
            ++it)
        {
            objects.push_back(new Object(*it));
        }
	}
    else
    {
        std::cout << "file not found" << std::endl;
    }
    return objects;
}

