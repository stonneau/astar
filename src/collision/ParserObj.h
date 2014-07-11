
#ifndef _CLASS_ParserObj
#define _CLASS_ParserObj

#include <string>
#include <vector>
#include <memory>

namespace planner
{
struct Object;
struct ParserPImpl;
struct ParserObj
{
public:
	 ParserObj();
	~ParserObj();

    std::vector<Object*> CreateWorld(const std::string& /*filename*/);


private:
	std::auto_ptr<ParserPImpl> pImpl_;
};
}//namespace planner;
#endif //_CLASS_ParserObj
