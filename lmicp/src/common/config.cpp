
#include "common/config.h"

namespace cxy
{

	config::config(std::string filename) : 
					serialization(filename)
					, filename_{filename}
		{};
	config::~config() {};
	
	void config::serialize()
			{}
	void config::unserialize()
	{
		if (fin_.is_open())
			fin_.close();
		fin_.open(filename_);
		std::cout<<filename_<<std::endl;
		if (! fin_.is_open())
			throw std::runtime_error("open file fail");
		std::string line;
		while (std::getline(fin_, line))
		{
			std::istringstream iss(line);
			std::string var_name;
			iss >> var_name;
			std::cout<<line<<std::endl;
			if ( "joint_number" == var_name)
			{
					iss >> joint_number; 
					break;
			}
		}		
	}
	

}