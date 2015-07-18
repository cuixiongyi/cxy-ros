#pragma once

#include <string>
#include <sstream>
#include <iostream>

#include "common/serialization.h"

namespace cxy
{
	class config : public serialization
	{
	public:
		config& operator=(const config&) = delete;	// Disallow copying
		config(const config&) = delete;
		config() = delete;
		config(std::string filename) : 
						serialization(filename)
						, filename_{filename}
			{};
		~config() {};
		
		virtual void serialize()
				{}
		virtual void unserialize()
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
		

				
	private:
		std::string filename_;

		int joint_number = {0};
	};
}