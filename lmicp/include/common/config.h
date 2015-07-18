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
		config(std::string filename);
		~config();
		
		virtual void serialize();

		virtual void unserialize();


				
	private:
		std::string filename_;

		int joint_number = {0};
	};
}