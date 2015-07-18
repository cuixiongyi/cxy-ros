#pragma once

#include <fstream>
#include <string>
#include <stdexcept>
namespace cxy
{

	class serialization
	{
	public:
		serialization& operator=(const serialization&) = delete;
		serialization(const serialization&) = delete;
		serialization() = delete;
		serialization(std::string filename) : filename_(filename)
			{
			}
		virtual ~serialization() {};

		virtual void serialize() = 0;
		virtual void unserialize() = 0;
		
	private:
		std::string filename_;

	protected:
		std::ifstream fin_;
	};
}