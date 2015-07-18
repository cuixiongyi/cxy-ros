#pragma once

#include <fstream>
#include <istream>

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

        /**
         * getline function return what the line is about
         * return -1 if there is no more line
         * return 0 if the line is ok
         * return -2 if the line contain # which is comment, should be ignore
         */
		virtual bool getline(std::istream& is, std::string& line, int8_t& lineStatus)
        {
            lineStatus = 0;

            bool result = std::getline(is, line);
            if (false == result)
            {
                lineStatus = -1;
                return false;
            }
            if (line.find("#") != std::string::npos)
            {
                lineStatus = -2;
                return true;
            }
			if ("" == line)
            {
                lineStatus = -2;
                return true;
            }
            return true;
        }
	private:
		std::string filename_;

	protected:
		std::ifstream fin_;
	};
}