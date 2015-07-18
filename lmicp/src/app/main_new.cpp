#include "common/config.h"



int main(int argc, char const *argv[])
{
	//cxy::config("common/config");
	cxy::config con("/home/xiongyi/cxy_workspace/src/cxyros/lmicp/include/common/config");
	con.unserialize();
	return 0;
}