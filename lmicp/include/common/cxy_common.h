
#ifndef CXY_COMMON
#define CXY_COMMON 

#define CXY_PI 3.1415926

#define Deg2Rad(deg) deg / 180.0 * CXY_PI
#define Rad2Deg(rad) rad * 180.0 / CXY_PI
namespace cxy
{
    enum Update_Status : int8_t
    {
        NotUptoDate = 0,
        BeingProcessed = 1,
        UptoDate = 2,
    };

}
#endif /* CXY_COMMON */