
#ifndef CXY_COMMON
#define CXY_COMMON 

#define CXY_PI 3.1415926

#define Deg2Rad(deg) deg / 180.0 * CXY_PI
#define Rad2Deg(rad) rad * 180.0 / CXY_PI
namespace cxy
{
    enum Update_Status : std::int8_t
    {
        NotUptoDate = 0,
        BeingProcessed = 1,
        UptoDate = 2,
    };

    enum Joint_Relation : std::int8_t
    {
        No_Relation = -1,
        Immediate_Parent = 0,
        Immediate_Child= 1,
        Parent = 2,
        Child = 3,
        Parallal = 4,
        No_Parent = 5,
    };
}

#endif /* CXY_COMMON */