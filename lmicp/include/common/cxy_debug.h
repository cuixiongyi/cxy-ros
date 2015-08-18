#pragma once
#include "iostream"
//#define ENABLE_CXY_ASSERT 

/*
#ifdef ENABLE_CXY_ASSERT
#   define CXY_ASSERT(condition, message) \
    do { \
        if (! (condition)) { \
            std::cerr << "Assertion `" #condition "` failed in " << __FILE__ \
                      << " line " << __LINE__ << ": " << message << std::endl; \
            std::exit(EXIT_FAILURE); \
        } \
    } while (false)
#else
#   define ASSERT(condition, message) do { } while (false)
#endif
*/

#ifdef ENABLE_CXY_ASSERT

    #   define CXY_ASSERT(condition) \
        do { \
            if (! (condition)) { \
                std::cerr << "Assertion `" #condition "` failed in " << __FILE__ \
                          << " line " << __LINE__ << ": " << std::endl; \
                std::exit(EXIT_FAILURE); \
            } \
        } while (false)

#   define CXY_ASSERT(condition) ;

#else
    #   define CXY_ASSERT(condition) do { } while (false)
#endif