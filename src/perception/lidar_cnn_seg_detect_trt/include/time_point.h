#ifndef _TIME_POINT_H_
#define _TIME_POINT_H_

#include <iostream>
#include <chrono>
#include <vector>
#include <fstream>

#define _GLOBAL_OUTPUT_

// #ifdef _GLOBAL_OUTPUT_
//     #define _T(expression,...) _TP(expression , __VA_ARGS__)
// #else
//     #define _T(expression, ...) 
// #endif 

#ifdef _GLOBAL_OUTPUT_
    #define _T(expression) _TP(expression)
#else
    #define _T(expression) {expression;}
#endif 

#define  _TP(expression) { \
    std::chrono::system_clock::time_point start_point = std::chrono::system_clock::now(); \
    expression; \
    std::chrono::system_clock::time_point end_point = std::chrono::system_clock::now(); \
    auto dura = (std::chrono::duration_cast<std::chrono::microseconds>(end_point - start_point)).count() / 1000.0f; \
    std::cout << #expression << " running time: " << dura << " ms." << std::endl; \
}

#define _TCSV_INIT() std::chrono::system_clock::time_point tcsv_start_point, tcsv_end_point; \
    std::vector<float> tcsv_duration

#define _TCSV_START() tcsv_start_point = std::chrono::system_clock::now()

#define _TCSV_END() tcsv_end_point = std::chrono::system_clock::now(); \
    tcsv_duration.push_back( (std::chrono::duration_cast<std::chrono::microseconds>(tcsv_end_point - tcsv_start_point)).count() / 1000.0f )

#define _TCSV_PRINT(filename, enable)  if (enable) { \
        std::ofstream csv_file(filename, std::ofstream::out | std::ofstream::app); \
                    for (auto t : tcsv_duration) { \
                        csv_file << t << "," ; \
                    } \
                    csv_file << "\n"; \    
    } else { \
                    for (auto t : tcsv_duration) { \
                        std::cout << t << " ms, " ; \
                    } \
                    std::cout << std::endl; \   
    } 



#endif // _TIME_POINT_H_