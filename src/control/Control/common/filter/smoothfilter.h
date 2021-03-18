/*
 * @Descripttion: save smooth filters
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-01 14:57:35
 * @LastEditors: xiangzhang
 * @LastEditTime: 2020-10-10 20:51:17
 */

#ifndef SMOOTHFILTER_H
#define SMOOTHFILTER_H

#include <cmath>
#include <numeric>
#include <vector>

using namespace std;
namespace Control {
/**
 * @name:
 * @brief:Mean filter
 * @param
 * @return
 */
double meanfilter(vector<double> &filterdata, double param, int smoothsize);

}  // namespace Control
#endif