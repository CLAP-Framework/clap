/*
 * @Descripttion:
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-01 14:59:30
 * @LastEditors: xiangzhang
 * @LastEditTime: 2020-09-27 19:04:17
 */

#include "smoothfilter.h"

namespace Control {

double meanfilter(vector<double> &filterdata, double param, int smoothsize) {
  if (filterdata.size() >= smoothsize) {
    for (int i = 0; i < smoothsize - 1; i++) {
      filterdata[i] = filterdata[i + 1];
    }
    filterdata[smoothsize - 1] = param;
  } else
    filterdata.push_back(param);
  double sum =
      std::accumulate(std::begin(filterdata), std::end(filterdata), 0.0);
  double mean = sum / filterdata.size();
  return mean;
}
}  // namespace Control