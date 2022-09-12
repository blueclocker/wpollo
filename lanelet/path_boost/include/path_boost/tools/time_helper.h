/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-08-30 22:39:10
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-08-31 20:17:03
 * @FilePath: /wpollo/src/lanelet/path_boost/include/path_boost/tools/time_helper.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef TIME_HELPER_H_
#define TIME_HELPER_H_

#include <vector>
#include <string>
#include <ctime>
#include "glog/logging.h"


namespace PathBoostNS
{
class TimeRecorder {
 public:
    TimeRecorder(const std::string& title) : title_(title) {}
    void recordTime(const std::string &name);
    void printTime() const;
    void clear();

 private:
    std::string title_;
    std::vector<std::string> names_;
    std::vector<clock_t> time_stamps_;
};

}//namespace PathBoostNS

#endif