/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-08-30 22:37:42
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-08-31 20:22:12
 * @FilePath: /wpollo/src/lanelet/path_boost/src/tools/time_helper.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "path_boost/tools/time_helper.h"
#include "path_boost/tools/tools.hpp"

namespace PathBoostNS
{
void TimeRecorder::recordTime(const std::string &name)
{
    time_stamps_.emplace_back(std::clock());
    names_.emplace_back(name);
}

void TimeRecorder::printTime() const
{
    if (time_stamps_.size() <= 1) {
        LOG(WARNING) << "time stamps size not enough!";
        return;
    }
    LOG(INFO) << "========Printing time for " << title_ << "========";
    for (size_t i = 0; i < time_stamps_.size() - 1; ++i) {
        LOG(INFO) << names_[i] << " cost " << time_ms(time_stamps_[i], time_stamps_[i + 1]) << " ms.";
    }
    if (time_stamps_.size() > 2) {
        LOG(INFO) << "Total time cost: " << time_ms(time_stamps_.front(), time_stamps_.back()) << " ms.";
    }
    LOG(INFO) << "========End printing time for " << title_ << "========";
}

void TimeRecorder::clear()
{
    time_stamps_.clear();
    names_.clear();
}

}//namespace PathBoostNS