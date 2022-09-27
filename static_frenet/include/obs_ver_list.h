#ifndef _OBS_VER_H
#define _OBS_VER_H

#include<iostream>
#include<vector>
#include<array>
#include<string>

class ObstacleVertexes{
public:
//定义一个二维容器，第一层存放端点列表，每个元素是端点；第二层端点列表中存放端点（x,y）;
  std::vector<std::vector<float>> vertexes_list;

  //定义一个二维容器，存放笛卡尔坐标下x,y方向速度;
  std::vector<float> speed_list;
};
using ListObsVer=std::vector<ObstacleVertexes>;

#endif