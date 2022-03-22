/*
 * @Author: your name
 * @Date: 2022-03-03 21:30:09
 * @LastEditTime: 2022-03-09 21:09:29
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/include/osmmap/map_base.h
 */
#ifndef MAP_BASE_H_
#define MAP_BASE_H_

#include <iostream>
#include <cmath>
#include <vector>
#include <unordered_map>
#include "tinyxml.h"

/*地图元素基本抽象类
* std::unordered_map<int, T*>表示存放的数据，使用哈系表
* numbers表示数据数量
* node_root表示解析osm文件的根指针
*/
namespace map
{
template <typename T>
class map_base
{
private:
    std::unordered_map<int, T*> Data;
    int numbers;
    TiXmlElement *node_root;
public:
    //map_base(/* args */);
    //基类初始化
    void Setroot(TiXmlElement *x) {node_root = x; numbers = 0;}
    //解析一个某类型的数据
    virtual void CreateOneObject(TiXmlElement *head) = 0;
    //解析某类型全部的数据
    void CreateObjects(TiXmlElement *tail)
    {
        for(auto it = node_root; it != tail; it = it->NextSiblingElement())
        {
            CreateOneObject(it);
            numbers++;
        }
    }
    //插入哈系表元素时，numbers增加1
    void Addnumbers() {numbers++;}
    //返回哈系表元素数量
    int Size() {return numbers;}
    //哈系表插入元素
    void Insert(int id_, T* x) {Data[id_] = x;}
    //根据索引（某类元素的ID号）从哈系表查找对应的具体数据
    T* Find(int id_) {return Data[id_];}
    //返回哈系表第一个元素的指针
    typename std::unordered_map<int, T*>::iterator Begin() {return Data.begin();}
    //返回哈系表元素的最后一个元素指针
    typename std::unordered_map<int, T*>::iterator End() {return Data.end();}
    //析构，销毁哈系表及指针，释放内存
    virtual ~map_base() 
    {
        //std::cout << "~map_base" << std::endl;
        for(auto it = Data.begin(); it != Data.end(); ++it)
        {
            delete it->second;
        }
        Data.clear();
    }
};




};//namespace map

#endif