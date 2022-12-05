/*
 * @Author: your name
 * @Date: 2022-03-03 21:29:50
 * @LastEditTime: 2022-10-03 15:35:19
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/include/osmmap/map_way.h
 */
#ifndef MAP_WAY_H_
#define MAP_WAY_H_

#include "map_base.h"

/*way类型元素
* 把读入数据存储在Line中
*
*/

namespace map
{
namespace way
{
enum class WayType
{
    unknown = 0,
    line_thin = 1,
    stop_line = 2,
    traffic_sign = 3,
    traffic_light = 4,
    light_bulbs = 5,
    road_border = 6
};

enum class WaySubtype
{
    unknown = 0,
    solid = 1
};

struct Line
{
    int ID_;
    int size_;//最大容量
    int length_;//实际元素个数
    int *nodeline_;//node的id
    WayType type_;
    WaySubtype subtype_;
    bool isVisual_;
    bool operator==(const Line &a) const
    {
        return (this->ID_ == a.ID_);
    }
    Line()
    {
        size_ = 5;
        length_ = 0;
        nodeline_ = new int[size_];
        type_ = WayType::unknown;
        subtype_ = WaySubtype::unknown;
        isVisual_ = false;
    }
    void Changesize()
    {
        int *nodelinenew = new int[size_*2];
        for(int i = 0; i < size_; ++i)
        {
            nodelinenew[i] = nodeline_[i];
        }
        size_ *= 2;
        delete [] nodeline_;
        nodeline_ = nodelinenew;
        
    }
    int Length() const
    {
        return length_;
    }
};

class Way: public map_base<Line>
{
private:
    //std::unordered_map<int, Line*> Data;
    //int numbers;
    //TiXmlElement *node_root;
    WayType Matchtype(const std::string s) const;
    WaySubtype MatchSubtype(const std::string s) const;
public:
    Way();
    Way(TiXmlElement *root);
    virtual void CreateOneObject(TiXmlElement *head);
    //virtual void CreateObjects(TiXmlElement *tail);
    //virtual int Size() const;
    //virtual Line* Find(int id_);
    virtual ~Way();
};







};//namespace way
};//namespace map


#endif