/*
 * @Author: your name
 * @Date: 2022-03-03 21:29:50
 * @LastEditTime: 2022-04-19 21:52:50
 * @LastEditors: Please set LastEditors
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
    int ID;
    int size;//最大容量
    int length;//实际元素个数
    int *nodeline;//node的id
    WayType type;
    WaySubtype subtype;
    bool isVisual;
    bool operator==(const Line &a)
    {
        return (this->ID == a.ID);
    }
    Line()
    {
        size = 5;
        length = 0;
        nodeline = new int[size];
        type = WayType::unknown;
        subtype = WaySubtype::unknown;
        isVisual = false;
    }
    void Changesize()
    {
        int *nodelinenew = new int[size*2];
        for(int i = 0; i < size; ++i)
        {
            nodelinenew[i] = nodeline[i];
        }
        size *= 2;
        delete [] nodeline;
        nodeline = nodelinenew;
        
    }
    int Length()
    {
        return length;
    }
};

class Way: public map_base<Line>
{
private:
    //std::unordered_map<int, Line*> Data;
    //int numbers;
    //TiXmlElement *node_root;
    WayType Matchtype(std::string s);
    WaySubtype MatchSubtype(std::string s);
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