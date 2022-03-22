/*
 * @Author: your name
 * @Date: 2022-03-03 21:30:01
 * @LastEditTime: 2022-03-16 19:49:05
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/include/osmmap/map_relation.h
 */
#ifndef MAP_RELATION_H_
#define MAP_RELATION_H_

#include "map_base.h"

/*relation类型元素
* 把读入数据存储在relationship中
*
*/

namespace map
{
namespace relation
{
enum class Edge
{
    unknown = 1,
    left = 2,
    right = 3
};

enum class RelationType
{
    unknown = 0,
    lanelet = 1,
    regulatory_element = 2
};

enum class RelationSubType
{
    unknown = 0,
    road = 1,
    traffic_sign = 2,
    traffic_light = 3
};

enum class WayDirection
{
    unknown = 0,
    straight = 1,
    left = 2,
    right = 3
};

struct WayEdge
{
    int ID;
    Edge edge;
};

struct relationship
{
    int ID;
    WayEdge leftedge;
    WayEdge rightedge;
    RelationType type;
    RelationSubType subtype;
    double speed_limit = 0;
    WayDirection turn_direction;
    //type = regulatory_element, 以下3个有意义，且表示对应way元素索引
    int refers;
    int ref_line;
    int light_bulbs;
    bool operator==(relationship &a)
    {
        return (this->ID == a.ID);
    }
};


class Relation: public map_base<relationship>
{
private:
    //std::unordered_map<int, relationship*> Data;
    //int numbers;
    //TiXmlElement *node_root;
public:
    Relation();
    Relation(TiXmlElement *root);
    virtual void CreateOneObject(TiXmlElement *head);
    //virtual void CreateObjects(TiXmlElement *tail);
    //virtual int Size() const;
    //virtual relationship* Find(int id_);
    RelationType Matchtype(std::string s);
    RelationSubType MatchSubtype(std::string s);
    WayDirection MatchDirection(std::string s);
    virtual ~Relation();
};



};//namespace relation
};//namespace map









#endif