/*
 * @Author: your name
 * @Date: 2022-03-03 21:30:01
 * @LastEditTime: 2022-10-03 16:00:36
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
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
    traffic_light = 3,
    park = 4,
    crosswalk = 5
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
    int ID_;
    Edge edge_;
};

struct regulatoryelement
{
    int ID_;
    RelationSubType subtype_;//交通标志类型
    int stoplineid_;//stop_line的id
    int laneletid_;//关联lanelet序号
    int centerpoint3did_;//关联到该lanelet的精确的中心点id
    regulatoryelement() {}
    regulatoryelement(const int id)
    {
        ID_ = id;
        subtype_ = RelationSubType::unknown;
        stoplineid_ = -1;
        laneletid_ = -1;
        centerpoint3did_ = -1;
    }
    bool operator==(regulatoryelement &a) const
    {
        return (a.ID_ == this->ID_);
    }
};


struct relationship
{
    int ID_;
    WayEdge leftedge_;
    WayEdge rightedge_;
    RelationType type_;
    RelationSubType subtype_;
    double speed_limit_ = 0;
    WayDirection turn_direction_;
    //type = regulatory_element, 以下3个有意义，且表示对应way元素索引
    int refers_;
    int ref_line_;
    int light_bulbs_;
    bool operator==(relationship &a) const
    {
        return (this->ID_ == a.ID_);
    }
};

struct parkspace
{
    int ID_;
    int leftID_;
    int rightID_;
    parkspace() {}
    parkspace(const int id)
    {
        ID_ = id;
        leftID_ = -1;
        rightID_ = -1;
    }
    bool operator==(const parkspace &a) const
    {
        return (this->ID_ == a.ID_);
    }
};

struct crosswalk
{
    int ID_;
    int leftID_;
    int rightID_;
    crosswalk() = default;
    crosswalk(const int id)
    {
        ID_ = id;
        leftID_ = -1;
        rightID_ = -1;
    }
    bool operator==(const crosswalk &a) const
    {
        return (this->ID_ == a.ID_);
    }
};




class Relation: public map_base<relationship>
{
private:
    //std::unordered_map<int, relationship*> Data;
    //int numbers;
    //TiXmlElement *node_root;
    std::unordered_map<int, regulatoryelement*> trafficSign_;
    std::unordered_map<int, parkspace*> parkLots_;
    std::unordered_map<int, crosswalk*> crosswalks_;
    RelationType Matchtype(const std::string s) const;
    RelationSubType MatchSubtype(const std::string s) const;
    WayDirection MatchDirection(const std::string s) const;
public:
    Relation();
    Relation(TiXmlElement *root);
    virtual void CreateOneObject(TiXmlElement *head);
    //virtual void CreateObjects(TiXmlElement *tail);
    //virtual int Size() const;
    //virtual relationship* Find(int id_);
    std::unordered_map<int, regulatoryelement*> GetTrafficSignHashMap() const {return trafficSign_;}
    std::unordered_map<int, regulatoryelement*>::const_iterator RegulatoryelementBegin() const {return trafficSign_.cbegin();}
    std::unordered_map<int, regulatoryelement*>::const_iterator RegulatoryelementEnd() const {return trafficSign_.cend();}
    regulatoryelement* FindRegulatoryelement(const int id) const {return trafficSign_.at(id);}

    std::unordered_map<int, parkspace*> GetParkLotsHashMap() const {return parkLots_;}
    std::unordered_map<int, parkspace*>::const_iterator ParklotsBegin() const {return parkLots_.cbegin();}
    std::unordered_map<int, parkspace*>::const_iterator ParklotsEnd() const {return parkLots_.cend();}
    parkspace* FindParkLot(const int id) const {return parkLots_.at(id);}

    std::unordered_map<int, crosswalk*> GetCrosswalksHashMap() const {return crosswalks_;}
    std::unordered_map<int, crosswalk*>::const_iterator CrosswalksBegin() const {return crosswalks_.cbegin();}
    std::unordered_map<int, crosswalk*>::const_iterator CrosswalksEnd() const {return crosswalks_.cend();}
    crosswalk* FindCrosswalk(const int id) const {return crosswalks_.at(id);}
    //判断该lanelet是否有交通标志，如果有则返回True，否则False
    bool IsRegulatoryElement(const int id) const;
    //判断该lanelet是否有停止线，如果有则返回True，否则False
    //当前只要有交通信号标志即有停止线, 等价于isRegulatoryelement()
    bool IsStopLine(const int id) const;
    //根据lanelet的id寻找与其对应的交通信号标志，返回全部的交通标志信息
    std::vector<regulatoryelement*> GetRegulatoryElement(const int id) const;
    virtual ~Relation();
};



};//namespace relation
};//namespace map









#endif