/*
 * @Author: your name
 * @Date: 2022-03-03 21:29:16
 * @LastEditTime: 2022-10-03 16:55:38
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/hdmap/map_relation.cpp
 */
#include "osmmap/map_relation.h"


namespace map
{
namespace relation
{
    
Relation::Relation()
{
}

Relation::Relation(TiXmlElement* root)
{
    Setroot(root);
}

void Relation::CreateOneObject(TiXmlElement *head)
{
    relationship *oneobject = new relationship;
    oneobject->ID_ = std::atoi(head->Attribute("id"));
    TiXmlElement *member_pin = head->FirstChildElement("member");
    TiXmlElement *tag_pin = head->FirstChildElement("tag");
    for(auto it = member_pin; it != tag_pin; it = it->NextSiblingElement())
    {
        std::string s(it->Attribute("role"));
        if(s == "left")
        {
            oneobject->leftedge_.edge_ = Edge::left;
            oneobject->leftedge_.ID_ = std::atoi(it->Attribute("ref"));
        }else if(s == "right"){
            oneobject->rightedge_.edge_ = Edge::right;
            oneobject->rightedge_.ID_ = std::atoi(it->Attribute("ref"));
        }else if(s == "refers"){
            oneobject->refers_ = std::atoi(it->Attribute("ref"));
        }else if(s == "ref_line"){
            oneobject->ref_line_ = std::atoi(it->Attribute("ref"));
        }else if(s == "light_bulbs"){
            oneobject->light_bulbs_ = std::atoi(it->Attribute("ref"));
        }else{
            continue;
        }
    }
    for(auto it = tag_pin; it != nullptr; it = it->NextSiblingElement())
    {
        std::string s(it->Attribute("k"));
        std::string p(it->Attribute("v"));
        if(s == "type")
        {
            oneobject->type_ = Matchtype(p);
        }else if(s == "subtype"){
            oneobject->subtype_ = MatchSubtype(p);
        }else if(s == "speed_limit"){
            oneobject->speed_limit_ = std::atof(it->Attribute("v"));
        }else if(s == "turn_direction"){
            oneobject->turn_direction_ = MatchDirection(p);
        }else{
            continue;
        }
    }
    Insert(oneobject->ID_, oneobject);

    //存储regulatoryelement,在此定义regulatoryelement，在centerway中Matchregulatoryelement(...)函数赋值
    if(oneobject->type_ == RelationType::regulatory_element)
    {
        regulatoryelement *onetrafficsign = new regulatoryelement(oneobject->ID_);
        trafficSign_[onetrafficsign->ID_] = onetrafficsign;
    }

    if(oneobject->subtype_ == RelationSubType::park)
    {
        parkspace *onepark = new parkspace(oneobject->ID_);
        onepark->leftID_ = oneobject->leftedge_.ID_;
        onepark->rightID_ = oneobject->rightedge_.ID_;
        parkLots_[onepark->ID_] = onepark; 
    }

    if(oneobject->subtype_ == RelationSubType::crosswalk)
    {
        crosswalk *onecrosswalk = new crosswalk(oneobject->ID_);
        onecrosswalk->leftID_ = oneobject->leftedge_.ID_;
        onecrosswalk->rightID_ = oneobject->rightedge_.ID_;
        crosswalks_[onecrosswalk->ID_] = onecrosswalk;
    }
}

RelationType Relation::Matchtype(const std::string s) const
{
    if(s == "lanelet")
    {
        return RelationType::lanelet;
    }else if(s == "regulatory_element"){
        return RelationType::regulatory_element;
    }else{
        return RelationType::unknown;
    }
}

RelationSubType Relation::MatchSubtype(const std::string s) const
{
    if(s == "road")
    {
        return RelationSubType::road;
    }else if(s == "traffic_sign"){
        return RelationSubType::traffic_sign;
    }else if(s == "traffic_light"){
        return RelationSubType::traffic_light;
    }else if(s == "park"){
        return RelationSubType::park;
    }else if(s == "crosswalk"){
        return RelationSubType::crosswalk;
    }else{
        return RelationSubType::unknown;
    }
}

WayDirection Relation::MatchDirection(const std::string s) const
{
    if(s == "straight")
    {
        return WayDirection::straight;
    }else if(s == "left"){
        return WayDirection::left;
    }else if(s == "right"){
        return WayDirection::right;
    }else{
        return WayDirection::unknown;
    }
}

bool Relation::IsRegulatoryElement(const int id) const
{
    bool flag = false;
    for(auto it = trafficSign_.begin(); it != trafficSign_.end(); ++it)
    {
        if(id == it->second->laneletid_)
        {
            flag = true;
            break;
        }
    }
    return flag;
}

bool Relation::IsStopLine(const int id) const
{
    bool flag = false;
    for(auto it = trafficSign_.begin(); it != trafficSign_.end(); ++it)
    {
        if(id == it->second->laneletid_)
        {
            flag = true;
            break;
        }
    }
    return flag;
}

std::vector<regulatoryelement*> Relation::GetRegulatoryElement(const int id) const
{
    std::vector<regulatoryelement*> res;
    for(auto it = trafficSign_.begin(); it != trafficSign_.end(); ++it)
    {
        if(id == it->second->laneletid_)
        {
            res.push_back(it->second);
        }
    }
    return res;
}

Relation::~Relation()
{
    // std::cout << "~relation" << std::endl;
    for(auto it = trafficSign_.begin(); it != trafficSign_.end(); ++it)
    {
        delete it->second;
    }
    trafficSign_.clear();
    for(auto it = parkLots_.begin(); it != parkLots_.end(); ++it)
    {
        delete it->second;
    }
    parkLots_.clear();
    for(auto it = crosswalks_.begin(); it != crosswalks_.end(); ++it)
    {
        delete it->second;
    }
    crosswalks_.clear();
}



};//namespace relation
};//namespace map