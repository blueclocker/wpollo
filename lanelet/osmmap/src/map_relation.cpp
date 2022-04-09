/*
 * @Author: your name
 * @Date: 2022-03-03 21:29:16
 * @LastEditTime: 2022-04-09 15:02:55
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/map_relation.cpp
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
    oneobject->ID = std::atoi(head->Attribute("id"));
    TiXmlElement *member_pin = head->FirstChildElement("member");
    TiXmlElement *tag_pin = head->FirstChildElement("tag");
    for(auto it = member_pin; it != tag_pin; it = it->NextSiblingElement())
    {
        std::string s(it->Attribute("role"));
        if(s == "left")
        {
            oneobject->leftedge.edge = Edge::left;
            oneobject->leftedge.ID = std::atoi(it->Attribute("ref"));
        }else if(s == "right"){
            oneobject->rightedge.edge = Edge::right;
            oneobject->rightedge.ID = std::atoi(it->Attribute("ref"));
        }else if(s == "refers"){
            oneobject->refers = std::atoi(it->Attribute("ref"));
        }else if(s == "ref_line"){
            oneobject->ref_line = std::atoi(it->Attribute("ref"));
        }else if(s == "light_bulbs"){
            oneobject->light_bulbs = std::atoi(it->Attribute("ref"));
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
            oneobject->type = Matchtype(p);
        }else if(s == "subtype"){
            oneobject->subtype = MatchSubtype(p);
        }else if(s == "speed_limit"){
            oneobject->speed_limit = std::atof(it->Attribute("v"));
        }else if(s == "turn_direction"){
            oneobject->turn_direction = MatchDirection(p);
        }else{
            continue;
        }
    }
    Insert(oneobject->ID, oneobject);

    //存储regulatoryelement,在此定义regulatoryelement，在centerway中Matchregulatoryelement(...)函数赋值
    if(oneobject->type == RelationType::regulatory_element)
    {
        regulatoryelement *onetrafficsign = new regulatoryelement(oneobject->ID);
        TrafficSign[onetrafficsign->ID] = onetrafficsign;
    }
}

RelationType Relation::Matchtype(std::string s)
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

RelationSubType Relation::MatchSubtype(std::string s)
{
    if(s == "road")
    {
        return RelationSubType::road;
    }else if(s == "traffic_sign"){
        return RelationSubType::traffic_sign;
    }else if(s == "traffic_light"){
        return RelationSubType::traffic_light;
    }else{
        return RelationSubType::unknown;
    }
}

WayDirection Relation::MatchDirection(std::string s)
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

bool Relation::isRegulatoryelement(const int id_)
{
    bool flag = false;
    for(auto it = TrafficSign.begin(); it != TrafficSign.end(); ++it)
    {
        if(id_ == it->second->laneletid)
        {
            flag = true;
            break;
        }
    }
    return flag;
}

std::vector<regulatoryelement*> Relation::getRegulatoryelement(const int id_)
{
    std::vector<regulatoryelement*> res;
    for(auto it = TrafficSign.begin(); it != TrafficSign.end(); ++it)
    {
        if(id_ == it->second->laneletid)
        {
            res.push_back(it->second);
        }
    }
    return res;
}

Relation::~Relation()
{
    //std::cout << "~relation" << std::endl;
    for(auto it = TrafficSign.begin(); it != TrafficSign.end(); ++it)
    {
        delete it->second;
    }
    TrafficSign.clear();
}



};//namespace relation
};//namespace map