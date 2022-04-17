/*
 * @Author: your name
 * @Date: 2022-03-03 21:29:06
 * @LastEditTime: 2022-04-16 16:32:34
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/map_way.cpp
 */

#include "osmmap/map_way.h"

namespace map
{
namespace way
{

Way::Way()
{
}

Way::Way(TiXmlElement *root)
{
    Setroot(root);
}

void Way::CreateOneObject(TiXmlElement *head)
{
    Line *oneobject = new Line;
    oneobject->ID = std::atoi(head->Attribute("id"));
    TiXmlElement *nd_pin = head->FirstChildElement("nd");
    TiXmlElement *tag_pin = head->FirstChildElement("tag");
    for(auto it = nd_pin; it != tag_pin; it = it->NextSiblingElement())
    {
        oneobject->nodeline[oneobject->length++] = std::atoi(it->Attribute("ref"));
        if(oneobject->length == oneobject->size)
        {
            oneobject->Changesize();
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
            //oneobject->subtype = static_cast<int>(WaySubtype::solid);
            oneobject->subtype = MatchSubtype(p);
        }else{
            continue;
        }
    }
    Insert(oneobject->ID, oneobject);
}

WayType Way::Matchtype(std::string s)
{
    if(s == "line_thin")
    {
        return WayType::line_thin;
    }else if(s == "stop_line"){
        return WayType::stop_line;
    }else if(s == "traffic_light"){
        return WayType::traffic_light;
    }else if(s == "traffic_sign"){
        return WayType::traffic_sign;
    }else if(s == "light_bulbs"){
        return WayType::light_bulbs;
    }else if(s == "road_border"){
        return WayType::road_border;
    }else{
        return WayType::unknown;
    }

}

WaySubtype Way::MatchSubtype(std::string s)
{
    if(s == "solid")
    {
        return WaySubtype::solid;
    }else{
        return WaySubtype::unknown;
    }
    
}

Way::~Way()
{
    //std::cout << "~way" << std::endl;
}


};//namespace way
};//namespace map