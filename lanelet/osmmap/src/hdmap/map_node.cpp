/*
 * @Author: your name
 * @Date: 2022-03-03 21:28:59
 * @LastEditTime: 2022-09-12 17:55:48
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/hdmap/map_node.cpp
 */
#include "../include/osmmap/map_node.h"

namespace map
{
namespace node
{

Node::Node()
{
    //
}

Node::Node(TiXmlElement *root)
{
    //std::cout << "new node" << std::endl;
    Setroot(root);
}

void Node::CreateOneObject(TiXmlElement *head)
{
    //node
    Point3D *oneobject = new Point3D;
    oneobject->ID = std::atoi(head->Attribute("id"));
    oneobject->latitude = std::atof(head->Attribute("lat"));
    oneobject->longitude = std::atof(head->Attribute("lon"));
    TiXmlElement *tag_pin = head->FirstChildElement("tag");
    //bool islocalx = false;
    //bool islocaly = false;
    for(auto it = tag_pin; it != nullptr; it = it->NextSiblingElement())
    {
        std::string s(it->Attribute("k"));
        if(s == "local_x")
        {
            oneobject->local_x = std::atof(it->Attribute("v"));
            //islocalx = true;
        }else if(s == "local_y"){
            oneobject->local_y = std::atof(it->Attribute("v"));
            //islocaly = true;
        }else if(s == "ele"){
            oneobject->elevation = std::atof(it->Attribute("v"));
        }else{
            continue;
        }
    }
    /*if(!islocalx || !islocaly)
    {
        MercatorGPS2xy(oneobject);
    }*/
    Insert(oneobject->ID, oneobject);
}

/*void Node::MercatorGPS2xy(Point3D *pin)
{
    //std::cout << "mercatorGPS2xy" << std::endl;
    double lat = std::min(89.5, std::max(pin->latitude, -89.5));
    double phi = lat * M_PI / 180.0;
    double con = Eccent * std::sin(phi);
    con = std::pow((1.0 - con) / (1.0 + con), 0.5 * Eccent);
    double ts = std::tan(0.5 * (M_PI * 0.5 - phi)) / con;
    const double y = -RMajor * std::log(ts);
    const double x = RMajor * pin->longitude * M_PI / 180.;
    pin->local_x = x;
    pin->local_y = y;
}

void Node::Mercatorxy2GPS(Point3D *pin)
{
    //std::cout << "mercatorxy2GPS" << std::endl;
    double ts = std::exp(-pin->local_y / RMajor);
    double phi = M_PI / 2 - 2 * std::atan(ts);
    double dphi = 1.0;
    for (int i = 0; fabs(dphi) > 0.000000001 && i < 15; i++) 
    {
        double con = Eccent * sin(phi);
        dphi = M_PI / 2 - 2 * atan(ts * pow((1.0 - con) / (1.0 + con), 0.5 * Eccent)) - phi;
        phi += dphi;
    }
    const double lat = 180 / M_PI * phi;
    const double lon = 180 / M_PI * pin->local_x / RMajor;
    pin->latitude = lat;
    pin->longitude = lon;
}*/

Node::~Node()
{
    // std::cout << "~Node" << std::endl;
}


};//namespace node
};//namespace map