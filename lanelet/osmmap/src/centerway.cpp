/*
 * @Author: your name
 * @Date: 2022-03-06 15:44:08
 * @LastEditTime: 2022-03-16 14:32:39
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/centerway.cpp
 */
#include "../include/osmmap/centerway.h"

namespace map
{
namespace centerway
{

CenterWay::CenterWay()
{
}

CenterWay::CenterWay(TiXmlElement *root)
{
    Setroot(root);
}

void CenterWay::CreateOneObject(TiXmlElement *head)
{
    return;
}

double CenterWay::NodeDistance(const node::Point3D *a, const node::Point3D *b)
{
    double x = a->local_x - b->local_x;
    double y = a->local_y - b->local_y;
    double z = a->elevation - b->elevation;
    return std::sqrt(x*x + y*y + z*z);
}

double CenterWay::EdgeLength(node::Node *nodes_, way::Line *line)
{
    double res = 0;
    for(int i = 0; i < line->Length() - 1; ++i)
    {
        res += NodeDistance(nodes_->Find(line->nodeline[i]), nodes_->Find(line->nodeline[i+1]));
    }
    return res;
}

std::vector<double> CenterWay::CalculateAccumulatedLengths(node::Node *nodes_, way::Line *line)
{
    std::vector<double> accumulated_lengths;
    double res = 0;
    accumulated_lengths.push_back(0);
    for(int i = 0; i < line->Length() - 1; ++i)
    {
        res += NodeDistance(nodes_->Find(line->nodeline[i]), nodes_->Find(line->nodeline[i+1]));
        accumulated_lengths.push_back(res);
    }
    return accumulated_lengths;
}

std::pair<size_t, size_t> CenterWay::FindNearestIndexPair(const std::vector<double> & accumulated_lengths, 
                                                          const double target_length)
{
    // 累加道路长度的数组的元素个数
    int N = accumulated_lengths.size();

    // 第一个
    if (target_length < accumulated_lengths.at(1)) 
    {
        return std::make_pair(0, 1);
    }

    // 最后一个
    if (target_length > accumulated_lengths.at(N - 2)) 
    {
        return std::make_pair(N - 2, N - 1);
    }

    // 中间
    for (auto i = 1; i < N; ++i) 
    {
        if (accumulated_lengths.at(i - 1) <= target_length && target_length <= accumulated_lengths.at(i)) 
        {
            return std::make_pair(i - 1, i);
        }
    }
}

std::vector<CenterPoint3D> CenterWay::ResamplePoints(node::Node *nodes_, way::Line *line, const int num_segments)
{
    double line_length = EdgeLength(nodes_, line);
    std::vector<double> accumulated_lengths = CalculateAccumulatedLengths(nodes_, line);
    std::vector<CenterPoint3D> resampled_points;
    for(int i = 0; i <= num_segments; ++i)
    {
        // 找到最近的两个点
        double target_length = (static_cast<double>(i) / num_segments) * line_length;
        auto index_pair = FindNearestIndexPair(accumulated_lengths, target_length);

        // 线性插值
        node::Point3D back_point = *nodes_->Find(line->nodeline[index_pair.first]);
        node::Point3D front_point = *nodes_->Find(line->nodeline[index_pair.second]);
        node::Point3D direction_vector = (front_point - back_point);

        double back_length = accumulated_lengths.at(index_pair.first);
        double front_length = accumulated_lengths.at(index_pair.second);
        double segment_length = front_length - back_length;
        node::Point3D target_point_ = back_point + 
                                    (direction_vector * (target_length - back_length) / segment_length);
        
        //保存插值结果
        CenterPoint3D target_point;
        target_point.x = target_point_.local_x;
        target_point.y = target_point_.local_y;
        target_point.ele = target_point_.elevation;
        target_point.ID = i;
        resampled_points.push_back(target_point);
    }
    return resampled_points;
    
}

std::vector<int> CenterWay::GenerateCenterline(node::Node *nodes_, way::Way *ways_, relation::relationship *relationship_, double resolution)
{
    // 寻找左右边界更长的一个
    double left_length = EdgeLength(nodes_, ways_->Find(relationship_->leftedge.ID));
    double right_length = EdgeLength(nodes_, ways_->Find(relationship_->rightedge.ID));
    double longer_distance = (left_length > right_length) ? left_length : right_length;
    int num_segments = std::max(static_cast<int>(ceil(longer_distance / resolution)), 1);

    // 重采样，将左右边界的点的数量统一
    std::vector<CenterPoint3D> left_points = ResamplePoints(nodes_, ways_->Find(relationship_->leftedge.ID), num_segments);
    std::vector<CenterPoint3D> right_points = ResamplePoints(nodes_, ways_->Find(relationship_->rightedge.ID), num_segments);
    
    // 左右边界点一一对应，计算中点
    std::vector<int> centerpointnumbers;
    for(int i = 0; i < num_segments + 1; i++)
    {
        CenterPoint3D *onecenterpoint = new CenterPoint3D;
        *onecenterpoint = (right_points.at(i) + left_points.at(i)) / 2;
        onecenterpoint->ID = relationship_->ID * 100 + i;//产生的新的中心点ID = 原来道路(relation)编号*100+点的顺序
        centerpointmap[onecenterpoint->ID] = onecenterpoint;
        centerpointnumbers.push_back(onecenterpoint->ID);
    }
    return centerpointnumbers;
}

void CenterWay::run(node::Node *nodes_, way::Way *ways_, relation::Relation *relations_)
{
    for(auto it = relations_->Begin(); it != relations_->End(); ++it)
    {
        if((it->second)->type == relation::RelationType::lanelet)
        {
            CenterWay3D *oneobject = new CenterWay3D;
            oneobject->ID = it->second->ID;//中心线ID沿用原始realtion的ID号
            oneobject->speed_limit = it->second->speed_limit;
            /*if((it->second)->turn_direction == relation::WayDirection::straight)
            {
                oneobject->isturn = false;
            }*/
            oneobject->direction = it->second->turn_direction;
            std::vector<int> onecenterline = GenerateCenterline(nodes_, ways_, it->second);
            
            //判断道路走向,确定source/target
            //取第一个道路中心线点
            CenterPoint3D c0 = *centerpointmap[*onecenterline.begin()];
            //取第二个道路中心线点
            CenterPoint3D c1 = *centerpointmap[*(onecenterline.begin() + 1)];
            //取道路左边界第一个点
            node::Point3D l0 = *nodes_->Find(ways_->Find(it->second->leftedge.ID)->nodeline[0]);
            //道路中心线第二个到第一个的连线向量----#1
            double c01x = c1.x - c0.x;
            double c01y = c1.y - c0.y;
            //对C01向量顺时针旋转90度，作为基准判定方向
            double xx = c01y;
            double xy = -c01x;
            //左边界第一点到道路中心线第一点的连线向量----#2
            double l0c0x = l0.local_x - c0.x;
            double l0c0y = l0.local_y - c0.y;
            //若向量#1与向量#2的点积 < 0, 说明道路方向与向量#1同向
            if(xx*l0c0x + xy*l0c0y < 0)
            {
                //std::cout << "correct order" << std::endl;
                oneobject->source = *onecenterline.begin();
                oneobject->target = *(onecenterline.end() - 1);
            }else{
                //否则，道路方向与向量#1反向
                //std::cout << "uncorrect order" << std::endl;
                oneobject->target = *onecenterline.begin();
                oneobject->source = *(onecenterline.end() - 1);
            }
            
            for(int i = 0; i < onecenterline.size(); ++i)
            {
                oneobject->centernodeline[oneobject->length++] = onecenterline[i];
                if(oneobject->length == oneobject->size)
                {
                    oneobject->Changesize();
                }
            } 
            Insert(it->second->ID, oneobject);
            Addnumbers();
            //std::cout << it->second->ID << ": " << Find(it->second->ID)->length << std::endl;
            //std::cout << "centerpointmap " << it->first << " size: " << centerpointmap.size() << std::endl;
        }
    }
}

CenterWay::~CenterWay()
{
    //std::cout << "~centerway" << std::endl;
    for(auto it = centerpointmap.begin(); it != centerpointmap.end(); ++it)
    {
        delete it->second;
    }
    centerpointmap.clear();

}


};//namespace centerway
};//namespace map