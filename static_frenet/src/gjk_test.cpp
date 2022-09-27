#include "collision.h"
#include "shape.h"
#include "vector2d.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

cv::Point2i cv_offset(
  float x, float y, int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(x * 100) + 300;
  output.y = image_height - int(y * 100) - image_height/3;
  return output;
  };

int main(){
  int num_vertices_1 = 3;
  int num_vertices_2 = 4;
  int num_vertices_3 = 5;
  float radius = 1.0;
  Vector2D *vector_1 =  new Vector2D[num_vertices_1];
  Vector2D *vector_2 =  new Vector2D[num_vertices_2];
  Vector2D *vector_3 =  new Vector2D[num_vertices_3];

  vector_1[0] = Vector2D(1.0, 0.0);
  vector_1[1] = Vector2D(1.0, 1.0);
  vector_1[2] = Vector2D(0.0, 1.0);

  vector_2[0] = Vector2D(0.0, 1.0);
  vector_2[1] = Vector2D(3.0, 1.0);
  vector_2[2] = Vector2D(3.0, 2.0);
  vector_2[3] = Vector2D(0.0, 2.0);

  vector_3[0] = Vector2D(1.0, 3.0);
  vector_3[1] = Vector2D(2.0, 3.0);
  vector_3[2] = Vector2D(3.0, 4.0);
  vector_3[3] = Vector2D(2.0, 5.0);
  vector_3[4] = Vector2D(1.0, 5.0);

  Polygon polygon1 = Polygon(num_vertices_1, vector_1);
  Polygon polygon2 = Polygon(num_vertices_2, vector_2);
  Polygon polygon3 = Polygon(num_vertices_3, vector_3);
  Circle circle1 = Circle( vector_1[1], radius);


  Simplex simplex;
  std::cout << "1与2 " << intersect(polygon1, polygon2) << std::endl;
  std::cout << "1与3 " << intersect(polygon1, polygon3) << std::endl;
  std::cout << "1与4 " << intersect(polygon2, circle1) << std::endl;
  std::cout << "2与3 " << intersect(polygon2, polygon3) << std::endl;
  std::cout << "2与4 " << intersect(polygon2, circle1) << std::endl;
  std::cout << "3与4 " << intersect(polygon3, circle1) << std::endl;
  // std::cout << gjk_intersect(polygon1, polygon2,  &simplex) << std::endl;
  // if(gjk_intersect(polygon1, polygon2,  &simplex)){
  //   std::cout << epa_penetration(polygon1, polygon2, simplex) << std::endl;
  //   };
   cv::Point points[3][20];
   cv::Point2i points_plt[3][20];
   points[0][0] = cv::Point( 1.0, 0.0 );
   points[0][1] = cv::Point( 1.0, 1.0 );
   points[0][2] = cv::Point( 0, 1 );

   points[1][0] = cv::Point( 0.0, 1.0 );
   points[1][1] = cv::Point( 3, 1 );
   points[1][2] = cv::Point( 3, 2 );
   points[1][3] = cv::Point( 0, 2 );

   points[2][0] = cv::Point( 1, 3 );
   points[2][1] = cv::Point( 2, 3 );
   points[2][2] = cv::Point( 3, 4 );
   points[2][3] = cv::Point( 2, 5 );
   points[2][4] = cv::Point( 1, 5 );



  cv::namedWindow("GJK", cv::WINDOW_NORMAL);
  cv::Mat backgroud(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));
  for (int i=0;i<3;i++)
  {
    for (int j=0;j<20;j++)
    points_plt[i][j] = cv_offset(points[i][j].x, points[i][j].y, backgroud.cols, backgroud.rows);  //图像输出点
  }
  const cv::Point2i* pt[3] = { points_plt[0], points_plt[1], points_plt[2] };
   int npt[3] = {3,4,5};
  cv::polylines(   //绘制多边形
    backgroud,
    pt,
    npt,
    3,
    1,
    cv::Scalar(0, 0, 0),
    2

  );
  /*for(unsigned int i=1; i<num_vertices_2; i++){
  cv::line(
        backgroud,
        cv_offset(vector_2[i-1].x, vector_2[i-1].y, backgroud.cols, backgroud.rows),
        cv_offset(vector_2[i].x, vector_2[i].y, backgroud.cols, backgroud.rows),
        cv::Scalar(0, 0, 0),
        2);
        }
  for(unsigned int i=1; i<num_vertices_1; i++){
  cv::line(
        backgroud,
        cv_offset(vector_1[i-1].x, vector_1[i-1].y, backgroud.cols, backgroud.rows),
        cv_offset(vector_1[i].x, vector_1[i].y, backgroud.cols, backgroud.rows),
        cv::Scalar(0, 0, 0),
        2);
        }
  for(unsigned int i=1; i<num_vertices_3; i++){
  cv::line(
        backgroud,
        cv_offset(vector_3[i-1].x, vector_3[i-1].y, backgroud.cols, backgroud.rows),
        cv_offset(vector_3[i].x, vector_3[i].y, backgroud.cols, backgroud.rows),
        cv::Scalar(0, 0, 0),
        2);
      } */
  cv::circle(
        backgroud,
        cv_offset(vector_1[1].x, vector_1[1].y, backgroud.cols, backgroud.rows),
        100*radius,
        cv::Scalar(0, 0, 0),
        2);

  cv::imshow("GJK", backgroud);
  cv::waitKey(100000000);
}


/*bool check_collision(FrenetPath path, const obstcle_lists ob, float heading_angle){
    for(unsigned int i = 0; i < path.x.size(); i++){
        int vehicle_vertices = 4;
        float vl = 2.0;
        float vw = 1.0;

        Vector2D *vector_vehicle = new Vector2D[vehicle_vertices];
        vector_vehicle[0] = Vector2D(path.x[i] - vl * cos(heading_angle) + vw * sin(heading_angle), path.y[i] - vl * sin(heading_angle) - vw * cos(heading_angle));
        vector_vehicle[1] = Vector2D(path.x[i] + vl * cos(heading_angle) + vw * sin(heading_angle), path.y[i] + vl * sin(heading_angle) - vw * cos(heading_angle));
        vector_vehicle[2] = Vector2D(path.x[i] + vl * cos(heading_angle) - vw * sin(heading_angle), path.y[i] + vl * sin(heading_angle) + vw * cos(heading_angle));
        vector_vehicle[3] = Vector2D(path.x[i] - vl * cos(heading_angle) - vw * sin(heading_angle), path.y[i] - vl * sin(heading_angle) + vw * cos(heading_angle));
        Polygon vehicle_polygon = Polygon(vehicle_vertices, vector_vehicle);

        // num_vertices, vector�Ӹ�֪���ݻ�ȡ���Գ������α����ϰ��������μ�����ײ
        for(unsigned int i = 0; i < ob.size(); i++){
        Vector2D *vector_1 = new Vector2D[num_vertices_1];
        Vector2D *vector_2 = new Vector2D[num_vertices_2];
        Polygon polygon1 = Polygon(num_vertices_1, vector_1);
        Polygon polygon2 = Polygon(num_vertices_2, vector_2);

        if(intersect(vehicle_polygon, polygon1)){
            return true;
        }
        }
    }
    return false;
}; */
