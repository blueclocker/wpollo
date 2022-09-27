///这不是标准库？？


#ifndef TF_VECTOR_H  //标识
#define TF_VECTOR_H

#include <iostream>
#include <cmath>

class Vector2D { //定义Vector2D类
public:
	float x;
	float y;  //公共成员

	
public:
	Vector2D();  //公共成员函数
	Vector2D(float x, float y);
	Vector2D(const Vector2D &v);  //引用型参数应该在能被定义为const的情况下，尽量定义为const
	Vector2D &operator=(const Vector2D &v);  

	Vector2D operator+(const Vector2D &v) const;
	Vector2D operator-(const Vector2D &v) const;
	Vector2D operator*(const Vector2D &v) const;
	Vector2D operator/(const Vector2D &v) const;
	void operator+=(const Vector2D &v);
	void operator-=(const Vector2D &v);
	void operator*=(const Vector2D &v);
	void operator/=(const Vector2D &v);

	Vector2D operator+(const float f) const;
	Vector2D operator-(const float f) const;
	Vector2D operator*(const float f) const;
	Vector2D operator/(const float f) const;
	void operator+=(const float f);
	void operator-=(const float f);
	void operator*=(const float f);
	void operator/=(const float f);

	Vector2D operator-() const;

	float length() const;
	float length_squared() const;
	void normalize();
	Vector2D normalized() const;
	Vector2D round(int decimal_places) const;  //小数位

	bool operator==(const Vector2D &v) const;
	bool operator!=(const Vector2D &v) const;
	bool operator<(const Vector2D &v) const;
	bool operator>(const Vector2D &v) const;
	bool is_null() const;

	float dot(const Vector2D &v) const;
	Vector2D projection_on(const Vector2D &v) const;

	Vector2D perpendicular_clockwise() const;  //垂直顺时针方向
	Vector2D perpendicular_counterclockwise() const;  //垂直逆时针方向

	friend std::ostream &operator<<(std::ostream &os, const Vector2D &v);    //友元函数重新定义输出流
};

Vector2D operator+(const float f, const Vector2D &v);
Vector2D operator*(const float f, const Vector2D &v);

// returns a perpendicular vector to ab that points towards p (or in the same direction as "ap")
Vector2D perpendicular_towards(const Vector2D &a, const Vector2D &b, const Vector2D &p);
// returns a perpendicular vector to ab that points away from p (or in the opposite direction as "ap")
Vector2D perpendicular_away(const Vector2D &a, const Vector2D &b, const Vector2D &p);

#endif