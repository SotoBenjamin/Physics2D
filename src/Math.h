#pragma once
#include <cmath>
#include <algorithm>
namespace Math {
    struct Vec2 {
        float x,y;
        Vec2(): x(0) , y(0){}
        Vec2(float _x , float _y) : x(_x) , y(_y){}
        Vec2 operator + (const Vec2& other) const {
            return {x + other.x , y + other.y};
        }
        Vec2 operator - (const Vec2& other) const {
            return {x - other.x  , y - other.y};
        }

        Vec2 operator*(const Vec2& o) const {
            return { x * o.x, y * o.y };
        }
        Vec2& operator*=(const Vec2& o) {
            x *= o.x;  y *= o.y;
            return *this;
        }

        Vec2 operator*(float s) const {
            return {x*s , y*s};
        }
        Vec2 operator/(float s) const {
            return {x/s,y/s};
        }
        Vec2& operator+=(const Vec2& other){
            x += other.x;
            y += other.y;
            return *this;
        }
        Vec2& operator -=(const Vec2& other) {
            x -= other.x;
            y -= other.y;
            return *this;
        }
        Vec2& operator *= (float s) {
            x*=s;
            y*=s;
            return *this;
        }
        Vec2& operator /= (float s) {
            return *this *= (1.0f/s);
        }
        Vec2 operator-() const { return {-x, -y}; }

    };
    struct Vec3{ float x,y,z;};
    struct Vec4 { float x,y,z,w; };
    struct Mat2 {
        float m00,m01,m10,m11;
        static Mat2 rotation(float angle) {
            float c = std::cos(angle);
            float s = std::sin(angle);
            return {c, -s, s, c};
        }
        Vec2 operator*(const Vec2& v) const {
            return {m00 * v.x + m01 * v.y,
            m10 * v.x + m11 * v.y};
        }
    };

    inline float dot (const Vec2& a , const  Vec2& b) { return a.x * b.x + a.y * b.y;}
    inline float cross(const Vec2& a , const Vec2& b){ return a.x * b.y - a.y * b.x;}
    inline Vec2 cross(const Vec2& v, float s){ return { s * v.y, -s * v.x };}
    inline Vec2 cross(float s, const Vec2& v){return {-s * v.y,  s * v.x };}
    inline float length(const Vec2& v){ return std::sqrt(dot(v, v));}
    inline Vec2 operator*(float s, const Vec2& v) { return v * s; }

    inline Vec2 normalize(const Vec2& v) {
        float len = length(v);
        return len > 1e-8f ? Vec2(v.x / len, v.y / len) : Vec2(1,0);
    }

    inline float clamp(float v, float lo, float hi){return v < lo ? lo : (v > hi ? hi : v);}

    inline Vec2  clamp(const Vec2& v, const Vec2& lo, const Vec2& hi) {
        return {clamp(v.x, lo.x, hi.x), clamp(v.y, lo.y, hi.y)};
    }




    inline Vec2 abs(const Vec2 &v)   { return {std::fabs(v.x), std::fabs(v.y)}; }
    inline Vec2 min(const Vec2 &a, const Vec2 &b) { return { std::min(a.x,b.x), std::min(a.y,b.y) }; }
    inline Vec2 max(const Vec2 &a, const Vec2 &b) { return { std::max(a.x,b.x), std::max(a.y,b.y) }; }

    inline float distance(const Vec2 &a, const Vec2 &b) { return length(a - b); }

    inline float radians(float deg) { return deg * (3.14159265358979323846f/180.0f); }
    inline float degrees(float rad) { return rad * (180.0f/3.14159265358979323846f); }

    inline Vec2 mix(const Vec2 &a, const Vec2 &b, float t) { return a*(1 - t) + b*t; }


    template<typename T>
    inline T min(T a, T b) { return a < b ? a : b; }

    template<typename T>
    inline T max(T a, T b) { return a > b ? a : b; }

};

