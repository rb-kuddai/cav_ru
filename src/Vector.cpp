#include "Vector.h"

#include <stdio.h>
#include <cmath>

Vector2::Vector2()
    : x(0.0f)
    , y(0.0f)
{};

Vector2::Vector2(float x, float y)
    : x(x)
    , y(y)
{};

Vector2 Vector2::One() {
    return Vector2(1.0, 1.0);
}

Vector2 Vector2::Zero() {
    return Vector2(0.0, 0.0);
}

float Vector2::Length(Vector2 v) {
    return sqrt(v.x * v.x + v.y * v.y);
}

float Vector2::Distance(Vector2 v1, Vector2 v2) {
    return Length(v1 - v2);
}

Vector2 Vector2::Normalize(Vector2 v) {
    float l = Length(v);
    return Vector2( v.x / l , v.y / l );
}

float Vector2::Dot(Vector2 v1, Vector2 v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

Vector2 Vector2::Max(Vector2 v, float val) {
    return Vector2(
               v.x > val ? val : v.x,
               v.y > val ? val : v.y
           );
}

Vector2 Vector2::Min(Vector2 v, float val) {
    return Vector2(
               v.x < val ? v.x : val,
               v.y < val ? v.y : val
           );
}

Vector2 Vector2::Clamp(Vector2 v, float bottom, float top) {
    Vector2 ret;
    ret = Vector2::Max(v, bottom);
    ret = Vector2::Min(v, top);
    return ret;
}

void Vector2::Print(Vector2 v) {
    printf("Vector2(%0.2f, %0.2f)", v.x, v.y);
}

float Vector2::operator[](int i) {
    if(i == 0) {
        return x;
    } else if (i == 1) {
        return y;
    } else {
        printf("Invalid Vector2 Subscript: %i\n", i);
        return 0;
    }
}

bool Vector2::operator==(Vector2 v) {
    return (x == v.x) && (y == v.y);
}

bool Vector2::operator!=(Vector2 v) {
    return (x != v.x) || (y != v.y);
}

Vector2 Vector2::operator*(float factor) {
    return Vector2( x * factor, y * factor );
}

Vector2 Vector2::operator*=(float factor) {
    x *= factor;
    y *= factor;
    return *this;
}

Vector2 Vector2::operator*(Vector2 v) {
    return Vector2( x * v.x, y * v.y );
}

Vector2 Vector2::operator*=(Vector2 v) {
    x *= v.x;
    y *= v.y;
    return *this;
}

Vector2 Vector2::operator/(float factor) {
    return Vector2( x / factor, y / factor );
}

Vector2 Vector2::operator/(Vector2 v) {
    return Vector2( x / v.x, y / v.y );
}

Vector2 Vector2::operator+(float factor) {
    return Vector2( x + factor, y + factor);
}

Vector2 Vector2::operator+(Vector2 v) {
    return Vector2( x + v.x, y + v.y);
}

Vector2 Vector2::operator+=(Vector2 v) {
    x += v.x;
    y += v.y;
    return *this;
}

Vector2 Vector2::operator-(Vector2 v) {
    return Vector2( x - v.x, y - v.y);
}

Vector2 Vector2::operator-=(Vector2 v) {
    x -= v.x;
    y -= v.y;
    return *this;
}

Vector2 Vector2::operator-=(float factor) {
    x -= factor;
    y -= factor;
    return *this;
}

Vector2 Vector2::operator-() {
    return Vector2(-x, -y);
}

Vector3::Vector3()
    : x(0.0f)
    , y(0.0f)
    , z(0.0f)
{};

Vector3::Vector3(float x, float y, float z)
    : x(x)
    , y(y)
    , z(z)
{};

Vector3 Vector3::One() {
    return Vector3(1.0, 1.0, 1.0);
}

Vector3 Vector3::Zero() {
    return Vector3(0.0, 0.0, 0.0);
}

float Vector3::r() {
    return x;
}

float Vector3::g() {
    return y;
}

float Vector3::b() {
    return z;
}

float Vector3::Length(Vector3 v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

float Vector3::Distance(Vector3 v1, Vector3 v2) {
    return Length(v1 - v2);
}

Vector3 Vector3::Normalize(Vector3 v) {
    float l = Length(v);
    return Vector3(v.x, v.y, v.z) / l;
}

float Vector3::Dot(Vector3 v1, Vector3 v2) {
    return (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
}

Vector3 Vector3::Cross(Vector3 v1, Vector3 v2) {
    return Vector3(
               v1.y * v2.z - v1.z * v2.y,
               v1.z * v2.x - v1.x * v2.z,
               v1.x * v2.y - v1.y * v2.x
           );
}

Vector3 Vector3::Max(Vector3 v, float val) {
    return Vector3(
               v.x > val ? val : v.x,
               v.y > val ? val : v.y,
               v.z > val ? val : v.z
           );
}

Vector3 Vector3::Min(Vector3 v, float val) {
    return Vector3(
               v.x < val ? v.x : val,
               v.y < val ? v.y : val,
               v.z < val ? v.z : val
           );
}

Vector3 Vector3::Clamp(Vector3 v, float bottom, float top) {
    Vector3 ret;
    ret = Vector3::Max(v, bottom);
    ret = Vector3::Min(v, top);
    return ret;
}

void Vector3::Print(Vector3 v) {
    printf("Vector3(%0.2f, %0.2f, %0.2f)", v.x, v.y, v.z);
}

float Vector3::operator[](int i) {
    if(i == 0) {
        return x;
    } else if (i == 1) {
        return y;
    } else if (i == 2) {
        return z;
    } else {
        printf("Invalid Vector3 Subscript: %i\n", i);
        return 0;
    }
}

bool Vector3::operator==(Vector3 v) {
    return (x == v.x) && (y == v.y) && (z == v.z);
}

bool Vector3::operator!=(Vector3 v) {
    return (x != v.x) || (y != v.y) || (z != v.z);
}

Vector3 Vector3::operator*(float factor) {
    return Vector3( x * factor, y * factor, z * factor );
}

Vector3 Vector3::operator*=(float factor) {
    x *= factor;
    y *= factor;
    z *= factor;
    return *this;
}

Vector3 Vector3::operator*(Vector3 v) {
    return Vector3( x * v.x, y * v.y, z * v.z );
}

Vector3 Vector3::operator*=(Vector3 v) {
    x *= v.x;
    y *= v.y;
    z *= v.z;
    return *this;
}

Vector3 Vector3::operator/(float factor) {
    return Vector3( x / factor, y / factor, z / factor );
}

Vector3 Vector3::operator/(Vector3 v) {
    return Vector3( x / v.x, y / v.y, z / v.z );
}

Vector3 Vector3::operator+(Vector3 v) {
    return Vector3( x + v.x, y + v.y, z + v.z);
}

Vector3 Vector3::operator+(float factor) {
    return Vector3( x + factor, y + factor, z + factor);
}

Vector3 Vector3::operator+=(Vector3 v) {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
}

Vector3 Vector3::operator-(Vector3 v) {
    return Vector3( x - v.x, y - v.y, z - v.z);
}

Vector3 Vector3::operator-=(Vector3 v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
}

Vector3 Vector3::operator-=(float factor) {
    x -= factor;
    y -= factor;
    z -= factor;
    return *this;
}

Vector3 Vector3::operator-() {
    return Vector3(-x, -y, -z);
}

Vector2 Vector3::xy() {
    return Vector2(x, y);
}

Vector4::Vector4()
    : x(0.0f)
    , y(0.0f)
    , z(0.0f)
    , w(0.0f)
{}

Vector4::Vector4(float x, float y, float z, float w)
    : x(x)
    , y(y)
    , z(z)
    , w(w)
{}

Vector4::Vector4(Vector2 v, float z, float w)
    : x(v.x)
    , y(v.y)
    , z(z)
    , w(w)
{}

Vector4::Vector4(Vector3 v, float w)
    : x(v.x)
    , y(v.y)
    , z(v.z)
    , w(w)
{}

Vector4 Vector4::One() {
    return Vector4(1.0, 1.0, 1.0, 1.0);
}

Vector4 Vector4::Zero() {
    return Vector4(0.0, 0.0, 0.0, 0.0);
}

float Vector4::Length(Vector4 v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
}

Vector4 Vector4::Normalize(Vector4 v) {
    return v / Vector4::Length(v);
}

float Vector4::Dot(Vector4 v1, Vector4 v2) {
    return (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w);
}

void Vector4::Print(Vector4 v) {
    printf("Vector4(%0.2f, %0.2f, %0.2f, %0.2f)", v.x, v.y, v.z, v.w);
}

float Vector4::r() {
    return x;
}

float Vector4::g() {
    return y;
}

float Vector4::b() {
    return z;
}

float Vector4::a() {
    return w;
}

Vector3 Vector4::xyz() {
    return Vector3(x, y, z);
}

Vector3 Vector4::rgb() {
    return Vector3(x, y, z);
}

Vector4 Vector4::Max(Vector4 v, float val) {
    return Vector4(
               v.x > val ? val : v.x,
               v.y > val ? val : v.y,
               v.z > val ? val : v.z,
               v.w > val ? val : v.w
           );
}

Vector4 Vector4::Min(Vector4 v, float val) {
    return Vector4(
               v.x < val ? v.x : val,
               v.y < val ? v.y : val,
               v.z < val ? v.z : val,
               v.w < val ? v.w : val
           );
}

Vector4 Vector4::Clamp(Vector4 v, float bottom, float top) {
    Vector4 ret;
    ret = Vector4::Max(v, bottom);
    ret = Vector4::Min(v, top);
    return ret;
}

Vector3 Vector4::FromHomogeneous(Vector4 v) {
    return Vector3(v.x, v.y, v.z) / v.w;
}

Vector4 Vector4::ToHomogeneous(Vector3 v) {
    return Vector4(v.x, v.y, v.z, 1.0);
}

float Vector4::operator[](int i) {
    if(i == 0) {
        return x;
    } else if (i == 1) {
        return y;
    } else if (i == 2) {
        return z;
    } else if (i == 3) {
        return w;
    } else {
        printf("Invalid Vector4 Subscript: %i\n", i);
        return 0;
    }
}

bool Vector4::operator==(Vector4 v) {
    return (x == v.x) && (y == v.y) && (z == v.z) && (w == v.w);
}

bool Vector4::operator!=(Vector4 v) {
    return (x != v.x) || (y != v.y) || (z != v.z) || (w != v.w);
}

Vector4 Vector4::operator*(float factor) {
    return Vector4( x * factor, y * factor, z * factor, w * factor );
}

Vector4 Vector4::operator*=(float factor) {
    x *= factor;
    y *= factor;
    z *= factor;
    w *= factor;
    return *this;
}

Vector4 Vector4::operator*(Vector4 v) {
    return Vector4( x * v.x, y * v.y, z * v.z, w * v.w );
}

Vector4 Vector4::operator*=(Vector4 v) {
    x *= v.x;
    y *= v.y;
    z *= v.z;
    w *= v.w;
    return *this;
}

Vector4 Vector4::operator/(float factor) {
    return Vector4( x / factor, y / factor, z / factor, w / factor );
}

Vector4 Vector4::operator/(Vector4 v) {
    return Vector4( x / v.x, y / v.y, z / v.z, w / v.w );
}

Vector4 Vector4::operator+(float factor) {
    return Vector4( x + factor, y + factor, z + factor, w + factor);
}

Vector4 Vector4::operator+(Vector4 v) {
    return Vector4( x + v.x, y + v.y, z + v.z, w + v.w);
}

Vector4 Vector4::operator+=(Vector4 v) {
    x += v.x;
    y += v.y;
    z += v.z;
    w += v.w;
    return *this;
}

Vector4 Vector4::operator-(Vector4 v) {
    return Vector4( x - v.x, y - v.y, z - v.z, w - v.w);
}

Vector4 Vector4::operator-=(Vector4 v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    w -= v.w;
    return *this;
}

Vector4 Vector4::operator-=(float factor) {
    x -= factor;
    y -= factor;
    z -= factor;
    w -= factor;
    return *this;
}

Vector4 Vector4::operator-() {
    return Vector4(-x, -y, -z, -w);
}
