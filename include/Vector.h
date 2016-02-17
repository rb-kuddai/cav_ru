#ifndef VECTOR_H
#define VECTOR_H

#pragma once

class Vector2 {
	
    public:
        float x, y;
        
        Vector2();
        Vector2(float x, float y);
        
        static Vector2 Zero();
        static Vector2 One();
        
        static float Length(Vector2 v);
        static float Distance(Vector2 v1, Vector2 v2);
        static Vector2 Normalize(Vector2 v);
        
        static float Dot(Vector2 v1, Vector2 v2);
        static Vector2 Max(Vector2 v, float val);
        static Vector2 Min(Vector2 v, float val);
        static Vector2 Clamp(Vector2 v, float bottom, float top);
        
        static void Print(Vector2 v);
        
        float operator[](int i);
        
        bool operator==(Vector2 v);
        bool operator!=(Vector2 v);
        
        Vector2 operator*(float factor);
        Vector2 operator*=(float factor);
        Vector2 operator*(Vector2 v);
        Vector2 operator*=(Vector2 v);
        Vector2 operator/(float factor);
        Vector2 operator/(Vector2 v);
        Vector2 operator+(float factor);
        Vector2 operator+(Vector2 v);
        Vector2 operator+=(Vector2 v);
        Vector2 operator-(Vector2 v);
        Vector2 operator-=(Vector2 v);
        Vector2 operator-=(float factor);
        Vector2 operator-();
};

class Vector3 {
    public:
        float x, y, z;
        
        Vector3();
        Vector3(float x, float y, float z);
        
        static Vector3 Zero();
        static Vector3 One();
        
        static float Length(Vector3 v);
        static float Distance(Vector3 v1, Vector3 v2);
        static Vector3 Normalize(Vector3 v);
        
        static float Dot(Vector3 v1, Vector3 v2);
        static Vector3 Cross(Vector3 v1, Vector3 v2);
        
        static Vector3 Max(Vector3 v, float val);
        static Vector3 Min(Vector3 v, float val);
        static Vector3 Clamp(Vector3 v, float bottom, float top);
          
        static void Print(Vector3 v);
        
        float r();
        float g();
        float b();
        
        Vector2 xy();
        
        float operator[](int i);
        
        bool operator==(Vector3 v);
        bool operator!=(Vector3 v);
        
        Vector3 operator*(float factor);
        Vector3 operator*=(float factor);
        Vector3 operator*(Vector3 v);
        Vector3 operator*=(Vector3 v);
        Vector3 operator/(float factor);
        Vector3 operator/(Vector3 v);
        Vector3 operator+(Vector3 v);
        Vector3 operator+(float factor);
        Vector3 operator+=(Vector3 v);
        Vector3 operator-(Vector3 v);
        Vector3 operator-=(Vector3 v);
        Vector3 operator-=(float factor);
        Vector3 operator-();
};

class Vector4 {
    public:
        float x, y, z, w;
        
        Vector4();
        Vector4(float x, float y, float z, float w);
        Vector4(Vector2 v, float z, float w);
        Vector4(Vector3 v, float w);
        
        static Vector4 Zero();
        static Vector4 One();
        
        static float Length(Vector4 v);
        static Vector4 Normalize(Vector4 v);
        
        static float Dot(Vector4 v1, Vector4 v2);
        
        static Vector4 Max(Vector4 v, float val);
        static Vector4 Min(Vector4 v, float val);
        static Vector4 Clamp(Vector4 v, float bottom, float top);
        
        static void Print(Vector4 v);
        
        static Vector3 FromHomogeneous(Vector4 v);
        static Vector4 ToHomogeneous(Vector3 v);
        
        float r();
        float g();
        float b();
        float a();
        
        Vector3 xyz();
        Vector3 rgb();
        
        float operator[](int i);
        
        bool operator==(Vector4 v);
        bool operator!=(Vector4 v);
        
        Vector4 operator*(float factor);
        Vector4 operator*=(float factor);
        Vector4 operator*(Vector4 v);
        Vector4 operator*=(Vector4 v);
        Vector4 operator/(float factor);
        Vector4 operator/(Vector4 v);
        Vector4 operator+(float factor);
        Vector4 operator+(Vector4 v);
        Vector4 operator+=(Vector4 v);
        Vector4 operator-(Vector4 v);
        Vector4 operator-=(Vector4 v);
        Vector4 operator-=(float factor);
        Vector4 operator-();
};

#endif
