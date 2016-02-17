#ifndef MATRIX_H
#define MATRIX_H

#include "Vector.h"

class Matrix_2x2 {
    public:
        float xx, xy;
        float yx, yy;
        
        Matrix_2x2();
        Matrix_2x2(float xx, float xy,
               float yx, float yy);
        Matrix_2x2(Vector2 r1, Vector2 r2);
               
        static Matrix_2x2 Id();
        static Matrix_2x2 Zero();
        static Matrix_2x2 Rotation(float a);
        static float Determinant(Matrix_2x2 m);
        
        static Matrix_2x2 Inverse(Matrix_2x2 m);
        
        static void Print(Matrix_2x2 m);
        
        Matrix_2x2 operator*(Matrix_2x2 m);
        Vector2 operator*(Vector2 v);
};

class Matrix_3x3 {

    public:
        float xx, xy, xz;
        float yx, yy, yz;
        float zx, zy, zz;
        
        Matrix_3x3();
        Matrix_3x3(float xx, float xy, float xz,
               float yx, float yy, float yz,
               float zx, float zy, float zz);
        Matrix_3x3(Vector3 r1, Vector3 r2, Vector3 r3);
               
        static Matrix_3x3 Id();
        static Matrix_3x3 Zero();
        
        static Matrix_3x3 RotationX(float a);
        static Matrix_3x3 RotationY(float a);
        static Matrix_3x3 RotationZ(float a);
        static Matrix_3x3 RotationAngleAxis(Vector3 axis, float angle);
        
        static float Determinant(Matrix_3x3 m);
        static Matrix_3x3 Inverse(Matrix_3x3 m);
        
        static void Print(Matrix_3x3 m);
        
        Matrix_3x3 operator*(Matrix_3x3 m);
        Vector3 operator*(Vector3 v);
};

class Matrix_4x4 {

    public:
        float xx, xy, xz, xw;
        float yx, yy, yz, yw;
        float zx, zy, zz, zw;
        float wx, wy, wz, ww;
        
        Matrix_4x4();
        Matrix_4x4(float xx, float xy, float xz, float xw,
               float yx, float yy, float yz, float yw,
               float zx, float zy, float zz, float zw,
               float wx, float wy, float wz, float ww);
        Matrix_4x4(Vector4 r1, Vector4 r2, Vector4 r3, Vector4 r4);
               
        static Matrix_4x4 Id();
        static Matrix_4x4 Zero();
        
        static Matrix_4x4 RotationX(float a);
        static Matrix_4x4 RotationY(float a);
        static Matrix_4x4 RotationZ(float a);
        static Matrix_4x4 RotationEuler(float a, float b, float c);
        static Matrix_4x4 RotationAngleAxis(Vector3 axis, float angle);
        
        static Matrix_4x4 Translation(Vector3 trans);
        static Matrix_4x4 Scale(Vector3 scale);
        
        static Matrix_4x4 ViewLookAt(Vector3 position, Vector3 target, Vector3 up);
        static Matrix_4x4 Perspective(float fov, float near, float far, float ratio);
        static Matrix_4x4 Orthographic(float left, float right, float bottom, float top, float near, float far);
          
        static Matrix_4x4 FromMatrix_3x3(Matrix_3x3 m);
        static Matrix_3x3 ToMatrix_3x3(Matrix_4x4 m);
        
        static Matrix_4x4 Transpose(Matrix_4x4 m);
        static float Determinant(Matrix_4x4 m);
        static Matrix_4x4 Inverse(Matrix_4x4 m);
        
        static void Print(Matrix_4x4 m);
        
        Matrix_4x4 operator*(Matrix_4x4 m);
        Matrix_4x4 operator*(Matrix_3x3 m);
        Vector4 operator*(Vector4 v);
        Vector3 operator*(Vector3 v);
        
        Matrix_4x4 operator*(float fac);
        Matrix_4x4 operator+(Matrix_4x4 m);
};

#endif