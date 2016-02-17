#include "Matrix.h"

#include <stdio.h>
#include <stdlib.h>
#include <cmath>

Matrix_2x2::Matrix_2x2() {}

Matrix_2x2::Matrix_2x2(float xx, float xy, float yx, float yy)
    : xx(xx), xy(xy), yx(yx), yy(yy) {}

Matrix_2x2::Matrix_2x2(Vector2 r1, Vector2 r2)
    : xx(r1.x), xy(r1.y), yx(r2.x), yy(r2.y) {}

Matrix_2x2 Matrix_2x2::Id() {
    return Matrix_2x2(1,0,0,1);
}

Matrix_2x2 Matrix_2x2::Zero() {
    return Matrix_2x2(0,0,0,0);
}

Matrix_2x2 Matrix_2x2::Rotation(float a) {
    return Matrix_2x2( cos(a), -sin(a), sin(a), cos(a) );
}

float Matrix_2x2::Determinant(Matrix_2x2 m) {
    return m.xx * m.yy - m.xy * m.yx;
}

void Matrix_2x2::Print(Matrix_2x2 m) {
    printf("| %0.2f, %0.2f |\n", m.xx, m.xy);
    printf("| %0.2f, %0.2f |\n", m.yx, m.yy);
}

Matrix_2x2 Matrix_2x2::operator*(Matrix_2x2 m) {
    return Matrix_2x2(
               xx * m.xx + xy * m.yx, xx * m.xy + xy * m.yy,
               yx * m.xx + yy * m.yx, yx * m.xy + yy * m.yy
           );
}

Vector2 Matrix_2x2::operator*(Vector2 v) {
    return Vector2( v.x * xx + v.y * xy , v.x * yx + v.y * yy);
}

Matrix_2x2 Matrix_2x2::Inverse(Matrix_2x2 m) {

    float det = Matrix_2x2::Determinant(m);
    if (det == 0) {
        printf("[ERROR]: Cannot Invert non-singular 2x2 matrix.\n");
        exit(EXIT_FAILURE);
    }
    float fac = 1.0 / det;

    return Matrix_2x2(
               fac * m.yy, fac * -m.xy,
               fac * -m.yx, fac * m.xx
           );

}

Matrix_3x3::Matrix_3x3() {}
Matrix_3x3::Matrix_3x3(float xx, float xy, float xz,
                       float yx, float yy, float yz,
                       float zx, float zy, float zz)
    : xx(xx), xy(xy), xz(xz)
    , yx(yx), yy(yy), yz(yz)
    , zx(zx), zy(zy), zz(zz)
{}

Matrix_3x3::Matrix_3x3(Vector3 r1, Vector3 r2, Vector3 r3)
    : xx(r1.x), xy(r1.y), xz(r1.z)
    , yx(r2.x), yy(r2.y), yz(r2.z)
    , zx(r3.x), zy(r3.y), zz(r3.z)
{}

Matrix_3x3 Matrix_3x3::Id() {
    return Matrix_3x3(
               1.0, 0.0, 0.0,
               0.0, 1.0, 0.0,
               0.0, 0.0, 1.0
           );
}

Matrix_3x3 Matrix_3x3::Zero() {
    return Matrix_3x3(
               0.0, 0.0, 0.0,
               0.0, 0.0, 0.0,
               0.0, 0.0, 0.0
           );
}

Matrix_3x3 Matrix_3x3::RotationX(float a) {
    return Matrix_3x3(
               1.0, 0.0, 0.0,
               0.0, cos(a), -sin(a),
               0.0, sin(a), cos(a)
           );
}

Matrix_3x3 Matrix_3x3::RotationY(float a) {
    return Matrix_3x3(
               cos(a), 0.0, sin(a),
               0.0, 1.0, 0.0,
               -sin(a), 0.0, cos(a)
           );
}

Matrix_3x3 Matrix_3x3::RotationZ(float a) {
    return Matrix_3x3(
               cos(a), -sin(a), 0.0,
               sin(a), cos(a), 0.0,
               0.0, 0.0, 1.0
           );
}

Matrix_3x3 Matrix_3x3::RotationAngleAxis(Vector3 v, float angle) {

    float c = cos(angle);
    float s = sin(angle);
    float nc = 1 - c;

    return Matrix_3x3(
               v.x * v.x * nc + c       , v.x * v.y * nc - v.z * s , v.x * v.z * nc + v.y * s,
               v.y * v.x * nc + v.z * s , v.y * v.y * nc + c       , v.y * v.z * nc - v.x * s,
               v.z * v.x * nc - v.y * s , v.z * v.y * nc + v.x * s , v.z * v.z * nc + c
           );

}

void Matrix_3x3::Print(Matrix_3x3 m) {
    printf("| %0.2f, %0.2f, %0.2f |\n", m.xx, m.xy, m.xz);
    printf("| %0.2f, %0.2f, %0.2f |\n", m.yx, m.yy, m.yz);
    printf("| %0.2f, %0.2f, %0.2f |\n", m.zx, m.zy, m.zz);
}

Matrix_3x3 Matrix_3x3::operator*(Matrix_3x3 m) {

    return Matrix_3x3(
               (xx * m.xx) + (xy * m.yx) + (xz * m.zx),
               (xx * m.xy) + (xy * m.yy) + (xz * m.zy),
               (xx * m.xz) + (xy * m.yz) + (xz * m.zz),

               (yx * m.xx) + (yy * m.yx) + (yz * m.zx),
               (yx * m.xy) + (yy * m.yy) + (yz * m.zy),
               (yx * m.xz) + (yy * m.yz) + (yz * m.zz),

               (zx * m.xx) + (zy * m.yx) + (zz * m.zx),
               (zx * m.xy) + (zy * m.yy) + (zz * m.zy),
               (zx * m.xz) + (zy * m.yz) + (zz * m.zz)
           );

}

Vector3 Matrix_3x3::operator*(Vector3 v) {

    return Vector3(
               (xx * v.x) + (xy * v.y) + (xz * v.z),
               (yx * v.x) + (yy * v.y) + (yz * v.z),
               (zx * v.x) + (zy * v.y) + (zz * v.z)
           );

}

float Matrix_3x3::Determinant(Matrix_3x3 m) {
    return (m.xx * m.yy * m.zz) + (m.xy * m.yz * m.zx) + (m.xz * m.yx * m.zy) -
           (m.xz * m.yy * m.zx) - (m.xy * m.yx * m.zz) - (m.xx * m.yz * m.zy);
}

Matrix_3x3 Matrix_3x3::Inverse(Matrix_3x3 m) {

    float det = Matrix_3x3::Determinant(m);
    if (det == 0) {
        printf("[ERROR]: Cannot Invert non-singular 3x3 matrix.\n");
        exit(EXIT_FAILURE);
    }

    float fac = 1.0 / det;

    return Matrix_3x3(
               fac * Matrix_2x2::Determinant(Matrix_2x2(m.yy, m.yz, m.zy, m.zz)),
               fac * Matrix_2x2::Determinant(Matrix_2x2(m.xz, m.xy, m.zz, m.zy)),
               fac * Matrix_2x2::Determinant(Matrix_2x2(m.xy, m.xz, m.yy, m.yz)),

               fac * Matrix_2x2::Determinant(Matrix_2x2(m.yz, m.yx, m.zz, m.zx)),
               fac * Matrix_2x2::Determinant(Matrix_2x2(m.xx, m.xz, m.zx, m.zz)),
               fac * Matrix_2x2::Determinant(Matrix_2x2(m.xz, m.xx, m.yz, m.yx)),

               fac * Matrix_2x2::Determinant(Matrix_2x2(m.yx, m.yy, m.zx, m.zy)),
               fac * Matrix_2x2::Determinant(Matrix_2x2(m.xy, m.xx, m.zy, m.zx)),
               fac * Matrix_2x2::Determinant(Matrix_2x2(m.xx, m.xy, m.yx, m.yy))
           );

}

Matrix_4x4::Matrix_4x4() {}
Matrix_4x4::Matrix_4x4(float xx, float xy, float xz, float xw,
                       float yx, float yy, float yz, float yw,
                       float zx, float zy, float zz, float zw,
                       float wx, float wy, float wz, float ww)
    : xx(xx), xy(xy), xz(xz), xw(xw)
    , yx(yx), yy(yy), yz(yz), yw(yw)
    , zx(zx), zy(zy), zz(zz), zw(zw)
    , wx(wx), wy(wy), wz(wz), ww(ww)
{}

Matrix_4x4::Matrix_4x4(Vector4 r1, Vector4 r2, Vector4 r3, Vector4 r4)
    : xx(r1.x), xy(r1.y), xz(r1.z), xw(r1.w)
    , yx(r2.x), yy(r2.y), yz(r2.z), yw(r2.w)
    , zx(r3.x), zy(r3.y), zz(r3.z), zw(r3.w)
    , wx(r4.x), wy(r4.y), wz(r4.z), ww(r4.w)
{}

Matrix_4x4 Matrix_4x4::Id() {
    return Matrix_4x4(
               1.0, 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 1.0
           );
}

Matrix_4x4 Matrix_4x4::Zero() {
    return Matrix_4x4(
               0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0
           );
}

Matrix_4x4 Matrix_4x4::RotationX(float a) {
    return Matrix_4x4(
               1.0, 0.0, 0.0, 0.0,
               0.0, cos(a), -sin(a), 0.0,
               0.0, sin(a), cos(a), 0.0,
               0.0, 0.0, 0.0, 1.0
           );
}

Matrix_4x4 Matrix_4x4::RotationY(float a) {
    return Matrix_4x4(
               cos(a), 0.0, sin(a), 0.0,
               0.0, 1.0, 0.0, 0.0,
               -sin(a), 0.0, cos(a), 0.0,
               0.0, 0.0, 0.0, 1.0
           );
}

Matrix_4x4 Matrix_4x4::RotationZ(float a) {
    return Matrix_4x4(
               cos(a), -sin(a), 0.0, 0.0,
               sin(a), cos(a), 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 1.0
           );
}

Matrix_4x4 Matrix_4x4::RotationEuler(float a, float b, float c) {

    return Matrix_4x4(
               cos(b) * cos(c),
               cos(b) * sin(c),
               -sin(b),
               0.0,

               -cos(a) * sin(c) + sin(a) * sin(b) * cos(c),
               cos(a) * cos(c) + sin(a) * sin(b) * sin(c),
               sin(a) * cos(b),
               0.0,

               sin(a) * sin(c) + cos(a) * sin(b) * cos(c),
               -sin(a) * cos(c) + cos(a) * sin(b) * sin(c),
               cos(a) * cos(b),
               0.0,

               0.0, 0.0, 0.0, 1.0
           );
}

Matrix_4x4 Matrix_4x4::RotationAngleAxis(Vector3 v, float angle) {

    float c = cos(angle);
    float s = sin(angle);
    float nc = 1 - c;

    return Matrix_4x4(
               v.x * v.x * nc + c       , v.x * v.y * nc - v.z * s , v.x * v.z * nc + v.y * s, 0.0,
               v.y * v.x * nc + v.z * s , v.y * v.y * nc + c       , v.y * v.z * nc - v.x * s, 0.0,
               v.z * v.x * nc - v.y * s , v.z * v.y * nc + v.x * s , v.z * v.z * nc + c, 0.0,
               0.0, 0.0, 0.0, 1.0
           );

}

Matrix_4x4 Matrix_4x4::Translation(Vector3 trans) {

    return Matrix_4x4(
               1.0, 0.0, 0.0, trans.x,
               0.0, 1.0, 0.0, trans.y,
               0.0, 0.0, 1.0, trans.z,
               0.0, 0.0, 0.0, 1.0
           );

}

Matrix_4x4 Matrix_4x4::Scale(Vector3 scale) {
    return Matrix_4x4(
               scale.x, 0.0, 0.0, 0.0,
               0.0, scale.y, 0.0, 0.0,
               0.0, 0.0, scale.z, 0.0,
               0.0, 0.0, 0.0, 1.0
           );
}

Matrix_4x4 Matrix_4x4::ViewLookAt(Vector3 position, Vector3 target, Vector3 up) {

    Vector3 zaxis = Vector3::Normalize( target - position );
    Vector3 xaxis = Vector3::Normalize( Vector3::Cross(up, zaxis) );
    Vector3 yaxis = Vector3::Cross(zaxis, xaxis);

    Matrix_4x4 view_matrix = Matrix_4x4(
                                 xaxis.x, xaxis.y, xaxis.z, 0.0,
                                 yaxis.x, yaxis.y, yaxis.z, 0.0,
                                 -zaxis.x, -zaxis.y, -zaxis.z, 0.0,
                                 0.0, 0.0, 0.0, 1.0
                             );

    view_matrix = view_matrix * Matrix_4x4::Translation(-position);

    return view_matrix;

}

Matrix_4x4 Matrix_4x4::Perspective(float fov, float near_clip, float far_clip, float ratio) {

    float right, left, bottom, top;

    right = -(near_clip * tanf(fov));
    left = -right;

    top = ratio * near_clip * tanf(fov);
    bottom = -top;

    return Matrix_4x4(
               (2.0 * near_clip) / (right - left), 0.0, (right + left) / (right - left), 0.0,
               0.0, (2.0 * near_clip) / (top - bottom), (top + bottom) / (top - bottom), 0.0,
               0.0, 0.0, (-far_clip - near_clip) / (far_clip - near_clip), ( -(2.0 * near_clip) * far_clip) / (far_clip - near_clip),
               0.0, 0.0, -1.0, 0.0
           );
}

Matrix_4x4 Matrix_4x4::Orthographic(float left, float right, float bottom, float top, float near, float far) {

    return Matrix_4x4(
               2 / (right - left), 0.0, 0.0, - (right + left) / (right - left),
               0.0, 2 / (top - bottom), 0.0, - (top + bottom) / (top - bottom),
               0.0, 0.0, -2 / (far - near), - (far + near) / (far - near),
               0.0, 0.0, 0.0, 1.0
           );

}

void Matrix_4x4::Print(Matrix_4x4 m) {
    printf("| %2.0f, %2.0f, %2.0f, %2.0f |\n", m.xx, m.xy, m.xz, m.xw);
    printf("| %2.0f, %2.0f, %2.0f, %2.0f |\n", m.yx, m.yy, m.yz, m.yw);
    printf("| %2.0f, %2.0f, %2.0f, %2.0f |\n", m.zx, m.zy, m.zz, m.zw);
    printf("| %2.0f, %2.0f, %2.0f, %2.0f |\n", m.wx, m.wy, m.wz, m.ww);
}

Matrix_4x4 Matrix_4x4::FromMatrix_3x3(Matrix_3x3 m) {

    return Matrix_4x4(
               m.xx, m.xy, m.xz, 0.0,
               m.yx, m.yy, m.yz, 0.0,
               m.zx, m.zy, m.zz, 0.0,
               0.0, 0.0, 0.0, 1.0
           );

}

Matrix_3x3 Matrix_4x4::ToMatrix_3x3(Matrix_4x4 m) {

    return Matrix_3x3(
               m.xx, m.xy, m.xz,
               m.yx, m.yy, m.yz,
               m.zx, m.zy, m.zz
           );

}

Matrix_4x4 Matrix_4x4::operator*(Matrix_4x4 m) {

    return Matrix_4x4(

               (xx * m.xx) + (xy * m.yx) + (xz * m.zx) + (xw * m.wx),
               (xx * m.xy) + (xy * m.yy) + (xz * m.zy) + (xw * m.wy),
               (xx * m.xz) + (xy * m.yz) + (xz * m.zz) + (xw * m.wz),
               (xx * m.xw) + (xy * m.yw) + (xz * m.zw) + (xw * m.ww),

               (yx * m.xx) + (yy * m.yx) + (yz * m.zx) + (yw * m.wx),
               (yx * m.xy) + (yy * m.yy) + (yz * m.zy) + (yw * m.wy),
               (yx * m.xz) + (yy * m.yz) + (yz * m.zz) + (yw * m.wz),
               (yx * m.xw) + (yy * m.yw) + (yz * m.zw) + (yw * m.ww),

               (zx * m.xx) + (zy * m.yx) + (zz * m.zx) + (zw * m.wx),
               (zx * m.xy) + (zy * m.yy) + (zz * m.zy) + (zw * m.wy),
               (zx * m.xz) + (zy * m.yz) + (zz * m.zz) + (zw * m.wz),
               (zx * m.xw) + (zy * m.yw) + (zz * m.zw) + (zw * m.ww),

               (wx * m.xx) + (wy * m.yx) + (wz * m.zx) + (ww * m.wx),
               (wx * m.xy) + (wy * m.yy) + (wz * m.zy) + (ww * m.wy),
               (wx * m.xz) + (wy * m.yz) + (wz * m.zz) + (ww * m.wz),
               (wx * m.xw) + (wy * m.yw) + (wz * m.zw) + (ww * m.ww)

           );

}

Matrix_4x4 Matrix_4x4::operator*(Matrix_3x3 m) {
    Matrix_4x4 m2 = Matrix_4x4::FromMatrix_3x3(m);
    return *this * m2;
}

Vector4 Matrix_4x4::operator*(Vector4 v) {
    return Vector4(
               (xx * v.x) + (xy * v.y) + (xz * v.z) + (xw * v.w),
               (yx * v.x) + (yy * v.y) + (yz * v.z) + (yw * v.w),
               (zx * v.x) + (zy * v.y) + (zz * v.z) + (zw * v.w),
               (wx * v.x) + (wy * v.y) + (wz * v.z) + (ww * v.w)
           );
}

Vector3 Matrix_4x4::operator*(Vector3 vec) {

    Vector4 v = Vector4::ToHomogeneous(vec);

    return Vector4::FromHomogeneous( Vector4(
                                         (xx * v.x) + (xy * v.y) + (xz * v.z) + (xw * v.w),
                                         (yx * v.x) + (yy * v.y) + (yz * v.z) + (yw * v.w),
                                         (zx * v.x) + (zy * v.y) + (zz * v.z) + (zw * v.w),
                                         (wx * v.x) + (wy * v.y) + (wz * v.z) + (ww * v.w)
                                     ));

}

Matrix_4x4 Matrix_4x4::Transpose(Matrix_4x4 m) {

    return Matrix_4x4(
               m.xx, m.yx, m.zx, m.wx,
               m.xy, m.yy, m.zy, m.wy,
               m.xz, m.yz, m.zz, m.wz,
               m.xw, m.yw, m.zw, m.ww
           );
}

float Matrix_4x4::Determinant(Matrix_4x4 m) {

    float cofact_xx =  Matrix_3x3::Determinant(Matrix_3x3(m.yy, m.yz, m.yw, m.zy, m.zz, m.zw, m.wy, m.wz, m.ww));
    float cofact_xy = -Matrix_3x3::Determinant(Matrix_3x3(m.yx, m.yz, m.yw, m.zx, m.zz, m.zw, m.wx, m.wz, m.ww));
    float cofact_xz =  Matrix_3x3::Determinant(Matrix_3x3(m.yx, m.yy, m.yw, m.zx, m.zy, m.zw, m.wx, m.wy, m.ww));
    float cofact_xw = -Matrix_3x3::Determinant(Matrix_3x3(m.yx, m.yy, m.yz, m.zx, m.zy, m.zz, m.wx, m.wy, m.wz));

    return (cofact_xx * m.xx) + (cofact_xy * m.xy) + (cofact_xz * m.xz) + (cofact_xw * m.xw);

}

Matrix_4x4 Matrix_4x4::Inverse(Matrix_4x4 m) {

    float det = Matrix_4x4::Determinant(m);
    if (det == 0) {
        printf("[ERROR]: Cannot Invert non-singular 4x4 matrix.\n");
        exit(EXIT_FAILURE);
    }

    float fac = 1.0 / det;

    Matrix_4x4 ret;
    ret.xx = fac *  Matrix_3x3::Determinant(Matrix_3x3(m.yy, m.yz, m.yw, m.zy, m.zz, m.zw, m.wy, m.wz, m.ww));
    ret.xy = fac * -Matrix_3x3::Determinant(Matrix_3x3(m.yx, m.yz, m.yw, m.zx, m.zz, m.zw, m.wx, m.wz, m.ww));
    ret.xz = fac *  Matrix_3x3::Determinant(Matrix_3x3(m.yx, m.yy, m.yw, m.zx, m.zy, m.zw, m.wx, m.wy, m.ww));
    ret.xw = fac * -Matrix_3x3::Determinant(Matrix_3x3(m.yx, m.yy, m.yz, m.zx, m.zy, m.zz, m.wx, m.wy, m.wz));

    ret.yx = fac * -Matrix_3x3::Determinant(Matrix_3x3(m.xy, m.xz, m.xw, m.zy, m.zz, m.zw, m.wy, m.wz, m.ww));
    ret.yy = fac *  Matrix_3x3::Determinant(Matrix_3x3(m.xx, m.xz, m.xw, m.zx, m.zz, m.zw, m.wx, m.wz, m.ww));
    ret.yz = fac * -Matrix_3x3::Determinant(Matrix_3x3(m.xx, m.xy, m.xw, m.zx, m.zy, m.zw, m.wx, m.wy, m.ww));
    ret.yw = fac *  Matrix_3x3::Determinant(Matrix_3x3(m.xx, m.xy, m.xz, m.zx, m.zy, m.zz, m.wx, m.wy, m.wz));

    ret.zx = fac *  Matrix_3x3::Determinant(Matrix_3x3(m.xy, m.xz, m.xw, m.yy, m.yz, m.yw, m.wy, m.wz, m.ww));
    ret.zy = fac * -Matrix_3x3::Determinant(Matrix_3x3(m.xx, m.xz, m.xw, m.yx, m.yz, m.yw, m.wx, m.wz, m.ww));
    ret.zz = fac *  Matrix_3x3::Determinant(Matrix_3x3(m.xx, m.xy, m.xw, m.yx, m.yy, m.yw, m.wx, m.wy, m.ww));
    ret.zw = fac * -Matrix_3x3::Determinant(Matrix_3x3(m.xx, m.xy, m.xz, m.yx, m.yy, m.yz, m.wx, m.wy, m.wz));

    ret.wx = fac * -Matrix_3x3::Determinant(Matrix_3x3(m.xy, m.xz, m.xw, m.yy, m.yz, m.yw, m.zy, m.zz, m.zw));
    ret.wy = fac *  Matrix_3x3::Determinant(Matrix_3x3(m.xx, m.xz, m.xw, m.yx, m.yz, m.yw, m.zx, m.zz, m.zw));
    ret.wz = fac * -Matrix_3x3::Determinant(Matrix_3x3(m.xx, m.xy, m.xw, m.yx, m.yy, m.yw, m.zx, m.zy, m.zw));
    ret.ww = fac *  Matrix_3x3::Determinant(Matrix_3x3(m.xx, m.xy, m.xz, m.yx, m.yy, m.yz, m.zx, m.zy, m.zz));

    ret = Matrix_4x4::Transpose(ret);

    return ret;

}

Matrix_4x4 Matrix_4x4::operator*(float fac) {
    return Matrix_4x4(
               xx * fac, xy * fac, xz * fac, xw * fac,
               yx * fac, yy * fac, yz * fac, yw * fac,
               zx * fac, zy * fac, zz * fac, zw * fac,
               wx * fac, wy * fac, wz * fac, ww * fac
           );
}

Matrix_4x4 Matrix_4x4::operator+(Matrix_4x4 m) {
    return Matrix_4x4(
               xx + m.xx, xy + m.xy, xz + m.xz, xw + m.xw,
               yx + m.yx, yy + m.yy, yz + m.yz, yw + m.yw,
               zx + m.zx, zy + m.zy, zz + m.zz, zw + m.zw,
               wx + m.wx, wy + m.wy, wz + m.wz, ww + m.ww
           );
}
