#include "Camera.h"

Camera::Camera()
    : m_position( Vector3(1.0, 1.0, 1.0) )
    , m_target( Vector3::Zero() )
    , m_fov(3.141 / 4)
    , m_near_clip(8)
    , m_far_clip(128)
{}

Camera::Camera(Vector3 position, Vector3 target)
    : m_position(position)
    , m_target(target)
    , m_fov(3.141 / 4)
    , m_near_clip(8)
    , m_far_clip(128)
{}

Camera::~Camera() {}

void Camera::SetPosition(Vector3 position) {
    m_position = position;
}

Vector3 Camera::GetPosition() {
    return m_position;
}

void Camera::SetTarget(Vector3 target) {
    m_target = target;
}

Vector3 Camera::GetTarget() {
    return m_target;
}

void Camera::SetDirection(Vector3 direction) {
    m_target = Vector3::Normalize(m_position + direction);
}

Vector3 Camera::GetDirection() {
    return Vector3::Normalize(m_target - m_position);
}

Vector3 Camera::PositionToTarget() {
    return m_target - m_position;
}

void Camera::SetFOV(float fov) {
    m_fov = fov;
}

float Camera::GetFOV() {
    return m_fov;
}

void Camera::SetNearClipPlane(float near_clip) {
    m_near_clip = near_clip;
}

float Camera::GetNearClipPlane() {
    return m_near_clip;
}

void Camera::SetFarClipPlane(float far_clip) {
    m_far_clip = far_clip;
}

float Camera::GetFarClipPlane() {
    return m_far_clip;
}