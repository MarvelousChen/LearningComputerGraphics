#include "primitive.h"

#include <cmath>
#include <limits>
using std::sqrt;

namespace cyr
{
Sphere::Sphere() {
}

Sphere::Sphere(const Eigen::Vector3f& center, float radius) {
    center_ = center;
    radius_ = radius;
}

Sphere::Sphere(float center[3], float radius)
{
    center_ = Eigen::Vector3f(center);
    radius_ = radius;
}
Sphere::~Sphere () {
}

void Sphere::SetRadius(float radius)
{
    radius_ = radius;
}

void Sphere::SetCenter(const Eigen::Vector3f& center)
{
    center_ = center;
}
bool Sphere::Hit(const Ray& ray, float t0, float t1, HitRecord *rec) {
    float A = ray.d.dot(ray.d);
    Eigen::Vector3f temp = ray.e - center_;
    float B = 2 * ray.d.dot(temp);
    float C = temp.dot(temp) - radius_ * radius_;
    
    float res;
    if (A == 0) {
        if (B == 0)
            return false;
        res = -C / B;
    } else {
        float deter = B * B - 4 * A * C;
        if (deter < 0)
            return false;
        if (deter == 0) {
            res = -B / A * 0.5f;
        }else {
            float res1 = (-B + sqrt(deter)) / A * 0.5;
            float res2 = (-B - sqrt(deter)) / A * 0.5;
            if (res1 < t0 || res1 > t1)
                res1 = FLT_MAX;
            if (res2 < t0 || res2 > t1)
                res2 = FLT_MAX;
            res = std::min(res1, res2);
            if (res == FLT_MAX)
                return false;
        }
    }

    if (res >= t0 && res <= t1) {
        // TODO: write hit record
        return true;
    }else {
        return false;
    }

    
}

Box Sphere::BoundingBox() const{
    Eigen::Vector3f min_point = center_ 
                                - Eigen::Vector3f(radius_, radius_, radius_);
    Eigen::Vector3f max_point = center_ 
                                + Eigen::Vector3f(radius_, radius_, radius_);
    return Box(min_point, max_point);
}

Triangle::Triangle(){
}
Triangle::Triangle(const float vertices[3][3]) {
    for (int i = 0; i < 3; ++i) {
        vertices_[i] = Eigen::Vector3f(vertices[i]);
    }
}
Triangle::Triangle(const Eigen::Vector3f vertices[3]) {
    for (int i = 0; i < 3; ++i) {
        vertices_[i] = vertices[i];
    }
}
Triangle::~Triangle() {
}
bool Triangle::Hit(const Ray& ray, float t0, float t1, HitRecord *rec) {
    float ei_hf = (vertices_[0].y() - vertices_[2].y()) * ray.d.z()
                 - ray.d.y() * (vertices_[0].z() - vertices_[2].z());
    float gf_di = (ray.d.x() * (vertices_[0].z() - vertices_[2].z())) 
                - (vertices_[0].x() - vertices_[2].x()) * ray.d.z();
    float dh_eg = (vertices_[0].x() - vertices_[2].x()) * ray.d.y()
                - (vertices_[0].y() - vertices_[2].y()) * ray.d.x();
    float ak_jb = (vertices_[0].x() - vertices_[1].x()) 
                * (vertices_[0].y() - ray.e.y())
                - (vertices_[0].x() - ray.e.x()) 
                * (vertices_[0].y() - vertices_[1].y());
    float jc_al = (vertices_[0].x() - ray.e.x()) 
                * (vertices_[0].z() - vertices_[1].z())
                - (vertices_[0].x() - vertices_[1].x()) 
                * (vertices_[0].z() - ray.e.z());
    float bl_kc = (vertices_[0].y() - vertices_[1].y()) 
                * (vertices_[0].z() - ray.e.z())
                - (vertices_[0].y() - ray.e.y()) 
                * (vertices_[0].z() - vertices_[1].z());
    float M = (vertices_[0].x() - vertices_[1].x()) * ei_hf 
            + (vertices_[0].y() - vertices_[1].y()) * gf_di
            + (vertices_[0].z() - vertices_[1].z()) * dh_eg;
    
    float t = -((vertices_[0].z() - vertices_[2].z()) * ak_jb 
            + (vertices_[0].y() - vertices_[2].y()) * jc_al 
            + (vertices_[0].x() - vertices_[2].x()) * bl_kc) / M;
    if (t < t0 || t > t1)
        return false;

    float beta = (vertices_[0].x() - ray.e.x() * ei_hf 
                + (vertices_[0].y() - ray.e.y()) * gf_di
                + (vertices_[0].z() - ray.e.z()) * dh_eg) / M;
    if (beta < 0 || beta > 1)
        return false;

    float gamma = (ray.d.z() * ak_jb + ray.d.y() * jc_al 
                + ray.d.x() * bl_kc) / M;
    if (gamma < 0 || gamma > 1)
        return false;
    // TODO: Set hit record
    return true;
    
}
std::tuple<float, float, float> Triangle::CalBariCoord(const Ray& ray) const{
    float ei_hf = (vertices_[0].y() - vertices_[2].y()) * ray.d.z()
                 - ray.d.y() * (vertices_[0].z() - vertices_[2].z());
    float gf_di = (ray.d.x() * (vertices_[0].z() - vertices_[2].z())) 
                - (vertices_[0].x() - vertices_[2].x()) * ray.d.z();
    float dh_eg = (vertices_[0].x() - vertices_[2].x()) * ray.d.y()
                - (vertices_[0].y() - vertices_[2].y()) * ray.d.x();
    float ak_jb = (vertices_[0].x() - vertices_[1].x()) * (vertices_[0].y() - ray.e.y())
                - (vertices_[0].x() - ray.e.x()) * (vertices_[0].y() - vertices_[1].y());
    float jc_al = (vertices_[0].x() - ray.e.x()) * (vertices_[0].z() - vertices_[1].z())
                - (vertices_[0].x() - vertices_[1].x()) * (vertices_[0].z() - ray.e.z());
    float bl_kc = (vertices_[0].y() - vertices_[1].y()) * (vertices_[0].z() - ray.e.z())
                - (vertices_[0].y() - ray.e.y()) * (vertices_[0].z() - vertices_[1].z());
    float M = (vertices_[0].x() - vertices_[1].x()) * ei_hf 
            + (vertices_[0].y() - vertices_[1].y()) * gf_di
            + (vertices_[0].z() - vertices_[1].z()) * dh_eg;
    float beta = (vertices_[0].x() - ray.e.x() * ei_hf + (vertices_[0].y() - ray.e.y()) * gf_di
                + (vertices_[0].z() - ray.e.z()) * dh_eg) / M;
    float gamma = (ray.e.z() * ak_jb + ray.e.y() * jc_al + ray.e.x() * bl_kc)
                    / M;
    float t = ((vertices_[0].z() - vertices_[2].z()) * ak_jb + (vertices_[0].y() 
            - vertices_[2].y()) * jc_al + (vertices_[0].x() - vertices_[2].x())
            * bl_kc) / M;

    return {beta, gamma, t};
}

Box Triangle::BoundingBox() const{
    return Box();
}
} // namespace cyr
