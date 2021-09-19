#ifndef _CYR_PRIMITIVE_H_
#define _CYR_PRIMITIVE_H_
#include "cyr_definitions.h"

#include <Eigen/Eigen>
#include <tuple>

namespace cyr
{
class Primitive {
public:
    Primitive () = default;
    virtual bool Hit(const Ray& ray, float t0, float t1, HitRecord *hit_record) = 0;

    virtual Box BoundingBox()const = 0 ;
}; 

class Sphere : public Primitive{
public:
    Sphere ();
    ~Sphere();
    Sphere(const Eigen::Vector3f& center, float radius);
    Sphere(float center[3], float radius);
    Box BoundingBox() const;
    
    bool Hit (const Ray& ray, float t0, float t1, HitRecord *hit_record);

    void SetRadius(float radius);
    void SetCenter(const Eigen::Vector3f& center);
    void SetCenter(float center[3] );

private:
    Eigen::Vector3f center_;
    float radius_;
};

class Triangle: public Primitive {
public:
    Triangle();
    Triangle(const float vertices[3][3]);
    Triangle(const Eigen::Vector3f vertices[3]);
    ~Triangle();

    Box BoundingBox() const;
    bool Hit (const Ray &ray, float t0, float t1, HitRecord *rec);
private:
    std::tuple<float, float, float> CalBariCoord(const Ray& ray) const;
    Eigen::Vector3f vertices_[3];
};
} // namespace cyr



#endif  // _CYR_PRIMITIVE_H_