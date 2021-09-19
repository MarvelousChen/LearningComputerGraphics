#ifndef _CYR_DEFINITIONS_H_
#define _CYR_DEFINITIONS_H_

#include <Eigen/Eigen>

namespace cyr
{
typedef struct _ray {
    Eigen::Vector3f e;
    Eigen::Vector3f d;
    // float t;
}Ray;

typedef struct _hit_record {

}HitRecord;

typedef struct _box {
    _box(const Eigen::Vector3f& min, const Eigen::Vector3f &max)
    {
        min_point = min;
        max_point = max;
    }
    _box(){}
    Eigen::Vector3f min_point;
    Eigen::Vector3f max_point;
}Box;
}


#endif  // _CYR_DEFINITIONS_H_