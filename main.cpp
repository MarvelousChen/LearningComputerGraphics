#include "primitive.h"
#include "cyr_definitions.h"

#include <Eigen/Eigen>
#include <iostream>


int main(int, char**) {

    cyr::Ray ray = {Eigen::Vector3f(1, 1, 1), Eigen::Vector3f(-1, -1, -1),};
    cyr::Sphere sphere({0, 0, 0}, 1);
    float ver[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    cyr::Triangle tri(ver);

    cyr::HitRecord hitRecord;
    printf("%d\n", tri.Hit(ray, 0, 50000, &hitRecord));
}
