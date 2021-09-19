#include "primitive.h"
#include "cyr_definitions.h"

#include <Eigen/Eigen>
#include <iostream>


void Exercises4_1() {
    cyr::Ray ray = {Eigen::Vector3f(1, 1, 1), Eigen::Vector3f(-1, -1, -1),};
    cyr::Sphere sphere({0, 0, 0}, 1);
    cyr::HitRecord hitRecord;
    printf("Answer to 4-1: %d\n", sphere.Hit(ray, 0, 50000, &hitRecord));
}

void Exercises4_2() {
    cyr::Ray ray = {Eigen::Vector3f(1, 1, 1), Eigen::Vector3f(-1, -1, -1),};
    float ver[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    cyr::Triangle tri(ver);
    cyr::HitRecord hitRecord;
    printf("Answer to 4-2: %d\n", tri.Hit(ray, 0, 50000, &hitRecord));
}

int main(int, char**) {
    Exercises4_1();
    Exercises4_2();
    
}
