//
// Created by haxic on 2016-06-17.
//

#ifndef PROJECT_VECTOR3_H
#define PROJECT_VECTOR3_H

/*
struct MyVector{

    MyVector(){

    }


};*/

class Vector3 {
private:


public:
    double x;
    double y;
    double z;

    Vector3(double newX, double newY, double newZ) : x(newX), y(newY), z(newZ) {
    }


    Vector3() {
        x = 0;
        y = 0;
        z = 0;
    }

    double distance() {
        return sqrt(x * x + y * y + z * z);
    }
};

#endif //PROJECT_VECTOR3_H

