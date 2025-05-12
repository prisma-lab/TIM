#include <iostream>
#include <random>

class radpi{
public:
    radpi(){
        this->ang = 0;
    }
    radpi(double a){
        this->ang = ( a>3.14159 ? a-(2*3.14159) : (a<=-3.14159 ? a+(2*3.14159) : a) );
    }

    // Overload (double) operator to return the angle value.
    operator double() { return this->ang; }

    // Overload + operator to add two ang180.
    double operator+(double a)
    {
        //std::cout<<"deg180: "<<this->ang<<" + "<<a<<" = "<<this->ang<<std::endl;
        double res = this->ang + a;
        res = ( res>3.14159 ? res-(2*3.14159) : (res<=-3.14159 ? res+(2*3.14159) : res) );

        return res;
    }

    // Overload - operator to subtradt two ang180.
    double operator-(double a)
    {
        //std::cout<<"deg180: "<<this->ang<<" - "<<a<<" = "<<this->ang<<std::endl;
        double res = this->ang - a;
        res = ( res>3.14159 ? res-(2*3.14159) : (res<=-3.14159 ? res+(2*3.14159) : res) );

        return res;
    }

    // Overload = operator to set from double.
    double operator=(double a)
    {
        this->ang = ( a>3.14159 ? a-(2*3.14159) : (a<=-3.14159 ? a+(2*3.14159) : a) );
        return this->ang;
    }
private:
    double ang;
};



class deg180{
public:
    deg180(){
        this->ang = 0;
    }
    deg180(double a){
        this->ang = ( a>180 ? a-360 : (a<=-180 ? a+360 : a) );
    }

    // Overload (double) operator to return the angle value.
    operator double() { return this->ang; }

    // Overload + operator to add two ang180.
    double operator+(double a)
    {
        //std::cout<<"deg180: "<<this->ang<<" + "<<a<<" = "<<this->ang<<std::endl;
        double res = this->ang + a;
        res = ( res>180 ? res-360 : (res<=-180 ? res+360 : res) );

        return res;
    }

    // Overload - operator to subtradt two ang180.
    double operator-(double a)
    {
        //std::cout<<"deg180: "<<this->ang<<" - "<<a<<" = "<<this->ang<<std::endl;
        double res = this->ang - a;
        res = ( res>180 ? res-360 : (res<=-180 ? res+360 : res) );

        return res;
    }

    // Overload = operator to set from double.
    double operator=(double a)
    {
        this->ang = ( a>180 ? a-360 : (a<=-180 ? a+360 : a) );
        return this->ang;
    }
private:
    double ang;
};

class Point3d{
public:
    Point3d(){
        x = 0;
        y = 0;
        z = 0;
    };
    Point3d(double nx, double ny, double nz){
        x = nx;
        y = ny;
        z = nz;
    };
    Point3d(double nx, double ny){
        x = nx;
        y = ny;
        z = 0; //std::nan;
    };
    double x; //m
    double y; //m
    double z; //m
};

class Pose3d{
public:
    Pose3d(){
        x = 0;
        y = 0;
        w = 0;
    };
    Pose3d(double nx, double ny, deg180 nw){
        x = nx;
        y = ny;
        w = nw;
    };
    Pose3d(double nx, double ny){
        x = nx;
        y = ny;
        w = 0; //std::nan;
    };
    // Overload = operator to set from Pose3d.
    Pose3d &operator=(Pose3d &a)
    {
        this->x = a.x;
        this->y = a.y;
        this->w = a.w;
        return a;
    }
    double x; //m
    double y; //m
    //double w; //deg
    deg180 w; //deg
};

//additional math utils!

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


class Random{
public:
    Random():
        random_generator(std::random_device()())
    {
            
    }
        
//    void reset(){
//        random_generator(std::random_device()());
//    }

    double real(double fMin, double fMax)
    {
        std::uniform_real_distribution<double> unif(fMin,fMax);
        return unif(random_generator); //unif(re);
    }

    int integer(int iMin, int iMax){
        std::uniform_int_distribution<int> unif(iMin,iMax);
        return unif(random_generator); //unif(re);
    }

protected:
    std::mt19937 random_generator; //Standard mersenne_twister_engine seeded with rd()

};