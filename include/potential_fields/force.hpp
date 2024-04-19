#ifndef FORCE_HPP_
#define FORCE_HPP_

#include <cmath>

class Force{
    public:
        Force(){
            x_ = 0; y_ = 0; size_ = 0;
        }
        void set(double x, double y){
            x_ = x;
            y_ = y;
            size_ = hypot(x, y);
        }
        double* get(){
            double* ret = new double[3];

            ret[0] = x_;
            ret[1] = y_;
            ret[2] = size_;

            return ret;
        }

    private:
        double x_, y_, size_;
};
#endif