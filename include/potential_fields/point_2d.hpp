#ifndef POINT_2D_HPP_
#define POINT_2D_HPP_

#include <cmath>
#include <iostream>

class Point2D{
    public:
        Point2D(){
            rect_ = new double[2];
            polar_ = new double[2];
        }
        
        void setXY(double x, double y){
            rect_[0] = x; rect_[1] = y;
            polar_[0] = hypot(x, y); polar_[1] = atan2(y, x);
        }
        void setRTHeta(double r, double theta){
            rect_[0] = r * cos(theta); rect_[1] = r * sin(theta);
            polar_[0] = r; polar_[1] = theta;

        }
        double* getXY(){
            return rect_;
        }
        double* getRTheta(){
            return polar_;
        }
        double getDistance(){
            return polar_[0];
        }
        double getTheta(){
            return polar_[1];
        }

        bool operator == (const Point2D& other){
            return this->polar_ == other.polar_;
        }
        bool operator < (const Point2D& other){
            return this->polar_[0] < other.polar_[0];
        }
        bool operator > (const Point2D& other){
            return this->polar_[0] > other.polar_[0];
        }
        bool operator <= (const Point2D& other){
            return this->polar_[0] <= other.polar_[0];
        }
        bool operator >= (const Point2D& other){
            return this->polar_[0] >= other.polar_[0];
        }

        friend std::ostream& operator << (std::ostream& out, const Point2D& point){
            out << "X: " << point.rect_[0] <<", Y: " << point.rect_[1];
            out << "\nR: " << point.polar_[0] << ", Theta: " << point.polar_[1] << "\n";
            return out;
        }

    private:
        double* rect_;
        double* polar_;

};

#endif
