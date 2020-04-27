#pragma once 

#include <cmath>
#include <vector>
// #include <>

namespace ele {

/**
 * (x1,y1) --> (x2, y2)
 * point (x1,y1), (x2,y2) on line, the line from (x1,y1) point to (x2,y2)
 * Line function : f = A*x + B*y + C 
 *                 A=y2-y1,  B=x1-x2,  C=x2*y1-x1*y2
 * Dis = A*xp + B*yp + C
 * if Dis < 0, Point is at left of the line; 
 * if Dis > 0, Point at at right of the line; 
 * if Dis = 0, Point is on the line.
*/
class BoundingLine {
public:
    #define EPS 0.000001
    enum BoundingType {
        Left = 1,
        Bottom= 2,
        Rotation = 3,
        Top = 4,
        Right = 5
    };

    BoundingLine(float theta, float x1, float y1, BoundingType type) : 
            x1_(x1), y1_(y1), type_(type) {
        if (type == BoundingType::Top || type == BoundingType::Bottom) {
            theta_ = theta + M_PI/2;
            if (theta_ > M_PI) theta_ = theta_ - 2 * M_PI;
        } else {
            theta_ = theta;
        }
        A_ = std::sin(theta_);
        B_ = -std::cos(theta_);
        UpdateC();
    }
    BoundingLine(float x1, float y1, float x2, float y2, BoundingType type) : 
            x1_(x1), y1_(y1), type_(type) {
        A_ = y2-y1;
        B_ = x1-x2;
        UpdateC();
    }
    ~BoundingLine() {}

    float F (float xp, float yp) {
        return (A_*xp + B_*yp + C_);
    }
    float A() { return A_; }
    float B() { return B_; }
    float C() { return C_; }

    bool Update (float xp, float yp) {
        if (type_ < BoundingType::Rotation) {
            if (F(xp, yp) < 0) {
                UpdateX1(xp, yp);
                UpdateC();
                return true;
            }
        } else {
            if (F(xp, yp) > 0) {
                UpdateX1(xp, yp);
                UpdateC();
                return true;
            }
        }
        return false;
    }

    bool GetCross (BoundingLine& l2, float& xc, float& yc) {
        float denominator = A_*l2.B() - l2.A()*B_;
        if (denominator < EPS && denominator > -EPS ) {
            return false;
        }
        float x_numerator = B_*l2.C() - l2.B()*C_;
        float y_numerator = C_*l2.A() - l2.C()*A_;
        xc = x_numerator / denominator;
        yc = y_numerator / denominator;
    }

    float GetDistance (BoundingLine& l2) {
        float denominator = A_*A_ + B_*B_;
        denominator = std::sqrt(denominator);
        float numerator = std::abs(C_-l2.C());
        return (numerator/denominator);
    }

private:
    void UpdateC () {
        // C_ = (x1_-B_)*y1_ - x1_*(y1_+A_);
        C_ = -(y1_*B_ + x1_*A_);
    }
    void UpdateX1 (float xp, float yp) {
        x1_ = xp;
        y1_ = yp;
    }
private:
    float A_;
    float B_;
    float C_;
    float x1_;
    float y1_;
    float theta_;
    BoundingType type_;
}; // class BoundingLine 


} // namespace ele 

