//
//  TwoDVector.h
//  OneDKalmanFilterExample
//
//  Created by Andrew Robertson on 05/03/2015.
//
//

#ifndef OneDKalmanFilterExample_TwoDVector_h
#define OneDKalmanFilterExample_TwoDVector_h


template <class T>

class TwoDVector {

    //a
    //b
public:
    TwoDVector(T _a, T _b){
        a = _a;
        b = _b;
    }
    
    void init(T _a, T _b){
        a = _a;
        b = _b;
    }
    
    void reset(const T& _a, const T& _b){
        a = _a;
        b = _b;
    }
    
    TwoDVector operator+(const TwoDVector VecB) const {
        TwoDVector addVec(a, b);
        addVec.a += VecB.a;
        addVec.b += VecB.b;
        return addVec;
    }
    
    TwoDVector operator-(const TwoDVector VecB) const {
        TwoDVector subVec(a, b);
        subVec.a -= VecB.a;
        subVec.b -= VecB.b;
        return subVec;
    }
    
    
    T dot(const TwoDVector& vecB) const {
        return a*vecB.a + (b*vecB.b);
    }
    
    
    void print() {
        printf("%f\n", (float)a);
        printf("%f\n\n", (float)b);
    }
    
    
//protected:
    T a;
    T b;
};

#endif
