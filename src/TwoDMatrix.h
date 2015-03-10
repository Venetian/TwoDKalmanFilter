//
//  2DMatrix.h
//  OneDKalmanFilterExample
//
//  Created by Andrew Robertson on 05/03/2015.
//
//

#ifndef __OneDKalmanFilterExample___DMatrix__
#define __OneDKalmanFilterExample___DMatrix__

#include <stdio.h>

#include "TwoDVector.h"

template <class T>

class TwoDMatrix {
  //make  friends later class TwoDVector<T>;
    //a b
    //c d
public:
    TwoDMatrix(T _a, T _b, T _c, T _d){
        a = _a;
        b = _b;
        c = _c;
        d = _d;
        
    }

    void init(T _a, T _b, T _c, T _d){
        a = _a;
        b = _b;
        c = _c;
        d = _d;
    }
    
    void reset(const T& _a, const T& _b, const T& _c, const T& _d){
        a = _a;
        b = _b;
        c = _c;
        d = _d;
    }
    
    TwoDMatrix operator+(const TwoDMatrix matB) {
        TwoDMatrix<T> addMat(a, b, c, d);
        addMat.a += matB.a;
        addMat.b += matB.b;
        addMat.c += matB.c;
        addMat.d += matB.d;
        return addMat;
    }
    
    TwoDMatrix operator-(const TwoDMatrix matB) {
        TwoDMatrix<T> subMat(a, b, c, d);
        subMat.a -= matB.a;
        subMat.b -= matB.b;
        subMat.c -= matB.c;
        subMat.d -= matB.d;
        return subMat;
    }
    
    TwoDMatrix operator*(const TwoDMatrix matB) {
        TwoDMatrix<T> storeMat(a, b, c, d);
        TwoDMatrix<T> timesMat(a, b, c, d);
        
        timesMat.a = storeMat.a*matB.a + storeMat.b*matB.c;
        timesMat.b = storeMat.a*matB.b + storeMat.b*matB.d;
        timesMat.c = storeMat.c*matB.a + storeMat.d*matB.c;
        timesMat.d = storeMat.c*matB.b + storeMat.d*matB.d;
        return timesMat;
    }
    
    TwoDVector<T> operator*(const TwoDVector<T>& vecB) {
        TwoDMatrix<T> storeMat(a, b, c, d);
        //TwoDMatrix<T> timesMat(a, b, c, d);
        TwoDVector<T> multVec(vecB.a, vecB.b);
        multVec.a = storeMat.a*vecB.a + storeMat.b*vecB.b;
        multVec.b = storeMat.c*vecB.a + storeMat.d*vecB.b;
        return multVec;
    }
    /*
    void operator=(const TwoDMatrix<T> &matB) {
        a = matB.a;
        b = matB.b;
        c = matB.c;
        d = matB.d;
        //return *this;
    }
    */
    /*
    void  operator= (const TwoDMatrix<T> & other)
    {
        if (this != &other) // protect against invalid self-assignment
        {
            //TwoDMatrix<T> newMatrix(other.a, other.b, other.c, other.d);
            //return &newMatrix;
            
            a = other.a;
            b = other.b;
            c = other.c;
            d = other.d;
        } //else
            
            return *this;
        // by convention, always return *this
    }
    */

    
    TwoDMatrix dot(const T scalar) {
        
        TwoDMatrix<T> timesMat(a, b, c, d);
        
        timesMat.a *= scalar;
        timesMat.b *= scalar;
        timesMat.c *= scalar;
        timesMat.d *= scalar;
        return timesMat;
    }
    
    TwoDMatrix Transpose() {
        
        TwoDMatrix<T> tranMat(a, c, b, d);
        return tranMat;
    }
    
    TwoDMatrix inverse() {
        
        TwoDMatrix<T> invMat(d, -1*b, -1*c, a);
        T det = 1.f/(a*d - b*c);
        return invMat.dot(det);
    }
    
    TwoDVector<T> times(const TwoDVector<T>& vec) {
        TwoDVector<T> multVec(vec.a, vec.b);
        
        multVec.a = a*vec.a + b *vec.b;
        multVec.b = c*vec.a + d *vec.b;
        
        return multVec;
    }

    void print() {
        printf("%f %f\n", (float)a, (float)b);
        printf("%f %f\n\n", (float)c, (float)d);
    }
    
    double getVal() {
        return a;
    }
    
private:
    T a;
    T b;
    T c;
    T d;
};

#endif /* defined(__OneDKalmanFilterExample___DMatrix__) */
