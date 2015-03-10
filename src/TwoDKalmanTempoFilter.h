//
//  TwoDKalmanBeatFilter.h
//  beatseeker~
//
//  Created by Andrew Robertson on 06/03/2015.
//
//

#ifndef __beatseeker___TwoDKalmanBeatFilter__
#define __beatseeker___TwoDKalmanBeatFilter__

#include "TwoDMatrix.h"

#include "DebugLogger.h"

#include <stdio.h>

class TwoDKalmanTempoFilter {
public:
    
        TwoDKalmanTempoFilter();
        
        TwoDVector<double> estimate;
        TwoDMatrix<double> A;
        TwoDMatrix<double> kalmanGain;
        TwoDMatrix<double> covarianceQ;
        TwoDMatrix<double> covarianceR;
        TwoDMatrix<double> Pk;
        TwoDMatrix<double> Pk_hat;
        TwoDMatrix<double> dummy;
        
        void init(double phase, double tempo);
        void initCovarianceParams();
    
        void initMatrix(TwoDMatrix<double>& matrix, double a, double b, double c, double d);
    
        void setProcessNoise(double& sigmaX, double& sigmaV);
        void setMeasurementNoise(double& sigmaX, double& sigmaV);
        
        void newMeasurement(const double& phaseMeasurement, const double& tempoMeasurement);
        void predictionUpdate();
        void measurementUpdate(double phaseMeasurement, double tempoMeasurement);
        void updateKalmanGain();
        void updatePk();
        void updateEstimate(TwoDVector<double>& newMeasurement);
        
 //       void setPk(double& X, double& V);
    
        
        double phaseEstimate() {
            return estimate.a;
        }
    
        double tempoEstimate() {
            return estimate.b;
        }
    
    void simpleExample();
};

#endif /* defined(__beatseeker___TwoDKalmanBeatFilter__) */
