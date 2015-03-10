//
//  TwoDKalmanBeatFilter.cpp
//  beatseeker~
//
//  Created by Andrew Robertson on 06/03/2015.
//
//

#include "TwoDKalmanTempoFilter.h"

//
//  TwoDKalmanTempoFilter.cpp
//  OneDKalmanFilterExample
//
//  Created by Andrew Robertson on 05/03/2015.
//
//
const bool printing = true;

#include "TwoDKalmanTempoFilter.h"
#include <Math.h>


TwoDKalmanTempoFilter::TwoDKalmanTempoFilter() : estimate (0.f, 500.f),
                                                A(1.0f, 1.0f, 0.f, 1.0f),
                                                kalmanGain(0.4f, 0.f, 0.f, 0.4f),
                                                covarianceQ(25.0f, 0.f, 0.f, 0.8f),
                                                covarianceR(100.0f, 0.f, 0.f, 16.f),
                                                Pk(15.f, 0.f, 0.f, 9.f),
                                                Pk_hat(15.f, 0.f, 0.f, 9.f),
                                                dummy(15.f, 0.5, 0.3f, 9.4f)
{
    double phase = 0.f;
    double tempo = 500.f;
    init(phase, tempo);
    
}



void TwoDKalmanTempoFilter::init(double phase, double tempo){
    
    //initCovarianceParams();//reset the matrices
    
   // initMatrix(kalmanGain, 0.5f, 0.f, 0.f, .5f);
   // initMatrix(covarianceQ, 25.0f, 0.f, 0.f, 0.64f);
   // initMatrix(covarianceR, 100.0f, 0.f, 0.f, 16.f);
   // initMatrix(Pk, 15.f, 0.f, 0.f, 3.f);
   // initMatrix(Pk_hat, 15.f, 0.f, 0.f, 3.f); // CRASH IF WE DO THIS - WHY?
    
    estimate.init(phase, tempo);
    
    //printf("anrfilter: set up filter to phase %f, tempo %f\n", phase, tempo);
    HSTREAM << "TWO_D_KALMAN RE-INIT phase " << phase << ", tempo " << tempo << std::endl;
    
    
}

void TwoDKalmanTempoFilter::initCovarianceParams() {
    initMatrix(kalmanGain, 0.5f, 0.f, 0.f, .5f);
    initMatrix(covarianceQ, 25.0f, 0.f, 0.f, 0.64f);
    initMatrix(covarianceR, 100.0f, 0.f, 0.f, 16.f);
    initMatrix(Pk, 15.f, 0.f, 0.f, 3.f);
    //initMatrix(Pk_hat, 15.f, 0.f, 0.f, 3.f);
}

void TwoDKalmanTempoFilter::initMatrix(TwoDMatrix<double>& matrix, double a, double b, double c, double d) {
//    matrix.init(a, b, c, d);
    matrix.reset(a, b, c, d);
}

void TwoDKalmanTempoFilter::setProcessNoise(double& sigmaX, double& sigmaV) {
    covarianceQ.init(sigmaX*sigmaX, 0.f, 0.f, sigmaV*sigmaV);
    
    //printf("cov Q\n");
    //covarianceQ.print();
    
}

void TwoDKalmanTempoFilter::setMeasurementNoise(double& sigmaX, double& sigmaV) {
    covarianceR.init(sigmaX*sigmaX, 0.f, 0.f, sigmaV*sigmaV);
   
    // printf("cov R\n");
   // covarianceR.print();
}

/*
void TwoDKalmanTempoFilter::setPk(double& X, double& V) {
    Pk.init(X, 0.f, 0.f, V);
    Pk_hat.init(X, 0.f, 0.f, V);
    //printf("Pk\n");
    //Pk.print();
}
*/
void TwoDKalmanTempoFilter::newMeasurement(const double& phaseMeasurement,
                                           const double& tempoMeasurement) {
    
    predictionUpdate();//phaseMeasurement);
    
    if (printing) {
        //printf("anrfilter: prediction\n");
        estimate.print();
    }
    
    measurementUpdate(phaseMeasurement, tempoMeasurement);
}

void TwoDKalmanTempoFilter::predictionUpdate() {
    
    estimate = A.times(estimate);//and no control noise B u_k
    
    TwoDMatrix<double> tranA = A.Transpose();
    
    if (printing) {
        Pk.print();
        tranA.print();
        (Pk*tranA).print();
    }
    
    TwoDMatrix<double> nextPk = A*(Pk*tranA);
    
    if (printing)
        nextPk.print();
    
    nextPk = nextPk + covarianceQ;
    
    //if (printing)
    //printf("Next Pk\n");
    //nextPk.print();
    
    Pk_hat = nextPk;
    
}


void TwoDKalmanTempoFilter::updateKalmanGain() {
    TwoDMatrix<double> denominator = Pk_hat + covarianceR;
    
    if (printing) {
        printf("Pk_hat\n");
        Pk_hat.print();
        printf("Pk_hat+R\n");
        denominator.print();
        printf("denom inverse:\n");
        denominator.inverse().print();
        
        kalmanGain.print();
    }
    
    kalmanGain = Pk_hat*(denominator.inverse());
    
    if (printing) {
        printf("new kalman gain\n");
        kalmanGain.print();
    }
    
}

void TwoDKalmanTempoFilter::updateEstimate(TwoDVector<double>& newMeasurement) {
    TwoDVector<double> diff = (newMeasurement- estimate);
    
    if (printing) {
        printf("new observation %f and tempo %f\nestimate is:\n", newMeasurement.a, newMeasurement.b);
        estimate.print();
        (newMeasurement - estimate).print();
        
        
        
        printf("new kalman gain\n");
        kalmanGain.print();
        printf("diff\n");
        diff.print();
        printf("product\n");
        (kalmanGain*diff).print();
    }
    
    estimate = estimate + (kalmanGain*(newMeasurement - estimate));
    
    if (printing) {
        printf("new estimate\n");
        estimate.print();
    }
}

void TwoDKalmanTempoFilter::updatePk() {
    TwoDMatrix<double> Identity(1.f, 0.f, 0.f, 1.f);
    if (printing) {
        
        printf("Pk\n");
        Pk.print();
        
        printf("Pk_hat\n");
        Pk_hat.print();
        
        
        
        printf("new kalman gain\n");
        kalmanGain.print();
        (Identity - kalmanGain).print();
    }
    
    Pk = (Identity - kalmanGain)*Pk_hat;
    
    if (printing) {
        printf("updated Pk\n");
        Pk.print();
    }
}

void TwoDKalmanTempoFilter::measurementUpdate(double phaseMeasurement, double tempoMeasurement) {
    updateKalmanGain();
    
    TwoDVector<double> newObservation(phaseMeasurement, tempoMeasurement);
    updateEstimate(newObservation);
    
    const bool estimateNotClose = fabs(phaseEstimate() - phaseMeasurement) > 100 || fabs(tempoMeasurement - tempoEstimate()) > 15;
    
    if (estimateNotClose) {
        HSTREAM << "TWO_D OUT AT " << phaseMeasurement << " vs " << phaseEstimate() << std::endl;
        HSTREAM << ", tempo" << tempoMeasurement << " vs " << tempoEstimate() << std::endl;
        init(phaseMeasurement, tempoMeasurement);
    } else {
        HSTREAM << "TWO_D estimate: " << phaseEstimate() << ", tempo " << tempoEstimate() << std::endl;
        updatePk();
    }
    //printf("anrFilter: after update\n");
    //estimate.print();
}


void TwoDKalmanTempoFilter::simpleExample() {
    init(2000, 500);//initiliase at 2000ms position and 500ms per beat (i.e. beat period)
    printf("our estimate starts out at 2000ms position and 500 ms beat period\n");
    
    
    //then we see some data
     newMeasurement(2510., 504.5);
     newMeasurement(3021, 513.);
     newMeasurement(3543., 521.);
     newMeasurement(4045, 507.);
}

