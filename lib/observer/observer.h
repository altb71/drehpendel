#pragma once
/* observer
*/
#include <cstdint>
#include "mbed.h"
#include <Eigen/Dense>
#define     N       2           // dimension of system
#define     N_meas  1           // number measurements

using namespace Eigen;
class observer
{
public:
    observer(){};               // default constructor
    observer(float);            // standard constructor
    virtual ~observer();        // deconstructor
    void init_matrices(void);   // initialize the relevant Mats with fixed values (A << ...)
    void do_step(float,float);  // calculate the derivative (at the "sum"- point in the observer)
    void integrate_states();    // do time-integration
    Matrix<float,N,1> get_x_obsv(); // get observed state x_hat
private:
    Matrix<float,N_meas,N> C;
    Matrix<float,N,N> A;
    Matrix<float,N,1> B;
    Matrix<float,N,N_meas> H;
    Matrix<float,N,1> x_hat,u_km1,dxdt;
    Matrix<float,N_meas,1> y_obsv;
    float Ts;
};
