#include "observer.h"
observer::observer(float Ts)   // constructor
{
    this->Ts = Ts;
    init_matrices();
}
observer::~observer() {}

// init the A, B, C, H Matrices
void observer::init_matrices()
{
    // Aufgabe 2.2
}
// calculate the observed states -> calc sum, integrate, store in x_hat
void observer::do_step(float u,float y_meas)
{
    // Aufgabe 2.3, 2.4
}
// to the integration dxdt -> x, use trapezoidal form
void observer::integrate_states()
{
    // Aufgabe 2.4
}
// get the observed states
Matrix<float,N,1> observer::get_x_obsv()
{
    return x_hat;
}