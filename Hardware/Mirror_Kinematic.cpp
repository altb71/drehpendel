#include "Mirror_Kinematic.h"
// THis class describes the mirror kinematic.
Mirror_Kinematic::Mirror_Kinematic(Data_Xchange *data) 
{
    this->m_data = data;
    screen_h = 102;
    screen_d = 4;
    dist_L = 16;
    inc_offset[0] = inc_offset[1] = 0;
    n = 1.585;  // index of refraction, Brechungsindex
    old_phi[0]=0.0;
    old_phi[1]=0.0;
    trafo_is_on = false;
    external_control = false;
    controller_is_on = true;
    max_num_it = 12;
    }
    
void Mirror_Kinematic::set_offsets(int16_t o1,int16_t o2)
{
    inc_offset[0] = o1;
    inc_offset[1] = o2;
}

// this is the transformation, transforming phi to xy values
bool Mirror_Kinematic::P2X(float *P, float *X)
{
    // calculation time 5.7usec on F446RE   
    float c1 = cosf(P[0]);
    float c2 = cosf(P[1]);
    float s1 = sinf(P[0]);
    float s2 = -sinf(2.0f*P[1]); // alwayx sin(2*phi2) is needed
                                // and: reverse the phi2 values!
    float n1x=c1;               // normal vector of 2nd laser beam (from mirror 1 -> mirror 2)
    float n1y=c1*s1;
    float n1z=s1*s1;
    float sq2=sqrt(.5);
    // i.e.: cos(2*x) = 2*cos(x)^2-1
    //float n2x = n1z*cosf(2.0f*P[1]) + n1x*sinf(2.0f*P[1]);
    float n2x = n1z*(2.0f*c2*c2-1.0f) + n1x*s2;  // normal vector of 3rd laser beam (mirror 2 -> screen)
    float n2y = n1y;
    float a1 = sq2*c2 - sq2*s1;
    float a2 = sq2*c2 + sq2*s1;
    float dum1 = n1x * a1 - n1z*a2;
    float n2z = n1x*(2.0f*c2*c2-1.0f) - n1z*s2;
    if(dum1*n2z == 0)
        return false;
    float dad = dist_L * a1/dum1;
    float Q2x = dad * n1x - dist_L;
    float Q2y = dad * n1y;
    float Q2z = dad * n1z;
    float x = atanf(n2x/n2z)/n;
    float y = atanf(n2y/n2z)/n;
    float dx = screen_d * x/sqrt(1-x*x);
    float dy = screen_d * y/sqrt(1-y*y);
    X[0] =  Q2x + (n2x*(screen_h - Q2z))/n2z - dx;
    X[1] =  Q2y + (n2y*(screen_h - Q2z))/n2z - dy;
    
    return true;
    }
// this is the trafo from xy -> phi1,phi2, not possible in an analytical form, use
// GN-iteration with a constant Jacobian
bool Mirror_Kinematic::X2P(float *X, float *P)
{
    float J12 = 0.0090517133f;
    float J21 = -0.0052923231f;     // on motor angle co-ordinates, we have a negative sensitivity
    float Xn[2];
    float dx = 1e4;
    float dy = 1e4;
    P[0] = old_phi[0];
    P[1] = old_phi[1];
    uint8_t k = 0;
    do
        {
            if(!P2X(P,Xn) || k==max_num_it)
                return false;
            dx = Xn[0]-X[0];
            dy = Xn[1]-X[1];
            P[0] -= J12 * dy;
            P[1] -= J21 * dx;
        }
        while((dx*dx+dy*dy) > 1e-3 && ++k<20);
    m_data->num_it = k;
    old_phi[0] = P[0];
    old_phi[1] = P[1];
    return true;
}