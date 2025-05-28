#pragma once


typedef struct{
    float sens_phi[2];
    float est_xy[2];
    float sens_Vphi[2];
    float cntrl_phi_des[2];
    float cntrl_Vphi_des[2];
    float cntrl_xy_des[2];
    float i_des[2];         // desired currents
    float wMot[2];          // desired speeds (rad/s)
    bool laser_on;
    uint8_t num_it;
    }Data_Xchange;
    
