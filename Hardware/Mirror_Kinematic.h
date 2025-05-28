#ifndef MIRROR_KINEMATIC_H_
#define MIRROR_KINEMATIC_H_

#include "mbed.h"
#include "data_structs.h"


class Mirror_Kinematic
{
public:
    Mirror_Kinematic(Data_Xchange *);
    void set_offsets(int16_t,int16_t);
    int16_t inc_offset[2];
    bool P2X(float *,float *);
    bool X2P(float *,float *);
    bool trafo_is_on;
    bool external_control;
    bool controller_is_on;
private:
    float old_phi[2];
    Data_Xchange *m_data;
    uint8_t max_num_it;
    float screen_h;
    float  n ;          //        Brechungsindex
    float dist_L;       // distance laser0 to 2nd axis
    float screen_d;     // thickness of screen / mmm
    };
    
#endif
    

