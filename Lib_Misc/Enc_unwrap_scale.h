#pragma once

#include <cstdint>
#include <mbed.h>

class Enc_unwrap_scale
{
public:

    Enc_unwrap_scale(){};
    Enc_unwrap_scale(uint16_t T, uint8_t bits);
    
    float operator()(short inc) {
        return doStep(inc);
    }
    
    virtual     ~Enc_unwrap_scale();
    
    float       doStep(int32_t inc);

private:

        int32_t incPast;
        int32_t max_delta;
        double  inc2rad;
        int32_t incPerRev;
        int32_t num_overflows;
};
