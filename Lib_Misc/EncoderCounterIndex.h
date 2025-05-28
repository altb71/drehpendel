/*
 * EncoderCounterIndex.h
 * Copyright (c) 2018, ZHAW
 * All rights reserved.
 *
 *  Created on: 13.08.2018
 *      Author: Marcel Honegger
 */

#ifndef ENCODER_COUNTER_INDEX_H_
#define ENCODER_COUNTER_INDEX_H_

#include <cstdlib>
#include <stdint.h>
#include <mbed.h>

class EncoderCounter;

/**
 * The <code>EncoderCounterIndex</code> class implements an interrupt service routine
 * to catch the current encoder position when an index pulse is received.
 */
class EncoderCounterIndex {
    
    public:
        
                    EncoderCounterIndex(EncoderCounter& encoderCounter, InterruptIn& channelIndex);
        virtual     ~EncoderCounterIndex();
        int32_t     getPositionAtIndexPulse();
        int32_t     positionAtIndexPulse;
        bool        is_referenced;
        
    private:
        
        EncoderCounter&         encoderCounter;
        InterruptIn&            channelIndex;
        void                    rise();
};

#endif /* ENCODER_COUNTER_INDEX_H_ */


