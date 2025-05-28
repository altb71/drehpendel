/*
 * EncoderCounterIndex.cpp
 * Copyright (c) 2018, ZHAW
 * All rights reserved.
 *
 *  Created on: 13.08.2018
 *      Author: Marcel Honegger
 */

#include <stdint.h>
#include "EncoderCounter.h"
#include "EncoderCounterIndex.h"

using namespace std;

/**
 * Creates an object with an interrupt service routine to catch
 * the current encoder position when an index pulse is received.
 */
EncoderCounterIndex::EncoderCounterIndex(EncoderCounter& encoderCounter, InterruptIn& channelIndex) : encoderCounter(encoderCounter), channelIndex(channelIndex) {
    
    // attach interrupt
    channelIndex.rise(callback(this, &EncoderCounterIndex::rise));
    is_referenced = false;
}

EncoderCounterIndex::~EncoderCounterIndex() {}

int32_t EncoderCounterIndex::getPositionAtIndexPulse() {
    
    return positionAtIndexPulse;
}

void EncoderCounterIndex::rise() {
    
    positionAtIndexPulse = encoderCounter;
    channelIndex.rise(NULL);
    is_referenced = true;
}


