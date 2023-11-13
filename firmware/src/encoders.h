#pragma once

#include <ch.h>
#include <hal.h>

class Encoder
{
public:
    Encoder(stm32_tim_t* tim);
    void init(bool inverted);
    int32_t get_value();
    void reset();

private:
    stm32_tim_t* tim;
    int32_t offset;
    
    // is the counter in the lower or upper half? Will be used to detect overflow/underflow
    bool lower_half;
};
