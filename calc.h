
#ifndef _CALC_H_
#define _CALC_H_
#pragma once

uint8_t countSetBits(uint16_t n) 
{ 
    uint8_t count = 0; 
    while (n) 
    { 
      n &= (n-1); 
      count++; 
    } 
    return count; 
} 

uint8_t getdigit(uint8_t num, uint8_t n)
{
    uint8_t r, t1, t2;
 
    t1 = pow(10, n+1);
    r = num % t1;
 
    if (n > 0)
    {
        t2 = pow(10, n);
        r = r / t2;
    }
 
    return r;
}

#endif