#include "main.h"
#include "math_u.h"

// SQUARE ROOT by Binary Search ------------------------------------------------
int s_sqrt(long data)
{   // sqrt algorithm with binary search
    
    int result = 0x3FFF;                // set binary search start
    int delta = 0x1FFF;                 // set start delta value
    long res2;                          // result ^ 2
    
    while(delta)
    {
        res2 = __builtin_mulss((int)result, (int)result);
        if(res2 > data)
            result = result - delta;
        else
            result = result + delta;
        
        delta = delta >> 1;
    }
    
    return result;
}

// RANDOM Number Generator -----------------------------------------------------
int n_rand(void)
{   // generate successive number
    static int rand_reg = 0xABCD;
    int partial;

    partial = (rand_reg & 0x0040) >> 6;
    partial = partial ^ ((rand_reg & 0x0080) >> 7);
    partial = partial ^ ((rand_reg & 0x8000) >> 15);
    rand_reg = rand_reg << 1;
    
    if(partial & 0x0001)
        rand_reg = rand_reg | 0x0001;

    return(rand_reg);
}
