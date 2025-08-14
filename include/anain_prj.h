#ifndef ANAIN_PRJ_H_INCLUDED
#define ANAIN_PRJ_H_INCLUDED

#include "hwdefs.h"

/* Here we specify how many samples are combined into one filtered result. Following values are possible:
*  - NUM_SAMPLES = 1: Most recent raw value is returned
*  - NUM_SAMPLES = 3: Median of last 3 values is returned
*  - NUM_SAMPLES = 9: Median of last 3 medians is returned
*  - NUM_SAMPLES = 12: Average of last 4 medians is returned
*/
#define NUM_SAMPLES 12
#define SAMPLE_TIME ADC_SMPR_SMP_7DOT5CYC //Sample&Hold time for each pin. Increases sample time, might increase accuracy

//Here you specify a list of analog inputs, see main.cpp on how to use them
#define ANA_IN_LIST \
   ANA_IN_ENTRY(ActCur_R, GPIOA, 0) \
   ANA_IN_ENTRY(ActCur_L, GPIOA, 1) \
   ANA_IN_ENTRY(Hall_R,   GPIOA, 2) \
   ANA_IN_ENTRY(Hall_L,   GPIOA, 3) \
   ANA_IN_ENTRY(Press,    GPIOA, 4) \
   ANA_IN_ENTRY(Sus_R,    GPIOA, 5) \
   ANA_IN_ENTRY(Sus_L,    GPIOA, 6) \
   ANA_IN_ENTRY(Comp_Tmp, GPIOA, 7) \
   ANA_IN_ENTRY(Vsense,   GPIOB, 0) \
   ANA_IN_ENTRY(Btn_Sig,  GPIOB, 1) \



#endif // ANAIN_PRJ_H_INCLUDED
