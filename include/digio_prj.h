#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "hwdefs.h"

/* Here you specify generic IO pins, i.e. digital input or outputs.
 * Inputs can be floating (INPUT_FLT), have a 30k pull-up (INPUT_PU)
 * or pull-down (INPUT_PD) or be an output (OUTPUT)
*/

#define DIG_IO_LIST \
    DIG_IO_ENTRY(led_out, 		GPIOC, GPIO13, 	PinMode::OUTPUT)\
    DIG_IO_ENTRY(FOR,     		GPIOB, GPIO6, 	PinMode::OUTPUT)\
	DIG_IO_ENTRY(REV,     		GPIOB, GPIO7, 	PinMode::OUTPUT)\
	DIG_IO_ENTRY(R1_EN,     	GPIOA, GPIO8, 	PinMode::OUTPUT)\
	DIG_IO_ENTRY(R2_EN,     	GPIOA, GPIO9, 	PinMode::OUTPUT)\
	DIG_IO_ENTRY(L1_EN,     	GPIOA, GPIO10, 	PinMode::OUTPUT)\
	DIG_IO_ENTRY(L2_EN,     	GPIOA, GPIO11, 	PinMode::OUTPUT)\
	DIG_IO_ENTRY(RR_Vlv,    	GPIOA, GPIO12, 	PinMode::OUTPUT)\
	DIG_IO_ENTRY(RL_Vlv,    	GPIOA, GPIO15, 	PinMode::OUTPUT)\
	DIG_IO_ENTRY(Amb_Vlv,   	GPIOB, GPIO3, 	PinMode::OUTPUT)\
	DIG_IO_ENTRY(Rvrs1_Vlv,		GPIOB, GPIO4, 	PinMode::OUTPUT)\
    DIG_IO_ENTRY(Rvrs2_Vlv, 	GPIOB, GPIO5, 	PinMode::OUTPUT)\
	DIG_IO_ENTRY(Compressor,	GPIOB, GPIO15, 	PinMode::OUTPUT)\
	DIG_IO_ENTRY(Aux_EN,		GPIOB, GPIO12, 	PinMode::OUTPUT)\
	DIG_IO_ENTRY(R_LED,    		GPIOB, GPIO13, 	PinMode::OUTPUT)\
	DIG_IO_ENTRY(G_LED,    		GPIOB, GPIO14, 	PinMode::OUTPUT)\
	
#endif // PinMode_PRJ_H_INCLUDED