/*
 * This file is part of the stm32-template project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/desig.h>
#include "stm32_can.h"
#include "canmap.h"
#include "cansdo.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "hwinit.h"
#include "anain.h"
#include "param_save.h"
#include "my_math.h"
#include "errormessage.h"
#include "printf.h"
#include "stm32scheduler.h"
#include "terminalcommands.h"
#include "my_string.h"
#define PRINT_JSON 0

extern "C" void __cxa_pure_virtual()
{
    while (1);
}

static Stm32Scheduler* scheduler;
static CanHardware* can;
static CanMap* canMap;
static CanSdo* canSdo;


int uauxGain = 224; //!! hard coded AUX gain
int analogGain = 620;
int R_Lock = 0;
int L_Lock = 0;
int R_Count = 0;
int L_Count = 0;
uint8_t Gcount = 0x00;

static void Ms1Task(void)
{
	
}

static void Ms10Task(void)
{
    //Set timestamp of error message
    ErrorMessage::SetTime(rtc_get_counter_val());
}


//sample 100ms task
static void Ms100Task(void)
{
    DigIo::led_out.Toggle();
    iwdg_reset();
    Param::SetFloat(Param::Vsense, 	 ((float)AnaIn::Vsense.Get()) / uauxGain);
	Param::SetFloat(Param::ActCur_R, ((float)AnaIn::ActCur_R.Get()));
	Param::SetFloat(Param::ActCur_L, ((float)AnaIn::ActCur_L.Get()));
	Param::SetFloat(Param::Hall_R, 	 ((float)AnaIn::Hall_R.Get()) / analogGain);
	Param::SetFloat(Param::Hall_L, 	 ((float)AnaIn::Hall_L.Get()) / analogGain);
	Param::SetFloat(Param::Press, 	 (float)AnaIn::Press.Get());
	Param::SetFloat(Param::Pressure, (((float)AnaIn::Press.Get()-162)/9.81));
	if ((Param::GetInt(Param::Pressure)) < 0) Param::SetFloat(Param::Pressure, 0);
	Param::SetFloat(Param::Sus_R, 	 (float)AnaIn::Sus_R.Get());
    Param::SetFloat(Param::Sus_L, 	 (float)AnaIn::Sus_L.Get());
	Param::SetFloat(Param::Comp_Tmp, (float)AnaIn::Comp_Tmp.Get());
	Param::SetFloat(Param::Btn_Sig,  (float)AnaIn::Btn_Sig.Get());
	float cpuLoad = scheduler->GetCpuLoad();
	Param::SetFloat(Param::cpuload, cpuLoad / 10);
	canMap->SendAll();
	Can_Tasks();
	eBrake_Control();
	
}

static void Ms200Task(void)
{
	if((Param::GetInt(Param::Vsense)) > 10.00) 
	{
		DigIo::Aux_EN.Set();
		Param::SetInt(Param::Aux_EN, 1);
	}
	else 
	{
		DigIo::Aux_EN.Clear();
		Param::SetInt(Param::Aux_EN, 0);
	}
	
}

void Analog_Outputs()
{	
	switch (Param::GetInt(Param::Air_Compressor))
	{
		case 0:
			DigIo::Compressor.Clear();
			Param::SetInt(Param::Compressor, 0);
			break;
		case 1:
			DigIo::Compressor.Set();
			Param::SetInt(Param::Compressor, 1);
			break;
	}
	
	switch (Param::GetInt(Param::Rear_Right_Valve))
	{
		case 0:
			DigIo::RR_Vlv.Clear();
			Param::SetInt(Param::RR_Vlv, 0);
			break;
		case 1:
			DigIo::RR_Vlv.Set();
			Param::SetInt(Param::RR_Vlv, 1);
			break;
	}
	
	switch (Param::GetInt(Param::Rear_Left_Valve))
	{
		case 0:
			DigIo::RL_Vlv.Clear();
			Param::SetInt(Param::RL_Vlv, 0);
			break;
		case 1:
			DigIo::RL_Vlv.Set();
			Param::SetInt(Param::RL_Vlv, 1);
			break;
	}
	
	switch (Param::GetInt(Param::Ambient_Valve))
	{
		case 0:
			DigIo::Amb_Vlv.Clear();
			Param::SetInt(Param::Amb_Vlv, 0);
			break;
		case 1:
			DigIo::Amb_Vlv.Set();
			Param::SetInt(Param::Amb_Vlv, 1);
			break;
	}
	
	switch (Param::GetInt(Param::Reverse1_Valve))
	{
		case 0:
			DigIo::Rvrs1_Vlv.Clear();
			Param::SetInt(Param::Rvrs1_Vlv, 0);
			break;
		case 1:
			DigIo::Rvrs1_Vlv.Set();
			Param::SetInt(Param::Rvrs1_Vlv, 1);
			break;
	}
	
	switch (Param::GetInt(Param::Reverse2_Valve))
	{
		case 0:
			DigIo::Rvrs2_Vlv.Clear();
			Param::SetInt(Param::Rvrs2_Vlv, 0);
			break;
		case 1:
			DigIo::Rvrs2_Vlv.Set();
			Param::SetInt(Param::Rvrs2_Vlv, 1);
			break;
	}
}

void Can_Tasks()
{
	
	uint8_t bytes[8];
    bytes[0]= 0xFF;
    bytes[1]= 0x00;
    bytes[2]= Gcount;
    bytes[3]= 0xCD;
    can->Send(0x1D2, bytes, 4); //Send on CAN1	
	Gcount = Gcount + 0x01;
   
   if (Gcount==0xFF)
   {
      Gcount=0x00;
   }
   
}
void DecodeCAN(int id, uint32_t* data)
{
	uint8_t* bytes = (uint8_t*)data;
	switch (id)
	{
		case 0x1AE:
			Param::SetInt(Param::opmode, bytes[0]);
			break;
		default:
						   
									 
			break;
	}
			
}
	

void PinIntialization()
{
	DigIo::FOR.Clear();
	DigIo::REV.Clear();
	DigIo::R1_EN.Clear();
	DigIo::R2_EN.Clear();
	DigIo::L1_EN.Clear();
	DigIo::L2_EN.Clear();
	DigIo::RR_Vlv.Clear();
	DigIo::RL_Vlv.Clear();
	DigIo::Amb_Vlv.Clear();
	DigIo::Rvrs1_Vlv.Clear();
	DigIo::Rvrs2_Vlv.Clear();
	DigIo::Compressor.Clear();
	DigIo::Aux_EN.Clear();
	DigIo::R_LED.Clear();
	DigIo::G_LED.Clear();
}

void eBrake_Control()
{
	switch (Param::GetInt(Param::Calibers_Control_Mode))
	{
		case 0:
			switch (Param::GetInt(Param::Calibers))
			{
				case 0:
					DigIo::R1_EN.Clear();
					DigIo::R2_EN.Clear();
					DigIo::L1_EN.Clear();
					DigIo::L2_EN.Clear();
				break;
				case 1: 
					DigIo::R1_EN.Set();
					DigIo::R2_EN.Set();
					DigIo::L1_EN.Clear();
					DigIo::L2_EN.Clear();
					switch (Param::GetInt(Param::LOCK))
					{
						case 0:
							if ((R_Lock == 1) && (Param::GetInt(Param::Hall_R)) > 1)
							{
								DigIo::FOR.Set();
								DigIo::REV.Clear();
								R_Count++;
								if (R_Count == 14) 
								{
									Disable_Calibers();
									R_Count = 0;
									R_Lock = 0;
									Param::SetInt(Param::Status, 0);
									DigIo::R_LED.Clear();
									Param::SetInt(Param::R_LED, 0);
									DigIo::G_LED.Set();
									Param::SetInt(Param::G_LED, 1);
								}
							}
						break;
						case 1:
							if ((R_Lock == 0) && (Param::GetInt(Param::Hall_R)) > 1)
							{
								DigIo::FOR.Clear();
								DigIo::REV.Set();
								R_Count++;
								if (R_Count == 15) 
								{
									Disable_Calibers();
									R_Count = 0;
									R_Lock = 1;
									Param::SetInt(Param::Status, 1);
									DigIo::R_LED.Set();
									Param::SetInt(Param::R_LED, 1);
									DigIo::G_LED.Clear();
									Param::SetInt(Param::G_LED, 0);
								}
							}
						break;
					}
				break;
				case 2: 
					DigIo::R1_EN.Clear();
					DigIo::R2_EN.Clear();
					DigIo::L1_EN.Set();
					DigIo::L2_EN.Set();
					switch (Param::GetInt(Param::LOCK))
					{
						case 0:
							if ((L_Lock == 1) && (Param::GetInt(Param::Hall_L)) > 1)
							{
								DigIo::FOR.Set();
								DigIo::REV.Clear();
								L_Count++;
								if (L_Count == 14) 
								{
									Disable_Calibers();
									L_Count = 0;
									L_Lock = 0;
									Param::SetInt(Param::Status, 0);
									DigIo::R_LED.Clear();
									Param::SetInt(Param::R_LED, 0);
									DigIo::G_LED.Set();
									Param::SetInt(Param::G_LED, 1);
								}
							}
						break;
						case 1:
							if ((L_Lock == 0) && (Param::GetInt(Param::Hall_L)) > 1)
							{
								DigIo::FOR.Clear();
								DigIo::REV.Set();
								L_Count++;
								if (L_Count == 15) 
								{
									Disable_Calibers();
									L_Count = 0;
									L_Lock = 1;
									Param::SetInt(Param::Status, 1);
									DigIo::R_LED.Set();
									Param::SetInt(Param::R_LED, 1);
									DigIo::G_LED.Clear();
									Param::SetInt(Param::G_LED, 0);
								}
							}
						break;
					}
				break;
				case 3: 
					DigIo::R1_EN.Set();
					DigIo::R2_EN.Set();
					DigIo::L1_EN.Set();
					DigIo::L2_EN.Set();
					switch (Param::GetInt(Param::LOCK))
					{
						case 0:
							if ((R_Lock == 1) && (Param::GetInt(Param::Hall_R)) > 1)
							{
								DigIo::FOR.Set();
								DigIo::REV.Clear();
								R_Count++;
								if (R_Count == 14) 
								{
									Disable_Calibers();
									R_Count = 0;
									R_Lock = 0;
									Param::SetInt(Param::Status, 0);
									DigIo::R_LED.Clear();
									Param::SetInt(Param::R_LED, 0);
									DigIo::G_LED.Set();
									Param::SetInt(Param::G_LED, 1);
								}
							}
						break;
						case 1:
							if ((R_Lock == 0) && (Param::GetInt(Param::Hall_R)) > 1)
							{
								DigIo::FOR.Clear();
								DigIo::REV.Set();
								R_Count++;
								if (R_Count == 15) 
								{
									Disable_Calibers();
									R_Count = 0;
									R_Lock = 1;
									Param::SetInt(Param::Status, 1);
									DigIo::R_LED.Set();
									Param::SetInt(Param::R_LED, 1);
									DigIo::G_LED.Clear();
									Param::SetInt(Param::G_LED, 0);
								}
							}
						break;
					}
				break;
			}
		break;
		case 1:
			DigIo::R1_EN.Set();
			DigIo::R2_EN.Set();
			DigIo::L1_EN.Set();
			DigIo::L2_EN.Set();
			switch (Param::GetInt(Param::LOCK))
			{
				case 0:
					if ((R_Lock == 1) && (Param::GetInt(Param::Hall_R)) > 1)
					{
						DigIo::FOR.Set();
						DigIo::REV.Clear();
						R_Count++;
						if (R_Count == 30) 
						{
							Disable_Calibers();
							R_Count = 0;
							R_Lock = 0;
							Param::SetInt(Param::Status, 0);
							DigIo::R_LED.Set();
							Param::SetInt(Param::R_LED, 1);
							DigIo::G_LED.Set();
							Param::SetInt(Param::G_LED, 1);
						}
					}
				break;
				case 1:
					if ((R_Lock == 0) && (Param::GetInt(Param::Hall_R)) > 1)
					{
						DigIo::FOR.Clear();
						DigIo::REV.Set();
						R_Count++;
						if (R_Count == 22) 
						{
							Disable_Calibers();
							R_Count = 0;
							R_Lock = 1;
							Param::SetInt(Param::Status, 1);
							DigIo::R_LED.Set();
							Param::SetInt(Param::R_LED, 1);
							DigIo::G_LED.Set();
							Param::SetInt(Param::G_LED, 1);
						}
					}
				break;
			}
		break;
	}						
}

void Disable_Calibers()
{
	DigIo::FOR.Clear();
	DigIo::REV.Clear();
	DigIo::R1_EN.Clear();
	DigIo::R2_EN.Clear();
	DigIo::L1_EN.Clear();
	DigIo::L2_EN.Clear();
}



//sample 10 ms task

static void SetCanFilters()
{
	can->RegisterUserMessage(0x605); //Can SDO
	can->RegisterUserMessage(0x1AE); //OI Control Message
}
	
static bool CanCallback(uint32_t id, uint32_t data[2], uint8_t dlc) //This is where we go when a defined CAN message is received.
{
    dlc = dlc;
	if (Param::GetInt(Param::CanCtrl)) DecodeCAN(id,data);		
	return false;
}

/** This function is called when the user changes a parameter */


void Param::Change(Param::PARAM_NUM paramNum)
{
    switch (paramNum)
    {
		case Param::NodeId:
			canSdo->SetNodeId(Param::GetInt(Param::NodeId));
			break;
		case Param::Air_Compressor:
			Analog_Outputs();
			break;
		case Param::Rear_Right_Valve:
			Analog_Outputs();
			break;
		case Param::Rear_Left_Valve:
			Analog_Outputs();
			break;
		case Param::Ambient_Valve:
			Analog_Outputs();
			break;
		case Param::Reverse1_Valve:
			Analog_Outputs();
			break;
		case Param::Reverse2_Valve:
			Analog_Outputs();
			break;
		case Param::CanCtrl:
			SetCanFilters();
			break;
	default:
        //Handle general parameter changes here. Add paramNum labels for handling specific parameters
		
        break;
    }
}

//Whichever timer(s) you use for the scheduler, you have to
//implement their ISRs here and call into the respective scheduler
extern "C" void tim2_isr(void)
{
    scheduler->Run();
}

extern "C" int main(void)
{
    extern const TERM_CMD termCmds[];

    clock_setup(); //Must always come first
    rtc_setup();
    ANA_IN_CONFIGURE(ANA_IN_LIST);
    DIG_IO_CONFIGURE(DIG_IO_LIST);
	AnaIn::Start(); //Starts background ADC conversion via DMA
	write_bootloader_pininit(); //Instructs boot loader to initialize certain pins
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_CAN1_REMAP_PORTB);//Remap CAN pins to Portb alt funcs.
    nvic_setup(); //Set up some interrupts
    parm_load(); //Load stored parameters
    Stm32Scheduler s(TIM2); //We never exit main so it's ok to put it on stack
    scheduler = &s;
    //Initialize CAN1, including interrupts. Clock must be enabled in clock_setup()
    Stm32Can c(CAN1, CanHardware::Baud500,true);
    FunctionPointerCallback cb(CanCallback, SetCanFilters);
	CanMap cm(&c);
	CanSdo sdo(&c, &cm);
	sdo.SetNodeId(Param::GetInt(Param::NodeId));
//store a pointer for easier access
    can = &c;
	canMap = &cm;
    canSdo = &sdo;
    c.AddCallback(&cb);
    Terminal t(USART3, termCmds);
    TerminalCommands::SetCanMap(canMap);
	
	s.AddTask(Ms1Task, 1);
    s.AddTask(Ms10Task, 10);
    s.AddTask(Ms100Task, 100);
	s.AddTask(Ms200Task, 200);

    Param::SetInt(Param::version, 4);
    Param::Change(Param::PARAM_LAST); //Call callback one for general parameter propagation
	PinIntialization();
	SetCanFilters();
	if (Param::GetInt(Param::CanCtrl) == 0)
	{
		
	}
	

    while(1)
    {
        char c = 0;
        t.Run();
        if (sdo.GetPrintRequest() == PRINT_JSON)
        {
            TerminalCommands::PrintParamsJson(&sdo, &c);
        }
    }

    return 0;
}

