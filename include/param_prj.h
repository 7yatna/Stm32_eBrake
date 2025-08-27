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

/* This file contains all parameters used in your project
 * See main.cpp on how to access them.
 * If a parameters unit is of format "0=Choice, 1=AnotherChoice" etc.
 * It will be displayed as a dropdown in the web interface
 * If it is a spot value, the decimal is translated to the name, i.e. 0 becomes "Choice"
 * If the enum values are powers of two, they will be displayed as flags, example
 * "0=None, 1=Flag1, 2=Flag2, 4=Flag3, 8=Flag4" and the value is 5.
 * It means that Flag1 and Flag3 are active -> Display "Flag1 | Flag3"
 *
 * Every parameter/value has a unique ID that must never change. This is used when loading parameters
 * from flash, so even across firmware versions saved parameters in flash can always be mapped
 * back to our list here. If a new value is added, it will receive its default value
 * because it will not be found in flash.
 * The unique ID is also used in the CAN module, to be able to recover the CAN map
 * no matter which firmware version saved it to flash.
 * Make sure to keep track of your ids and avoid duplicates. Also don't re-assign
 * IDs from deleted parameters because you will end up loading some random value
 * into your new parameter!
 * IDs are 16 bit, so 65535 is the maximum
 */

//Define a version string of your firmware here
#define VER 0.02AK

/* Entries must be ordered as follows:
   1. Saveable parameters (id != 0)
   2. Temporary parameters (id = 0)
   3. Display values
 */
//Next param id (increase when adding new parameter!): 33
//Next value Id: 2028
/*              category     name         unit       min     max     default id */
#define PARAM_LIST \
    PARAM_ENTRY(CAT_HBridge,      Calibers_Control_Mode, 		MODE,       0,      1,  	1,   1 ) \
	PARAM_ENTRY(CAT_HBridge,      LOCK,   					 	OFFON,      0,     	1, 		1,   2 ) \
	PARAM_ENTRY(CAT_HBridge,      Calibers, 					CALBRS,     0,      3,      3,   3 ) \
	PARAM_ENTRY(CAT_AIRSUS,		  AirSuspension_Control_Mode, 	MODE,       0,      2,  	1,   12 )\
	PARAM_ENTRY(CAT_AIRSUS,       Air_Compressor, 				OFFON,     	0,      1,      0,   13 )\
	PARAM_ENTRY(CAT_AIRSUS,       Rear_Right_Valve, 			OFFON,     	0,      1,      0,   14 )\
	PARAM_ENTRY(CAT_AIRSUS,       Rear_Left_Valve, 				OFFON,     	0,      1,      0,   15 )\
	PARAM_ENTRY(CAT_AIRSUS,       Ambient_Valve, 				OFFON,     	0,      1,      0,   16 )\
	PARAM_ENTRY(CAT_AIRSUS,       Reverse1_Valve, 				OFFON,     	0,      1,      0,   17 )\
	PARAM_ENTRY(CAT_AIRSUS,       Reverse2_Valve, 				OFFON,     	0,      1,      0,   18 )\
	VALUE_ENTRY(opmode,        	  OPMODES,	2000 )\
    VALUE_ENTRY(version,          VERSTR,	2001 )\
	VALUE_ENTRY(Vsense, 	      "V",		2002 )\
	VALUE_ENTRY(ActCur_R, 	      "V",		2003 )\
	VALUE_ENTRY(ActCur_L, 	      "V",		2004 )\
	VALUE_ENTRY(Hall_R, 	      "V",		2005 )\
	VALUE_ENTRY(Hall_L, 	      "V",		2006 )\
	VALUE_ENTRY(Status, 	      LKMODES,	2025 )\
	VALUE_ENTRY(Press, 	   	      "V",		2007 )\
	VALUE_ENTRY(Sus_R, 	   	      "V",		2008 )\
	VALUE_ENTRY(Sus_L, 	          "V",		2009 )\
	VALUE_ENTRY(Comp_Tmp, 	      "V",		2010 )\
	VALUE_ENTRY(Btn_Sig, 	      "V",		2011 )\
	VALUE_ENTRY(R1_EN, 	   		  OFFON,	2012 )\
	VALUE_ENTRY(R2_EN, 	   	  	  OFFON,	2013 )\
	VALUE_ENTRY(L1_EN, 	   	      OFFON,	2014 )\
	VALUE_ENTRY(L2_EN, 	   		  OFFON,	2015 )\
	VALUE_ENTRY(Compressor, 	  OFFON,	2026 )\
	VALUE_ENTRY(RR_Vlv, 	   	  OFFON,	2017 )\
	VALUE_ENTRY(RL_Vlv, 	   	  OFFON,	2018 )\
	VALUE_ENTRY(Amb_Vlv, 	   	  OFFON,	2019 )\
	VALUE_ENTRY(Rvrs1_Vlv, 	   	  OFFON,	2020 )\
	VALUE_ENTRY(Rvrs2_Vlv, 	   	  OFFON,	2021 )\
	VALUE_ENTRY(Aux_EN, 	   	  OFFON,	2022 )\
	VALUE_ENTRY(R_LED, 	   		  OFFON,	2023 )\
	VALUE_ENTRY(G_LED, 	   		  OFFON,	2024 )\
    VALUE_ENTRY(cpuload,          "%", 		2030 )


/***** Enum String definitions *****/
#define OPMODES      "0=Off, 1=Run"
#define MODE         "0=Auto, 1=Service"
#define OFFON        "0=OFF, 1=ON"
#define CALBRS       "0=OFF, 1=Right, 2=Left, 3=RightLeft"
#define LKMODES      "0=UnLocked, 1=Locked"
#define CAT_AIRSUS 	 "Manual Air Suspensio Control"
#define CAT_HBridge  "eBrake Actuator Control"
#define CAT_SYSCON 	 "System General Control"
#define VERSTR STRINGIFY(4=VER)

/***** enums ******/




enum _modes
{
    MOD_OFF = 0,
    MOD_RUN,
    MOD_LAST
};

//Generated enum-string for possible errors
extern const char* errorListString;
