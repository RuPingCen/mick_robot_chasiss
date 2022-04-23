#ifndef __BMP085_H
#define __BMP085_H

#include "IOI2C.h"
#include "delay.h"


#define BMP085_ADDR                 0xEE     // default I2C address
#define BUFFER_SIZE                 3

#define AUTO_UPDATE_TEMPERATURE     1    //default is true
        // when true, temperature is measured everytime pressure is measured (Auto).
        // when false, user chooses when to measure temperature (just call calcTrueTemperature()).
        // used for dynamic measurement to increase sample rate (see BMP085 modes below).
       
/* ---- Registers ---- */
#define CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define CAL_AC3           0xAE  // R   Calibration data (16 bits)    
#define CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define CAL_B1            0xB6  // R   Calibration data (16 bits)
#define CAL_B2            0xB8  // R   Calibration data (16 bits)
#define CAL_MB            0xBA  // R   Calibration data (16 bits)
#define CAL_MC            0xBC  // R   Calibration data (16 bits)
#define CAL_MD            0xBE  // R   Calibration data (16 bits)
#define CONTROL           0xF4  // W   Control register 
#define CONTROL_OUTPUT    0xF6  // R   Output registers 0xF6=MSB, 0xF7=LSB, 0xF8=XLSB

// unused registers
#define SOFTRESET         0xE0
#define VERSION           0xD1  // ML_VERSION  pos=0 len=4 msk=0F  AL_VERSION pos=4 len=4 msk=f0
#define CHIPID            0xD0  // pos=0 mask=FF len=8
                                // BMP085_CHIP_ID=0x55

/************************************/
/*    REGISTERS PARAMETERS          */
/************************************/
// BMP085 Modes
#define MODE_ULTRA_LOW_POWER    0 //oversampling=0, internalsamples=1, maxconvtimepressure=4.5ms, avgcurrent=3uA, RMSnoise_hPA=0.06, RMSnoise_m=0.5
#define MODE_STANDARD           1 //oversampling=1, internalsamples=2, maxconvtimepressure=7.5ms, avgcurrent=5uA, RMSnoise_hPA=0.05, RMSnoise_m=0.4
#define MODE_HIGHRES            2 //oversampling=2, internalsamples=4, maxconvtimepressure=13.5ms, avgcurrent=7uA, RMSnoise_hPA=0.04, RMSnoise_m=0.3
#define MODE_ULTRA_HIGHRES      3 //oversampling=3, internalsamples=8, maxconvtimepressure=25.5ms, avgcurrent=12uA, RMSnoise_hPA=0.03, RMSnoise_m=0.25
                  
// Control register
#define READ_TEMPERATURE        0x2E 
#define READ_PRESSURE           0x34 
//Other
#define MSLP                    101325          // Mean Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar)


//以下程序为BMP085驱动供外部调用的API
void BMP085_init(void);	//初始化BMP085
void BMP085_getTemperat(int32_t *_Temperature);	//Temp(0.1C):  读取温度
void BMP085_getPress(int32_t *_TruePressure);  //Pressure(Pa) 读取气压值
void BMP085_getAlt(int32_t *_centimeters); //Alt(cm) 读取高度
void BMP085_Routing(void);	  //这个程序需要用户定时调用，以更新当前温度和气压值 
void BMP085_ResetAlt(int32_t _centimeters);	//复位气压高度。把当前的高度设置成 0米

#endif

//------------------End of File----------------------------
