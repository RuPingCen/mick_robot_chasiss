#ifndef __BMP180_H
#define __BMP180_H

#include "stm32f10x.h"


#define SCTemperature     0x01// ÆøÑ¹¼Æ×´Ì¬»ú
#define CTemperatureing   0x02
#define SCPressure        0x03
#define SCPressureing     0x04

#define BUFFER_SIZE                 3	  



#define CAL_AC1           0xAA  // R   Calibration data (16 bits)/* ---- Registers ---- */
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

#define BMP180_ADDR                 0xEE     // default I2C address




// unused registers
#define SOFTRESET         0xE0
#define VERSION           0xD1  // ML_VERSION  pos=0 len=4 msk=0F  AL_VERSION pos=4 len=4 msk=f0
#define CHIPID            0xD0  // pos=0 mask=FF len=8
                                // BMP085_CHIP_ID=0x55

/************************************/
/*    REGISTERS PARAMETERS          */
/************************************/
// BMP180 Modes
#define MODE_ULTRA_LOW_POWER    0 //oversampling=0, internalsamples=1, maxconvtimepressure=4.5ms, avgcurrent=3uA, RMSnoise_hPA=0.06, RMSnoise_m=0.5
#define MODE_STANDARD           1 //oversampling=1, internalsamples=2, maxconvtimepressure=7.5ms, avgcurrent=5uA, RMSnoise_hPA=0.05, RMSnoise_m=0.4
#define MODE_HIGHRES            2 //oversampling=2, internalsamples=4, maxconvtimepressure=13.5ms, avgcurrent=7uA, RMSnoise_hPA=0.04, RMSnoise_m=0.3
#define MODE_ULTRA_HIGHRES      3 //oversampling=3, internalsamples=8, maxconvtimepressure=25.5ms, avgcurrent=12uA, RMSnoise_hPA=0.03, RMSnoise_m=0.25
                  
// Control register
#define READ_TEMPERATURE        0x2E 
#define READ_PRESSURE           0x34 
//Other
#define MSLP                    101325          // Mean Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar)



void BMP180_Routing(void);
void BMP180_init(void) ;
void BMP180_getCalData(void);
void BMP180_calcTruePressure(u8 writeread);
void BMP180_calcTrueTemperature(u8 rw);
void BMP180_readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]);
void BMP180_writemem(uint8_t _addr, uint8_t _val);
void Nem_Pa_Data_FIFO(int32_t temp_da);
void Nem_centimeters_Data_FIFO(int32_t temp_da);
void Nem_Temperature_Data_FIFO(int32_t temp_da);

#endif

//------------------End of File----------------------------
