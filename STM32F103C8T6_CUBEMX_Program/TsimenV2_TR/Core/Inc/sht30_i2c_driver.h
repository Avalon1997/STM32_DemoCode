#ifndef __SHT30_I2C_DRIVER
#define __SHT30_I2C_DRIVER

#include "I2C.h"
/* ADDR引脚直接 VSS*/
#define SHT30_ADDR_WRITE    0x44<<1         //10001000
#define SHT30_ADDR_READ     (0x44<<1)+1     //10001001

typedef enum
{
    /* 软件复位指令 */
    SOFT_RESET_CMD = 0x30A2,

    /**
    * 单次测量模式
    * 指令命名格式： Repeatability_ClockStretching_CMD
    */
    HIGH_ENABLED_CMD    = 0x2C06,
    MEDIUM_ENABLED_CMD  = 0x2C0D,
    LOW_ENABLED_CMD     = 0x2C10,
    
    HIGH_DISABLED_CMD   = 0x2400,
    MEDIUM_DISABLED_CMD = 0x240B,
    LOW_DISABLED_CMD    = 0x2416,

} SHT30_CMD;


uint8_t SHT30_ValGet (uint8_t* dat);    //单次测量值获取
uint8_t SHT30_Dat_To_Float(uint8_t* const dat,float* temperature,float* humidity);  //浮点型温度湿度值转换

void Measure_TR(void);  //执行测量函数



#endif  //end of __SHT30_I2C_DRIVER
