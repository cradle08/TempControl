/**
  ******************************************************************************
  * @file    errcode.h
  * @author  Firmware-Team
  * @version xx.xx.xx
  * @date    27-07-2016
  * @brief   温控板的故障信息表
  *
  * 故障码分区
  * 0x0000-0x09FF	温控应用软件专用
  * 0x0A00-0x0AFF   应用软件公用
  * 0x0B00-0x0FFF   温控平台驱动
  *
  ******************************************************************************
**/


#ifndef _ERRCODE_H_
#define _ERRCODE_H_

#define    SUCCESS1  (0x00)
#define    ERR_TARGET_TEMP_OVER_UPPER_LIMIT   	    0x0101		//Target-Temperature beyond the Up-Limit
#define    ERR_TARGET_TEMP_OVER_LOWER_LIMIT   	    0x0102		//Target-Temperature beyond the Down-Limit
//#define    ERR_TARGET_TEMP_OVER_UPPER_LIMIT_B   	0x0103		//B Channel)Target-Temperature beyond the Up-Limit
//#define    ERR_TARGET_TEMP_OVER_LOWER_LIMIT_B   	0x0104		//B Channel)Target-Temperature beyond the Down-Limit
#define    ERR_TEMP_OVER_W_UPPER_LIMIT        		0x0103		//Current-Temperature beyond the Warning-Up-Limit
#define    ERR_TEMP_OVER_W_LOWER_LIMIT        		0x0104		//Current-Temperature beyond the Warning-Down-Limit
//#define    ERR_TEMP_OVER_E_UPPER_LIMIT        		0x0107		//Current-Temperature beyond the Error-Up-Limit
//#define    ERR_TEMP_OVER_E_LOWER_LIMIT        		0x0108		//Current-Temperature beyond the Error-Down-Limit
//#define    ERR_TEMP_OVER_W_UPPER_LIMIT_B        	0x0109		//B Channel)Current-Temperature beyond the Warning-Up-Limit
//#define    ERR_TEMP_OVER_W_LOWER_LIMIT_B        	0x010A		//B Channel)Current-Temperature beyond the Warning-Down-Limit
//#define    ERR_TEMP_OVER_E_UPPER_LIMIT_B        	0x010B		//B Channel)Current-Temperature beyond the Error-Up-Limit
//#define    ERR_TEMP_OVER_E_LOWER_LIMIT_B        	0x010C		//B Channel)Current-Temperature beyond the Error-Down-Limit
//#define    ERR_TEMPRETURE_SENSOR              	    0x010D		//Temperature-Sensor exception
//#define    ERR_CHANNEL_NUM                    		0x010E		//Channel-Number exception
//#define    ERR_PID_PARAM                      		0x010F		//Kp\Ki\Kd exception
//#define    ERR_PID_OVERTIME                   		0x0110		//PID调节超时
#define    ERR_COOL_OVERTIME                           0x0105      //制冷超时
#define    ERR_HOT_OVERTIME                              0x0106      //加热超时
#define    ERR_COOL_OVERTIME_B                      0x0107
#define    ERR_HOT_OVERTIME_B                         0x0108

#define    ERR_COOL_OVERTIME_C                      0x0109
#define    ERR_HOT_OVERTIME_C                         0x010A

#define    ERR_COOL_OVERTIME_D                      0x010B
#define    ERR_HOT_OVERTIME_D                         0x010C

#define    ERR_TEMPRETURE_SENSOR_A                  0x010D
#define    ERR_TEMPRETURE_SENSOR_B                  0x010E
#define    ERR_TEMPRETURE_SENSOR_C                  0x010F
#define    ERR_TEMPRETURE_SENSOR_D                  0x0110

#define    ERR_CHANNEL_NUM                    		0x0111		//Channel-Number exception
#define    ERR_ADC_ABNORMAL                         0x0112
#define    ERR_REFRI_TEMPERATURE              0x0113      //冰箱温度异常

#define    ERR_TARGET_TEMP_OVER_UPPER_LIMIT_B   	    0x0120		//Target-Temperature beyond the Up-Limit
#define    ERR_TARGET_TEMP_OVER_LOWER_LIMIT_B   	    0x0121		//Target-Temperature beyond the Down-Limit

#define    ERR_TARGET_TEMP_OVER_UPPER_LIMIT_C   	    0x0122		//Target-Temperature beyond the Up-Limit
#define    ERR_TARGET_TEMP_OVER_LOWER_LIMIT_C   	    0x0123		//Target-Temperature beyond the Down-Limit

#define    ERR_TARGET_TEMP_OVER_UPPER_LIMIT_D   	    0x0124		//Target-Temperature beyond the Up-Limit
#define    ERR_TARGET_TEMP_OVER_LOWER_LIMIT_D   	    0x0125		//Target-Temperature beyond the Down-Limit

#define    ERR_TEMP_OVER_W_UPPER_LIMIT_B        		0x0126		//Current-Temperature beyond the Warning-Up-Limit
#define    ERR_TEMP_OVER_W_LOWER_LIMIT_B        		0x0127		//Current-Temperature beyond the Warning-Down-Limit

#define    ERR_TEMP_OVER_W_UPPER_LIMIT_C       		0x0128		//Current-Temperature beyond the Warning-Up-Limit
#define    ERR_TEMP_OVER_W_LOWER_LIMIT_C        		0x0129		//Current-Temperature beyond the Warning-Down-Limit

#define    ERR_TEMP_OVER_W_UPPER_LIMIT_D        		0x012A		//Current-Temperature beyond the Warning-Up-Limit
#define    ERR_TEMP_OVER_W_LOWER_LIMIT_D       		0x012B		//Current-Temperature beyond the Warning-Down-Limit

#define    WAR_NOT_CLOSE_FANA                                        0x0134       // for SP960/SP960XL ,if  the TEC is working ,not close the cooling fan 
#define    WAR_NOT_CLOSE_FANB                                        0x0135       // for SP960/SP960XL ,if  the TEC is working ,not close the cooling fan 
#define    WAR_NOT_CLOSE_FANC                                        0x0136       // for SP960/SP960XL ,if  the TEC is working ,not close the cooling fan 
#define    WAR_NOT_CLOSE_FAND                                        0x0137       // for SP960/SP960XL ,if  the TEC is working ,not close the cooling fan 

#define    ERR_FRAME_INEXISTENCE                  0x0001      //命令帧不存在
#define    ERR_FRAME_LENTH_ERROR               0x0002      //帧长度错误
#define    ERR_PARAM1_ABNOMAL                       0x0003      //Parameter1异常
#define    ERR_PARAM_ABNOMAL                         0x0004      //Parameter异常
//#define    ERR_FRAMETYPE_ERROR                      0x0205      //帧类型错误

#define    ERR_DRIVER_EEPROM                        0x0B00      //eeprom驱动错误
#define    ERR_VIRSION                                          0x0B01      //版本错误（主要是获取版本函数返回的长度<=0，则认为异常）




#endif


