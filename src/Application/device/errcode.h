/**
  ******************************************************************************
  * @file    errcode.h
  * @author  Firmware-Team
  * @version xx.xx.xx
  * @date    27-07-2016
  * @brief   �¿ذ�Ĺ�����Ϣ��
  *
  * ���������
  * 0x0000-0x09FF	�¿�Ӧ�����ר��
  * 0x0A00-0x0AFF   Ӧ���������
  * 0x0B00-0x0FFF   �¿�ƽ̨����
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
//#define    ERR_PID_OVERTIME                   		0x0110		//PID���ڳ�ʱ
#define    ERR_COOL_OVERTIME                           0x0105      //���䳬ʱ
#define    ERR_HOT_OVERTIME                              0x0106      //���ȳ�ʱ
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
#define    ERR_REFRI_TEMPERATURE              0x0113      //�����¶��쳣

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

#define    ERR_FRAME_INEXISTENCE                  0x0001      //����֡������
#define    ERR_FRAME_LENTH_ERROR               0x0002      //֡���ȴ���
#define    ERR_PARAM1_ABNOMAL                       0x0003      //Parameter1�쳣
#define    ERR_PARAM_ABNOMAL                         0x0004      //Parameter�쳣
//#define    ERR_FRAMETYPE_ERROR                      0x0205      //֡���ʹ���

#define    ERR_DRIVER_EEPROM                        0x0B00      //eeprom��������
#define    ERR_VIRSION                                          0x0B01      //�汾������Ҫ�ǻ�ȡ�汾�������صĳ���<=0������Ϊ�쳣��




#endif


