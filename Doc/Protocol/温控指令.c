TEMPA
55 AA 08 00 01 04 00 0B 06 00 01 01 E1 15  //TEMPA开启

55 AA 0B 00 01 05 00 0B 05 00 01 88 13 00 00 E7 FA  //设置温度60°

55 AA 0B 00 01 05 00 0B 05 00 01 98 3A 00 00 32 F2  //设置温度150


55 AA 08 00 01 7E 00 0B 06 00 01 00 FB 12  //关闭TEMPA

55 AA 07 00 01 76 00 0B 0B 00 01 A9 B0 //查询当前温度




TEMPB: 设置为numlen=3

55 AA 08 00 01 05 00 0B 06 00 03 01 F0 B5  //TEMPB开启


55 AA 0B 00 01 20 00 0B 05 00 03 A0 0F 00 00 47 26 //TEMPB 40°


55 AA 07 00 01 21 00 0B 0B 00 03 25 96  //查询当前温度

55 AA 07 00 01 22 00 0B 0B 00 03 25 A5 //查询当前温度

55 AA 07 00 01 23 00 0B 0B 00 03 24 74 //查询当前温度

55 AA 07 00 01 24 00 0B 0B 00 03 25 C3  //查询当前温度
 
55 AA 07 00 01 25 00 0B 0B 00 03 24 12  //查询当前温度
 
55 AA 07 00 01 26 00 0B 0B 00 03 24 21  //查询当前温度
 
55 AA 07 00 01 27 00 0B 0B 00 03 25 F0  //查询当前温度
 
 
 
 
 
 
55 AA 08 00 01 9A 00 0B 06 00 03 00 5E 7C   //关闭TEMPB
55 AA 08 00 01 9C 00 0B 06 00 03 00 38 7C   //关闭TEMPB


















ChannelNum:1
AA 07 00 01 0D 00 0B 0B 00 01 A3 FB

55 AA 06 00 02 0D 00 01 0B 00 7A D3 
55 AA 0C 00 03 0D 00 01 0B 00 00 01   D6 08 00 00    F7 C1 
ChannelNum:1. CurrentTemp = 0x000008D6==>22.62


ChannelNum:1
AA 07 00 01 0F 00 0B 0B 00 01 A2 19

55 AA 06 00 02 0F 00 01 0B 00 03 13 
55 AA 0C 00 03 0F 00 01 0B 00 00 01 E9 08 00 00 F0 6D 
ChannelNum:1. CurrentTemp = 0x000008E9==>22.81








