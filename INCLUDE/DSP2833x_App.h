
#ifndef DSP2833x_D013_H
#define DSP2833x_D013_H


#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint16_t Byte_number;//0x493端口输出字节数//3.29自己添加


extern int16 MVB_Address;//MVB地址线
extern int16 MVB_Sink_FN;//SINK800输入超时故障
extern uint16_t MVB2_SINK800_T4[1];//0x800端口输入
extern uint16_t MVB2_SINK4X8_T2[16];//0x498端口输入
extern uint16_t MVB2_SINK4X9_T3[16];//0x499端口输入
extern uint16_t MVB2_SINK4XA_T4[16];//0x49A端口输入
extern uint16_t MVB1_SINK516_T3[16];//0x516端口输入
extern uint16_t MVB1_SINK300_T2[1];//0x300端口输入
extern uint16_t MVB1_SINK440_T3[1];//0x440端口输入

extern uint16_t MVB2_SRC4X0_T2[16];//0x490端口输出
extern uint16_t MVB2_SRC4X1_T3[16];//0x491端口输出
extern uint16_t MVB2_SRC4X2_T4[16];//0x492端口输出
extern uint16_t MVB1_SRC4X6_T3[16];//0x496端口输出

extern uint16_t MVB1_SRC308_T2[16];//0x308端口输出
extern uint16_t MVB1_SRC448_T3[1];//0x448端口输出

extern uint16_t MVB2_SRC4X3_T4[16];//0x493端口输出//11.26自己添加



#ifdef __cplusplus
}
#endif /* extern "C" */

#endif  // end of DSP2833x_D013_H definition
