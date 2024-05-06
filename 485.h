#ifndef __RS485_COMM_PROC_H
#define __RS485_COMM_PROC_H

#include "stm32f10x.h"
#include "includes.h"

#ifdef ENABLE_EXT_RS485

#define RS485_TASK_PRIO           11
#define RS485_STK_SIZE            200

//RS485控制IO  输出高：485发送  输出低：485接收，初始化接收模式
#define  RS485_CTRL_PORT            GPIOB
#define  RS485_CTRL_BIT             GPIO_Pin_13
#define  Rs485SendMode()            GPIO_SetBits(RS485_CTRL_PORT, RS485_CTRL_BIT)
#define  Rs485RecvMode()            GPIO_ResetBits(RS485_CTRL_PORT, RS485_CTRL_BIT)  

#define  Rs485Com_Rx_LEN         50

#define  PARAMETER_TRUE      0x01
#define  PARAMETER_FALSE     0x00

typedef enum
{
    QUERY_BATT_TEMPERATURE = 0x08,                    //查询电池温度
    QUERY_BATT_TOTAL_VOLT = 0x09,                     //查询电池组总电压
    QUERY_BATT_CURRENT = 0x0A,                        //查询实时电流
    QUERY_BATT_RELATIVE_CAPACITY = 0x0D,              //查询相对容量
    QUERY_BATT_ABSOLUTE_CAPACITY = 0x0E,              //查询绝对容量
    QUERY_BATT_RESIDUE_CAPACITY = 0x0F,               //查询剩余容量
    QUERY_BATT_FULL_CAPACITY = 0x10,                  //查询满电容量
    QUERY_BATT_STAUTS = 0x16,                         //查询电池状态
    QUERY_BATT_CYCLE_INDEX = 0x17,                    //查询循环次数
    QUERY_BATT_MANUF_NAME = 0x20,                     //查询电池组制造厂名称
    QUERY_BATT_ONE_SEVEN_VOLT = 0x24,                 //查询1~7节电池电压
    QUERY_BATT_EIGHT_TWENTY_VOLT = 0x25,              //查询8~20节电池电压
    QUERY_BATT_CUR_CHARGE_TIME = 0x47,                //查询当前充电间隔时间
    QUERY_BATT_MAX_CHARGE_TIME = 0x48,                //查询最大充电间隔时间
    QUERY_BATT_SOH = 0x0C,                            //查询SOH
    QUERY_BATT_VERSION = 0x7F,                        //查询版本号
    QUERY_BATT_ID_CODE = 0x7E,                        //查询电池组ID条码
    QUERY_BATT_POWER_OUTPUT = 0x81,                   //查询BMS供电输出
    QUERY_BATT_OVER_DISCHARGE_PROTE = 0x82,           //查询电池过放保护
    //=================控制类=================
    CTRL_BATT_POWER_OUTPUT = 0x80,                    //控制BMS供电输出，数据区00：关闭，01：打开
    CTRL_BATT_CHARGE_TYPE = 0x83,                     //控制BMS充电类型，数据区00：需要通讯，01：判断电压
    
    QUERY_CTRL_CMD = 0xF0,
    RESPONSE_CTRL_CMD = 0xF1,
    RS485_CONFIG_CMD = 0xFF, 
}bms_batt_em;

typedef struct tagStructRs485CtrlData
{
	uint8_t       SendCtrlFlag;        //发送控制是否成功，由接收控制器应答判断，1=未收到应答，0=收到应答
    uint16_t      SendTimeoutCnt;
    uint8_t       SendCtrlCnt;         //发送控制次数，从机不应答，最多发5次
	
}StructRs485CtrlData;

//APP数据结构
typedef struct tagRs485_DataStruct
{       
    int16_t temperature;                  //电池内部温度，最高位符号位，1表示负数，0表示正数，放大10倍后
    uint32_t total_volt;                  //电池总电压，单位mV 
    uint32_t ave_current;                 //5秒内平均电流 
    uint8_t  soc1;                        //相对容量SOC，0~100%
    uint8_t  soc2;                        //绝对容量SOC，0~100%
    uint16_t residue_capacity;            //剩余容量，单位mAh
    uint16_t full_capacity;               //满电容量，单位mAh
    uint8_t status[3];                    //电池状态，3字节，每bit代表一种状态
    uint16_t cycle_charge_cnt;            //循环充电次数，2字节
    uint8_t manufacturer_name[20];        //电池组制造商名称，ascii码
    uint8_t cell_volt[20][2];             //每节电芯电压，用2byte表示，最多20节
    uint8_t soh;                          //soh,0~100%
    uint16_t cur_charge_time;             //当前充电间隔时间，单位分钟
    uint16_t max_charge_time;             //最大充电间隔时间，单位小时
    uint8_t hw_ver;                       //硬件版本号，从整数0x64开始，例：0x65 = V101
    uint8_t sf_ver;                       //软件版本号，从整数0x64开始
    uint8_t id_code[16];                  //条形码ID，ascii码,16位字符
    uint8_t power_output_state;           //BMS供电输出状态，0=关闭输出，1=打开输出
    uint8_t over_discharge_state;         //BMS过放保护状态，0=正常，1=电量过低，需要进入过放保护，此时bms会断开gps供电
    
	uint8_t RecLen;
    uint8_t bRs485PacketRcvSuccess;       //接收到正确包标志
}Rs485DataStruct;

#define RS485_MAX_BUF_LEN  50
typedef struct
{
    char  RxBuf[RS485_MAX_BUF_LEN];	
    char  TxBuf[RS485_MAX_BUF_LEN];
    uint16_t rx_size;
}Rs485Com_st;

extern Rs485Com_st Rs485Com;

extern OS_SEM rs485_recv_sem;
extern OS_SEM rs485_send_sem;
extern Rs485DataStruct Rs485Data;

void rs485_hw_init(void);
void rs485_output_ctrl(uint8_t state);
void bms_output_ctrl(uint8_t state);
void bms_set_charge_type(uint8_t state);
void rs485_dma_write(uint8_t *buf, uint16_t len);
void RcvRs485DataFromIsr(uint8_t tmp);
void Rs485CommRecvProc(uint8_t rev_data);
void Rs485RcvDataProc(void);
void SendLedScreenData(uint8_t cmd,uint8_t *ledBuff,uint8_t len);
void rs485_create(void);

#endif //#ifdef ENABLE_EXT_RS485
#endif /* __RS485_COMM_PROC_H */
