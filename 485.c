#include "includes.h"

#ifdef ENABLE_EXT_RS485

CPU_STK Rs485TaskStk[RS485_STK_SIZE];
OS_SEM rs485_send_sem;
OS_SEM rs485_recv_sem;
OS_TCB  Rs485TackTCB;

Rs485DataStruct Rs485Data;
Rs485Com_st Rs485Com;

uint8_t rs485_output_enable = FALSE;
uint8_t rs485_rev_buf[100] = {0};
static uint8_t rs485_send_buf[50] = {0};
uint16_t Rs485ComRecvPtr = 0;

bms_batt_em query_batt_state = QUERY_BATT_TEMPERATURE;

extern OS_TCB    RFDecodeTaskTCB;

uint8_t recConfigFlag = FALSE;
uint8_t ConfigBuf[8];
uint8_t Rs485Com_RX_BUF[Rs485Com_Rx_LEN];
uint8_t query_lio_batt_cnt=0;

void DebugStatusInfo(void);
extern uint8_t GetShakeLevel(void);

/*****************************************************************************
 函 数 名  : rs485_hw_init
 功能描述  : RS485初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年3月13日, 星期二
    作    者   : Billy yu  
    修改内容   : 新生成函数

*****************************************************************************/
void rs485_hw_init(void)
{
    GPIO_InitTypeDef  GpioInit;
    
    unsigned  char i;
	
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);

    //RS485控制IO初始化，  高发送，低接收
    GpioInit.GPIO_Pin    = RS485_CTRL_BIT;
    GpioInit.GPIO_Mode   = GPIO_Mode_OUT;
    GpioInit.GPIO_OType  = GPIO_OType_PP;
    GpioInit.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(RS485_CTRL_PORT, &GpioInit);
    Rs485RecvMode();
    
    bsp_InitUart(RS485_COM,9600);
    //McuDMAChannel_Config(DMA1_Channel6,(u32)&USART3->RDR,(uint32_t)Rs485Com.RxBuf,\
    //                     DMA_DIR_PeripheralSRC,DMA_Mode_Normal,RS485_MAX_BUF_LEN);                        
    //DMA_Cmd(DMA1_Channel6, ENABLE);
    
	for (i = 0; i < 8; i++) 
    {
		ConfigBuf[i] = 0;
    }
	
    memset(&Rs485Data,0,sizeof(Rs485DataStruct));
    
    for (i = 0; i < 16; i++)
    {
        Rs485Data.id_code[i] = '0';
    }
    
    for (i = 0; i < 20; i++)
    {
        Rs485Data.manufacturer_name[i] = '0';
    }
    
    Rs485Data.bRs485PacketRcvSuccess = FALSE;
}

//控制485调试信息是否输出，默认不输出
void rs485_output_ctrl(uint8_t state)
{
    if(state)
        rs485_output_enable = TRUE;
    else
        rs485_output_enable = FALSE;
}

/*
*********************************************************************************************************
*	函 数 名: rs485_send_data
*	功能说明: 使用串口5发送数据
*	形    参: buf--存放数据buf,len--长度
*	返 回 值: 无
*********************************************************************************************************
*/
void rs485_send_data(uint8_t *buf, uint16_t len)
{  
    OS_ERR err;
    uint16_t i;
    
    OSSemPend((OS_SEM      *)&rs485_send_sem,
              (OS_TICK      )0,
              (OS_OPT       )OS_OPT_PEND_BLOCKING,
              (CPU_TS      *)0,
              (OS_ERR      *)&err);
    
    Rs485SendMode();  
    //delay_ms(1);
    //printf("\r\nRS485 Send data:");
    for(i = 0;i < len;i++)
    {
        UART5_SendChar(buf[i]);
    }
    //USART_SendArray(USART3,buf,len);
    
    //delay_ms(1);
    Rs485RecvMode();
    OSSemPost(&rs485_send_sem,OS_OPT_POST_1,&err);
    
    #ifdef  DEBUG_OUTPUT
    if(rs485_output_enable == TRUE)
    {
        printf("Rs485SendData: ");
        for(i = 0;i < len;i++)
           printf("%02X ",buf[i]); 
        printf("\r\n");
    }
    #endif
}

/*
*********************************************************************************************************
*	函 数 名: lionBat_Send_cmd 
*	功能说明: 使用串口5发送命令
*	形    参: cmd--命令,param--参数
*	返 回 值: 无
*********************************************************************************************************
*/
void lionBat_Send_cmd(bms_batt_em cmd,uint8_t param)
{
    uint8_t i;
    uint16_t checkSum = 0;
    uint8_t tmpBuf[15]={0};
    
    //0x3A,0x16,0x08,0x01,0x00,0x1F,0x00,0x0D,0x0A
    
    tmpBuf[0] = 0x3A;         //包头
    tmpBuf[1] = 0x16;         //地址
    tmpBuf[2] = (uint8_t)cmd; //命令
    tmpBuf[3] = 0x01;         //长度
    tmpBuf[4] = param;        //内容
    
    //从地址到内容的累加和
    for(i = 0; i < 5; i++)
    {
        checkSum += tmpBuf[i+1]; 
    }
    tmpBuf[5] = checkSum&0xFF;   //小端模式
    tmpBuf[6] = checkSum>>8;

    tmpBuf[7] = 0x0D;         //包尾
    tmpBuf[8] = 0x0A;
    
    rs485_send_data(tmpBuf,9);
}

/*
*********************************************************************************************************
*	函 数 名: bms_output_ctrl
*	功能说明: 控制BMS供电输出
*	形    参: 0:关闭，1:打开
*	返 回 值: 无
*********************************************************************************************************
*/
void bms_output_ctrl(uint8_t state)
{
    if(state>1) state = 0;
    
    if(state)
    {
        lionBat_Send_cmd(CTRL_BATT_POWER_OUTPUT,PARAMETER_TRUE);
        #ifdef DEBUG_OUTPUT
        printf("\r\n使能BMS供电输出!!!\r\n");
        #endif
    }
    else
    {
        lionBat_Send_cmd(CTRL_BATT_POWER_OUTPUT,PARAMETER_FALSE);
        #ifdef DEBUG_OUTPUT
        printf("\r\n失能BMS供电输出!!!\r\n");
        #endif
    }
}

/*
*********************************************************************************************************
*	函 数 名: bms_set_charge_type
*	功能说明: 设置BMS充电类型
*	形    参: 0:盲充，通过判断电压，1:握手通讯
*	返 回 值: 无
*********************************************************************************************************
*/
void bms_set_charge_type(uint8_t state)
{
    if(state>1) state = 0;
    
    if(state)
    {
        lionBat_Send_cmd(CTRL_BATT_CHARGE_TYPE,PARAMETER_TRUE);
        #ifdef DEBUG_OUTPUT0
        printf("\r\n设置通讯方式充电!!!\r\n");
        #endif
    }
    else
    {
        lionBat_Send_cmd(CTRL_BATT_CHARGE_TYPE,PARAMETER_FALSE);
        #ifdef DEBUG_OUTPUT0
        printf("\r\n设置判断电压充电!!!\r\n");
        #endif
    }
}

/*
*********************************************************************************************************
*	函 数 名: rs485_dma_write
*	功能说明: 使用DMA方式向串口3发送数据
*	形    参: buf--存放数据buf,len--长度
*	返 回 值: 无
*********************************************************************************************************
*/
void rs485_dma_write(uint8_t *buf, uint16_t len)
{  
    OS_ERR err;
    
    OSSemPend((OS_SEM      *)&rs485_send_sem,
              (OS_TICK      )0,
              (OS_OPT       )OS_OPT_PEND_BLOCKING,
              (CPU_TS      *)0,
              (OS_ERR      *)&err);
    
    if (buf)
        memcpy(rs485_send_buf, buf, len);
    
    Rs485SendMode();   
//    USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);
//    McuDmaResetMemoryBaseAddr(DMA1_Channel7, (uint32_t)rs485_send_buf, len);
//    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
}

unsigned char CheckRs485DataPacket(unsigned char *pData,unsigned start,unsigned char len)
{
    uint8_t i,ucCheckValue;
    
    //计算LRC校验值(累加和取反加1)
    ucCheckValue = 0;
    for (i = start; i < len; i++)
    {
        ucCheckValue += *(pData + i);
    }
    ucCheckValue = (~ucCheckValue) + 1;
	
    if(ucCheckValue == *(pData + i)) //比较校验和
    {
        return TRUE;
    }
    else
    {
        DEBUG_UART("\r\nRS485校验和错误");
        return FALSE;
    }
}

/*****************************************************************************
 函 数 名  : CheckRs485DataPacket_Add
 功能描述  : 校验485接收数据包,累加校验，传过来的是小端模式
 输入参数  : unsigned char *pData  
             unsigned start        
             unsigned char len     
 输出参数  : 无
 返 回 值  : unsigned
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2018年7月17日, 星期二
    作    者   : Billy yu
    修改内容   : 新生成函数

*****************************************************************************/
unsigned char CheckRs485DataPacket_Add(unsigned char *pData,unsigned start,unsigned char len)
{
    uint16_t i,ucCheckValue;
    
    ucCheckValue = 0;
    for (i = start; i < len; i++)
    {
        ucCheckValue += *(pData + i);
    }
	
    if(((ucCheckValue&0xFF) == *(pData + i))&&(ucCheckValue>>8 == *(pData + i + 1))) //比较校验和
    {
        return TRUE;
    }
    else
    {
        DEBUG_UART("\r\nRS485校验和错误");
        return FALSE;
    }
}

//**********************************************************************
//新增测试协议，通过485发送命令测试硬件  //7E 55 01 5A A5 FF 2A
//1、0x7E 0x55 0x01 0x5A 0xA5 0x00 0x0A ---打开一键启动GPIO
//2、0x7E 0x55 0x02 0x5A 0xA5 0xFF 0x0A ---打开一键启动GPIO
//3、0x7E 0x55 0x03 0x1C 0x60 0x81 0x0A ---配置遥控ID码
//4、0x7E 0x55 0x04 0x5A 0xA5 0xFD 0x0A ---开启工程调试模式
//5、0x7E 0x55 0x05 0x5A 0xA5 0xFC 0x0A ---开启生产测试模式
//===========================
/*****************************************************************************
 函 数 名  : RcvRs485DataFromIsr
 功能描述  : 485接收处理函数，由串口中断调用
 输入参数  : uint8_t tmp  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2018年7月17日, 星期二
    作    者   : Billy yu
    修改内容   : 新生成函数

*****************************************************************************/
void RcvRs485DataFromIsr(uint8_t tmp)
{
    static unsigned char ucRcvDataState = 0, ucRcvDataPtr = 0;  
    unsigned char i;
    
//    if (Rs485Data.bRs485PacketRcvSuccess == TRUE)
//    {
//        return;  //接收的包还没处理
//    }
    
    switch (ucRcvDataState)
    {
        case 0:
          //if ((tmp == 0xB7) ||(tmp == 0xA6) || (tmp == 0x7E)) //是否收到包开始字节  7E 
		  if ((tmp == 0x3A) || (tmp == 0x7E)) //包头
          {
              ucRcvDataState = 1;
              ucRcvDataPtr = 0; 
              Rs485Com_RX_BUF[ucRcvDataPtr ++] = tmp;
          }
          else
          {
              ucRcvDataState = 0;
          }
          break;

        case 1:
          Rs485Com_RX_BUF[ucRcvDataPtr ++] = tmp;
          if (ucRcvDataPtr >= 200) 
          {
              ucRcvDataState = 0;
              ucRcvDataPtr   = 0;
          }

		  if (Rs485Com_RX_BUF[ucRcvDataPtr - 1] == 0x0A) //判断结束标志
          {
			  Rs485Data.RecLen = ucRcvDataPtr;
              
			  if((Rs485Com_RX_BUF[0] == 0x7E) && (Rs485Com_RX_BUF[1] == 0x55))
			  {
				  #ifdef RCV_DATA_1527_MSB
				  if (CheckRs485DataPacket(Rs485Com_RX_BUF,2,5) == CHECK_SUCCESS)
				  {
					  for(i = 0;i < MotoData.RecLen;i++)
					  {
						  ConfigBuf[i] = Rs485Com_RX_BUF[i];
					  }
					  
					  MotoData.bMotoPacketRcvSuccess = TRUE;
					  recConfigFlag = TRUE;
				  }
				  #else
				  if(Rs485Com_RX_BUF[2] == 0x03)
				  {
					  if (CheckRs485DataPacket(Rs485Com_RX_BUF,2,6) == CHECK_SUCCESS)
					  {
						  for(i = 0;i < Rs485Data.RecLen;i++)
						  {
							 ConfigBuf[i] = Rs485Com_RX_BUF[i];
						  }

						  //Rs485Data.bRs485PacketRcvSuccess = TRUE;
						  recConfigFlag = TRUE;
					  }
				  }
				  else
				  {
                      //7E 55 01 5A A5 00 0A
					  if (CheckRs485DataPacket(Rs485Com_RX_BUF,2,5) == CHECK_SUCCESS)
					  {
						  for(i = 0;i < Rs485Data.RecLen;i++)
						  {
							  ConfigBuf[i] = Rs485Com_RX_BUF[i];
						  }
						  
						  //Rs485Data.bRs485PacketRcvSuccess = TRUE;
						  recConfigFlag = TRUE;
					  }
				  }
				  #endif
			  }
              
              ucRcvDataState = 0;
              ucRcvDataPtr   = 0;
          }
          break;
        default :
          ucRcvDataState = 0;
          ucRcvDataPtr   = 0;
          break;
    }
}

//(30.0V3.8V012612866104025832856898607b2111790015610C43F124A48FC00123000053E481XR701-F303-V1.0.8)
/*****************************************************************************
 函 数 名  : Rs485RspCheckState
 功能描述  : 测试工具应答数据包
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2018年7月17日, 星期二
    作    者   : Billy yu
    修改内容   : 新生成函数

*****************************************************************************/
void Rs485RspCheckState(void)
{
    CPU_SR_ALLOC();
    uint8_t tmpBuf[100];
    uint8_t i;
    
    tmpBuf[0] = '(';  //包头
    
    //外部电压
    tmpBuf[1] = ValueToChar(DeviceInfo.InputValue/10000);  //26000mV
    tmpBuf[2] = ValueToChar(DeviceInfo.InputValue%10000/1000); 
    tmpBuf[3] = '.';
    tmpBuf[4] = ValueToChar(DeviceInfo.InputValue%1000/100);
    tmpBuf[5] = 'V';
    
    //电池电压
    tmpBuf[6] = ValueToChar(DeviceInfo.VbatValue/1000);  //4120mV
    tmpBuf[7] = '.';
    tmpBuf[8] = ValueToChar(DeviceInfo.VbatValue%1000/100); 
    tmpBuf[9] = 'V';
    
    //SIM卡状态，0=未插卡，1=已插卡
    if(gprs_get_state() > GPRS_INIT)
    {
        tmpBuf[10] = '1';
    }
    else
    {
        tmpBuf[10] = '0';
    }
    
    //平台登录状态，0=未上线，1=已上线
    if (PlatformData.DeviceRegisterSuccess == TRUE)
    {
        tmpBuf[11] = '1';
    }
    else
    {
        tmpBuf[11] = '0';
    }
    
    //GSM信号强度0--31
    tmpBuf[12] = ValueToChar(DeviceInfo.ModuleSignalValue % 100 /10); 
    tmpBuf[13] = ValueToChar(DeviceInfo.ModuleSignalValue%10);
    
    //GPS卫星数0--25
    tmpBuf[14] = ValueToChar(DeviceInfo.GpsSatelliteTotal % 100 / 10);
    tmpBuf[15] = ValueToChar(DeviceInfo.GpsSatelliteTotal % 10);
    
    //IMEI 866104025832856
    for (i = 0; i < 15; i ++)
    {
        tmpBuf[16 + i] = PlatformData.DeviceID[i];
    }
    
    //ICCID 898607b2111790015610
    for (i = 0; i < 20; i ++)
    {
        tmpBuf[31 + i] = DeviceInfo.SimCCIDNumber[i];
    }
    
    //蓝牙地址 C43F124A48FC
    for (i = 0; i < 12; i++)
    {
        tmpBuf[51 + i] = BleData.BleMAC[i];
    }
    
    //发送成功计数 0--65535
    tmpBuf[63] = ValueToChar(PlatformData.StatisticSendSuccessSum / 10000);
    tmpBuf[64] = ValueToChar(PlatformData.StatisticSendSuccessSum % 10000/1000);
    tmpBuf[65] = ValueToChar(PlatformData.StatisticSendSuccessSum % 1000/100);
    tmpBuf[66] = ValueToChar(PlatformData.StatisticSendSuccessSum % 100/10);
    tmpBuf[67] = ValueToChar(PlatformData.StatisticSendSuccessSum % 10);
    
    //发送失败计数 0--65535
    tmpBuf[68] = ValueToChar(PlatformData.StatisticSendFailSum / 10000);
    tmpBuf[69] = ValueToChar(PlatformData.StatisticSendFailSum % 10000/1000);
    tmpBuf[70] = ValueToChar(PlatformData.StatisticSendFailSum % 1000/100);
    tmpBuf[71] = ValueToChar(PlatformData.StatisticSendFailSum % 100/10);
    tmpBuf[72] = ValueToChar(PlatformData.StatisticSendFailSum % 10);
    
    //遥控码 00000---FFFFF
    tmpBuf[73] = ValueToChar(DeviceData.KeyAddress >> 16);
    tmpBuf[74] = ValueToChar((DeviceData.KeyAddress & 0x0F000) >> 12);
    tmpBuf[75] = ValueToChar((DeviceData.KeyAddress & 0x00F00) >> 8);
    tmpBuf[76] = ValueToChar((DeviceData.KeyAddress & 0x000F0) >> 4);
    tmpBuf[77] = ValueToChar((DeviceData.KeyAddress & 0x0000F));
    
    //版本号，XR701-F303-V1.0.7
    for (i = 0; i < 17; i ++)
    {
        tmpBuf[78 + i] = app_Version[i];
    }
    tmpBuf[95] = ')';
    
    OS_CRITICAL_ENTER();
    RS485_Send_Data(tmpBuf,96);
    OS_CRITICAL_EXIT();
    
    #ifdef  DEBUG_OUTPUT0
    printf("\r\nSendToPc: ");
    for(i = 0;i < 96;i++)
       printf("%02X ",tmpBuf[i]); 
    printf("\r\n");
	#endif
}

void Open_all_IO(void)
{
	OpenOneKeyStartPin();
	OpenCenterCtrlPin();
    OpenSeatLock();
    OpenBattLock();
    OpenDoubleFlash();
}

void Close_all_IO(void)
{
	CloseOneKeyStartPin();
	CloseCenterCtrlPin();
    CloseSeatLock();
    CloseBattLock();
    CloseDoubleFlash();
}

/*****************************************************************************
 函 数 名  : Rs485PacketParse
 功能描述  : 测试工具命令处理
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2018年7月17日, 星期二
    作    者   : Billy yu
    修改内容   : 新生成函数

*****************************************************************************/
void Rs485PacketParse(void) 
{
    #ifdef  DEBUG_OUTPUT0
    uint8_t i;
    #endif
    
    //收到数据包，解析数据
	if(recConfigFlag == TRUE)
	{
        DEBUG_PRINT("REM_ADDRESS:%05X",RemKeyData.KeyAddress);
        DEBUG_PRINT("CmdType: %02X\r\n",ConfigBuf[2]); 
        
        query_lio_batt_cnt = 0;
		recConfigFlag = FALSE;
		
		switch(ConfigBuf[2])
		{
		  case 0x01:  //打开GPIO  7E 55 01 5A A5 00 0A

              DEBUG_PRINT("\r\n打开GPIO!!!");

			  Open_all_IO();
		  break;
		  
		  case 0x02:  //关闭GPIO  7E 55 02 5A A5 FF 0A

              DEBUG_PRINT("\r\n关闭GPIO!!!");

			  Close_all_IO();
		  break;
		  
		  case 0x03:  //配置遥控ID     7E 55 03 1C 60 81 0A /   7E 55 03 03 4E 81 2B 0A

              DEBUG_PRINT("\r\n配置遥控器ID!!!");

			  #ifdef RCV_DATA_1527_MSB
			  RemKeyData.KeyAddress = (ConfigBuf[3] << 8) | ConfigBuf[4];	
			  #else
			  RemKeyData.KeyAddress = ((ConfigBuf[3] << 16) | (ConfigBuf[4] << 8) | ConfigBuf[5]) & 0x000FFFFF;	
			  #endif

              DEBUG_PRINT("\r\nNEW_ADDRESS:%05X",RemKeyData.KeyAddress);

			  SaveRemCtrState();  //保存新遥控器ID
		  break;   

          case 0x04:  //进入配对模式 7E 55 04 5A A5 FD 0A  
              RemKeyData.PairingMode = TRUE;
          break;
          
          case 0x07:  //查询状态  7E 55 07 5A A5 FA 0A 
			  //DispTaskInfo();
              Rs485RspCheckState(); //应答测试软件查询设备信息
              //DebugStatusInfo();
		  break;

		  default :
		  break;
		}
	}
	
	#ifdef  DEBUG_OUTPUT0
	#ifdef RCV_DATA_1527_MSB
	DEBUG_PRINT("\r\nREM_ADDRESS:%04X",RemKeyData.KeyAddress);
	#else
	DEBUG_PRINT("\r\nREM_ADDRESS:%05X",RemKeyData.KeyAddress);
	#endif
	DEBUG_PRINT("\r\nRs485RcvBuf: ");
    for(i = 0;i < 7;i++)
       printf(" %02X",ConfigBuf[i]); 
	#endif
}

//******************************************************************************
void Rs485RcvDataProc(void)
{
    //判断是否收到数据包
    if (Rs485Data.bRs485PacketRcvSuccess == TRUE)
    {
        Rs485PacketParse();
    }
    Rs485Data.bRs485PacketRcvSuccess = FALSE;
}

/*
*********************************************************************************************************
*	函 数 名: ctrl_cmd_proc
*	功能说明: 锂电gps发的控制命令处理
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
#ifdef ENABLE_MEM_MANAGEMENT
extern OS_MEM UserBufPoolMem;
#endif
static uint8_t *p485Mem = NULL;
void ctrl_cmd_proc(uint8_t *buf)
{
    OS_ERR err;
    static uint8_t firstUpPwr = FALSE;
    //uint8_t tempBuf[4]={0};
 
    if(buf[4] == 0xF0) return;
    
    p485Mem = OSMemGet((OS_MEM      *)&UserBufPoolMem,
                       (OS_ERR      *)&err);

    if((p485Mem == NULL) && (err==OS_ERR_MEM_NO_FREE_BLKS))
    {
        DEBUG_PRINT("===p485Mem malloc fail!===\r\n");
        return ;
    }
    
    switch(buf[4])
    {
        case 0x08: //设防
            firstUpPwr = TRUE;
            p485Mem[0] = 0x08; 
            p485Mem[1] = 0x00;
            p485Mem[2] = 0x00;
            p485Mem[3] = 0x00;
            break;
        
        case 0x04: //启动
            firstUpPwr = TRUE;
            p485Mem[0] = 0x01; 
            p485Mem[1] = 0x00;
            p485Mem[2] = 0x00;
            p485Mem[3] = 0x00;
            break;
        
        case 0x00: //解防
            if(firstUpPwr == TRUE)
            {
                p485Mem[0] = 0x04; 
                p485Mem[1] = 0x00;
                p485Mem[2] = 0x00;
                p485Mem[3] = 0x00;
            }
            break;
        
        default:
        break;
    }
    
    if(firstUpPwr == TRUE)
    {
        OSTaskQPost ((OS_TCB      *)&RFDecodeTaskTCB,
                     (void        *)p485Mem,
                     (OS_MSG_SIZE  )4,
                     (OS_OPT       )OS_OPT_POST_FIFO,
                     (OS_ERR      *)&err);
    }
}

/*
*********************************************************************************************************
*	函 数 名: rs485_config_proc
*	功能说明: 处理RS485测试命令数据
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
//3A 16 FF 01 xx sum1 sum2 0D 0A   -- 0xFF -- RS485配置命令，xx -- 子命令
void rs485_config_proc(uint8_t *buf)
{ 
    query_lio_batt_cnt = 0;
    
    #ifdef  DEBUG_OUTPUT0
    Debug_printf(DEBUG_485,"REM_ADDRESS:%05X",RemKeyData.KeyAddress);
    Debug_printf(DEBUG_485,"CmdType: %02X\r\n",buf[4]); 
    #endif
    
    switch(buf[4])
    {
      case 0x01:  //打开GPIO  3A 16 FF 01 01 17 01 0D 0A 
          DEBUG_UART("\r\n打开GPIO!!!");
          Open_all_IO();
      break;
      
      case 0x02:  //关闭GPIO  3A 16 FF 01 02 18 01 0D 0A 
          DEBUG_UART("\r\n关闭GPIO!!!");
          Close_all_IO();
      break;
      
//		  case 0x03:  //配置遥控ID     7E 55 03 1C 60 81 0A /   7E 55 03 03 4E 81 2B 0A
//              #ifdef DEBUG_OUTPUT
//              Debug_printf(DEBUG_485,"\r\n配置遥控器ID!!!");
//              #endif
//			  #ifdef RCV_DATA_1527_MSB
//			  RemKeyData.KeyAddress = (ConfigBuf[3] << 8) | ConfigBuf[4];	
//			  #else
//			  RemKeyData.KeyAddress = ((ConfigBuf[3] << 16) | (ConfigBuf[4] << 8) | ConfigBuf[5]) & 0x000FFFFF;	
//			  #endif
//              #ifdef  DEBUG_OUTPUT
//              Debug_printf(DEBUG_485,"\r\nNEW_ADDRESS:%05X",RemKeyData.KeyAddress);
//              #endif
//			  SaveRemCtrState();  //保存新遥控器ID
//		  break;   

      case 0x04:  //进入配对模式 3A 16 FF 01 04 1A 01 0D 0A  
          RemKeyData.PairingMode = TRUE;
      break;
      
      case 0x07:  //查询状态  3A 16 FF 01 07 1D 01 0D 0A 
          Rs485RspCheckState(); //应答测试软件查询设备信息
          //DebugStatusInfo();
      break;

      default :
      break;
    }
}

/*
*********************************************************************************************************
*	函 数 名: lio_batt_proc
*	功能说明: 处理锂电返回数据
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t tmp_code[16]={0};
void lio_batt_proc(uint8_t *buf)
{
    bms_batt_em recv_state;
    uint8_t i,j,k;
    uint16_t tmp = 0;

    recv_state = (bms_batt_em)buf[2];
    switch (recv_state) //判断命令字
    {
//--------------------RS485测试工具命令处理---------------------------------------------
        case RS485_CONFIG_CMD: 
            rs485_config_proc(buf);
            break;
//--------------------向601查询控制命令的应答------------------------------------------- 
        case QUERY_CTRL_CMD:        //示例:3A 16 F0 01 xx 07 01 0D 0A  -- xx低4bit表示601当前的遥控器状态，00:解防，04:启动，08:设防
            ctrl_cmd_proc(buf);
        break;

//--------------------向601通知遥控状态的应答-------------------------------------------    
    case RESPONSE_CTRL_CMD:         //示例:3A 16 F1 01 00 08 01 0D 0A
        RemSyncCount = 0;
        RemSetCmdFlag = FALSE;
        break;
    
//--------------------查询电池温度-------------------------------------------------------
    case QUERY_BATT_TEMPERATURE:    //示例:3A 16 08 02 FE 0B 29 01 0D 0A
        query_batt_state = QUERY_BATT_TOTAL_VOLT;
        tmp = ((buf[5] << 8) | buf[4]);
        Rs485Data.temperature = tmp - 2731;
#ifdef DEBUG_OUTPUT
        if(rs485_output_enable == TRUE)
        {
            DEBUG_UART("\r\ntmp=%d,temperature=%d\r\n",tmp,Rs485Data.temperature);
        }
#endif
        break;

//--------------------查询电池总电压----------------------------------------------------        
    case QUERY_BATT_TOTAL_VOLT:    //示例:3A 16 09 04 0D E8 00 00 18 01 0D 0A 
        query_batt_state = QUERY_BATT_CURRENT;
        Rs485Data.total_volt = ((buf[7] << 24) |(buf[6] << 16) |(buf[5] << 8) | buf[4]);

        if(rs485_output_enable == TRUE)
        {
            DEBUG_UART("\r\ntotal_volt=%dmV\r\n",Rs485Data.total_volt);
        }       
        break;
        
//--------------------查询实时电流-------------------------------------------------------        
    case QUERY_BATT_CURRENT:     //示例: 3A 16 0A 04 00 00 00 00 24 00 0D 0A 
        query_batt_state = QUERY_BATT_RELATIVE_CAPACITY;
        
        Rs485Data.ave_current = ((buf[7] << 24) |(buf[6] << 16) |(buf[5] << 8) | buf[4]);
        if(Rs485Data.ave_current > 0)
            Rs485Data.ave_current = ~(Rs485Data.ave_current-1);
    
        if(rs485_output_enable == TRUE)
        {
            printf("\r\nave_current=%dmA\r\n",Rs485Data.ave_current);
        }     
        break;
        
//--------------------查询电池相对容量---------------------------------------------------        
    case QUERY_BATT_RELATIVE_CAPACITY:    //示例:3A 16 0D 01 00 24 00 0D 0A  
        query_batt_state = QUERY_BATT_ABSOLUTE_CAPACITY;
        Rs485Data.soc1 = buf[4];
  
        if(rs485_output_enable == TRUE)
        {
            DEBUG_UART("\r\n===soc1=%d===\r\n",Rs485Data.soc1);
        }        
        break;
        
//---------------------查询电池绝对容量--------------------------------------------------        
    case QUERY_BATT_ABSOLUTE_CAPACITY:   //示例:3A 16 0E 01 00 25 00 0D 0A 
        query_batt_state = QUERY_BATT_RESIDUE_CAPACITY;
        Rs485Data.soc2 = buf[4];
      
        if(rs485_output_enable == TRUE)
        {
            DEBUG_UART("\r\n===soc2=%d===\r\n",Rs485Data.soc2);
        }       
        break;
        
//---------------------查询电池剩余容量------------------------------------------------  
    case QUERY_BATT_RESIDUE_CAPACITY:    //示例:3A 16 0F 02 54 3D B8 00 0D 0A 
        //if(LionBatData.full_capacity == 0) //开机查一次
            query_batt_state = QUERY_BATT_FULL_CAPACITY;
        //else
        //    query_batt_state = QUERY_BATT_STAUTS;
        
        Rs485Data.residue_capacity = ((buf[5] << 8) | buf[4]);

        if(rs485_output_enable == TRUE)
        {
            DEBUG_UART("\r\nresidue_capacity=%dmAh\r\n",Rs485Data.residue_capacity);
        }   
        break;
        
//---------------------查询电池满充容量----------------------------------------------        
    case QUERY_BATT_FULL_CAPACITY:     //示例:3A 16 10 02 80 3E E6 00 0D 0A
        query_batt_state = QUERY_BATT_STAUTS;
        Rs485Data.full_capacity = ((buf[5] << 8) | buf[4]);
      
        if(rs485_output_enable == TRUE)
        {
            DEBUG_UART("\r\nfull_capacity=%dmAh\r\n",Rs485Data.full_capacity);
        }  
        break;
        
//---------------------查询电池状态--------------------------------------------------
    case QUERY_BATT_STAUTS:     //示例:3A 16 16 03 08 46 14 91 00 0D 0A   
        query_batt_state = QUERY_BATT_CYCLE_INDEX;
        Rs485Data.status[0] = buf[4];
        Rs485Data.status[1] = buf[5];
        Rs485Data.status[2] = buf[6];
        
        if(rs485_output_enable == TRUE)
        {
            DEBUG_UART("\r\nstatus_0=0x%X,nstatus_1=0x%X,nstatus_2=0x%X\r\n",\
                Rs485Data.status[0],Rs485Data.status[1],Rs485Data.status[2]);
        }
        break;
        
//---------------------查询电池循环次数------------------------------------------------  
    case QUERY_BATT_CYCLE_INDEX:    //示例:3A 16 17 02 00 00 2F 00 0D 0A 
        //if(LionBatData.manufacturer_name[0] == 0x00) 
            query_batt_state = QUERY_BATT_MANUF_NAME;
       //else
       //     query_batt_state = QUERY_BATT_ONE_SEVEN_VOLT;
        
        Rs485Data.cycle_charge_cnt = ((buf[5] << 8) | buf[4]);
    
        if(rs485_output_enable == TRUE)
        {
            DEBUG_UART("\r\ncycle_charge_cnt=%d\r\n",Rs485Data.cycle_charge_cnt);
        }
        break;
        
//---------------------查询电池组制造商(GUOSHENG BATTERY)--------------------------------        
    case QUERY_BATT_MANUF_NAME:    //示例：3A 16 20 10 47 55 4F 53 48 45 4E 47 20 42 41 54 54 45 52 59 E1 04 0D 0A
        query_batt_state = QUERY_BATT_ONE_SEVEN_VOLT;
        for(i = 0; i < buf[3]; i++)
        {
            Rs485Data.manufacturer_name[i] = buf[i+4];
        }
        Rs485Data.manufacturer_name[i] = '\0';
        
        if(rs485_output_enable == TRUE)
        {
            DEBUG_UART("\r\n%s",Rs485Data.manufacturer_name);
            /*printf("\r\nmanufacturer:");
            for(i = 0; i < buf[3]; i++)
                printf("%02X ",Rs485Data.manufacturer_name[i]);*/
        }
        break;
    
//---------------------查询1~7电芯电压-----------------------------------------------------
//示例:3A 16 24 0E 89 0C 7A 0C 6D 0C 85 0C 85 0C 66 0C 89 0C 05 04 0D 0A 
    case QUERY_BATT_ONE_SEVEN_VOLT:    
        query_batt_state = QUERY_BATT_EIGHT_TWENTY_VOLT;
        k = 0;
        for(i = 5; i < (buf[3]+5); i+=2,k++)
        {
            for(j = 0; j < 2; j++)
            {
                Rs485Data.cell_volt[k][j] = buf[i-j];
            }
        }
 
        if(rs485_output_enable == TRUE)
        {
            for(i = 0; i < 7; i++)
                DEBUG_UART("\r\ncell_volt_%d=%dmV",i+1,(Rs485Data.cell_volt[i][0]<<8|Rs485Data.cell_volt[i][1]));
        }
        break;
    
//---------------------查询8~20电芯电压---------------------------------------------------
//示例:3A 16 25 18 8C 0C 8A 0C 67 0C BA 0B F0 0B EE 0B EC 0B EA 0B F3 0B F1 0B E9 0B F2 0B 84 0A 0D 0A        -- 19串
     //3A 16 25 1A 87 0E 61 0E EB 0D 26 0E CE 0D F1 0D 24 0E 14 0E C9 0D 20 0E 27 0E FD 0D 00 00 F5 06 0D 0A  -- 20串
    case QUERY_BATT_EIGHT_TWENTY_VOLT: 
        query_batt_state = QUERY_BATT_CUR_CHARGE_TIME;
        k = 7;
        for(i = 5; i < (buf[3]+5); i+=2,k++)
        {
            for(j = 0; j < 2; j++)
                Rs485Data.cell_volt[k][j] = buf[i-j];
        }
        
        if(rs485_output_enable == TRUE)
        {
            for(i = 7; i < 20; i++)
                DEBUG_UART("\r\ncell_volt_%d=%dmV",i+1,(Rs485Data.cell_volt[i][0]<<8|Rs485Data.cell_volt[i][1]));
        }
        break;
    
//---------------------当前充电间隔时间--------------------------------------------------- 
    case QUERY_BATT_CUR_CHARGE_TIME:    //示例:3A 16 47 02 FF FF 5D 02 0D 0A 
        query_batt_state = QUERY_BATT_MAX_CHARGE_TIME;
        Rs485Data.cur_charge_time = ((buf[5] << 8) | buf[4]);
         
        if(rs485_output_enable == TRUE)
        {
            DEBUG_UART("\r\ncur_charge_time=%d\r\n",Rs485Data.cur_charge_time);
        }
        break;

//---------------------最大充电间隔时间---------------------------------------------------         
    case QUERY_BATT_MAX_CHARGE_TIME:    //示例:3A 16 48 02 00 00 60 00 0D 0A 
        query_batt_state = QUERY_BATT_SOH;
        Rs485Data.max_charge_time = ((buf[5] << 8) | buf[4]);
    
        if(rs485_output_enable == TRUE)
        {
            DEBUG_UART("\r\nmax_charge_time=%d\r\n",Rs485Data.max_charge_time);
        }
        break;

//---------------------查询电池SOH------------------------------------------------------         
    case QUERY_BATT_SOH:    //示例:3A 16 0C 02 64 00 88 00 0D 0A 
        query_batt_state = QUERY_BATT_VERSION;
        Rs485Data.soh = buf[4];
        
        if(rs485_output_enable == TRUE)
        {
            DEBUG_UART("\r\n===soh=%d===\r\n",Rs485Data.soh);
        }
        break;
        
//---------------------读取版本---------------------------------------------------------- 
    case QUERY_BATT_VERSION:    //示例:3A 16 7F 03 00 65 64 61 01 0D 0A
        //if(LionBatData.id_code[0] == 0x00)
            query_batt_state = QUERY_BATT_ID_CODE;
        //else
        //    query_batt_state = QUERY_BATT_POWER_OUTPUT;
        
        Rs485Data.sf_ver = buf[5];
        Rs485Data.hw_ver = buf[6];
    
        if(rs485_output_enable == TRUE)
        {
            DEBUG_UART("\r\nsf_ver=V%d\r\n",Rs485Data.sf_ver);
            DEBUG_UART("\r\nhw_ver=V%d\r\n",Rs485Data.hw_ver);
        }
        break;

//---------------------读取成品条形码----------------------------------------------------------                
    case QUERY_BATT_ID_CODE: 
        query_batt_state = QUERY_BATT_POWER_OUTPUT;
        for(i = 0; i < buf[3]; i++)
        {
            Rs485Data.id_code[i] = buf[i+4];
        }

        //ID不一样，表示已发生换电，需要立即上报新电池ID
        for(i = 0; i < buf[3]; i++)
        {
            if(Rs485Data.id_code[i] != tmp_code[i])
            {
                DEBUG_UART("\r\n检测到新电池插入!!!");
                
                for(i = 0; i < buf[3]; i++)
                {
                    tmp_code[i] = Rs485Data.id_code[i];
                }
                DeviceInfo.SimBmsIdUpFlag = FALSE;
                DeviceInfo.SendBmsIdTimeCnt = FIVE_SECOND;
            }
        }
        
        if(rs485_output_enable == TRUE)
        {
            DEBUG_UART("\r\nid_code:%s",Rs485Data.id_code);
//            printf("\r\nid_code:");
//            for(i = 0; i < buf[3]; i++)
//                printf("%02X ",Rs485Data.id_code[i]);
        }                 
        break;
 
//---------------------查询BMS供电输出----------------------------------------------------------
    case QUERY_BATT_POWER_OUTPUT:  //示例:3A 16 81 01 00 98 00 0D 0A 
        query_batt_state = QUERY_BATT_OVER_DISCHARGE_PROTE;
        
        if(buf[4] == 0x01) //01:供电打开，00:供电关闭
        {
            gCarFunctionState |= FUNC_BATT_POWER_OUT_BIT;
            
            if(rs485_output_enable == TRUE)
            {
                DEBUG_UART("\r\nBMS供电输出已打开!!!\r\n");
            }       
            Rs485Data.power_output_state = 0x01;
            
        }
        else if(buf[4] == 0x00)
        {
            gCarFunctionState &= ~FUNC_BATT_POWER_OUT_BIT;

            if(rs485_output_enable == TRUE)
            {
                DEBUG_UART("\r\nBMS供电输出已关闭!!!\r\n");
            }
            Rs485Data.power_output_state = 0x00;
        }
        break;

//---------------------应答设置BMS供电输出-------------------------------------------------------
    case CTRL_BATT_POWER_OUTPUT:  //示例:3A 16 80 01 65 FC 00 0D 0A
        if(rs485_output_enable == TRUE)
        {
            DEBUG_UART("应答设置BMS供电输出\r\n");
        }  
        break;

//---------------------查询电池过放保护----------------------------------------------------------        
    case QUERY_BATT_OVER_DISCHARGE_PROTE:  //示例:3A 16 82 01 01 9A 00 0D 0A
        query_batt_state = QUERY_BATT_TEMPERATURE;
        if(buf[4] == 0x01) //00:正常，01:电量过低
        {
            if(rs485_output_enable == TRUE)
            {
                DEBUG_UART("\r\nBMS电量过低，需要保护!!!\r\n");
            }
            Rs485Data.over_discharge_state = 0x01;
            gCarFunctionState |= FUNC_BATT_OVER_DISCHARGE;
        }
        else if(buf[4] == 0x00)
        {
            if(rs485_output_enable == TRUE)
            {
                DEBUG_UART("\r\nBMS电量正常，不需要保护!!!\r\n");
            }

            Rs485Data.over_discharge_state = 0x00;
            gCarFunctionState &= ~FUNC_BATT_OVER_DISCHARGE;
        }
        break;

//---------------------应答设置BMS充电类型-------------------------------------------------------
    case CTRL_BATT_CHARGE_TYPE:  //示例:3A 16 83 01 01 A7 00 0D 0A 
        if(rs485_output_enable == TRUE)
        {
            DEBUG_UART("应答设置BMS充电类型\r\n");
        }

        break;
        
    default:
        break;
    }
}

/*
*********************************************************************************************************
*	函 数 名: cycle_send_query_cmd 
*	功能说明: 循环定时发送查询命令
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void cycle_send_query_cmd(void)
{
    switch(query_batt_state)
    {
        case QUERY_BATT_TEMPERATURE: //查询电池温度
            lionBat_Send_cmd(QUERY_BATT_TEMPERATURE,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_TOTAL_VOLT: //查询电池组总电压
            lionBat_Send_cmd(QUERY_BATT_TOTAL_VOLT,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_CURRENT: //查询实时电流
            lionBat_Send_cmd(QUERY_BATT_CURRENT,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_RELATIVE_CAPACITY: //查询相对容量
            lionBat_Send_cmd(QUERY_BATT_RELATIVE_CAPACITY,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_ABSOLUTE_CAPACITY: //查询绝对容量
            lionBat_Send_cmd(QUERY_BATT_ABSOLUTE_CAPACITY,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_RESIDUE_CAPACITY: //查询剩余容量
            lionBat_Send_cmd(QUERY_BATT_RESIDUE_CAPACITY,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_FULL_CAPACITY: //查询满电容量
            lionBat_Send_cmd(QUERY_BATT_FULL_CAPACITY,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_STAUTS:        //查询电池状态
            lionBat_Send_cmd(QUERY_BATT_STAUTS,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_CYCLE_INDEX: //查询循环次数
            lionBat_Send_cmd(QUERY_BATT_CYCLE_INDEX,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_MANUF_NAME: //查询电池组制造厂名称
            lionBat_Send_cmd(QUERY_BATT_MANUF_NAME,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_ONE_SEVEN_VOLT: //查询1~7节电池电压
            lionBat_Send_cmd(QUERY_BATT_ONE_SEVEN_VOLT,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_EIGHT_TWENTY_VOLT: //查询8~20节电池电压
            lionBat_Send_cmd(QUERY_BATT_EIGHT_TWENTY_VOLT,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_CUR_CHARGE_TIME: //查询当前充电间隔时间
            lionBat_Send_cmd(QUERY_BATT_CUR_CHARGE_TIME,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_MAX_CHARGE_TIME: //查询最大充电间隔时间
            lionBat_Send_cmd(QUERY_BATT_MAX_CHARGE_TIME,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_SOH: //查询SOH
            lionBat_Send_cmd(QUERY_BATT_SOH,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_VERSION: //查询版本号
            lionBat_Send_cmd(QUERY_BATT_VERSION,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_ID_CODE: //查询电池组ID条码
            lionBat_Send_cmd(QUERY_BATT_ID_CODE,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_POWER_OUTPUT: //查询BMS供电输出
            lionBat_Send_cmd(QUERY_BATT_POWER_OUTPUT,PARAMETER_FALSE);
            
            break;
        
        case QUERY_BATT_OVER_DISCHARGE_PROTE: //查询电池过放保护
            lionBat_Send_cmd(QUERY_BATT_OVER_DISCHARGE_PROTE,PARAMETER_FALSE);
            
            break;
        
        default :
            break;
    }
}

void DebugStatusInfo(void)
{
    uint8_t i;
    
    printf("\r\n-----------------------------------------");
    printf("\r\nAPP_version:%s",app_Version);
    printf("\r\nDate:%s Time:%s",__DATE__, __TIME__);

    if(DeviceInfo.ucAdminState)
    {
        for (i = 0; i < 5; i ++)
        {
             if ((DeviceInfo.Telephone[i].Number[0] != 0))
                printf("\r\nAdmin Number[%d]:%s",i+1,DeviceInfo.Telephone[i].Number);
        }
    }
    else
    {
        printf("\r\nAdmin Flag: No Admin");
    }
    printf("\r\nmain IP:%d.%d.%d.%d Port:%d",PlatformData.PlatformIpAddr[0],
    PlatformData.PlatformIpAddr[1],PlatformData.PlatformIpAddr[2],
    PlatformData.PlatformIpAddr[3],PlatformData.PlatformPort);
 
#ifdef DOUBLE_IP_FUN     
    printf("\r\nBackup IP:%d.%d.%d.%d Port:%d",PlatformData.BackupIpAddr[0], 
    PlatformData.BackupIpAddr[1],PlatformData.BackupIpAddr[2],
    PlatformData.BackupIpAddr[3],PlatformData.PlatformPort);
#endif
    
    printf("\r\nHY Protocol"); 
    Debug_PrintData("Device ID:",PlatformData.DeviceID,PlatformData.DeviceID+15); 
    
    printf("\r\nPassword:%2.6s",DeviceInfo.ucPassword);
    printf("\r\nTimeZone:%d",DeviceData.TimeZone);
    printf("\r\nReportTime: %dS", DeviceInfo.AccOpenInterval);
    //printf("\r\nAcc Off: %dS", DeviceInfo.AccCloseInterval);
    printf("\r\nAPN:%s",DeviceData.GprsApn);
    if ((GpsFunctionState & FUNC_VIBRATION_ON_OFF_BIT) != 0)
    {
        printf("\r\nVib On! Vib Level-->%d",GetShakeLevel());
    }
    else
    {
        printf("\r\nVib Off!");
    }
    printf("\r\nSendFailSum:%d",PlatformData.StatisticSendFailSum);
    printf("\r\nSendSuccessSum:%d",PlatformData.StatisticSendSuccessSum);
    //printf("\r\nBlindWriteCount:%d", RecordState.WriteOffsetPtr);
    //printf("\r\nBlindReadCount:%d", RecordState.ReadOffsetPtr);
    printf("\r\nGSM Rssi:%d",DeviceInfo.ModuleSignalValue);
    printf("\r\nGPS C/N:%d",DeviceInfo.GpsSatelliteTotal);
    printf("\r\nBat_Voltage=%.2fV",(float)DeviceInfo.VbatValue/1000);
    printf("\r\nExt_Voltage=%.2fV",(float)DeviceInfo.InputValue/1000);
    printf("\r\n-----------------------------------------");
}

/*
*********************************************************************************************************
*	函 数 名: rs485_check
*	功能说明: 判断输入命令
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void rs485_check(uint8_t *buf,uint16_t len)
{
    if((buf[0] != 0x3A) && (buf[1] == 0x0A)) //判断包头包尾，必须是3A，0A结尾
        return;  
    
    switch(buf[1]) //判断地址
    {
        case 0x16:  //锂电池
            lio_batt_proc(buf);
            break;
        
        case 0x28:  //LED广告屏
            //ledScreen_resp(buf);
            break;
              
        default:
        break;
    }
}

//void Rs485CommRecvProc(uint8_t rev_data)
//{
//    OS_ERR err;
//    
//    if (Rs485ComRecvPtr == 0)
//    {  
//        Rs485Com.RxBuf[Rs485ComRecvPtr ++] = rev_data;
//    }
//    else
//    {
//        if (Rs485ComRecvPtr < MAX_BUF_LEN)
//        {
//            Rs485Com.RxBuf[Rs485ComRecvPtr ++] = rev_data;
//        }
//        
//        if((Rs485Com.RxBuf[Rs485ComRecvPtr-1] == 0x0A)&&(Rs485Com.RxBuf[Rs485ComRecvPtr-2]==0x0D))
//        {
//            //if(rs485_recv_sem)
//            {
//                OSSemPost(&rs485_recv_sem,OS_OPT_POST_1,&err);
//            }
//        }
//    }
//}

void Rs485CommRecvProc(uint8_t rev_data)
{
    OS_ERR err;
    
    if (Rs485ComRecvPtr == 0)
    {  
        if (rev_data == 0x3A) //是否收到包开始字节
            Rs485Com.RxBuf[Rs485ComRecvPtr ++] = rev_data;
    }
    else
    {
        if (Rs485ComRecvPtr < MAX_BUF_LEN)
        {
            Rs485Com.RxBuf[Rs485ComRecvPtr ++] = rev_data;
        }
        else Rs485ComRecvPtr = 0;
        
        //if((Rs485Com.RxBuf[Rs485ComRecvPtr-1] == 0x0A)&&(Rs485Com.RxBuf[Rs485ComRecvPtr-2]==0x0D))
        if((Rs485Com.RxBuf[Rs485ComRecvPtr-1] == 0x0A)
            &&(Rs485Com.RxBuf[Rs485ComRecvPtr-2]==0x0D)
            &&Rs485ComRecvPtr == (Rs485Com.RxBuf[3]+8)) //解决数据包中出现0D0A导致接收异常
        {
            //if(rs485_recv_sem)
            {
                OSSemPost(&rs485_recv_sem,OS_OPT_POST_1,&err);
            }
        }
    }
}

/*
*********************************************************************************************************
*	函 数 名: rs485_read_from_uart
*	功能说明: 从硬件串口读取数据
*	形    参：buf -- 指向接收缓存区地址
*	返 回 值: 返回接收数据长度
*********************************************************************************************************
*/
static int rs485_read_from_uart(uint8_t *buf)
{
    int recv_len = Rs485ComRecvPtr;
    
    memcpy(buf,Rs485Com.RxBuf,recv_len);
    memset(Rs485Com.RxBuf,0,RS485_MAX_BUF_LEN);
    Rs485ComRecvPtr = 0;
    
    return recv_len;
}

/*
*********************************************************************************************************
*	函 数 名: rs485_read
*	功能说明: 以阻塞方式读取缓存区数据
*	形    参：buf -- 指向接收缓存区地址
*	返 回 值: 返回接收数据长度
*********************************************************************************************************
*/
static int rs485_read(uint8_t *buf, uint32_t timeout)
{
    OS_ERR err;
    
    OSSemPend((OS_SEM      *)&rs485_recv_sem,
              (OS_TICK      )timeout,
              (OS_OPT       )OS_OPT_PEND_BLOCKING,
              (CPU_TS      *)0,
              (OS_ERR      *)&err);
    
    if(OS_ERR_TIMEOUT == err)
        return 0;
    
    return rs485_read_from_uart(buf);
}

//同步遥控器设防解防命令给601
//3A 16 F1 01 aa bb bb 0D 0A   -- aa:设防解防状态
void SyncRemCmdTo601(void)
{
    uint8_t i;
    uint16_t checkSum;
	uint8_t tmpBuf[20];
    //CPU_SR_ALLOC();
    
    //OS_CRITICAL_ENTER(); 
    //3A 16 F1 01 xx sum1 sum2 0D 0A
    tmpBuf[0] = 0x3A;  //包头
    tmpBuf[1] = 0x16;  //地址
    tmpBuf[2] = 0xF1;  //命令
    tmpBuf[3] = 0x01;  //内容长度
    
    //控制命令
    tmpBuf[4] = gCarFunctionState & (~0xF1);//高4位和bit0是控制器通讯状态位，不需要 
    
	checkSum = 0;
    for (i = 1; i < 5; i++)
    {
        checkSum += tmpBuf[i];
    }
    //累加校验和,先传低字节，再传高字节
	tmpBuf[5] = checkSum; 
    tmpBuf[6] = checkSum>>8;
    
    //结束
    tmpBuf[7] = 0x0D;
    tmpBuf[8] = 0x0A;

    rs485_send_data(tmpBuf,9);
    //OS_CRITICAL_EXIT();
    
//    #ifdef  DEBUG_OUTPUT
//    if(rs485_output_enable == TRUE)
//    {
//        printf("\r\nSyncTo601: ");
//        for(i = 0;i < 9;i++)
//           printf("%02X ",tmpBuf[i]); 
//        printf("\r\n");
//    }
//	#endif
}

/*
*********************************************************************************************************
*	函 数 名: Rs485TaskProc
*	功能说明: Rs485任务
*	形    参：p_arg --- 可选参数
*	返 回 值: 无
*********************************************************************************************************
*/  //3A 16 F0 01 00 07 01 0D 0A
uint8_t query_lio_batt_cmd[9] = {0x3A, 0x16, 0xF0, 0x01, 0x00, 0x07, 0x01, 0x0D, 0x0A};
void Rs485TaskProc(void *p_arg)
{
    //CPU_SR_ALLOC();
    #ifdef ENABLE_IDWG
    OS_ERR err;
    #endif
    uint8_t rs485len=0,i,queryFlag=0;
    (void)p_arg;
    
    //printf("\r\nrs485 task run!!!\r\n");
    
    for(;;)
    {
#ifdef FTP_UPGRAGE
        if(FtpUpFlag == FALSE)
#endif
        {
            rs485len = rs485_read(rs485_rev_buf, 100); //100*5=500ms 
            if( rs485len > 0)
            {
                #ifdef DEBUG_OUTPUT
                if(rs485_output_enable == TRUE)
                {
                    printf("Rs485RecvData:");
                    for(i = 0; i < rs485len; i++)
                    {
                        printf("%02X ",rs485_rev_buf[i]);
                    }
                    printf("\r\n");
                }
                #endif
                //3A 16 F0 01 00 07 01 0D 0A 
                if((rs485_rev_buf[0]==0x3A)&&(rs485_rev_buf[rs485len-1] == 0x0A))
                {
                    if (CheckRs485DataPacket_Add(rs485_rev_buf,1,rs485len-4) == TRUE)
                    {
                        rs485_check(rs485_rev_buf,rs485len);
                    }
                }
                
                rs485len = 0;
                memset(rs485_rev_buf, 0, sizeof(rs485_rev_buf));
            }
            
            //同步遥控状态给601 
            if((RemSetCmdFlag == TRUE)&&(RemSyncCount>0))
            {
                RemSyncCount--;
                query_lio_batt_cnt = 0;
                //OS_CRITICAL_ENTER(); 
                SyncRemCmdTo601();
                //OS_CRITICAL_EXIT();
            }
            
            if(++query_lio_batt_cnt >= 2)  //间隔1秒向BMS查询
            {
                query_lio_batt_cnt = 0;
                if(queryFlag == 0) //查询601是否有启动设防命令发给705
                {
                    queryFlag = 1;
                    
                    //OS_CRITICAL_ENTER(); 
                    rs485_send_data(query_lio_batt_cmd,9);
                    //OS_CRITICAL_EXIT();
                }
                else
                {
                    queryFlag = 0;
                    cycle_send_query_cmd();
                }
            }
        }
        
        delay_ms(50);
        
        #ifdef ENABLE_IDWG
        OSFlagPost ((OS_FLAG_GRP  *)&FLAG_TaskRunStatus,
                    (OS_FLAGS      )RS485_CAN_TASK_FEED_DOG,  /* 设置bit7 */
                    (OS_OPT        )OS_OPT_POST_FLAG_SET,
                    (OS_ERR       *)&err);
        #endif
    }
}

/*
*********************************************************************************************************
*	函 数 名: rs485_create
*	功能说明: 创建rs485任务
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void rs485_create(void)
{
    OS_ERR err;
    
    OSSemCreate ((OS_SEM    *)&rs485_send_sem,
                 (CPU_CHAR  *)"rs485_send_sem",
                 (OS_SEM_CTR )1,
                 (OS_ERR    *)&err);
    if(err != OS_ERR_NONE)
    {
        printf("\r\nCreate 'rs485_send_sem' fail!!!");
    }
    OSSemCreate ((OS_SEM    *)&rs485_recv_sem,
                 (CPU_CHAR  *)"rs485_send_sem",
                 (OS_SEM_CTR )0,
                 (OS_ERR    *)&err);
    if(err != OS_ERR_NONE)
    {
        printf("\r\nCreate 'rs485_recv_sem' fail!!!");
    }
                
    OSTaskCreate((OS_TCB     *)&Rs485TackTCB,
                 (CPU_CHAR   *)"Rs485TaskProc",
                 (OS_TASK_PTR ) Rs485TaskProc,
                 (void       *) 0,
                 (OS_PRIO     ) RS485_TASK_PRIO,
                 (CPU_STK    *)&Rs485TaskStk[0],
                 (CPU_STK_SIZE) RS485_STK_SIZE / 10,
                 (CPU_STK_SIZE) RS485_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err); 
}

#endif
