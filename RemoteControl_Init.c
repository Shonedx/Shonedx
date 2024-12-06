
#include "stm32f4xx.h"
#include "Allheaderfile.h"

#define RC_CHANNAL_ERROR_VALUE 700

//取正函数
static int16_t RC_abs(int16_t value);

//遥控器处理函数
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//遥控器控制变量
//static RC_ctrl_t rc_ctrl;
 RC_ctrl_t rc_ctrl;//为了查看rc_ctrl中的数据，将static去掉

//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t SBUS_rx_buf[2][SBUS_RX_BUF_NUM];
// uint8_t SBUS_rx_buf[2][SBUS_RX_BUF_NUM];

/**
初始化DMA，串口1
注意static定义的不可以debug查看SBUS_rx_buf中的数据
**/

		int left_x,left_y,right_x,right_y;

extern 	int left_push_stick, right_push_stick; //左上，右上推杆





void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
        /* -------------- Enable Module Clock Source ----------------------------*/
	    	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE );
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);  
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  
	
			RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,DISABLE);
			RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);
	
			GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_USART2); //¸´ÓÃ
			
	
        /* -------------- Configure GPIO ---------------------------------------*/
        {
                GPIO_InitTypeDef GPIO_InitStructure;
                USART_InitTypeDef USART_InitStructure;
                GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_Init(GPIOA, &GPIO_InitStructure);

                USART_DeInit(USART2);
			
			/*串口通信协议已由大疆所给官方手册确定,包括波特率,数据长度,校验方式*/
                USART_InitStructure.USART_BaudRate = 100000;
                USART_InitStructure.USART_WordLength = USART_WordLength_8b;
                USART_InitStructure.USART_StopBits = USART_StopBits_1;
                USART_InitStructure.USART_Parity = USART_Parity_Even;
                USART_InitStructure.USART_Mode = USART_Mode_Rx;
                USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
                USART_Init(USART2, &USART_InitStructure);

                USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

                USART_ClearFlag(USART2, USART_FLAG_IDLE);
				USART_ClearFlag(USART2,USART_FLAG_RXNE);
				
				USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
				
                USART_Cmd(USART2, ENABLE);
        }

        /* -------------- Configure NVIC ---------------------------------------*/
        {
                NVIC_InitTypeDef NVIC_InitStructure;
                NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
                NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
                NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
                NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                NVIC_Init(&NVIC_InitStructure);
			
			
                NVIC_InitStructure.NVIC_IRQChannel =DMA1_Stream5_IRQn;
                NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
                NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
                NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                NVIC_Init(&NVIC_InitStructure);
        }

        //DMA2 stream5 ch4  or (DMA2 stream2 ch4)    !!!!!!! P206 of the datasheet
		//DMA2 通道4数据流2
        /* -------------- Configure DMA -----------------------------------------*/
        {
                DMA_InitTypeDef DMA_InitStructure;
                DMA_DeInit(DMA1_Stream5);

                DMA_InitStructure.DMA_Channel = DMA_Channel_4;
                DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);
                DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx1_buf;               //£¨*£©
                DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
                DMA_InitStructure.DMA_BufferSize = dma_buf_num;
                DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
                DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
                DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
                DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
                DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
			    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium ;
                DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;//ÓÉÓÚÃ»ÓÐÊ¹ÄÜFIFO£¬Ïà¹ØÉèÖÃ²»ÓÃÌØÒâÉèÖÃ
			DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//FIFOÏà¹Ø£ºãÐÖµ
                DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//FIFOÏà¹Ø£º´æ´¢Æ÷Êý¾ÝÍ»·¢
                DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//FIFOÏà¹Ø£ºÍâÉèÍ»·¢£¬Í»·¢¿ÉÒÔÓÃÀ´½ÓÊÕ²»¶¨³¤Êý¾Ý
                DMA_Init(DMA1_Stream5, &DMA_InitStructure);
                DMA_DoubleBufferModeConfig(DMA1_Stream5, (uint32_t)rx2_buf, DMA_Memory_0);//  £¨**£©
                DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);
				DMA_Cmd(DMA1_Stream5, DISABLE);
                DMA_Cmd(DMA1_Stream5, ENABLE);
        }
}
/**²¹³补充说明
1,双缓冲模式(存储器0存储器1)
在(*)与(*)中,将rxl_buf与DMA_Memory_0,rx2_buf与DMA_Memory_1关联


FIFO是先进先出,用于在源数据传输道目标之前临时存储数据,到达阈值后DMA再
搬运数据,在本遥控实验中对数据的反应要求高,需要对接收到数据后立刻搬运到
存储器中进行数据拼接所以禁止FIFO模式,不需要将数据存到一定量再搬运做到即发即收
即收即搬
**/

//失能usart2
void RC_unable(void)
{
        USART_Cmd(USART2, DISABLE);
}
//RC 重启
void RC_restart(uint16_t dma_buf_num)
{
        USART_Cmd(USART2, DISABLE);
        DMA_Cmd(DMA1_Stream5, DISABLE);
        DMA_SetCurrDataCounter(DMA1_Stream5, dma_buf_num);

        USART_ClearFlag(USART2, USART_FLAG_IDLE);

        DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF2);
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF2);
        DMA_Cmd(DMA1_Stream5, ENABLE);
        USART_Cmd(USART2, ENABLE);
}

void remote_control_init(void)
{
		 RC_Init(SBUS_rx_buf[0], SBUS_rx_buf[1], RC_FRAME_LENGTH);
//	 RC_Init(SBUS_rx_buf[0], SBUS_rx_buf[1], SBUS_RX_BUF_NUM);

}

//?????????,????????????
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//???????????,
uint8_t RC_data_is_error(void)
{
    //???go to?? ?????????????????
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

//??RC
void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
//??RC
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
u16 a,b,c,d,e,f;
//????
static uint16_t this_time_rx_len = 0;

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//???????
    {
		a++;
		(void)USART2->SR;
		(void)USART2->DR;
		
    }
    else if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)//???????
    {
		b++;
       
		//??IDLE??????????????,????????????
        (void)USART2->SR;
		(void)USART2->DR;
		
        if(DMA_GetCurrentMemoryTarget(DMA1_Stream5)==0 )
        {
			
			c++;
            //????DMA
			this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream5);
            DMA_Cmd(DMA1_Stream5, DISABLE);
			
            
            DMA_SetCurrDataCounter(DMA1_Stream5, SBUS_RX_BUF_NUM);
            DMA1_Stream5->CR |= DMA_SxCR_CT;
			
            DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
            DMA_Cmd(DMA1_Stream5, ENABLE);
			
			SBUS_TO_RC(SBUS_rx_buf[0], &rc_ctrl);
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {	
				d++;
                //???????
                SBUS_TO_RC(SBUS_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {	
			e++;
            //????DMA
            DMA_Cmd(DMA1_Stream5, DISABLE);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream5);
            DMA_SetCurrDataCounter(DMA1_Stream5, SBUS_RX_BUF_NUM);
            DMA1_Stream5->CR &= ~(DMA_SxCR_CT);
            //?DMA????
            DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
            DMA_Cmd(DMA1_Stream5, ENABLE);
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
				f++;
                //???????
                SBUS_TO_RC(SBUS_rx_buf[1], &rc_ctrl);
            }

        }
    }
}

//????
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
// ?SBUS???????????rc_ctrl
/***************************************************************************
*?????????,???????????????,?????????????
*8+3=11bits??????
*????,????1024???????
*****************************************************************************/

static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
  
		rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

  LPS=rc_ctrl->rc.s[1];
  RPS=rc_ctrl->rc.s[0];

		if(rc_ctrl->rc.s[1] == 3)
		{
			left_x=rc_ctrl->rc.ch[2];
			left_y=rc_ctrl->rc.ch[3];
			
	  }  
		if(rc_ctrl->rc.s[1] == 1)
		{
		  left_x=rc_ctrl->rc.ch[2];
			left_y=rc_ctrl->rc.ch[3];
		}
		
			if(rc_ctrl->rc.s[0] == 3)
		{
			right_x=rc_ctrl->rc.ch[0];
			right_y=rc_ctrl->rc.ch[1];
			
	  }  
		if(rc_ctrl->rc.s[0] == 1)
		{
		  right_x=rc_ctrl->rc.ch[0];
			right_y=rc_ctrl->rc.ch[1];
		}

}







