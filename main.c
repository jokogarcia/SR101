// Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"



#include <stdio.h>
#define ARRAYSIZE 9
#define ADC1_DR    ((uint32_t)0x4001244C)
volatile uint16_t ADC_values[ARRAYSIZE];
volatile uint32_t status = 0;
uint8_t channels[9],keys[9];

void ADCInit(void);
void DMAInit(void);
void ADC_process();
void trigger(int, int);
void ADCInit(void) {
	//--Enable ADC1 --
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure; //Variable used to setup the GPIO pins
	//First, PA:
	//==Configure ADC pins (PA1,PA2,PA3,PB0,PB1,PC1,PC2,PC4,PC5) as analog inputs==
	GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//PB:
	GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//PC:
	GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//Habilitar los puertos A, B y C
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
	ADC_InitTypeDef ADC_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;


	//ADC1 configuration
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfConversion = 9;
	ADC_Init(ADC1, &ADC_InitStruct);

	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div8;
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStruct);








	ADC_TempSensorVrefintCmd(ENABLE);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 4, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 5, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 6, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 7, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 8, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 9, ADC_SampleTime_28Cycles);
	//Enable ADC1
	ADC_Cmd(ADC1, ENABLE);
	//enable DMA for ADC
	ADC_DMACmd(ADC1, ENABLE);

}
void DMAInit(void) {
	//enable DMA1 clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	//create DMA structure
	DMA_InitTypeDef DMA_InitStructure;
	//reset DMA2 Stream 0 to default values;
	DMA_DeInit(DMA2_Stream0);
	//setting normal mode (non circular)
	DMA_InitStructure.DMA_Channel=DMA_Channel_0;
	//Peripherial base address
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR;
	//Destination base address
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) ADC_values;
	//Location assigned to peripheral register will be source
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	//chunk of data to be transfered
	DMA_InitStructure.DMA_BufferSize = ARRAYSIZE;
	//source address increment disable
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//automatic memory destination increment enable.
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//source and destination data size word=32bit
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	//DMA mode
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	//medium priority
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	//FIFO mode
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	//DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;




	//Disable DMA2 Stream 0 and wait for it to be disabled
	DMA_Cmd(DMA2_Stream0, DISABLE);
	while (DMA2_Stream0->CR & DMA_SxCR_EN);
	//send values to DMA registers
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	//Clear all DMA interrupt flags
	DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_FEIF0|DMA_FLAG_DMEIF0|DMA_FLAG_TEIF0|DMA_FLAG_HTIF0|DMA_FLAG_TCIF0);
	// Enable DMA1 Channel Transfer Complete interrupt
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Stream0, ENABLE); //Enable the DMA2 Stream0
	NVIC_InitTypeDef NVIC_InitStructure;
	//Enable DMA1 channel IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
#define TRIG 200
void ADC_process(void) {
	static uint16_t c[ARRAYSIZE], ADC_oldvalues[ARRAYSIZE];
	for (int i = 0; i < ARRAYSIZE; i++) {
		if (c[i] > 0) {
			c[i]--;
			continue;
		}
		if (ADC_values[i] > TRIG && ADC_values[i] < ADC_oldvalues[i]) {
			trigger(i, ADC_values[i]);
			c[i] = 10;
			ADC_oldvalues[i] = 0;
			continue;
		}
		else
			ADC_oldvalues[i] = ADC_values[i];
	}
}
void trigger(int cuerpo, int veloc) {
	//note on
	while( !(USART1->SR & 0x00000040) );
	USART_SendData(USART1, 0x90|channels[cuerpo]);
	while( !(USART1->SR & 0x00000040) );
	USART_SendData(USART1, 0x7f & keys[cuerpo]);
	while( !(USART1->SR & 0x00000040) );
	USART_SendData(USART1, 0x7f & veloc);
	return;
}
void DMA2_Stream0_IRQHandler(void)
{
  //Test on DMA1 Channel1 Transfer Complete interrupt
  if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0))
  {
	 status=1;
   //Clear DMA1 interrupt pending bits
    DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
  }
  if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_HTIF0))
    {
  	  status=0;
     //Clear DMA1 interrupt pending bits
      DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TEIF0);
    }
  if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TEIF0))
    {
  	  status=0;
     //Clear DMA1 interrupt pending bits
      DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TEIF0 );
    }
  if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_FEIF0))
    {
  	  status=0;
     //Clear DMA1 interrupt pending bits
      DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_FEIF0 );
    }


}
void UART_Initialize(void)
{
 /* Enable peripheral clock for USART1 */
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

/* USART1 configured as follow:
 * BaudRate 31250 baud
 * Word Length 8 Bits
 * 1 Stop Bit
 * No parity
 * Hardware flow control disabled
 * Receive and transmit enabled
 */
 USART_InitTypeDef USART_InitStructure;
 USART_InitStructure.USART_BaudRate = 31250;
 USART_InitStructure.USART_WordLength = USART_WordLength_8b;
 USART_InitStructure.USART_StopBits = USART_StopBits_1;
 USART_InitStructure.USART_Parity = USART_Parity_No;
 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

USART_Init(USART1, &USART_InitStructure); // USART configuration
USART_Cmd(USART1, ENABLE); // Enable USART
}
int main (void)
{
	int c=0;
	ADC_values[0]=33;
	ADC_values[1]=34;
	ADC_values[2]=35;
	ADC_values[3]=36;
	ADC_values[4]=37;
	ADC_values[5]=38;
	ADCInit();
	DMAInit();
	ADC_SoftwareStartConv(ADC1); //Start the conversion
	channels[0] = 10;
	channels[1] = 10;
	channels[2] = 10;
	channels[3] = 10;
	channels[4] = 10;
	channels[5] = 10;
	channels[6] = 10;
	channels[7] = 10;
	channels[8] = 10;
	channels[9] = 10;

	keys[0] = 38; //Snare
	keys[1] = 50; //Hi-Tom
	keys[2] = 48; //Hi mid tom
	keys[3] = 47; //Lo mid tom
	keys[4] = 45; //lo tom
	keys[5] = 43; //hi floor tom
	keys[6] = 41; //lo floor tom
	keys[7] = 49; //crash cymb 1
	keys[8] = 51; //ride cynb 1
	keys[9] = 57; //crash cymb 2


//	while(!DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0));
	printf("Hello there!\n");
	while (status == 0);

	printf("Scan complete\nADC values are:\n");
	for (int i=0;i<9;i++)
	{
		printf("Channel %d = %d\n",i+1,ADC_values[i]);
	}

	while(1)
	{
		if(status == 1)
		{
			ADC_process();
			ADC_SoftwareStartConv(ADC1);
			status=0;
			c++;
			if (c%25 == 0)
			{
				printf("Scan complete\nADC values are:\n");
				for (int i=0;i<9;i++)
				{
					printf("Chan %d = %d\n",i+1,ADC_values[i]);
				}
			}

		}
	}
	return 0;

}
