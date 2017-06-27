/**
  ******************************************************************************
  * @file    TIM/TIM_PWMOutput/Src/main.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    26-February-2014
  * @brief   This sample code shows how to use STM32F4xx TIM HAL API to generate
  *          4 signals in PWM.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "cmsis_os.h"
#include "stm32f4xx_hal_adc.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup TIM_PWM_Output
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/

/* ADC handler declaration */
 ADC_HandleTypeDef    AdcHandle1, AdcHandle2, AdcHandle3;
ADC_ChannelConfTypeDef adcConfig1, adcConfig2, adcConfig3;
ADC_ChannelConfTypeDef sConfig;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Variable used to get converted value */
__IO uint32_t uhADCxRight;
__IO uint32_t uhADCxForward;
__IO uint32_t uhADCxLeft;	




/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
void Error_Handler(void);
static void SystemClock_Config2(void);


//TaskHandle 선언부 

xTaskHandle TaskHandle[5];



	
#define  PERIOD_VALUE       0xFFFF  /* Period Value  */
#define  PULSE1_VALUE       0xFFFF        /* Capture Compare 1 Value  */
#define  PULSE2_VALUE       900         /* Capture Compare 2 Value  */
#define  PULSE3_VALUE       600         /* Capture Compare 3 Value  */
#define  PULSE4_VALUE       450         /* Capture Compare 4 Value  */

#define TaskHandle(__INDEX__)   ((__INDEX__) == 0) ? TaskHandle_0 :\
                                                     ((__INDEX__) == 1) ? TaskHandle_1 :\
																										 ((__INDEX__) == 2) ? TaskHandle_2 :
				

// 타이머 만들기 				


/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle1, TimHandle2, TimHandle3, TimHandle4;
TIM_IC_InitTypeDef     sICConfig;
	
/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig1, sConfig2, sConfig3;

/* Counter Prescaler value */
uint32_t uwPrescalerValue = 0;
uint32_t motorInterrupt1 = 0;
uint32_t motorInterrupt2 = 0;
uint32_t temp = 0;
uint8_t encoder_right = READY ;
uint8_t encoder_left  = READY ;
	 
 /* Captured Values */
uint32_t               uwIC2Value1 = 0;
uint32_t               uwIC2Value2 = 0;
uint32_t               uwDiffCapture1 = 0;
	
uint32_t               uwIC2Value3 = 0;
uint32_t               uwIC2Value4 = 0;
uint32_t               uwDiffCapture2 = 0;

uint32_t               uwIC2Value5 = 0;
uint32_t               uwIC2Value6= 0;
uint32_t               uwDiffCapture3 = 0;
uint32_t							direction = 2; // 출구 방향, 초기값 전면 
uint32_t							move = 0; //움직일 방향 
uint32_t							status = 0;
uint32_t               uwFrequency = 0;
uint32_t               R = 0;
uint32_t               L = 0;
static uint32_t ch2 = 0 ;   //내가선언한거 

/* Private function prototypes -----------------------------------------------*/
static void ADC_Initt(ADC_HandleTypeDef* hadc);

static void SystemClock_Config(void);
void Motor_Forward(void);
void Motor_Backward(void);
void Motor_Left(void);
void Motor_Right(void);
void Motor_Stop(void);
void Motor_Speed_Up_Config(void);
void Motor_Speed_Down_Config(void);
void Motor_Speed_Set_Config(int a);
void changeAlgo(int a,int b);
static void EXTILine_Config(void);
static void Error_Handler(void);
long lExpireCounters =0;
void moveTo();
void judge();
/* Private functions --------
-------------------------------------------------*/

extern UART_HandleTypeDef UartHandle1, UartHandle2;

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
	
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&UartHandle1, (uint8_t *)&ch, 1, 0xFFFF); 

	return ch;
}


// 학번 : 201102430 이름 : 문찬용


static void turn_Left(){

	if(L!=0){
		Motor_Left();
osDelay(950); // 950ms 동안 좌회전 
	Motor_Stop();
	
	osDelay(500);

	if(direction == 1) // 회전 후 원래 향하던 방향 업데이트 
		direction = 4 ; 
	else 
				direction = direction -1 ;
}
	else
		L++;
				
			}



static void turn_Right(){
	if(R!=0){
	Motor_Right();
		osDelay(1200); //  1200ms동안 우회전

	Motor_Stop();
		if(direction == 4) // 회전 후 원래 향하던 방향 업데이트 
		direction = 1 ;
	else 
				direction =  direction+1 ;
}
	else
		R++;
	
}





// 학번 : 201102430 이름 : 문찬용
void moveTo() {
				if(move == 1){
				turn_Left(); 
					Motor_Forward();
					return;
				}
			else if(move == 3){
				turn_Right();
					Motor_Forward();
				return;
				}
			else if (move == 4 )
			{
				turn_Right();
				turn_Right();
				Motor_Forward();
				return;
			}
			else if(move == 2)
			{Motor_Forward();
			return;
			}
}

// move 3 은오른쪽//move 1 은 왼쪽 
void judge() {
	if( direction == 2 ){
		//  좌우 검사 
		// (uwDiffCapture1/58)
			if((uwDiffCapture1/58) > (uwDiffCapture3/58))
			{	move = 3 ; 
			return;
			}
			else 
			{move = 1;
			return;
			}
	}
else if(direction == 1) 
	{
		// 우검사 막히면 뒤로 
					if(uwDiffCapture1/58 > 35)
					{	move = 3;
						return;
			}
					else
					{		move = 4;
					return;
			}	
					
						}
	else if(direction == 3 ){
		// 좌검사 막히면 뒤로 
							if(uwDiffCapture3/58 > 35)
							{		move = 1;
								return;
			}
					else
					{		move = 4;
						return;
			}
	}
	else if (direction == 4 )
		{
			move = 4;return;
			
	}
}


void movejudge() {

	
	if(direction == 1)  //  입구에서 왼쪽 
	{
					if(uwDiffCapture1/58 > 35)
					{move = 3;
						return;
					}
					else{
						move = 2;
						return;
					}
					
						}
	else if(direction == 3 ){ //입구에서 오른쪽 
							if(uwDiffCapture3/58 > 35)
							{move = 1;
								return;
					}
					else
					{move = 2;
						return;
					}
	}
	else if (direction == 4 ) // 입구에서 정면 
		{
								if(uwDiffCapture1/58 > 35)
								{move = 3;
									return;
					}
				else if(uwDiffCapture3/58 > 35)
				{move = 1;
					return;
					}
				else 
				{	move = 2;
					return;
					}
	}
}


void Detect_obstacle(void *params){
			
	
	  for (;;) { 

//			printf("\r전면부 초음파 센서 : %dcm",uwDiffCapture2/58);
			
			// 좌측 적외선 측정값
			
HAL_ADC_Start(&AdcHandle1);
		uhADCxLeft = HAL_ADC_GetValue(&AdcHandle1);
		HAL_ADC_PollForConversion(&AdcHandle1, 0xFF);
			
		//	printf("\좌측 센서 : %dcm",uhADCxLeft);
			// 우측 적외선 측정값 
			HAL_ADC_Start(&AdcHandle2);
			uhADCxRight = HAL_ADC_GetValue(&AdcHandle2);
		HAL_ADC_PollForConversion(&AdcHandle2, 0xFF);
			
				
//printf("우측 센서 : %dcm",uhADCxRight);
					// ch2 초음파 전면 측정값 
				ch2 = uwDiffCapture2/58;
			
			
//		printf("\rDetect\n");
						printf("방향 : %d",direction);

					vTaskDelay(1);
				
}
		
}


void first_Algo_Left(void *params){
	for (;;) {
		
									vTaskDelay(1); //  현재 태스크 블락으로  detact_obstacle TASK 실행 , ADC 값 읽어 옴 
						if ( uhADCxLeft > 1200   )	 //  전좌측 적외선센서로 30도 정도 회피
						{
							Motor_Stop();
							Motor_Right();
							osDelay(300); // 동작 delay를 통해 Right 시간 지연 후 스톱 ( 300ms 만큼 동작후 스톱) 
							Motor_Stop(); 
							osDelay(500); // 멈춘 후 갑자기 모터를 구동하면 차체에 움직임에 문제 가 생길 수 있어서 지연후 모터구동
							Motor_Forward(); 
						 vTaskDelay(1);    //detact_obstacle TASK 실행

							continue; // TASK를 위에부터 다시 시작 

						}
												vTaskDelay(1);  //  detact_obstacle TASK 실행 
						
						if(uhADCxRight > 1200 ) //  전우측 적외선센서로 25도 정도로 회피
						{
							Motor_Stop();
							Motor_Left();
							osDelay(250);// 동작 delay를 통해 left 시간 지연 후 스톱 ( 250ms 만큼 동작후 스톱) 
														Motor_Stop();
														osDelay(500); // 멈춘 후 갑자기 모터를 구동하면 차체에 움직임에 문제 가 생길 수 있어서 지연후 모터구동
							Motor_Forward();
						vTaskDelay(1);  //  detact_obstacle TASK 실행 

							continue;// TASK를 위에부터 다시 시작 

						}							
						
						
						
						if(uwDiffCapture2/58  < 25)  // 전방으로 이동이 못할때 실행하는  코드 
						{  //앞이 막혀있을때 반응 
						Motor_Stop();
							
							vTaskDelay(1);
								osDelay(1000);
							judge();  // 어느방향으로 갈지 판단하는 함수 
							
							moveTo() ; // 정한 방향으로 이동하는 함수
									
							vTaskDelay(1);

							continue; // 코드 기능 하나를 실행하였으면 태스크를 다시 시작한다. 

						}
						else if ((((uwDiffCapture3/58) >35 ) && ( direction == 3  ))  || (((uwDiffCapture1/58) > 35 ) && ( direction == 1 ))) // 방향에따라 왼쪽오른쪽센서의 길이값이 고정 35이상일때 실행 
						{					 // 이동중 출구방향으로 진입할수 있는지 확인하는 코드 
							
							
									osDelay(700);
								Motor_Stop();
									osDelay(1000);
								movejudge(); // 이동중 어느방향으로 이동하는 함수 
							moveTo(); // 정한 방향으로 이동하는 함수
							
							vTaskDelay(1);

							continue; // 코드 기능 하나를 실행하였으면 태스크를 다시 시작한다. 
						
						}
						
}
}  // void first_Algo_Left(void *params) end 







/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */




int main(void)
{

	GPIO_InitTypeDef  GPIO_InitStruct;
	
	

	/* STM32F4xx HAL library initialization:
	   - Configure the Flash prefetch, instruction and Data caches
	   - Configure the Systick to generate an interrupt each 1 msec
	   - Set NVIC Group Priority to 4
	   - Global MSP (MCU Support Package) initialization
	 */
	HAL_Init();

	
	/* Configure the system clock to have a system clock = 180 Mhz */
		SystemClock_Config2();  
	BSP_COM1_Init();

//적외선 설정 


	//##-1- Configure the ADC peripheral #######################################*/
	AdcHandle1.Instance          = ADC3;   // ADC 3번분
  
	AdcHandle1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	AdcHandle1.Init.Resolution = ADC_RESOLUTION12b;
	AdcHandle1.Init.ScanConvMode = DISABLE;
	// Mode 설정
	AdcHandle1.Init.ContinuousConvMode = DISABLE;
	AdcHandle1.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle1.Init.NbrOfDiscConversion = 0;  
	AdcHandle1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	AdcHandle1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	AdcHandle1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AdcHandle1.Init.NbrOfConversion = 1;
	//DMA(Direct Memory Access)
	AdcHandle1.Init.DMAContinuousRequests = DISABLE;
	AdcHandle1.Init.EOCSelection = DISABLE;
	
	HAL_ADC_Init(&AdcHandle1);//ADC Initialized

	/*##-2- Configure ADC regular channel ######################################*/ 
	adcConfig1.Channel = ADC_CHANNEL_11; //채널 설정
	adcConfig1.Rank = 1;
	adcConfig1.SamplingTime = ADC_SAMPLETIME_480CYCLES; //샘플링 주기 설정
	adcConfig1.Offset = 0;

	HAL_ADC_ConfigChannel(&AdcHandle1, &adcConfig1);
		
	AdcHandle2.Instance          = ADC2;   // ADC부분

	AdcHandle2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	AdcHandle2.Init.Resolution = ADC_RESOLUTION12b;
	AdcHandle2.Init.ScanConvMode = DISABLE;
	AdcHandle2.Init.ContinuousConvMode = DISABLE;
	AdcHandle2.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle2.Init.NbrOfDiscConversion = 0;  
	AdcHandle2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	AdcHandle2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	AdcHandle2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AdcHandle2.Init.NbrOfConversion = 1;
	AdcHandle2.Init.DMAContinuousRequests = DISABLE;
	AdcHandle2.Init.EOCSelection = DISABLE;

	HAL_ADC_Init(&AdcHandle2);

	/*##-2- Configure ADC regular channel ######################################*/ 
	adcConfig2.Channel = ADC_CHANNEL_14;
	adcConfig2.Rank = 1;
	adcConfig2.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	adcConfig2.Offset = 0;	
	HAL_ADC_ConfigChannel(&AdcHandle2, &adcConfig2);
	
	AdcHandle3.Instance          = ADC1;   // ADC부분

	AdcHandle3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	AdcHandle3.Init.Resolution = ADC_RESOLUTION12b;
	AdcHandle3.Init.ScanConvMode = DISABLE;
	AdcHandle3.Init.ContinuousConvMode = DISABLE;
	AdcHandle3.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle3.Init.NbrOfDiscConversion = 0;  
	AdcHandle3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	AdcHandle3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	AdcHandle3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AdcHandle3.Init.NbrOfConversion = 1;
	AdcHandle3.Init.DMAContinuousRequests = DISABLE;
	AdcHandle3.Init.EOCSelection = DISABLE;

	HAL_ADC_Init(&AdcHandle3);

	/*##-2- Configure ADC regular channel ######################################*/ 
	adcConfig3.Channel = ADC_CHANNEL_15;
	adcConfig3.Rank = 1;
	adcConfig3.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	adcConfig3.Offset = 0;	
	HAL_ADC_ConfigChannel(&AdcHandle3, &adcConfig3);
				


//적외선 설정 끝 
	
	









/* Compute the prescaler value to have TIM4,TIM8 counter clock equal to 2 MHz */
	uwPrescalerValue = (SystemCoreClock/2)/1000000;
	

	// PB2 모터 전원 인가를 위한 GPIO 초기화
	__GPIOB_CLK_ENABLE();
		
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // MC_EN(PB2) 모터 전원 
	
	
	/*##-1- Configure the TIM peripheral #######################################*/ 
	/* Initialize TIMx peripheral as follow:
	   + Prescaler = (SystemCoreClock/2)/18000000
	   + Period = 1800  (to have an output frequency equal to 10 KHz)
	   + ClockDivision = 0
	   + Counter direction = Up
	*/
	sConfig1.OCMode     = TIM_OCMODE_PWM1;
	sConfig1.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig1.OCFastMode = TIM_OCFAST_DISABLE;
	sConfig1.Pulse = 21500;
	
	// 왼쪽 Motor가 연결되어 있는 PIN은 회로도를 보면 PC6 / PC7 이며 Datasheet를 보면 PC6 / P7로 사용할 수 있는 Timer와 채널이 나와있다.
	TimHandle1.Instance = TIM8;	// MC_1A(PC6) -> TIM8_CH1, MC_2A(PC7) -> TIM8_CH2
	TimHandle1.Init.Prescaler     = uwPrescalerValue;
	TimHandle1.Init.Period        = 20000; 
	TimHandle1.Init.ClockDivision = 0;
	TimHandle1.Init.CounterMode   = TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(&TimHandle1);
	
	HAL_TIM_PWM_ConfigChannel(&TimHandle1, &sConfig1, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&TimHandle1, &sConfig1, TIM_CHANNEL_2);
		
	/*## Configure the PWM channels #########################################*/ 
	/* Common configuration for all channels */
	sConfig2.OCMode     = TIM_OCMODE_PWM1;
	sConfig2.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig2.OCFastMode = TIM_OCFAST_DISABLE;
	sConfig2.Pulse = 20000;
	
	TimHandle2.Instance = TIM4;	// MC_1B(PD12) -> TIM4_CH1, MC_2B(PD13) -> TIM4_CH2
	TimHandle2.Init.Prescaler     = uwPrescalerValue;
	TimHandle2.Init.Period        = 20000;
	TimHandle2.Init.ClockDivision = 0;
	TimHandle2.Init.CounterMode   = TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(&TimHandle2);

	HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig2, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig2, TIM_CHANNEL_2);

	EXTILine_Config(); // Encoder Interrupt Setting
	
	uwPrescalerValue = ((SystemCoreClock / 2) / 1000000) - 1;	
		/*##-1- Configure the TIM peripheral #######################################*/ 
	/* Set TIMx instance */
	TimHandle3.Instance = TIM3;

	/* Initialize TIMx peripheral as follow:
	   + Period = 0xFFFF
	   + Prescaler = 0
	   + ClockDivision = 0
	   + Counter direction = Up
	*/
	TimHandle3.Init.Period        = 0xFFFF;
	TimHandle3.Init.Prescaler     = uwPrescalerValue;
	TimHandle3.Init.ClockDivision = 0;
	TimHandle3.Init.CounterMode   = TIM_COUNTERMODE_UP; 
	
	if(HAL_TIM_IC_Init(&TimHandle3) != HAL_OK){ Error_Handler();}
	 
	/*##-2- Configure the Input Capture channel ################################*/ 
	/* Configure the Input Capture of channel 2 */
	sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
	sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
	sICConfig.ICFilter    = 0;   
	
	HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_1);
	HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_2);
	HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_3);
	HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_4);

	HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_2) ;
	HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_3) ;
	HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_4) ;

	uwPrescalerValue = (SystemCoreClock / 2 / 131099) - 1;
		
	/*##-1- Configure the TIM peripheral #######################################*/ 
	/* Initialize TIMx peripheral as follow:
	   + Prescaler = SystemCoreClock/327675
	   + Period = 65535  (to have an output frequency equal to 10 Hz)
	   + ClockDivision = 0
	   + Counter direction = Up
	*/
	/* Compute the prescaler value to have TIM10 counter clock equal to 131.099 kHz */
	uwPrescalerValue = (SystemCoreClock / 2 / 131099) - 1;
		
	TimHandle4.Instance = TIM10;

	TimHandle4.Init.Prescaler     = uwPrescalerValue;
	TimHandle4.Init.Period        = 0xFFFF;
	TimHandle4.Init.ClockDivision = 0;
	TimHandle4.Init.CounterMode   = TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(&TimHandle4);
	printf("\r\n 6 :%d",uwPrescalerValue/1);
	
	/*##-2- Configure the PWM channels #########################################*/ 
	/* Common configuration for all channels */
	sConfig3.OCMode     = TIM_OCMODE_PWM1;
	sConfig3.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig3.OCFastMode = TIM_OCFAST_DISABLE;

	/* Set the pulse value for channel 1 */
	sConfig3.Pulse = 2;
	HAL_TIM_PWM_ConfigChannel(&TimHandle4, &sConfig3, TIM_CHANNEL_1);
  
	/*##-3- Start PWM signals generation #######################################*/ 
	/* Start channel 3 */	
	HAL_TIM_PWM_Start(&TimHandle4, TIM_CHANNEL_1);
	
	
	/*
	printf("\r\n ------------------- Encoder Motor Example -------------------");
	printf("\r\n'w' = Forward, 's' = Stop, 'x' = Backward, 'a' = Left, 'd' = Right ");
	printf("\r\n'1' = Speed Up, '2' = Speed Down");
			*/
			
			
		//	Motor_Forward();
Motor_Forward();





								xTaskCreate(first_Algo_Left,   // pointer to task function
              "base_Algo_Left",	// name
              400,      // stack size: 100 * 4 byte
              NULL,     // parameters
              5,        // priority: number between 0 and max-priority
              TaskHandle[0]     // handle
              );

									xTaskCreate(Detect_obstacle,   // pointer to task function
              "Detect",	// name
              400,      // stack size: 100 * 4 byte
              NULL,     // parameters
              3,        // priority: number between 0 and max-priority
              NULL      // handle
              );
						vTaskStartScheduler();

	
	
	

	
	
	
}







void Motor_Forward(void)
{
	HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2);

	
	   
	
}






void Motor_Backward(void)
{
	HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);
}
void Motor_Left(void)
{
	HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2);
	
}
void Motor_Right(void)
{
	HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);
}

void Motor_Stop(void)
{
	
	
	HAL_TIM_PWM_Stop(&TimHandle1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&TimHandle1, TIM_CHANNEL_2);				
	HAL_TIM_PWM_Stop(&TimHandle2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&TimHandle2, TIM_CHANNEL_2);
	
}

void Motor_Speed_Up_Config(void)
{
	sConfig1.Pulse  = sConfig1.Pulse + 100;
	sConfig2.Pulse  = sConfig2.Pulse + 100;
	TIM8->CCR1 = sConfig1.Pulse;
	TIM8->CCR2 = sConfig1.Pulse;
	TIM4->CCR1 = sConfig2.Pulse;
	TIM4->CCR2 = sConfig2.Pulse;
}

void Motor_Speed_Set_Config(int a)
{
	sConfig1.Pulse  = a;
	sConfig2.Pulse  = a;
	TIM8->CCR1 = sConfig1.Pulse;
	TIM8->CCR2 = sConfig1.Pulse;
	TIM4->CCR1 = sConfig2.Pulse;
	TIM4->CCR2 = sConfig2.Pulse;
}
void Motor_Speed_Down_Config(void)
{
	sConfig1.Pulse  = sConfig1.Pulse - 100;
	sConfig2.Pulse  = sConfig2.Pulse - 100;
	TIM8->CCR1 = sConfig1.Pulse;
	TIM8->CCR2 = sConfig1.Pulse;
	TIM4->CCR1 = sConfig2.Pulse;
	TIM4->CCR2 = sConfig2.Pulse;
}
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */

static void _Config2(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__PWR_CLK_ENABLE();

	/* The voltage scaling allows optimizing the power consumption when the device is 
	 clocked below the maximum system frequency, to update the voltage scaling value 
	 regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 288;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 6;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
	 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}



static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__PWR_CLK_ENABLE();

	/* The voltage scaling allows optimizing the power consumption when the device is 
	 clocked below the maximum system frequency, to update the voltage scaling value 
	 regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 360;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/* Activate the Over-Drive mode */
	HAL_PWREx_ActivateOverDrive();

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
	 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}
static void SystemClock_Config2(void)
{
   RCC_ClkInitTypeDef RCC_ClkInitStruct;
   RCC_OscInitTypeDef RCC_OscInitStruct;

   /* Enable Power Control clock */
   __PWR_CLK_ENABLE();

   /* The voltage scaling allows optimizing the power consumption when the device is 
    clocked below the maximum system frequency, to update the voltage scaling value 
    regarding system frequency refer to product datasheet.  */
   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

   /* Enable HSE Oscillator and activate PLL with HSE as source */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
   RCC_OscInitStruct.PLL.PLLM = 25;
   RCC_OscInitStruct.PLL.PLLN = 288;
   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
   RCC_OscInitStruct.PLL.PLLQ = 6;
   HAL_RCC_OscConfig(&RCC_OscInitStruct);

   /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
    clocks dividers */
   RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
   HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}

#endif

 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
	switch(GPIO_Pin)
	{
		case GPIO_PIN_15 :
			encoder_right = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3);
			if(encoder_right == 0)
			{
				if(motorInterrupt1==0)
					motorInterrupt1 = 20000;
				motorInterrupt1--;
				encoder_right = READY;
			}
			else if(encoder_right == 1)
			{
				
				motorInterrupt1++;
				encoder_right = READY;
								if(motorInterrupt1==20000)
					motorInterrupt1 = 0;
			}
			break;
		
		case GPIO_PIN_4 :
			encoder_left = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);
			if(encoder_left == 0)
			{
				motorInterrupt2++;				
				encoder_left = READY;
								if(motorInterrupt2==20000)
					motorInterrupt2 = 0;
			}		
			else if(encoder_left == 1)
			{
								if(motorInterrupt1==0)
					motorInterrupt1 = 20000;
				motorInterrupt2--;
				encoder_left = READY;
			}	
			break;
	}
 }

 
 
/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  htim : hadc handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if((TIM3->CCER & TIM_CCER_CC2P) == 0)
			{
				/* Get the 1st Input Capture value */
				uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
				TIM3->CCER |= TIM_CCER_CC2P;
			}
			else if((TIM3->CCER & TIM_CCER_CC2P) == TIM_CCER_CC2P)
			{
				/* Get the 2nd Input Capture value */
				uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); 
				
				/* Capture computation */
				if (uwIC2Value2 > uwIC2Value1)
				{
						uwDiffCapture1 = (uwIC2Value2 - uwIC2Value1); 
				}
				else if (uwIC2Value2 < uwIC2Value1)
				{
						uwDiffCapture1 = ((0xFFFF - uwIC2Value1) + uwIC2Value2); 
				}
				else
				{
						uwDiffCapture1 = 0;
				}
			//	printf("\r\n Value Right : %d cm", uwDiffCapture1/58);
					
				uwFrequency = (2*HAL_RCC_GetPCLK1Freq()) / uwDiffCapture1;
				TIM3->CCER &= ~TIM_CCER_CC2P;
			}
		}

		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if((TIM3->CCER & TIM_CCER_CC3P) == 0)
			{
				/* Get the 1st Input Capture value */
				uwIC2Value3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
				TIM3->CCER |= TIM_CCER_CC3P;
			}
			else if((TIM3->CCER & TIM_CCER_CC3P) == TIM_CCER_CC3P)
			{
				/* Get the 2nd Input Capture value */
				uwIC2Value4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); 
				
				/* Capture computation */
				if (uwIC2Value4 > uwIC2Value3)
				{
					uwDiffCapture2 = (uwIC2Value4 - uwIC2Value3); 
				}
				else if (uwIC2Value4 < uwIC2Value3)
				{
					uwDiffCapture2 = ((0xFFFF - uwIC2Value3) + uwIC2Value4); 
				}
				else
				{
					uwDiffCapture2 = 0;
				}
			//	printf("\r\n Value Forward : %d cm", uwDiffCapture2/58);
					
				uwFrequency = (2*HAL_RCC_GetPCLK1Freq()) / uwDiffCapture2;
				TIM3->CCER &= ~TIM_CCER_CC3P;
			}		
		}
		
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if((TIM3->CCER & TIM_CCER_CC4P) == 0)
			{
				/* Get the 1st Input Capture value */
				uwIC2Value5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
				TIM3->CCER |= TIM_CCER_CC4P;
			}
			else if((TIM3->CCER & TIM_CCER_CC4P) == TIM_CCER_CC4P)
			{
				/* Get the 2nd Input Capture value */
				uwIC2Value6 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); 
				
				/* Capture computation */
				if (uwIC2Value6 > uwIC2Value5)
				{
					uwDiffCapture3 = (uwIC2Value6 - uwIC2Value5); 
				}
				else if (uwIC2Value6 < uwIC2Value5)
				{
					uwDiffCapture3 = ((0xFFFF - uwIC2Value5) + uwIC2Value6); 
				}
				else
				{
					uwDiffCapture3 = 0;
				}
			//	printf("\r\n Value Left: %d cm", uwDiffCapture3/58);
					
				uwFrequency = (2*HAL_RCC_GetPCLK1Freq()) / uwDiffCapture3;
				TIM3->CCER &= ~TIM_CCER_CC4P;
			}
		}
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
    /* Turn LED3 on */
    BSP_LED_On(LED3);
    while(1)
    {
    }
}

/**
  * @brief  Configures EXTI Line (connected to PA15, PB3, PB4, PB5 pin) in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTILine_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOA clock */
  __GPIOA_CLK_ENABLE();
  
  /* Configure PA15 pin  */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_15 ;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
/* Enable and set EXTI Line15 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
		
	
 /* Enable GPIOB clock */	
 __GPIOB_CLK_ENABLE();

  /* Configure PB3, PB4, PB5 pin  */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
  /* Enable and set EXTI Line4 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}


static void ADC_Initt(ADC_HandleTypeDef* hadc)
{
  
  /* Set ADC parameters */
  /* Set the ADC clock prescaler */
  ADC->CCR &= ~(ADC_CCR_ADCPRE);
  ADC->CCR |=  hadc->Init.ClockPrescaler;
  
  /* Set ADC scan mode */
  hadc->Instance->CR1 &= ~(ADC_CR1_SCAN);
  hadc->Instance->CR1 |=  __HAL_ADC_CR1_SCANCONV(hadc->Init.ScanConvMode);
  
  /* Set ADC resolution */
  hadc->Instance->CR1 &= ~(ADC_CR1_RES);
  hadc->Instance->CR1 |=  hadc->Init.Resolution;
  
  /* Set ADC data alignment */
  hadc->Instance->CR2 &= ~(ADC_CR2_ALIGN);
  hadc->Instance->CR2 |= hadc->Init.DataAlign;
  
  /* Select external trigger to start conversion */
  hadc->Instance->CR2 &= ~(ADC_CR2_EXTSEL);
  hadc->Instance->CR2 |= hadc->Init.ExternalTrigConv;

  /* Select external trigger polarity */
  hadc->Instance->CR2 &= ~(ADC_CR2_EXTEN);
  hadc->Instance->CR2 |= hadc->Init.ExternalTrigConvEdge;
  
  /* Enable or disable ADC continuous conversion mode */
  hadc->Instance->CR2 &= ~(ADC_CR2_CONT);
  hadc->Instance->CR2 |= __HAL_ADC_CR2_CONTINUOUS(hadc->Init.ContinuousConvMode);
  
  if (hadc->Init.DiscontinuousConvMode != DISABLE)
  {
    assert_param(IS_ADC_REGULAR_DISC_NUMBER(hadc->Init.NbrOfDiscConversion));
  
    /* Enable the selected ADC regular discontinuous mode */
    hadc->Instance->CR1 |= (uint32_t)ADC_CR1_DISCEN;
    
    /* Set the number of channels to be converted in discontinuous mode */
    hadc->Instance->CR1 &= ~(ADC_CR1_DISCNUM);
    hadc->Instance->CR1 |=  __HAL_ADC_CR1_DISCONTINUOUS(hadc->Init.NbrOfDiscConversion);
  }
  else
  {
    /* Disable the selected ADC regular discontinuous mode */
    hadc->Instance->CR1 &= ~(ADC_CR1_DISCEN);
  }
  
  /* Set ADC number of conversion */
  hadc->Instance->SQR1 &= ~(ADC_SQR1_L);
  hadc->Instance->SQR1 |=  __HAL_ADC_SQR1(hadc->Init.NbrOfConversion);
  
  /* Enable or disable ADC DMA continuous request */
  hadc->Instance->CR2 &= ~(ADC_CR2_DDS);
  hadc->Instance->CR2 |= __HAL_ADC_CR2_DMAContReq(hadc->Init.DMAContinuousRequests);
  
  /* Enable or disable ADC end of conversion selection */
  hadc->Instance->CR2 &= ~(ADC_CR2_EOCS);
  hadc->Instance->CR2 |= __HAL_ADC_CR2_EOCSelection(hadc->Init.EOCSelection);
}



HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef* hadc)
{
  /* Check ADC handle */
  if(hadc == NULL)
  {
     return HAL_ERROR;
  }
  
  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
  assert_param(IS_ADC_CLOCKPRESCALER(hadc->Init.ClockPrescaler));
  assert_param(IS_ADC_RESOLUTION(hadc->Init.Resolution));
  assert_param(IS_FUNCTIONAL_STATE(hadc->Init.ScanConvMode));
  assert_param(IS_FUNCTIONAL_STATE(hadc->Init.ContinuousConvMode));
  assert_param(IS_ADC_EXT_TRIG_EDGE(hadc->Init.ExternalTrigConvEdge)); 
  assert_param(IS_ADC_EXT_TRIG(hadc->Init.ExternalTrigConv));
  assert_param(IS_ADC_DATA_ALIGN(hadc->Init.DataAlign));
  assert_param(IS_ADC_REGULAR_LENGTH(hadc->Init.NbrOfConversion));
  assert_param(IS_FUNCTIONAL_STATE(hadc->Init.DMAContinuousRequests));
  assert_param(IS_ADC_EOCSelection(hadc->Init.EOCSelection));
  assert_param(IS_FUNCTIONAL_STATE(hadc->Init.DiscontinuousConvMode));
  
  if(hadc->State == HAL_ADC_STATE_RESET)
  {
    /* Init the low level hardware */
    HAL_ADC_MspInit(hadc);
  }
  
  /* Initialize the ADC state */
  hadc->State = HAL_ADC_STATE_BUSY;
  
  /* Set ADC parameters */
  ADC_Initt(hadc);
  
  /* Set ADC error code to none */
  hadc->ErrorCode = HAL_ADC_ERROR_NONE;
  
  /* Initialize the ADC state */
  hadc->State = HAL_ADC_STATE_READY;

  /* Release Lock */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig)
{
  /* Check the parameters */
  assert_param(IS_ADC_CHANNEL(sConfig->Channel));
  assert_param(IS_ADC_REGULAR_RANK(sConfig->Rank));
  assert_param(IS_ADC_SAMPLE_TIME(sConfig->SamplingTime));
  
  /* Process locked */
  __HAL_LOCK(hadc);
    
  /* if ADC_Channel_10 ... ADC_Channel_18 is selected */
  if (sConfig->Channel > ADC_CHANNEL_9)
  {
    /* Clear the old sample time */
    hadc->Instance->SMPR1 &= ~__HAL_ADC_SMPR1(ADC_SMPR1_SMP10, sConfig->Channel);
    
    /* Set the new sample time */
    hadc->Instance->SMPR1 |= __HAL_ADC_SMPR1(sConfig->SamplingTime, sConfig->Channel);
  }
  else /* ADC_Channel include in ADC_Channel_[0..9] */
  {
    /* Clear the old sample time */
    hadc->Instance->SMPR2 &= ~__HAL_ADC_SMPR2(ADC_SMPR2_SMP0, sConfig->Channel);
    
    /* Set the new sample time */
    hadc->Instance->SMPR2 |= __HAL_ADC_SMPR2(sConfig->SamplingTime, sConfig->Channel);
  }
  
  /* For Rank 1 to 6 */
  if (sConfig->Rank < 7)
  {
    /* Clear the old SQx bits for the selected rank */
    hadc->Instance->SQR3 &= ~__HAL_ADC_SQR3_RK(ADC_SQR3_SQ1, sConfig->Rank);
    
    /* Set the SQx bits for the selected rank */
    hadc->Instance->SQR3 |= __HAL_ADC_SQR3_RK(sConfig->Channel, sConfig->Rank);
  }
  /* For Rank 7 to 12 */
  else if (sConfig->Rank < 13)
  {
    /* Clear the old SQx bits for the selected rank */
    hadc->Instance->SQR2 &= ~__HAL_ADC_SQR2_RK(ADC_SQR2_SQ7, sConfig->Rank);
    
    /* Set the SQx bits for the selected rank */
    hadc->Instance->SQR2 |= __HAL_ADC_SQR2_RK(sConfig->Channel, sConfig->Rank);
  }
  /* For Rank 13 to 16 */
  else
  {
    /* Clear the old SQx bits for the selected rank */
    hadc->Instance->SQR1 &= ~__HAL_ADC_SQR1_RK(ADC_SQR1_SQ13, sConfig->Rank);
    
    /* Set the SQx bits for the selected rank */
    hadc->Instance->SQR1 |= __HAL_ADC_SQR1_RK(sConfig->Channel, sConfig->Rank);
  }
  
  /* if ADC1 Channel_18 is selected enable VBAT Channel */
  if ((hadc->Instance == ADC1) && (sConfig->Channel == ADC_CHANNEL_VBAT))
  {
    /* Enable the VBAT channel*/
    ADC->CCR |= ADC_CCR_VBATE;
  }
  
  /* if ADC1 Channel_16 or Channel_17 is selected enable TSVREFE Channel(Temperature sensor and VREFINT) */
  if ((hadc->Instance == ADC1) && ((sConfig->Channel == ADC_CHANNEL_TEMPSENSOR) || (sConfig->Channel == ADC_CHANNEL_VREFINT)))
  {
    /* Enable the TSVREFE channel*/
    ADC->CCR |= ADC_CCR_TSVREFE;
  }
  
  /* Process unlocked */
  __HAL_UNLOCK(hadc);
  
  /* Return function status */
  return HAL_OK;
}



uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef* hadc)
{       
  /* Return the selected ADC converted value */ 
  return hadc->Instance->DR;
}

HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc)
{
  uint16_t i = 0;
  
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(hadc->Init.ContinuousConvMode));
  assert_param(IS_ADC_EXT_TRIG_EDGE(hadc->Init.ExternalTrigConvEdge)); 
  
  /* Process locked */
  __HAL_LOCK(hadc);
  
  /* Check if an injected conversion is ongoing */
  if(hadc->State == HAL_ADC_STATE_BUSY_INJ)
  {
    /* Change ADC state */
    hadc->State = HAL_ADC_STATE_BUSY_INJ_REG;  
  }
  else
  {
    /* Change ADC state */
    hadc->State = HAL_ADC_STATE_BUSY_REG;
  } 
    
  /* Check if ADC peripheral is disabled in order to enable it and wait during 
     Tstab time the ADC's stabilization */
  if((hadc->Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON)
  {  
    /* Enable the Peripheral */
    __HAL_ADC_ENABLE(hadc);
    
    /* Delay inserted to wait during Tstab time the ADC's stabilazation */
    for(; i <= 540; i++)
    {
      __NOP();
    }
  }

  /* Check if Multimode enabled */
  if(HAL_IS_BIT_CLR(ADC->CCR, ADC_CCR_MULTI))
  {
    /* if no external trigger present enable software conversion of regular channels */
    if(hadc->Init.ExternalTrigConvEdge == ADC_EXTERNALTRIGCONVEDGE_NONE)
    {
      /* Enable the selected ADC software conversion for regular group */
      hadc->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
    }
  }
  else
  {
    /* if instance of handle correspond to ADC1 and  no external trigger present enable software conversion of regular channels */
    if((hadc->Instance == ADC1) && (hadc->Init.ExternalTrigConvEdge == ADC_EXTERNALTRIGCONVEDGE_NONE))
    {
      /* Enable the selected ADC software conversion for regular group */
        hadc->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
    }
  }
  
  /* Process unlocked */
  __HAL_UNLOCK(hadc);
  
  /* Return function status */
  return HAL_OK;
}
	


HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout)
{
  uint32_t timeout;
 
  /* Get timeout */
  timeout = HAL_GetTick() + Timeout;  

  /* Check End of conversion flag */
  while(!(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC)))
  {
    /* Check for the Timeout */
    if(Timeout != HAL_MAX_DELAY)
    {
      if(HAL_GetTick() >= timeout)
      {
        hadc->State= HAL_ADC_STATE_TIMEOUT;
        /* Process unlocked */
        __HAL_UNLOCK(hadc);
        return HAL_TIMEOUT;
      }
    }
  }
  
  /* Check if an injected conversion is ready */
  if(hadc->State == HAL_ADC_STATE_EOC_INJ)
  {
    /* Change ADC state */
    hadc->State = HAL_ADC_STATE_EOC_INJ_REG;  
  }
  else
  {
    /* Change ADC state */
    hadc->State = HAL_ADC_STATE_EOC_REG;
  }
  
  /* Return ADC state */
  return HAL_OK;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
   GPIO_InitTypeDef          GPIO_InitStruct;
	// ADC GPIO 설정
	if(hadc->Instance == ADC3)
	{
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* ADC1 Periph clock enable */

		__ADC3_CLK_ENABLE();
		/* Enable GPIO clock ****************************************/
		__GPIOC_CLK_ENABLE();

		/*##-2- Configure peripheral GPIO ##########################################*/ 
		/* ADC1 Channel11 GPIO pin configuration */
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	}
	
	if(hadc->Instance == ADC2)
	{
		/* ADC1 Periph clock enable */
		__ADC2_CLK_ENABLE();
		/* Enable GPIO clock ****************************************/
		__GPIOC_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_4 ;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	}
	
	if(hadc->Instance == ADC1)
	{
		/* ADC1 Periph clock enable */
		__ADC1_CLK_ENABLE();
		/* Enable GPIO clock ****************************************/
		__GPIOC_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	}
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
