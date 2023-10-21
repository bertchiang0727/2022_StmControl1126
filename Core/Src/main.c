/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mainpp.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim23;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
//motor 1 / encoder(time2)/ INA(PD8) INB(PD14) PWM(time12_ch2)
#define DCmotor1_INA_Port GPIOD
#define DCmotor1_INA_Pin GPIO_PIN_14
#define DCmotor1_INB_Port GPIOD
#define DCmotor1_INB_Pin GPIO_PIN_8

//motor 2/ encoder(time5)/ INA(PB12) INB(PB13) PWM(time12_ch1)
#define DCmotor2_INA_Port GPIOB
#define DCmotor2_INA_Pin GPIO_PIN_12
#define DCmotor2_INB_Port GPIOB
#define DCmotor2_INB_Pin GPIO_PIN_13

//motor 3/ encoder(time3)/ INA(PE2) INB(PE3) PWM(time15_ch1)
#define DCmotor3_INA_Port GPIOE
#define DCmotor3_INA_Pin GPIO_PIN_3
#define DCmotor3_INB_Port GPIOE
#define DCmotor3_INB_Pin GPIO_PIN_2

//motor 4/ encoder(time4)/ INA(PE4) INB(PC13) PWM(time15_ch2)
#define DCmotor4_INA_Port GPIOE
#define DCmotor4_INA_Pin GPIO_PIN_4
#define DCmotor4_INB_Port GPIOC
#define DCmotor4_INB_Pin GPIO_PIN_13


//step 1/ steppin(PF9)/ dir(PC5)/ MS1(PB2)/ MS2(PB1)
#define Step1_STEP_Port GPIOF
#define Step1_STEP_Pin GPIO_PIN_9
#define Step1_DIR_Port GPIOC
#define Step1_DIR_Pin GPIO_PIN_5
#define Step1_MS1_Port GPIOB
#define Step1_MS1_Pin GPIO_PIN_2
#define Step1_MS2_Port GPIOB
#define Step1_MS2_Pin GPIO_PIN_1

//step 2/ steppin(PF8)/ dir(PF11)
#define Step2_STEP_Port GPIOF
#define Step2_STEP_Pin GPIO_PIN_8
#define Step2_DIR_Port GPIOF
#define Step2_DIR_Pin GPIO_PIN_11

//step 3/ steppin(PB9)/ dir(PF15)
#define Step3_STEP_Port GPIOB
#define Step3_STEP_Pin GPIO_PIN_9
#define Step3_DIR_Port GPIOF
#define Step3_DIR_Pin GPIO_PIN_15

//step data
#define PI 3.14159265
#define L1 18.06  //large arm's length
#define L2 10.975 //small arm's length
#define error 2.03 //the distance between servo and middle of circle

//step function
double arctangent2 (double x,double y);
double arctangent2 (double x,double y){
    double answer;
    if(x>0) answer=(atan(y/x))*180.0/PI;
    else if(x<0&&y>=0) answer=atan(y/x)*180.0/PI+180;
    else if(x==0&&y>0) answer=90;
    else if(x<0&&y<0) answer=atan(y/x)*180.0/PI-180;
    else if(x==0&&y<0) answer=-90;
    return answer;
}
//PUMP
#define Pump1_IN1_PORT GPIOE
#define Pump1_IN1_Pin GPIO_PIN_15
#define Pump2_IN3_PORT GPIOE
#define Pump2_IN3_Pin GPIO_PIN_10
#define Pump3_IN4_PORT GPIOE
#define Pump3_IN4_Pin GPIO_PIN_8




// DC motor data
// DC1(right back wheel) / DC2(left back wheel) / DC3(right front wheel) / DC4(left front wheel)
double DC1_RpsGoal = 0, DC2_RpsGoal = 0, DC3_RpsGoal = 0, DC4_RpsGoal = 0;

int ms4=0, n=0;
int pulse_1=0,pulse_2=0,pulse_3=0,pulse_4=0;
int res_encoder=512;
int16_t CountNow_1,CountNow_2,CountNow_3,CountNow_4;
double time=0.001;
double rps_1,rps_2,rps_3,rps_4;
double P_1=0,P_2=0,P_3=0,P_4=0;
double I_1=0,I_2=0,I_3=0,I_4=0;
double e_1=0,e_2=0,e_3=0,e_4=0;
double u_1=0,u_2=0,u_3=0,u_4=0;
double sr_ratio=16;
//P,I
double p_1=500,i_1=250;
double p_2=500,i_2=250;
double p_3=500,i_3=250;
double p_4=500,i_4=250;


//stepper data
//Step1(large arm) / Step2(small arm) / Step3(up down)
//double x=-10.5,y=10.3,z=0;
double Step1_AngleGoal = 0, Step2_AngleGoal = 0, Step3_AngleGoal = 0;

int ms1=0,ms2=0,ms3=0;
int dir_state1 = 1, step_state1 = 1;
int dir_state2 = 1, step_state2 = 1;
int dir_state3 = 1, step_state3 = 1;
int step1=0,step2=0,step3=0;
double angle_before1=98.151,angle_before2=125.696, angle_before3=0;

double Le=0,Lcorrect=0;
double c1=0,c2=0,c3=0,cf=0,cerror=0;
double angle1_Now=0,angle2_Now=0,angle_error=0,anglef=0,angleb=0,angle1=0,angle2=0;
int temp=0;
double angle1_before=0,angle2_before=0;
double angle_true=0;
double x_true=0,y_true=0;
int pump1=1,pump2=2,pump3=3;


//servo data(servo1 -> for the claw/ servo2 -> for the container(180->90))
int Servo1_AngleGoal = 0, Servo2_AngleGoal = 180;

int servo_pulse1=0, servo_pulse2=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM23_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	Step1_AngleGoal = 0;Step2_AngleGoal = 0;Step3_AngleGoal = 0;
	angle_before1=98.151;angle_before2=125.69;angle_before3=0;
	temp=0;


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM15_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM23_Init();
  /* USER CODE BEGIN 2 */
  	setup();

    HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1); //encoder 1/ time2
    HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_1); //encoder 2/ time5
    HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1); //encoder 3/ time3
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1); //encoder 4/ time4
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2); //vnh pwm 1
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1); //vnh pwm 2
    HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1); //vnh pwm 3
    HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2); //vnh pwm 4

    HAL_TIM_Base_Start_IT(&htim13); //clock for dcmotor
    HAL_TIM_Base_Start_IT(&htim6); //clock for step1
    HAL_TIM_Base_Start_IT(&htim7); //clock for step2
    HAL_TIM_Base_Start_IT(&htim8); //clock for step3

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //servo1
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); //servo2
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    	/*
    	// step ms1 ms2 initail -> 16 steps
    	HAL_GPIO_WritePin(Step1_MS1_Port, Step1_MS1_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(Step1_MS2_Port, Step1_MS2_Pin, GPIO_PIN_SET);
    	*/



    	 // step calculate
    		float a = 12;
    		float b = 9.65;
    	   DC3_RpsGoal = vel[1] - vel[0] + vel[2]*(a+b);
    	   DC4_RpsGoal = vel[1] + vel[0] - vel[2]*(a+b);
    	   DC2_RpsGoal = vel[1] - vel[0] - vel[2]*(a+b);
    	   DC1_RpsGoal = vel[1] + vel[0] + vel[2]*(a+b);


			temp = 0;
			angleb=arctangent2(x,y);
			Lcorrect=pow((x*x+y*y-error*error),0.5);
			cerror=(x*x+y*y+Lcorrect*Lcorrect-error*error)/(2*pow(x*x+y*y,0.5)*Lcorrect);
			if(cerror>1||cerror<-1) temp=1;
			angle_error=acos(cerror);
			if(temp==1)angle_true=angleb-angle_error;
			x_true=Lcorrect*cos(angle_true);
			y_true=Lcorrect*sin(angle_true);
			c2=(x_true*x_true+y_true*y_true-L1*L1-L2*L2)/(2*L1*L2);
			if(c2>1||c2<-1) temp=1;
			if(temp==0) angle2_Now=(acos(c2))*180.0/PI;//small arm's angle

			if(angle2_Now<(-54.31)&&angle_before2<125.69&&angle_before2>0) angle2_Now=360+angle2_Now;
			if(angle2_Now<125.69&&angle2_Now>0&&angle_before2>305.69&&angle_before2>0) angle2_Now=360-angle2_Now;

			if(temp==0){
				cf=(L2*L2-(x_true*x_true+y_true*y_true)-L1*L1)/((-2)*L1*pow(x_true*x_true+y_true*y_true,0.5));
				if(cf>1||cf<-1) temp=1;
				anglef=(acos(cf))*180.0/PI;
				if(temp==0){                                //large arm's angle
					if(angle2_Now>0) angle1_Now=angleb+anglef;
					if(angle2_Now<0) angle1_Now=angleb-anglef;
				}
			}
			if(temp==0){
				Step1_AngleGoal =angle1_Now;
				Step2_AngleGoal =angle2_Now;
			}
		 // step initailize????
		 if(Step1_AngleGoal!=angle_before1){

			 step1=0;
			 step1=(Step1_AngleGoal - angle_before1)*1.6/1.8;
			 angle_before1=Step1_AngleGoal;
			 if(step1>=0) dir_state1=1;
			 else if(step1<0){
				 step1=-step1;
				 dir_state1=0;
			 }
			 n++;
		 }
		 if(Step2_AngleGoal!=angle_before2){
			 step2=0;
			 step2=(Step2_AngleGoal - angle_before2)*2.2/1.8;
			 angle_before2=Step2_AngleGoal;
			 if(step2>=0) dir_state2=1; // goes down
			 else if(step2<0){
				 step2=-step2;
				 dir_state2=0;
			 }
		 }
		 if(Step3_AngleGoal!=angle_before3){
			 step3=0;
			 n++;
			 step3=(Step3_AngleGoal - angle_before3)/1.8;
			 angle_before3=Step3_AngleGoal;
			 if(step3>=0) dir_state3=1;
			 else if(step3<0){
				 step3=-step3;
				 dir_state3=0;
			 }
		 }
//		 if(pump1=1) HAL_GPIO_WritePin(Pump1_IN1_PORT,Pump1_IN1_Pin,GPIO_PIN_SET);
//		 if(pump1=0) HAL_GPIO_WritePin(Pump1_IN1_PORT,Pump1_IN1_Pin,GPIO_PIN_RESET);
//		 if(pump2=1) HAL_GPIO_WritePin(Pump2_IN3_PORT,Pump2_IN3_Pin,GPIO_PIN_SET);
//		 if(pump2=0) HAL_GPIO_WritePin(Pump2_IN3_PORT,Pump2_IN3_Pin,GPIO_PIN_RESET);
//		 if(pump3=1) HAL_GPIO_WritePin(Pump3_IN4_PORT,Pump3_IN4_Pin,GPIO_PIN_SET);
//		 if(pump3=0) HAL_GPIO_WritePin(Pump3_IN4_PORT,Pump3_IN4_Pin,GPIO_PIN_RESET);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 loop();

 	 // servo control/ servo1(timer2ch3)/ servo2(timer16ch1)
 	 servo_pulse1 = 600 + 10 * Servo1_AngleGoal;
 	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, servo_pulse1);
 	 servo_pulse2 = 600 + 10 * Servo2_AngleGoal;
 	 __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, servo_pulse2);


   }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 63;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 19999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 63;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 63;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 83;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 63;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 19999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM23 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM23_Init(void)
{

  /* USER CODE BEGIN TIM23_Init 0 */

  /* USER CODE END TIM23_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM23_Init 1 */

  /* USER CODE END TIM23_Init 1 */
  htim23.Instance = TIM23;
  htim23.Init.Prescaler = 63;
  htim23.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim23.Init.Period = 19999;
  htim23.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim23.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim23) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim23, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim23, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM23_Init 2 */

  /* USER CODE END TIM23_Init 2 */
  HAL_TIM_MspPostInit(&htim23);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE8
                           PE10 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF8 PF9 PF11 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB12 PB13
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == TIM13) {
	ms4++;
	time=0.001;

	//motor 1 / encoder(time2)/ INA(PD8) INB(PD14) PWM(time12_ch2)
	CountNow_1=__HAL_TIM_GetCounter(&htim2);
	rps_1 = (double) CountNow_1/ 4 /res_encoder/sr_ratio/time;
	__HAL_TIM_SetCounter(&htim2,0);

	e_1=DC1_RpsGoal-rps_1;
	P_1=p_1*e_1;
	I_1=I_1+i_1*e_1*time;

	if(rps_1==0) I_1=0;
	else if (I_1>200) I_1=200;
	else if(I_1<-200) I_1=-200;

	u_1=P_1+I_1;

	if(u_1>=0) {
		HAL_GPIO_WritePin(DCmotor1_INA_Port,DCmotor1_INA_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DCmotor1_INB_Port,DCmotor1_INB_Pin,GPIO_PIN_RESET);
	}
	else if(u_1<0) {
		HAL_GPIO_WritePin(DCmotor1_INA_Port,DCmotor1_INA_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DCmotor1_INB_Port,DCmotor1_INB_Pin,GPIO_PIN_SET);
		u_1=-u_1;
	}

	pulse_1=(int)u_1*1000;
	if(pulse_1>65535) pulse_1=65535;

	__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,pulse_1);


	//motor 2/ encoder(time5)/ INA(PB12) INB(PB13) PWM(time12_ch1)
	CountNow_2=__HAL_TIM_GetCounter(&htim5);
	rps_2 = (double) CountNow_2/ 4 /res_encoder/sr_ratio/time;
	__HAL_TIM_SetCounter(&htim5,0);

	e_2=DC2_RpsGoal-rps_2;
	P_2=p_2*e_2;
	I_2=I_2+i_2*e_2*time;

	if(rps_2==0) I_2=0;
	else if (I_2>200) I_2=200;
	else if(I_2<-200) I_2=-200;

	u_2=P_2+I_2;

	if(u_2>=0) {
		HAL_GPIO_WritePin(DCmotor2_INA_Port,DCmotor2_INA_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DCmotor2_INB_Port,DCmotor2_INB_Pin,GPIO_PIN_RESET);
	}
	else if(u_2<0) {
		HAL_GPIO_WritePin(DCmotor2_INA_Port,DCmotor2_INA_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DCmotor2_INB_Port,DCmotor2_INB_Pin,GPIO_PIN_SET);
		u_2=-u_2;
	}

	pulse_2=(int)u_2*1000;
	if(pulse_2>65535)
	{
		pulse_2=65535;
	}

	__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,pulse_2);

	//motor 3/ encoder(time3)/ INA(PE2) INB(PE3) PWM(time15_ch1)
	CountNow_3=__HAL_TIM_GetCounter(&htim3);
	rps_3 = (double) CountNow_3/ 4 /res_encoder/sr_ratio/time;
	__HAL_TIM_SetCounter(&htim3,0);

	e_3=DC3_RpsGoal-rps_3;
	P_3=p_3*e_3;
	I_3=I_3+i_3*e_3*time;

	if(rps_3==0) I_3=0;
	else if (I_3>200) I_3=200;
	else if(I_3<-200) I_3=-200;

	u_3=P_3+I_3;

	if(u_3>=0) {
		HAL_GPIO_WritePin(DCmotor3_INA_Port,DCmotor3_INA_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DCmotor3_INB_Port,DCmotor3_INB_Pin,GPIO_PIN_RESET);
	}
	else if(u_3<0) {
		HAL_GPIO_WritePin(DCmotor3_INA_Port,DCmotor3_INA_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DCmotor3_INB_Port,DCmotor3_INB_Pin,GPIO_PIN_SET);
		u_3=-u_3;
	}

	pulse_3=(int)u_3*1000;
	if(pulse_3>65535)
	{
		pulse_3=65535;
	}

	__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,pulse_3);

	//motor 4/ encoder(time4)/ INA(PE4) INB(PC13) PWM(time15_ch2)
	CountNow_4=__HAL_TIM_GetCounter(&htim4);
	rps_4 = (double) CountNow_4/ 4 /res_encoder/sr_ratio/time;
	__HAL_TIM_SetCounter(&htim4,0);

	e_4=DC4_RpsGoal-rps_4;
	P_4=p_4*e_4;
	I_4=I_4+i_4*e_4*time;

	if(rps_4==0) I_4=0;
	else if (I_4>200) I_4=200;
	else if(I_4<-200) I_4=-200;

	u_4=P_4+I_4;

	if(u_4>=0) {
		HAL_GPIO_WritePin(DCmotor4_INA_Port,DCmotor4_INA_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DCmotor4_INB_Port,DCmotor4_INB_Pin,GPIO_PIN_RESET);
	}
	else if(u_4<0) {
		HAL_GPIO_WritePin(DCmotor4_INA_Port,DCmotor4_INA_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DCmotor4_INB_Port,DCmotor4_INB_Pin,GPIO_PIN_SET);
		u_4=-u_4;
	}

	pulse_4=(int)u_4*1000;
	if(pulse_4>65535)
	{
		pulse_4=65535;
	}

	__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,pulse_4);

	encoder[0] = rps_1;
	encoder[1] = rps_2;
	encoder[2] = rps_3;
	encoder[3] = rps_4;

}
	// control step 1/timer6
	if (htim->Instance == TIM6) {
		ms1++;


		if(step1!=0){
			if (dir_state1 == 1) HAL_GPIO_WritePin(Step1_DIR_Port, Step1_DIR_Pin, GPIO_PIN_SET);
			else if (dir_state1 == 0) HAL_GPIO_WritePin(Step1_DIR_Port, Step1_DIR_Pin, GPIO_PIN_RESET);

			if (step_state1 == 1) {
				HAL_GPIO_WritePin(Step1_STEP_Port, Step1_STEP_Pin, GPIO_PIN_SET);
				step_state1 = 0;
			}
			else if (step_state1 == 0) {
				HAL_GPIO_WritePin(Step1_STEP_Port, Step1_STEP_Pin, GPIO_PIN_RESET);
				step_state1 = 1;
				step1--;
			}
		}
	}

	// control step 2/timer7
	if (htim->Instance == TIM7) {
		ms2++;


		if(step2!=0){
			if (dir_state2 == 1) HAL_GPIO_WritePin(Step2_DIR_Port, Step2_DIR_Pin, GPIO_PIN_SET);
			else if (dir_state2 == 0) HAL_GPIO_WritePin(Step2_DIR_Port, Step2_DIR_Pin, GPIO_PIN_RESET);

			if (step_state2 == 1) {
				HAL_GPIO_WritePin(Step2_STEP_Port, Step2_STEP_Pin, GPIO_PIN_SET);
				step_state2 = 0;
			}
			else if (step_state2 == 0) {
				HAL_GPIO_WritePin(Step2_STEP_Port, Step2_STEP_Pin, GPIO_PIN_RESET);
				step_state2 = 1;
				step2--;
			}
		}
	}

	// control step 3/timer8
	if (htim->Instance == TIM8) {
		ms3++;

		if(step3!=0){
			if (dir_state3 == 1) HAL_GPIO_WritePin(Step3_DIR_Port, Step3_DIR_Pin, GPIO_PIN_SET);
			else if (dir_state3 == 0) HAL_GPIO_WritePin(Step3_DIR_Port, Step3_DIR_Pin, GPIO_PIN_RESET);

			if (step_state3 == 1) {
				HAL_GPIO_WritePin(Step3_STEP_Port, Step3_STEP_Pin, GPIO_PIN_SET);
				step_state3 = 0;
			}
			else if (step_state3 == 0) {
				HAL_GPIO_WritePin(Step3_STEP_Port, Step3_STEP_Pin, GPIO_PIN_RESET);
				step_state3 = 1;
				step3--;
			}
		}
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

