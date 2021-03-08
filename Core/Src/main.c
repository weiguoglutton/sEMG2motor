/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void UserInit(void);
void PowerOut(void);
uint8_t  GetValueLen(uint8_t channal);
void TimerInit(TIM_HandleTypeDef htim, uint16_t period);
void KeyService(void);
void Set_Value(uint8_t channel);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//ADC_Buff
uint8_t ADC_Flag = 0U;
static const uint16_t adcLen = S_CHANNEL__NUM * ADC_LEN;
uint16_t ADC_Buff[adcLen];
uint16_t S_Value[S_CHANNEL__NUM][SUBDIVIDE + 1];
static const uint16_t S_Value_Vector = 0xFFF / SUBDIVIDE;
uint8_t S_Value_Centerline[S_CHANNEL__NUM] = {S_VALUE_CENTERLINE_CHA, S_VALUE_CENTERLINE_CHB};
uint8_t S_Value_Top[S_CHANNEL__NUM] = {S_VALUE_TOP_CHA, S_VALUE_TOP_CHB};
uint16_t S_Value_Set_Cycle_Tick = 0;
uint8_t S_Value_Pos[S_CHANNEL__NUM][2];
uint8_t S_Value_Len[S_CHANNEL__NUM];
static const uint8_t S_Value_Centerline_Set_Min[S_CHANNEL__NUM] = {SET_S_VALUE_CENTERLINE_CHA_MIN, SET_S_VALUE_CENTERLINE_CHB_MIN};
static const uint8_t S_Value_Centerline_Set_Max[S_CHANNEL__NUM] = {SET_S_VALUE_CENTERLINE_CHA_MAX, SET_S_VALUE_CENTERLINE_CHB_MAX};

float Sig2Speed[S_CHANNEL__NUM] = {(float)S_VALUE_TOP_CHA / SPEED_SUBDIVIDE, (float)S_VALUE_TOP_CHB / SPEED_SUBDIVIDE};
static const uint16_t Speed_Top = MOTOR_A_TIM_PERIOD * 0.6 ; //
static const float Speed_Sub = (float)Speed_Top / SPEED_SUBDIVIDE;

float Sig2Led = (float)SUBDIVIDE / LED_SUBDIVIDE;
uint16_t Led_ARR[S_CHANNEL__NUM] = {LED_SUBDIVIDE, LED_SUBDIVIDE};
uint16_t Led_CNT[S_CHANNEL__NUM] = {LED_SUBDIVIDE, LED_SUBDIVIDE};
uint16_t Led_CCR[S_CHANNEL__NUM] = {0};
static const uint16_t  Led_Keep_ON = (LED_TIM_PERIOD + 1) / LED_SUBDIVIDE;

uint8_t Value_Status = CH_ALL;
uint8_t Key_status[S_CHANNEL__NUM] = {0};
uint8_t Key_state[S_CHANNEL__NUM] = {0};
uint16_t Key_Down_Tick[S_CHANNEL__NUM] = {0};
uint16_t Key_Up_Tick[S_CHANNEL__NUM] = {0};

uint8_t Centerline_Set = CH_DIS;
uint8_t Value_Set_Channel = CH_DIS;
uint8_t Value_Set_Status = 0;
uint16_t Value_Set_Tem = 0;
uint8_t Value_Set_Flag = 1U;

uint16_t Sig2SpeedLevel[S_CHANNEL__NUM];

uint8_t PWR_Flag = 1U;
uint16_t PWR_Value;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

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
    MX_DMA_Init();
    MX_ADC_Init();
    MX_TIM3_Init();
    MX_TIM1_Init();
    MX_TIM14_Init();
    MX_TIM16_Init();

    /* USER CODE BEGIN 2 */
    for(uint16_t i = 0; i < 20; i++)
    {
        HAL_GPIO_TogglePin (LED_CHA_GPIO_Port, LED_CHA_Pin);
        HAL_GPIO_TogglePin (LED_CHB_GPIO_Port, LED_CHB_Pin);
        HAL_Delay(200);
    }

    UserInit();

//hadc.Instance->DR;
//S_Value_Len S_Value_Centerline
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {

        PowerOut();
        KeyService();
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
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

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14 | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
    RCC_OscInitStruct.HSI14CalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef *htim )
{

    if ( htim->Instance == LED_TIM.Instance )
    {
        if(ADC_Flag == 0U && Centerline_Set == CH_DIS && Value_Set_Channel == CH_DIS)
        {
            Led_ARR[CHA] = (uint16_t)(S_Value_Len[CHA] / Sig2Led);
            Led_ARR[CHB] = (uint16_t)(S_Value_Len[CHB] / Sig2Led);

        }

//Led_CNT
        if(Led_ARR[CHA] < LED_CENTERLINE)
            Led_ARR[CHA] = 0;

        if(Led_ARR[CHB] < LED_CENTERLINE)
            Led_ARR[CHB] = 0;

        if(Led_CNT[CHA] < Led_ARR[CHA])
        {
            if(Led_CCR[CHA] < LED_kEEP_ON)
            {
                Led_CCR[CHA]++;
                HAL_GPIO_WritePin (LED_CHA_GPIO_Port, LED_CHA_Pin, !0U);
            }
            else
            {
                Led_CNT[CHA] = LED_SUBDIVIDE - 1;
                Led_CCR[CHA] = 0;
                HAL_GPIO_WritePin (LED_CHA_GPIO_Port, LED_CHA_Pin, 0U);
            }

        }
        else if(Led_ARR[CHA] > 0)
        {
            Led_CNT[CHA]--;
        }
        else
        {

            HAL_GPIO_WritePin (LED_CHA_GPIO_Port, LED_CHA_Pin, 0U);
        }

        if(Led_CNT[CHB] < Led_ARR[CHB])
        {
            if(Led_CCR[CHB] < LED_kEEP_ON)
            {
                Led_CCR[CHB]++;
                HAL_GPIO_WritePin (LED_CHB_GPIO_Port, LED_CHB_Pin, !0U);
            }
            else
            {
                Led_CNT[CHB] = LED_SUBDIVIDE - 1;
                Led_CCR[CHB] = 0;
                HAL_GPIO_WritePin (LED_CHB_GPIO_Port, LED_CHB_Pin, 0U);
            }
        }
        else if(Led_ARR[CHB] > 0)
        {
            Led_CNT[CHB]--;
        }
        else
            HAL_GPIO_WritePin (LED_CHB_GPIO_Port, LED_CHB_Pin, 0U);

    }

    if( htim->Instance == KEY_TIM.Instance )
    {
        //Key_Down_Tick Key_Up_Tick
        if(HAL_GPIO_ReadPin(KEY_A_GPIO_Port, KEY_A_Pin) == 0U)
            Key_Down_Tick[CHA]++;
        else
        {
            Key_Down_Tick[CHA] = 0;
            Key_Up_Tick[CHA]++;
        }

        if(HAL_GPIO_ReadPin(KEY_B_GPIO_Port, KEY_B_Pin) == 0U)
            Key_Down_Tick[CHB]++;
        else
        {

            Key_Up_Tick[CHB]++;
            Key_Down_Tick[CHB] = 0;
        }
    }

}

void PowerOut(void)
{

    if(Centerline_Set == CH_DIS && Value_Set_Channel == CH_DIS)
    {
        if(GetValueLen(CH_ALL))
        {

            Sig2SpeedLevel[CHA] = S_Value_Len[CHA] / Sig2Speed[CHA];
            Sig2SpeedLevel[CHB] = S_Value_Len[CHB] / Sig2Speed[CHB] ;

            if(Sig2SpeedLevel[CHA] > Sig2SpeedLevel[CHB]
							&&Sig2SpeedLevel[CHA] - Sig2SpeedLevel[CHB] > COMPARE_TH)
            {
                    Value_Status = CHA;
            }
            else if(Sig2SpeedLevel[CHB] > Sig2SpeedLevel[CHA]
							&&Sig2SpeedLevel[CHB] - Sig2SpeedLevel[CHA] > COMPARE_TH)
            {
                    Value_Status = CHB;
            }
            else
                Value_Status = CH_DIS;

            if(Value_Status == CHA)
            {
                MOTOR_A_CCRA = Sig2SpeedLevel[CHA] * Speed_Sub; //Sig2SpeedLevel
                MOTOR_A_DIR_A_EN;
            }
            else if(Value_Status == CHB)
            {
                MOTOR_A_CCRB = Sig2SpeedLevel[CHB] * Speed_Sub; //Sig2SpeedLevel
                MOTOR_A_DIR_B_EN;
            }
            else
                MOTOR_A_DIS;



        }


    }
    else if(Value_Set_Channel < CH_DIS)
    {
        Set_Value(Value_Set_Channel);
        Value_Status = CH_DIS;
        MOTOR_A_DIS;
    }

}

uint8_t GetValueLen(uint8_t channal)
{

    if(ADC_Flag)
    {
        ADC_Flag = 0U;
        Value_Set_Flag = 1U;
        HAL_TIM_Base_Stop_IT(&LED_TIM);

        if(PWR_Flag)
        {
            PWR_Flag = 0U;

            for(uint16_t i = 0, j = 0; i < adcLen; i++, j++)
            {
                S_Value[CHA][ADC_Buff[i] / S_Value_Vector]++;
                i++;
                S_Value[CHB][ADC_Buff[i] / S_Value_Vector]++;

            }

            hadc.Instance->CHSELR = PWR_ADC;
            HAL_ADC_Start_DMA ( &hadc, ( uint32_t* ) &PWR_Value, 1 );

            for(uint16_t i = 0, j[S_CHANNEL__NUM] = {0U}; i < SUBDIVIDE; i++)
            {
                if(channal == CH_ALL)
                {
                    if(S_Value[CHA][i] > S_VALUE_COUNT_TH)
                    {
                        if(	j[CHA] == 0U)
                            S_Value_Pos[CHA][1] = i;
                        else
                            S_Value_Pos[CHA][0] = i;

                        j[CHA] = 1U;
                    }

                    if(S_Value[CHB][i] > S_VALUE_COUNT_TH)
                    {
                        if(	j[CHB] == 0U)
                            S_Value_Pos[CHB][1] = i;
                        else
                            S_Value_Pos[CHB][0] = i;

                        j[CHB] = 1U;
                    }

                }
                else
                {
                    if(S_Value[channal][i] > S_VALUE_COUNT_TH)
                    {
                        if(	j[channal] == 0U)
                            S_Value_Pos[channal][1] = i;
                        else
                            S_Value_Pos[channal][0] = i;

                        j[channal] = 1U;
                    }
                }
            }

            if(channal == CH_ALL)
            {
                S_Value_Len[CHA] = S_Value_Pos[CHA][0] - S_Value_Pos[CHA][1];
                S_Value_Len[CHB] = S_Value_Pos[CHB][0] - S_Value_Pos[CHB][1];
            }
            else
                S_Value_Len[channal] = S_Value_Pos[channal][0] - S_Value_Pos[channal][1];

            for(uint16_t i = 0; i < SUBDIVIDE; i++)
            {
                for(uint16_t j = 0; j < S_CHANNEL__NUM; j++)
                    S_Value[j][i] = 0;
            }



        }//PWR_Value
        else
        {
            if(PWR_Value > 2450)
            {
                PWR_Flag = 1U;
                hadc.Instance->CHSELR = EMG_ADC;
                HAL_ADC_Start_DMA ( &hadc, ( uint32_t* ) &ADC_Buff, adcLen );

            }
            else
            {
                MOTOR_A_DIS;
                HAL_ADC_Start_DMA ( &hadc, ( uint32_t* ) &PWR_Value, 1 );
            }

        }

        HAL_TIM_Base_Start_IT(&LED_TIM);
        return 1U;
    }


}


void KeyService(void)
{
//判断是否长按
    if((Key_Up_Tick[CHA] > KEY_TIME_OUT) || (Key_Up_Tick[CHB] > KEY_TIME_OUT))
    {
        Centerline_Set = CH_DIS;

        for(uint8_t i = 0; i < S_CHANNEL__NUM; i++)
        {
            Key_status[i] = 0;
            Key_Up_Tick[i] = 0;
            Led_ARR[i] = 0;
        }

        HAL_TIM_Base_Stop_IT(&KEY_TIM);
    }

    if(Key_Down_Tick[CHA] > KEY_LONG || Key_Down_Tick[CHB] > KEY_LONG)
    {
        HAL_TIM_Base_Stop_IT(&KEY_TIM);
        uint8_t channel = Key_Down_Tick[CHA] > KEY_LONG ? CHA : CHB;
        Centerline_Set = CH_DIS;
        Value_Set_Channel = channel;

        for(uint8_t i = 0; i < S_CHANNEL__NUM; i++)
        {
            Key_Down_Tick[i] = 0;
        }
    }
    else if((Key_Down_Tick[CHA] > KEY_SHORT || Key_Down_Tick[CHB] > KEY_SHORT) && Value_Set_Channel == CH_DIS)
    {
        //如果不是长按，则判断两个按键是否都处于松开状态
        if((HAL_GPIO_ReadPin(KEY_A_GPIO_Port, KEY_A_Pin) == !0U) && (HAL_GPIO_ReadPin(KEY_B_GPIO_Port, KEY_B_Pin) == !0U))
        {
            uint8_t channel = Key_Down_Tick[CHA] > KEY_SHORT ? CHA : CHB;
            Key_Down_Tick[channel] = 0;

            if(Centerline_Set == CH_DIS)
            {
                switch(Key_status[channel])
                {
                    case 0:
                        Key_status[channel]++;
                        break;

                    case 1:
                        Led_ARR[channel] = LED_SUBDIVIDE;

                        for(uint8_t i = 0U; i < S_CHANNEL__NUM; i++)
                        {
                            if(i != channel)
                                Led_ARR[i] = 0;
                        }

                        Centerline_Set = channel;		//当判断为双击，则进入Centerline设置状态
                        break;
                }
            }
            else
            {
                Key_Up_Tick[CHA] = 0;
                Key_Up_Tick[CHB] = 0;

                //调节Centerline幅值
                switch(channel)
                {
                    case CHA:
                        if(S_Value_Centerline[Centerline_Set] > S_Value_Centerline_Set_Min[Centerline_Set])
                            S_Value_Centerline[Centerline_Set]--;

                        break;

                    case CHB:
                        if(S_Value_Centerline[Centerline_Set] < S_Value_Centerline_Set_Max[Centerline_Set])
                            S_Value_Centerline[Centerline_Set]++;

                        break;
                }
            }

        }

    }
}


void Set_Value(uint8_t channel)
{
    GetValueLen(channel);

    if(ADC_Flag == 0U)
    {
        if(Value_Set_Flag)
        {
            Value_Set_Flag = 0U;

            switch(Value_Set_Status)
            {
                case 0:

                    //当采集次数超出阈值则进入下一状态
                    if(S_Value_Set_Cycle_Tick > S_VALUE_SET_CYCLE)
                    {
                        Value_Set_Status++;
                        S_Value_Top[channel] = 0;
                    }
                    else
                    {
                        //设置led输出pwm占空比
                        Led_ARR[channel] = LED_SUBDIVIDE * 0.5;

                        for(uint8_t i = 0U; i < S_CHANNEL__NUM; i++)
                        {
                            if(i != channel)
                                Led_ARR[i] = 0;
                        }

                        //获取最大值作为Centerline基准
                        if(S_Value_Len[channel] > S_Value_Centerline[channel]
                                && S_Value_Len[channel] > S_Value_Centerline_Set_Min[channel]
                                && S_Value_Len[channel] < S_Value_Centerline_Set_Max[channel])
                            S_Value_Centerline[channel] = S_Value_Len[channel] + S_VALUE_SET_TH;

                    }

                    break;

                case 1:

                    if(S_Value_Set_Cycle_Tick > S_VALUE_SET_CYCLE * 2)
                    {
                        //修改速度区间并退出设置状态
                        Led_ARR[channel] = 0;
                        Value_Set_Status = 0;
                        S_Value_Set_Cycle_Tick = 0;
                        Value_Set_Channel = CH_DIS;
                        Sig2Speed[channel] = (float)(S_Value_Top[channel] - S_Value_Centerline[channel]) / SPEED_SUBDIVIDE;
                    }
                    else
                    {

                        //设置led为常亮
                        Led_ARR[channel] = LED_SUBDIVIDE;

                        //获取最大值作为Top线
                        if(S_Value_Len[channel] > S_Value_Top[channel])
                            S_Value_Top[channel] = S_Value_Len[channel] + S_VALUE_SET_TH;

                    }

                    break;
            }

            if(Value_Set_Channel < CH_DIS)
                S_Value_Set_Cycle_Tick++;
        }

    }

}

void HAL_ADC_ConvCpltCallback ( ADC_HandleTypeDef* AdcHandle )
{
    HAL_ADC_Stop_DMA(&hadc);
    ADC_Flag = 1U;
}

void HAL_GPIO_EXTI_Callback ( uint16_t GPIO_Pin )
{
    switch(GPIO_Pin)
    {
        case KEY_A_Pin:
            HAL_TIM_Base_Start_IT ( &KEY_TIM );
            break;

        case KEY_B_Pin:
            HAL_TIM_Base_Start_IT ( &KEY_TIM );
            break;
    }
}

void UserInit(void)
{
    for(uint16_t i = 0; i < SUBDIVIDE; i++)
    {
        for(uint16_t j = 0; i < S_CHANNEL__NUM; i++)
            S_Value[j][i] = 0;
    }

    S_Value_Pos[CHA][0] = 0;
    S_Value_Pos[CHB][0] = 0;
    S_Value_Pos[CHA][1] = 0xFF;
    S_Value_Pos[CHB][1] = 0xFF;
    //TIM3->CNT  S_Value

    TimerInit(ADC_TIM, ADC_TIM_PERIOD);
    HAL_TIM_Base_Start ( &ADC_TIM );
    TimerInit(LED_TIM, LED_TIM_PERIOD);
    HAL_TIM_Base_Start_IT ( &LED_TIM );
    TimerInit(MOTOR_A_TIM, MOTOR_A_TIM_PERIOD);
    HAL_TIM_Base_Start ( &MOTOR_A_TIM );
    TimerInit(KEY_TIM, KEY_TIM_PERIOD);
    HAL_TIM_Base_Start_IT ( &KEY_TIM );

    HAL_ADC_Start_DMA ( &hadc, ( uint32_t* ) &ADC_Buff, adcLen );

}

void TimerInit(TIM_HandleTypeDef htim, uint16_t period)
{
    //TIM14->CNT TIM16->CNT TIM1->CNT TIM3->CNT
    htim.Instance->ARR = period;
    htim.Instance->CCER = 0U;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
