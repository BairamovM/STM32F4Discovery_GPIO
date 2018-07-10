
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "stdbool.h" // library for BOOLEAN type variables
// C:\Ac6\SystemWorkbench\plugins\fr.ac6.mcu.externaltools.arm-none.win32_1.15.0.201708311556\tools\compiler\lib\gcc\arm-none-eabi\6.3.1\include

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

bool Status_user_btn = 1; // variable to check user_button push or not

bool Status_JOY_A = 0;    // variable to check JOY_A_button push or not
bool Status_JOY_B = 0;    // variable to check JOY_B_button push or not
bool Status_JOY_C = 0;    // variable to check JOY_C_button push or not
bool Status_JOY_D = 0;    // variable to check JOY_D_button push or not

bool Status_JOY_CTR = 1;  // variable to check JOY_CTR_button push or not
bool Status_sett = 1;     // variable to wait for JOY_*_button push

uint32_t Delay_JOY;       // value to Delay Blink Features when pushed JOY_*_button


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void blink1_func(void);
void blink2_func(void);
void blink3_func(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//-------------------------------------------------------------------------------------------------------------
void blink1_func()
{
  for(int i=0; i<=4; i++)
	 {
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); // Turn on LED3_Orange
	  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET); // Turn on LED5_Red
	  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Turn on LED6_Blue
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Turn on LED4_Green
	  HAL_Delay(Delay_JOY);                                         // Delay 1s

	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Turn off LED3_Orange
	  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // Turn off LED5_Red
	  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET); // Turn off LED6_Blue
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET); // Turn off LED4_Green
	  HAL_Delay(Delay_JOY);                                           // Delay 1s

	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Turn off LED3_Orange
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Turn off LED4_Green
	  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // Turn off LED5_Red
	  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Turn off LED6_Blue
	  HAL_Delay(Delay_JOY);
	 }
}
//-------------------------------------------------------------------------------------------------------------
void blink2_func()
{
  for(int i=0; i<=4; i++)
	 {
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Turn on LED3_Orange
	  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET); // Turn on LED5_Red
	  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Turn on LED6_Blue
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET); // Turn on LED4_Green
	  HAL_Delay(Delay_JOY);                                         // Delay 1s

	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); // Turn off LED3_Orange
	  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // Turn off LED5_Red
	  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET); // Turn off LED6_Blue
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Turn off LED4_Green
	  HAL_Delay(Delay_JOY);                                           // Delay 1s

	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Turn off LED3_Orange
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Turn off LED4_Green
	  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // Turn off LED5_Red
	  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Turn off LED6_Blue
	  HAL_Delay(Delay_JOY);
	 }
}
//-------------------------------------------------------------------------------------------------------------
void blink3_func()
{
  for(int i=0; i<=4; i++)
	 {
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); // Turn on LED3_Orange
	  HAL_Delay(Delay_JOY);

	  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET); // Turn on LED5_Red
	  HAL_Delay(Delay_JOY);

	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Turn on LED3_Orange
	  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET); // Turn on LED6_Blue
	  HAL_Delay(Delay_JOY);


	  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // Turn on LED5_Red
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET); // Turn on LED4_Green
	  HAL_Delay(Delay_JOY);                                         // Delay 1s

	  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Turn on LED6_Blue
	  HAL_Delay(Delay_JOY);

	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Turn on LED4_Green
	  HAL_Delay(Delay_JOY);

	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Turn off LED3_Orange
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Turn off LED4_Green
	  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // Turn off LED5_Red
	  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Turn off LED6_Blue
	  HAL_Delay(Delay_JOY);
	 }
}
//-------------------------------------------------------------------------------------------------------------
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  /* USER CODE BEGIN 2 */

//---- Test LEDS --------------------------------------------------------------------------------
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); // Turn on LED3_Orange
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET); // Turn on LED4_Green
  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET); // Turn on LED5_Red
  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET); // Turn on LED6_Blue
  HAL_Delay(1000);                                         // Delay 1s

  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Turn off LED3_Orange
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Turn off LED4_Green
  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // Turn off LED5_Red
  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Turn off LED6_Blue
  HAL_Delay(1000);                                           // Delay 1s
//---- END Test LEDS ------------------------------------------------------------------------------

//---- Delay settings for Blink Features -------------------------------------------------------------------------
  while(Status_JOY_CTR) // Main wait for JOY_*_button push
       {
	    while(Status_sett) // wait for JOY_*_button push (ABCD)
             {
                 /* A - LD4 */   // This Function check if button JOY_A pushed or not
	          if(HAL_GPIO_ReadPin(JOY_A_GPIO_Port, JOY_A_Pin) == GPIO_PIN_RESET)
	            {
	              HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);   // Turn on LED4_Green
	              HAL_Delay(250);                                            // Delay 250ms
	              HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Turn off LED4_Green
	              Delay_JOY = 250;  // Value 250ms for All Blink Features
	              Status_JOY_A = 1;   // button JOY_A pushed
	              Status_sett = 0;    // button JOY_A pushed
	            }

                 /* B - LD3 */   // This Function check if button JOY_B pushed or not
	          if(HAL_GPIO_ReadPin(JOY_B_GPIO_Port, JOY_B_Pin) == GPIO_PIN_RESET)
	            {
	              HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);   // Turn on LED3_Orange
	              HAL_Delay(500);                                            // Delay 500ms
	              HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Turn off LED3_Orange
	              Delay_JOY = 500; // Value 500ms for All Blink Features
	              Status_JOY_B = 1;  // button JOY_B pushed
	              Status_sett = 0;   // button JOY_B pushed
	            }

                 /* C - LD6 */   // This Function check if button JOY_C pushed or not
	          if(HAL_GPIO_ReadPin(JOY_C_GPIO_Port, JOY_C_Pin) == GPIO_PIN_RESET)
	            {
	              HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);   // Turn on LED6_Blue
	              HAL_Delay(750);                                            // Delay 500ms
	              HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Turn off LED6_Blue
	              Delay_JOY = 750; // Value 750ms for All Blink Features
	              Status_JOY_C = 1;  // button JOY_C pushed
	              Status_sett = 0;   // button JOY_C pushed
	            }

                 /* D - LD5 */   // This Function check if button JOY_D pushed or not
	          if(HAL_GPIO_ReadPin(JOY_D_GPIO_Port, JOY_D_Pin) == GPIO_PIN_RESET)
	            {
	              HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);   // Turn on LED5_Red
	              HAL_Delay(1000);                                           // Delay 1s
	              HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // Turn off LED5_Red
	              Delay_JOY = 1000; // Value 1000ms for All Blink Features
	              Status_JOY_D = 1;   // button JOY_D pushed
	              Status_sett = 0;    // button JOY_D pushed
	            }
             }

	         while(Status_JOY_CTR) // wait for JOY_CTR_button push
                  {
	    	        /* PRESS - LD4 LD3 LD6 LD5 */  // This Function check if button JOY_CTR pushed or not
	    	        if(HAL_GPIO_ReadPin(JOY_CTR_GPIO_Port, JOY_CTR_Pin) == GPIO_PIN_RESET)
                      {
	    	            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); // Turn on LED3_Orange
	    	            HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET); // Turn on LED4_Green
	    	            HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET); // Turn on LED5_Red
	    	            HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET); // Turn on LED6_Blue
	    	            HAL_Delay(100); // Delay 100ms

	    	            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Turn off LED3_Orange
	    	            HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Turn off LED4_Green
	    	            HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // Turn off LED5_Red
	    	            HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Turn off LED6_Blue

	    	               // check if one of then buttons JOY_* pushed
	    	               // If pushed - it can confirm Delay Blink settings
	    	            if ((Status_JOY_A || Status_JOY_B || Status_JOY_C || Status_JOY_D) ==1)
	    	               {
	    	                Status_JOY_CTR = 0; // button JOY_CTR pushed and All settings for
	    	               	 	    	        // Blink Features ready
	    	               }
                      }
                   }
       }

//---- END Delay settings for Blink Features ----------------------------------------------------------------------

//---- Test USER_BUTTON ---------------------------------------------------------------------------
  while(Status_user_btn) //wait for pushed the USER_BUTTON
       {
	     // This Function check if button pushed or not
	     if(HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_RESET)
	       {
	    	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Turn off LED3_Orange
	    	HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Turn off LED4_Green
	    	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // Turn off LED5_Red
	    	HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Turn off LED6_Blue
	       }
	       else
	          {
	    	   HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); // Turn on LED3_Orange
	    	   HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET); // Turn on LED4_Green
	    	   HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET); // Turn on LED5_Red
	    	   HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET); // Turn on LED6_Blue
	    	   HAL_Delay(1000);                                         // Delay 1s
	    	   Status_user_btn = 0; // Button is pushed and exit from algorithm
	          }
       }
//---- END Test USER_BUTTON -----------------------------------------------------------------------

//---- Test TogglePin Function --------------------------------------------------------------------
  for(int i=0; i<=10; i++) // Blink All LEDS for 5 times
     {
	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin); //Change PinState LED3_Orange
	  HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin); //Change PinState LED4_Green
	  HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin); //Change PinState LED5_Red
	  HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin); //Change PinState LED6_Blue
	  HAL_Delay(250); // Delay 250 ms
     }
//---- END TogglePin Function ---------------------------------------------------------------------

//---- Blink Function with Delay Values -----------------------------------------------------------------------
blink1_func();
blink2_func();
blink3_func();
//---- END Blink Function with Delay Values -------------------------------------------------------------------

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
      {

    /* A - LD4 */   // This Function check if button JOY_A pushed or not
             if(HAL_GPIO_ReadPin(JOY_A_GPIO_Port, JOY_A_Pin) == GPIO_PIN_RESET)
                HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET); // Turn on LED4_Green
             else
                HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);  // Turn off LED4_Green

    /* B - LD3 */   // This Function check if button JOY_B pushed or not
             if(HAL_GPIO_ReadPin(JOY_B_GPIO_Port, JOY_B_Pin) == GPIO_PIN_RESET)
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); // Turn on LED3_Orange
             else
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Turn off LED3_Orange

    /* C - LD6 */   // This Function check if button JOY_C pushed or not
             if(HAL_GPIO_ReadPin(JOY_C_GPIO_Port, JOY_C_Pin) == GPIO_PIN_RESET)
                HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET); // Turn on LED6_Blue
             else
                HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Turn off LED6_Blue

    /* D - LD5 */   // This Function check if button JOY_D pushed or not
             if(HAL_GPIO_ReadPin(JOY_D_GPIO_Port, JOY_D_Pin) == GPIO_PIN_RESET)
                HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET); // Turn on LED5_Red
             else
                HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // Turn off LED5_Red

    /* PRESS - LD4 LD3 LD6 LD5 */    // This Function check if button JOY_CTR pushed or not
             if(HAL_GPIO_ReadPin(JOY_CTR_GPIO_Port, JOY_CTR_Pin) == GPIO_PIN_RESET)
               {
                HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET); // Turn on LED4_Green
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); // Turn on LED3_Orange
                HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET); // Turn on LED6_Blue
                HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET); // Turn on LED5_Red
               }
               else
                  {
                   HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Turn off LED4_Green
                   HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Turn off LED3_Orange
                   HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Turn off LED6_Blue
                   HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // Turn off LED5_Red
                  }

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
