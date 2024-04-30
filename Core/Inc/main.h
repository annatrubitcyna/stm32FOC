/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
struct trAngle{
	float an;
	float sin;
	float cos;
};

//structure for Kalman filter
struct Observer{
	float y_hat;
	float x1_hat;
	float x2_hat;
	float l1;
	float l2;
	//params
	float q1;
	float q2;
	float r;
};

struct Filter{
	float x[4][3];
	float x_med[4];
	float k;
	float dx_max;      // if current change for that value k change to k_max
	float k_max;
	float x_hat[4];
};
struct Controller{
	float Kp;
	float Ki;
	float Kd;
};

struct Motor{
	uint32_t zp;       // number of pole pairs
	float R;           // Resistance of stator winding
	float L;           // d-axis and q-axis inductance of stator winding
	float psiF;        // rotor permanent magnets flux linkage
	float startAngle;  // encoder and flux zero angle shift
	uint32_t isEncInit;
	int32_t initDir;

	uint32_t data;          //array for encoder data
	uint16_t adcData[2];    //array for current sensor data (12-bit adc)
	float mechAngle;        // angle (encoder data)
	float pMAngle;          // previous angle to find turn
	int32_t kTurns;
	int32_t prevkTurns;     // to count speed
	float prevMAngle;       // previous angle to count speed
	struct trAngle elAngle; // rotor flux angle ( A phase - 0)
	float speed;            // rotor speed (rad/sec)
	float time;

	float Umax;             // max voltage (about power supply)
	float Imax;
	float speedMax;
	float U;
	float alpha;
	float Uq;               // synchronous axis reference frame quantities
	float Ud;
	float IqRef;
	float IdRef;
	float Iq;
	float Id;
	float IA;
	float IB;
	float IAs;              // for averaging
	float IBs;

	float refAngle;
	float startRefSpeed;
	float refSpeed;         // if speed controller
	float dSpeed;
	float eIAngle;          // I-regulators sum
	float eISpeed;
	float eIq;
	float eId;
	struct Controller sC;   // speed controller

	struct Observer obs;
	struct Filter filter;

	uint32_t speedPerCounter;  // periods counter
	uint32_t speedFrequency;   // speed count frequency
	uint32_t flagSpeed;        // for first speed counting

	int32_t speedSign;         // to change speed with button
	CORDIC_HandleTypeDef* hcordic;
};

struct SentData{
	uint32_t header;
	float refAngle;
	float angle;
	float refSpeed;
	float speed;
	float Id;
	float Iq;
	float time;
	float Uq;
	uint32_t terminator;
};

struct RefData{
	 uint32_t header;
//	 int32_t refSpeed;
	 float Kp;
	 float Ki;
	 float Kd;
	 uint32_t terminator;
};

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_CSn_Pin GPIO_PIN_4
#define SPI1_CSn_GPIO_Port GPIOA
#define BUTTON_Pin GPIO_PIN_12
#define BUTTON_GPIO_Port GPIOB
#define BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define EN_Pin GPIO_PIN_14
#define EN_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
