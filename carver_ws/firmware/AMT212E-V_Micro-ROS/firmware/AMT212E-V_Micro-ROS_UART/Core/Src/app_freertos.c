/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dma.h"
#include "iwdg.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include "AMT212EV.h"
#include "PWM.h"
#include "Controller.h"
#include "Cytron_MDXX.h"
#include "math.h"

#include <amt212ev_interfaces/msg/amt_read.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int8.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ALPHA 0.1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RCSOFTCHECK(fn) if (fn != RCL_RET_OK) {};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
AMT212EV amt;
MDXX motor;
PID_CONTROLLER pid_pos;
PID_CONTROLLER pid_vel;

int32_t t =0;
float theta=0;
#define SAMPLE_RATE 1000 // Define the sample rate in Hz
#define FREQUENCY 0.5f   // Define the sine wave frequency in Hz



float kp_pos = 6.0;
float ki_pos = 0.0;
float kd_pos= 0.0;
float u_max_pos = 6.0;

float kp_vel = 7200;
float ki_vel = 220.0;
float kd_vel= 0.0;
float u_max_vel = 65535.0;

float cmd_vx = 0;
float cmd_ux = 0;
float setpoint = 0;

float error_pose = 0.0;
float filtered_value = 0.0;
float steering_angle = 0.0;

float vel_filt = 0.0;
float pos_filt = 0.0;

int8_t steering_mode = 0.0;

int16_t sync_counter = 1000;

rcl_node_t node;

rcl_publisher_t amt_publisher;
amt212ev_interfaces__msg__AmtRead amt_msg_pub;

rcl_subscription_t amt_subscription;
amt212ev_interfaces__msg__AmtRead amt_msg_sub;

rcl_subscription_t amt_subscription_mode;
amt212ev_interfaces__msg__AmtRead amt_msg_sub_mode;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void timer_callback(rcl_timer_t * timer, int64_t last_call_time);

void amt_publish(double rads, double radps);
void subscription_callback(const void * msgin);
void subscription_callback_mode(const void * msgin);

float update_filter(float input);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	AMT212EV_Init(&amt, &huart1, 1000, 16384);
	PID_CONTROLLER_Init(&pid_pos, kp_pos, ki_pos, kd_pos, u_max_pos);
	PID_CONTROLLER_Init(&pid_vel, kp_vel, ki_vel, kd_vel, u_max_vel);
	MDXX_init(&motor, &htim8, TIM_CHANNEL_3, &htim8, TIM_CHANNEL_1);
	MDXX_set_range(&motor, 2000, 0);
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  // micro-ROS configuration
  rmw_uros_set_custom_transport(
	true,
	(void *) &hlpuart1,
	cubemx_transport_open,
	cubemx_transport_close,
	cubemx_transport_write,
	cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	  printf("Error on default allocators (line %d)\n", __LINE__);
  }

	// micro-ROS app
  rcl_timer_t AMT_timer;
  rclc_support_t support;
  rclc_executor_t executor;
  rcl_allocator_t allocator;
  rcl_init_options_t init_options;

  const unsigned int timer_period = RCL_MS_TO_NS(1);
  const int timeout_ms = 5000;
  int executor_num = 3;

  const rosidl_message_type_support_t * amt_pub_type_support =
  	  ROSIDL_GET_MSG_TYPE_SUPPORT(amt212ev_interfaces, msg, AmtRead);

  const rosidl_message_type_support_t * amt_sub_type_support =
    	  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);

  const rosidl_message_type_support_t * amt_sub_type_support_mode =
    	  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8);

  allocator = rcl_get_default_allocator();

  executor = rclc_executor_get_zero_initialized_executor();

  init_options = rcl_get_zero_initialized_init_options();

  RCSOFTCHECK(rcl_init_options_init(&init_options, allocator));
  RCSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 198));

  // create support init_options
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // create timer
  rclc_timer_init_default(&AMT_timer, &support, timer_period, timer_callback);

  // create node
  rclc_node_init_default(&node, "uros_AMT_Node", "", &support);

  // create publisher
  rclc_publisher_init_best_effort(&amt_publisher, &node, amt_pub_type_support, "amt_publisher");

  // create subscriber
  rclc_subscription_init_default(&amt_subscription, &node, amt_sub_type_support, "steering_angle");
  rclc_subscription_init_default(&amt_subscription_mode, &node, amt_sub_type_support_mode, "steering_mode");
  // create service server

  // create service client

  // create executor
  rclc_executor_init(&executor, &support.context, executor_num, &allocator);

  rclc_executor_add_timer(&executor, &AMT_timer);
  rclc_executor_add_subscription(&executor, &amt_subscription, &amt_msg_sub, &subscription_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &amt_subscription_mode, &amt_msg_sub_mode, &subscription_callback_mode, ON_NEW_DATA);

  rclc_executor_spin(&executor);
  rmw_uros_sync_session(timeout_ms);

  for(;;)
  {
//	osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
//void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
//{
//	if (timer != NULL)
//	{
//		amt_publish(amt.rads,amt.radps);
//		if (uwTick >= sync_counter) {  // Sync session at lower frequency
//			rmw_uros_sync_session(1000);
//			sync_counter += 1000;
//		}
//		HAL_IWDG_Refresh(&hiwdg);
//		cc++;
//	}
//}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    if (timer != NULL)
    {
        // Publish data
        amt_publish(amt.rads, amt.radps);

        static uint32_t sync_counter = 0;
        if (sync_counter <= HAL_GetTick()) {

            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                rmw_uros_sync_session(1000); // Sync if agent is reachable
            }
            else {
                // If the agent is not reachable, let the watchdog reset the MCU
            	NVIC_SystemReset();
            }

            sync_counter = HAL_GetTick() + 1000;
        }

        // Refresh watchdog to prevent reset during normal operation
        HAL_IWDG_Refresh(&hiwdg);

    }
}


void amt_publish(double rads, double radps)
{
	amt_msg_pub.rads = rads;
	amt_msg_pub.radps = radps;
	rcl_ret_t ret = rcl_publish(&amt_publisher, &amt_msg_pub, NULL);
	if (ret != RCL_RET_OK) printf("Error publishing (line %d)\n", __LINE__);
}

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Float32 * amt_msg_sub = (const std_msgs__msg__Float32 *)msgin;
	steering_angle = amt_msg_sub->data;
}

void subscription_callback_mode(const void * msgin)
{
	const std_msgs__msg__Int8 * amt_msg_sub_mode = (const std_msgs__msg__Int8 *)msgin;
	steering_mode = amt_msg_sub_mode->data;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	  if (htim->Instance == TIM1) {
	    HAL_IncTick();
	  }

	  if (htim->Instance == TIM2)
	  {

		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
		{
			AMT212EV_SetZero(&amt);
		}

	    AMT212EV_ReadPosition(&amt);
	    AMT212EV_DiffCount(&amt);
	    AMT212EV_Compute(&amt);

	    vel_filt = -update_filter(amt.radps);
	    pos_filt = -amt.rads;

	    if (steering_angle >= (0.61))
	    	{
	    	steering_angle = 0.6;
	    	}
	    if (steering_angle <= (-0.61))
	    	{
	    	steering_angle = -0.6;
	    	}
	    if (pos_filt >= (0.7))
			{
			steering_angle = 0.6;
			}
		if (pos_filt <= (-0.7))
			{
			steering_angle = -0.6;
			}


	    if (uwTick > 1000)
	    {
//			static int toggle_flag = 0; // Keeps track of PWM toggle state
//
//			if (t >= 3000) {
//				// Toggle PWM between 45 and -45
//				if (toggle_flag == 0) {
//					setpoint = 0.5;
//					toggle_flag = 1;
//				} else {
//					setpoint = -0.5;
//					toggle_flag = 0;
//				}
//				t = 0; // Reset timer
//			}
//			static float phase = 0.0f; // Phase accumulator for the sine wave
//			theta = sinf(phase) * 0.5; // Scale sine wave amplitude to +/- 45
//			phase += (2 * M_PI * FREQUENCY) / SAMPLE_RATE; // Increment phase based on sample rate
//			if (phase >= 2 * M_PI) {
//				phase -= 2 * M_PI; // Wrap phase within [0, 2*PI]
//			}

			error_pose = steering_angle - pos_filt;

	    	cmd_vx = PID_CONTROLLER_Compute(&pid_pos,error_pose);
	    	cmd_ux = PWM_Satuation(PID_CONTROLLER_Compute(&pid_vel, cmd_vx -vel_filt), 65535, -65535);
//			MDXX_set_range(&motor, 2000, cmd_ux);
	    }
	    else
	    {
	    	cmd_ux = 0;
	    }

	    MDXX_set_range(&motor, 2000,cmd_ux);

//	    t+=1;

	  }
}

float update_filter(float input) {
    // Low-pass filter formula
    filtered_value = ALPHA * input + (1.0 - ALPHA) * filtered_value;
    return filtered_value;
}
/* USER CODE END Application */

