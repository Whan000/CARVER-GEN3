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

#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <micro_ros_utilities/string_utilities.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float64_multi_array.h>

#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/bool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
typedef enum{
	MANUAL,
	TELEOP,
	AUTO,
	JOYSTICK,
	DEBOUNCE
}MODE;

typedef enum{
	FORWARD,
	BACKWARD,
}DIRECTION;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 40
#define RCSOFTCHECK(fn) if (fn != RCL_RET_OK) {};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
rcl_node_t node;

rcl_publisher_t xrl8_publisher;
std_msgs__msg__UInt16 XRL8_msg;

rcl_publisher_t emer_publisher;
std_msgs__msg__Bool emer_msg;

rcl_publisher_t direction_publisher;
std_msgs__msg__Int8 dir_msg;

rcl_publisher_t mode_publisher;
std_msgs__msg__Int8 mode_msg;

rcl_subscription_t light_signal_subscriber;
std_msgs__msg__Int8 light_signal_msg;
uint8_t auto_light_signal = 0;  // 0=none, 1=right, 2=left, 3=brake, 4=right+brake, 5=left+brake

uint8_t sync_counter = 0;

uint8_t Manual_mode = 0; // Manual
uint8_t Teleop_mode = 0; // Teleop
uint8_t Auto_mode = 0; // Auto
uint8_t Joystick_mode = 0; // Joystick
uint8_t forward = 0;
uint8_t backward = 0;
uint8_t l_switch = 0;
uint8_t r_switch = 0;
uint8_t l_break = 0;
uint8_t r_break = 0;


uint8_t cmd_mode1 = 0;
uint8_t cmd_mode2 = 0;
uint8_t cmd_mode3 = 0;
uint8_t cmd_mode4 = 0;
uint8_t cmd_forward = 0;
uint8_t cmd_backward = 0;
uint8_t cmd_front_light = 0;
uint8_t cmd_brake_light = 0;
uint8_t cmd_left_signal_light = 0;
uint8_t cmd_right_signal_light = 0;


MODE mode = MANUAL;
DIRECTION manual_direction = FORWARD;
float accelerator = 0.0;
GPIO_PinState emer = 0;

uint16_t adc_buffer[BUFFER_SIZE];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 5000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .priority = (osPriority_t) osPriorityHigh,
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
void light_signal_callback(const void * msgin);
uint16_t calculate_average(uint16_t *buffer, uint16_t length);
void xlr8_publish(uint16_t xlr8);
void emergency_publish(GPIO_PinState emer_state);
void accel_direction_publish(int8_t dir);
void mode_publish(MODE current_mode);
void interfaces_status();
void mode_cycle();
void mode_light_indicator();

void left_turn(uint16_t _t);
void right_turn(uint16_t _t);
void harzard(uint16_t _t);
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
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, BUFFER_SIZE);
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
	rcl_timer_t XLR8_timer;
	rclc_support_t support;
	rclc_executor_t executor;
	rcl_allocator_t allocator;
	rcl_init_options_t init_options;

	const unsigned int timer_period = RCL_MS_TO_NS(8);
	const int timeout_ms = 1000;
    int executor_num = 2;

	const rosidl_message_type_support_t * uint16_type_support =
	  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16);

	const rosidl_message_type_support_t * bool_type_support =
	  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);

	const rosidl_message_type_support_t * int8_type_support =
	  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8);

	allocator = rcl_get_default_allocator();

	executor = rclc_executor_get_zero_initialized_executor();

	init_options = rcl_get_zero_initialized_init_options();

	RCSOFTCHECK(rcl_init_options_init(&init_options, allocator));
	RCSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 127));

	// create support init_options
	rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

	// create timer
	rclc_timer_init_default(&XLR8_timer, &support, timer_period, timer_callback);

	// create node
	rclc_node_init_default(&node, "uros_carver_interface_node", "", &support);

	// create publisher
	rclc_publisher_init_default(&xrl8_publisher, &node, uint16_type_support, "accl_publisher");
	rclc_publisher_init_default(&emer_publisher, &node, bool_type_support, "carver_emergency");
	rclc_publisher_init_default(&direction_publisher, &node, int8_type_support, "accel_direction");
	rclc_publisher_init_default(&mode_publisher, &node, int8_type_support, "carver_mode");

	// create subscriber
	rclc_subscription_init_default(&light_signal_subscriber, &node, int8_type_support, "light_signal_command");

	// create service server

	// create service client

 	// create executor
	rclc_executor_init(&executor, &support.context, executor_num, &allocator);

	rclc_executor_add_timer(&executor, &XLR8_timer);
	rclc_executor_add_subscription(&executor, &light_signal_subscriber, &light_signal_msg, &light_signal_callback, ON_NEW_DATA);

	rclc_executor_spin(&executor);
	rmw_uros_sync_session(timeout_ms);

//	for(;;)
//	{
//	//	osDelay(10);
//	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	static MODE previous_mode = MANUAL;  // Track previous mode

	if (timer != NULL) {

		if (sync_counter++ >= 254) {
			rmw_uros_sync_session(1000);
			sync_counter = 0;
		}

		interfaces_status();
		emergency_publish(emer);

		mode_cycle();

		// Detect mode change and reset lights when switching modes
		if(previous_mode != mode){
			cmd_left_signal_light = RESET;
			cmd_right_signal_light = RESET;
			cmd_brake_light = RESET;
			auto_light_signal = 0;
			previous_mode = mode;
		}

		mode_light_indicator();
		mode_publish(mode);

		cmd_front_light = SET;  // Always turn on front light

		if(mode == MANUAL){
			accelerator = calculate_average(adc_buffer, BUFFER_SIZE);
			xlr8_publish(accelerator);

			if(manual_direction == FORWARD){
				cmd_forward = SET;
				cmd_backward = RESET;

				if(l_switch == RESET){
					left_turn(100);
				}

				if(r_switch == RESET){
					right_turn(100);
				}

				if(l_switch == SET && r_switch == SET){
					cmd_left_signal_light = RESET;
					cmd_right_signal_light = RESET;
				}

				if(l_break == RESET){
					cmd_brake_light = SET;
				}
				else{
					cmd_brake_light = RESET;
				}

				accel_direction_publish(1);
			}
			else if(manual_direction == BACKWARD){
				cmd_forward = RESET;
				cmd_backward = SET;
				cmd_brake_light = SET;

				harzard(200);

				accel_direction_publish(-1);
			}
		}
		else if(mode == AUTO){
			// Handle light signals in AUTO mode
			switch(auto_light_signal){
				case 0:  // None
					cmd_left_signal_light = RESET;
					cmd_right_signal_light = RESET;
					cmd_brake_light = RESET;
					break;

				case 1:  // Right only
					right_turn(100);
					cmd_brake_light = RESET;
					break;

				case 2:  // Left only
					left_turn(100);
					cmd_brake_light = RESET;
					break;

				case 3:  // Brake only
					cmd_left_signal_light = RESET;
					cmd_right_signal_light = RESET;
					cmd_brake_light = SET;
					break;

				case 4:  // Right + Brake
					right_turn(100);
					cmd_brake_light = SET;
					break;

				case 5:  // Left + Brake
					left_turn(100);
					cmd_brake_light = SET;
					break;

				default:  // Invalid, turn all off
					cmd_left_signal_light = RESET;
					cmd_right_signal_light = RESET;
					cmd_brake_light = RESET;
					break;
			}
		}
		else{
			// Other modes (TELEOP, JOYSTICK): turn off all signal lights
			cmd_left_signal_light = RESET;
			cmd_right_signal_light = RESET;
			cmd_brake_light = RESET;
		}

		HAL_IWDG_Refresh(&hiwdg);
	}
}

void light_signal_callback(const void * msgin)
{
	const std_msgs__msg__Int8 * msg = (const std_msgs__msg__Int8 *)msgin;
	auto_light_signal = msg->data;
}

void xlr8_publish(uint16_t xlr8)
{
	XRL8_msg.data = xlr8;
	rcl_ret_t ret = rcl_publish(&xrl8_publisher, &XRL8_msg, NULL);
	if (ret != RCL_RET_OK) printf("Error publishing (line %d)\n", __LINE__);
}

void emergency_publish(GPIO_PinState emer_state)
{
	emer_msg.data = !emer_state;
	rcl_ret_t ret = rcl_publish(&emer_publisher, &emer_msg, NULL);
	if (ret != RCL_RET_OK) printf("Error publishing (line %d)\n", __LINE__);
}

void accel_direction_publish(int8_t dir)
{
	dir_msg.data = dir;
	RCSOFTCHECK(rcl_publish(&direction_publisher, &dir_msg, NULL));
}

void mode_publish(MODE current_mode)
{
	mode_msg.data = (int8_t)current_mode;
	RCSOFTCHECK(rcl_publish(&mode_publisher, &mode_msg, NULL));
}

uint16_t calculate_average(uint16_t *buffer, uint16_t length) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i < length; i++) {
        sum += buffer[i];
    }
    return (uint16_t)(sum / length);
}

void interfaces_status(){

	emer = HAL_GPIO_ReadPin(Emergency_GPIO_Port, Emergency_Pin);

	Manual_mode = HAL_GPIO_ReadPin(Manual_GPIO_Port, Manual_Pin);
	Teleop_mode = HAL_GPIO_ReadPin(Teleop_GPIO_Port, Teleop_Pin);
	Auto_mode = HAL_GPIO_ReadPin(Auto_GPIO_Port, Auto_Pin);
	Joystick_mode = HAL_GPIO_ReadPin(Joystick_GPIO_Port, Joystick_Pin);
	forward = HAL_GPIO_ReadPin(Forward_GPIO_Port, Forward_Pin);
	backward = HAL_GPIO_ReadPin(Backward_GPIO_Port, Backward_Pin);
	l_switch = HAL_GPIO_ReadPin(L_Switch_GPIO_Port, L_Switch_Pin);
	r_switch = HAL_GPIO_ReadPin(R_Switch_GPIO_Port, R_Switch_Pin);
	l_break = HAL_GPIO_ReadPin(L_Break_GPIO_Port, L_Break_Pin);
	r_break = HAL_GPIO_ReadPin(R_Break_GPIO_Port, R_Break_Pin);


	HAL_GPIO_WritePin(Lamp_Mode1_GPIO_Port, Lamp_Mode1_Pin, cmd_mode1);
	HAL_GPIO_WritePin(Lamp_Mode2_GPIO_Port, Lamp_Mode2_Pin, cmd_mode2);
	HAL_GPIO_WritePin(Lamp_Mode3_GPIO_Port, Lamp_Mode3_Pin, cmd_mode3);
	HAL_GPIO_WritePin(Lamp_Mode4_GPIO_Port, Lamp_Mode4_Pin, cmd_mode4);

	HAL_GPIO_WritePin(Signal_F_GPIO_Port, Signal_F_Pin, cmd_front_light);
	HAL_GPIO_WritePin(Signal_B_GPIO_Port, Signal_B_Pin, cmd_brake_light);
	HAL_GPIO_WritePin(Signal_L_GPIO_Port, Signal_L_Pin, cmd_left_signal_light);
	HAL_GPIO_WritePin(Signal_R_GPIO_Port, Signal_R_Pin, cmd_right_signal_light);
}

void mode_cycle(){
	if(Manual_mode == 1){
		mode = MANUAL;
	}
	else if(Teleop_mode == 1){
		mode = TELEOP;
	}
	else if(Auto_mode == 1){
		mode = AUTO;
	}
	else if(Joystick_mode == 1){
		mode = JOYSTICK;
	}


	if(forward == 1){
		manual_direction = FORWARD;
	}
	else if(backward == 1){
		manual_direction = BACKWARD;
	}
}

void mode_light_indicator(){

	if(mode == MANUAL){
		cmd_mode1 = SET;
		cmd_mode2 = RESET;
		cmd_mode3 = RESET;
		cmd_mode4 = RESET;
	}
	else if(mode == TELEOP){
		cmd_mode2 = SET;
		cmd_mode1 = RESET;
		cmd_mode3 = RESET;
		cmd_mode4 = RESET;
	}
	else if(mode == AUTO){
		cmd_mode3 = SET;
		cmd_mode1 = RESET;
		cmd_mode2 = RESET;
		cmd_mode4 = RESET;
	}
	else if(mode == JOYSTICK){
		cmd_mode4 = SET;
		cmd_mode1 = RESET;
		cmd_mode2 = RESET;
		cmd_mode3 = RESET;
	}

	if(mode != MANUAL){
		cmd_forward = 0;
		cmd_backward = 0;
		manual_direction = FORWARD;
		accel_direction_publish(0);
	}

	HAL_GPIO_WritePin(Lamp_Mode1_GPIO_Port, Lamp_Mode1_Pin, cmd_mode1);
	HAL_GPIO_WritePin(Lamp_Mode2_GPIO_Port, Lamp_Mode2_Pin, cmd_mode2);
	HAL_GPIO_WritePin(Lamp_Mode3_GPIO_Port, Lamp_Mode3_Pin, cmd_mode3);
	HAL_GPIO_WritePin(Lamp_Mode4_GPIO_Port, Lamp_Mode4_Pin, cmd_mode4);
	HAL_GPIO_WritePin(Lamp_Forward_GPIO_Port, Lamp_Forward_Pin, cmd_forward);
	HAL_GPIO_WritePin(Lamp_Backward_GPIO_Port, Lamp_Backward_Pin, cmd_backward);


	HAL_GPIO_WritePin(Signal_F_GPIO_Port, Signal_F_Pin, cmd_front_light);
	HAL_GPIO_WritePin(Signal_B_GPIO_Port, Signal_B_Pin, cmd_brake_light);
	HAL_GPIO_WritePin(Signal_L_GPIO_Port, Signal_L_Pin, cmd_left_signal_light);
	HAL_GPIO_WritePin(Signal_R_GPIO_Port, Signal_R_Pin, cmd_right_signal_light);

}

void left_turn(uint16_t _t){
	if (uwTick % (_t*2) < _t) {
		cmd_left_signal_light = SET;
	} else {
		cmd_left_signal_light = RESET;
	}
}

void right_turn(uint16_t _t){
	if (uwTick % (_t*2) < _t) {
		cmd_right_signal_light = SET;
	} else {
		cmd_right_signal_light = RESET;
	}
}

void harzard(uint16_t _t){
	if (uwTick % (_t*2) < _t) {
	    cmd_left_signal_light = SET;
	    cmd_right_signal_light = SET;
	} else {
	    cmd_left_signal_light = RESET;
	    cmd_right_signal_light = RESET;
	}
}
/* USER CODE END Application */

