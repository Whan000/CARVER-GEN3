# AMT212E-V with Micro-ROS
First, initiate .ioc normally like how you would do without Micro-ROS.

** But do not include the code in <main.c> since it will be in <app_freertos.c> instead

- [AMT212E-V setup before Micro-ROS](../AMT212E-V_NoUros/README.md)


Next, you will need to install Micro-ROS utilities in your STM32 project.

- [Carver utils installation](https://github.com/CARVER-NEXT-GEN/micro_ros_stm32cubemx_utils)

- (backup)[General Micro-ROS utils installation](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils)

Open your project via terminal, for example; `cd AMT212E-V_Micro-ROS/firmware/<project_name>/`

```
git clone https://github.com/CARVER-NEXT-GEN/micro_ros_stm32cubemx_utils.git
```
or alternatively
```
git clone -b humble https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git
```

If you are using firmware folder to store Micro-ROS from pure ROS2 code dont forget to add COLCON_IGNORE file.
```
cd ..
touch COLCON_IGNORE
```

## Setup Micro-ROS environment in your project properties.

Assuming you already have a docker;

1. Go to `Project -> Settings -> C/C++ Build -> Settings -> Build Steps Tab` and in `Pre-build steps` add:
	```
	docker pull microros/micro_ros_static_library_builder:humble && docker run --rm -v ${workspace_loc:/${ProjName}}:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:humble
	```

2. Add micro-ROS include directory. In `Project -> Settings -> C/C++ Build -> Settings -> Tool Settings Tab -> MCU GCC Compiler -> Include paths` add
	```
	../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include
	```

3. Add the micro-ROS precompiled library. In `Project -> Settings -> C/C++ Build -> Settings -> MCU GCC Linker -> Libraries`
	- -L
	```
	../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
	```
	- -l
	```
	microros
	```

	Rebuild the index if asked to do.

4. Add the following source code files to your project, copy them to source folder:

	- [extra_sources/microros_time.c](https://github.com/CARVER-NEXT-GEN/micro_ros_stm32cubemx_utils/blob/humble/extra_sources/microros_time.c)
	- [extra_sources/microros_allocators.c](https://github.com/CARVER-NEXT-GEN/micro_ros_stm32cubemx_utils/blob/humble/extra_sources/microros_allocators.c)
	- [extra_sources/custom_memory_manager.c](https://github.com/CARVER-NEXT-GEN/micro_ros_stm32cubemx_utils/blob/humble/extra_sources/custom_memory_manager.c)
	- [extra_sources/microros_transports/dma_transport.c](https://github.com/CARVER-NEXT-GEN/micro_ros_stm32cubemx_utils/blob/humble/extra_sources/microros_transports/dma_transport.c) or your transport selection.

## setting up .ioc

### LPUART1

- Set baud rate to 2000000
- Enable Asynchronus mode for LPUART1
- Add Rx and Tx in DMA Settings both with `very high` priority
- Select Circular mode for Rx
- Enable global interrupt in NVIC setting

![IOC_13.png](/AMT212E-V_Micro-ROS/Assets/IOC_13.png)

![IOC_11.png](/AMT212E-V_Micro-ROS/Assets/IOC_11.png)
![IOC_12.png](/AMT212E-V_Micro-ROS/Assets/IOC_12.png)

### FreeRTOS
- In the tab `Tasks and Queues`, select CMSIS_V2 option for interface
- change stack size to 3000
- Set Allocation to static

** if you intend to write in main change Code Generation option to `as weak` but normally if you write in app_freertos this is nor required.

![IOC_8.png](/AMT212E-V_Micro-ROS/Assets/IOC_8.png)

### Watchdog

- Activate IWDG
- Set down-counter reload value to 2499

![IOC_9.png](/AMT212E-V_Micro-ROS/Assets/IOC_9.png)

### Optional

Select `Generate peripheral initialization as a pair of '.c/.h' files per peripherlal` to make main.c more tidy.

![IOC_10.png](/AMT212E-V_Micro-ROS/Assets/IOC_10.png)

### Grant permission for docker

To grant non-root users access to the Docker daemon, run the following command in your terminal:

```
sudo chmod 666 /var/run/docker.sock
```

### Build your project 

![build.png](/AMT212E-V_Micro-ROS/Assets/build.png)

The build process may take some time, Please wait patiently until it completes.

### App_freertos.c

Note that some of the following code requires a custom message that's from:
[Carver's micro_ros_stm32cubemx_utils](https://github.com/CARVER-NEXT-GEN/micro_ros_stm32cubemx_utils)

#includes has to be done manually in this file.

> Add the following code in the `USER CODE BEGIN Includes` section

```c
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

#include <amt212ev_interfaces/msg/amt_read.h>
#include <std_msgs/msg/float32.h>
```

> Define Alpha for <amt.radps> low pass filter usage in the section `USER CODE BEGIN PD`
```c
#define ALPHA 0.1
```
> Define check function in `USER CODE BEGIN PM`
```c
#define RCSOFTCHECK(fn) if (fn != RCL_RET_OK) {};
```
> Generate variables for Micro-ROS in `USER CODE BEGIN Variables`
```c
AMT212EV amt;
MDXX motor;
PID_CONTROLLER pid_pos;
PID_CONTROLLER pid_vel;

float kp_pos = 4.0;
float ki_pos = 0.004;
float kd_pos= 0.0;
float u_max_pos = 65535.0;

float kp_vel = 11500.0;
float ki_vel = 0.02;
float kd_vel= 0.0;
float u_max_vel = 65535.0;

float cmd_vx;
float cmd_ux;

float error_pose = 0.0;
float filtered_value = 0.0;
float steering_angle = 0.0;

int16_t sync_counter = 1000;

rcl_node_t node;

rcl_publisher_t amt_publisher;
amt212ev_interfaces__msg__AmtRead amt_msg_pub;

rcl_subscription_t amt_subscription;
amt212ev_interfaces__msg__AmtRead amt_msg_sub;
```
> Paste the following code into `USER CODE BEGIN FunctionPrototypes`
```c
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

float update_filter(float input);
```
> In `USER CODE BEGIN Init` include this code and remove both of them from <main.c> if it's present there.
```c
AMT212EV_Init(&amt, &huart1, 1000, 16384);
PID_CONTROLLER_Init(&pid_pos, kp_pos, ki_pos, kd_pos, u_max_pos);
MDXX_init(&motor, &htim8, TIM_CHANNEL_3, &htim8, TIM_CHANNEL_1);
MDXX_set_range(&motor, 1000, 0);
HAL_TIM_Base_Start_IT(&htim2);
```
> Add Micro-ROS structure in `USER CODE BEGIN StartDefaultTask`
```c
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
  int executor_num = 2;

  const rosidl_message_type_support_t * amt_pub_type_support =
  	  ROSIDL_GET_MSG_TYPE_SUPPORT(amt212ev_interfaces, msg, AmtRead);

  const rosidl_message_type_support_t * amt_sub_type_support =
    	  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);

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
  // create service server

  // create service client

  // create executor
  rclc_executor_init(&executor, &support.context, executor_num, &allocator);

  rclc_executor_add_timer(&executor, &AMT_timer);
  rclc_executor_add_subscription(&executor, &amt_subscription, &amt_msg_sub, &subscription_callback, ON_NEW_DATA);

  rclc_executor_spin(&executor);
  rmw_uros_sync_session(timeout_ms);
  ```

> You can also comment out delay for faster speed optimization.
```c
  for(;;)
  {
//	osDelay(10);
  }
```

> Finally, add this code to `USER CODE BEGIN Application`
```c 
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	if (timer != NULL)
	{
		amt_publish(amt.rads,amt.radps);
		if (uwTick >= sync_counter) {  // Sync session at lower frequency
			rmw_uros_sync_session(1000);
			sync_counter += 1000;
		}
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

	    amt.radps = update_filter(amt.radps);

	    if (steering_angle >= (0.61))
	    	{
	    	steering_angle = 0.6;
	    	}
	    if (steering_angle <= (-0.61))
	    	{
	    	steering_angle = -0.6;
	    	}
	    if (amt.rads >= (0.7))
			{
			steering_angle = 0.6;
			}
		if (amt.rads <= (-0.7))
			{
			steering_angle = -0.6;
			}

	    error_pose = steering_angle - amt.rads;

	    cmd_vx = PID_CONTROLLER_Compute(&pid_pos,error_pose);
		cmd_ux = PWM_Satuation(PID_CONTROLLER_Compute(&pid_vel, cmd_vx), 65535, -65535);
		MDXX_set_range(&motor, 1000, cmd_ux * -1);
	  }
}

float update_filter(float input) {
    // Low-pass filter formula
    filtered_value = ALPHA * input + (1.0 - ALPHA) * filtered_value;
    return filtered_value;
}
```

Now the AMT212E-V should be able to communicate via ROS2 topic and PID control loop should be working properly if tuned correctly.