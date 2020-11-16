#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_uros/options.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_system.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"

#define GPIO_PWM0A_OUT  16   
#define GPIO_PWMINA_OUT 14

#define GPIO_ENCODER_A  18
#define GPIO_ENCODER_B  19

#define MOTOR_ABS(x) (x < 0) ? -x : x
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 message;
char esp32_device_topic[48];
uint8_t esp32_device_id_raw[6];

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	int motor_cmd = msg->data;

	//Staturate the motor command:
	if(motor_cmd > 10000) {
		motor_cmd= 10000;
	} else if (motor_cmd < -10000) {
		motor_cmd = -10000;
	}

	gpio_set_level(GPIO_PWMINA_OUT, motor_cmd < 0 ? 0 : 1);
	motor_cmd = MOTOR_ABS(motor_cmd);

	//Give some extra energy to impulse motor at low-speeds:
	if(motor_cmd <= 2000) {
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 100.0f);
		mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0f);
		mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 20.0f);
		mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
	} 

	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (float)motor_cmd / 100.0f);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
	
	int16_t pulses; 
	pcnt_get_counter_value(PCNT_UNIT_0, &pulses);
	message.data = (int32_t)pulses;

	RCSOFTCHECK(rcl_publish(&publisher, &message, NULL));
}

void appMain(void * arg)
{
    pcnt_config_t dev_config = {
        .pulse_gpio_num = GPIO_ENCODER_A,
        .ctrl_gpio_num = GPIO_ENCODER_B,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_0,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 16384,
        .counter_l_lim = -16384,
    };

	pcnt_unit_config(&dev_config);

	dev_config.pulse_gpio_num = GPIO_ENCODER_B;
    dev_config.ctrl_gpio_num = GPIO_ENCODER_A;
    dev_config.channel = PCNT_CHANNEL_1;
    dev_config.pos_mode = PCNT_COUNT_INC;
    dev_config.neg_mode = PCNT_COUNT_DEC;

	pcnt_unit_config(&dev_config);
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
	pcnt_counter_resume(PCNT_UNIT_0);

	gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1 << GPIO_PWMINA_OUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	gpio_set_level(GPIO_PWMINA_OUT, 1);

	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
	mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings


	esp_efuse_mac_get_default(esp32_device_id_raw);

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	//Create device node name:
	sprintf(esp32_device_topic, "esp32_0x%X%X%X%X%X%X", esp32_device_id_raw[0],
												esp32_device_id_raw[1],
												esp32_device_id_raw[2],
												esp32_device_id_raw[3],
												esp32_device_id_raw[4],
												esp32_device_id_raw[5]);


	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, esp32_device_topic, "", &support));

	//Create device pub topic name:
	sprintf(esp32_device_topic, "/esp32_%X%X%X%X%X%X", esp32_device_id_raw[0],
												esp32_device_id_raw[1],
												esp32_device_id_raw[2],
												esp32_device_id_raw[3],
												esp32_device_id_raw[4],
												esp32_device_id_raw[5]);
	strcat(esp32_device_topic, "/motor/state");

	printf("%s \n\n\n\n",esp32_device_topic);


	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		esp32_device_topic));

	//Create sub topic name:
	sprintf(esp32_device_topic, "/esp32_%X%X%X%X%X%X", esp32_device_id_raw[0],
												esp32_device_id_raw[1],
												esp32_device_id_raw[2],
												esp32_device_id_raw[3],
												esp32_device_id_raw[4],
												esp32_device_id_raw[5]);

	strcat(esp32_device_topic, "/motor/cmd");

	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		esp32_device_topic));


	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &message, &subscription_callback, ON_NEW_DATA));

	rclc_executor_spin(&executor);
	
	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node))
	RCCHECK(rcl_node_fini(&node))

  	vTaskDelete(NULL);
}
