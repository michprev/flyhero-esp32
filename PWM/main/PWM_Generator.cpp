/*
 * PWM_Generator.cpp
 *
 *  Created on: 11. 12. 2016
 *      Author: michp
 */

#include "PWM_Generator.h"


namespace flyhero
{

PWM_Generator &PWM_Generator::Instance()
{
    static PWM_Generator instance;

    return instance;
}

PWM_Generator::PWM_Generator()
{
    gpio_config_t conf;
    conf.intr_type = GPIO_INTR_DISABLE;
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.pin_bit_mask = GPIO_SEL_14 | GPIO_SEL_25 | GPIO_SEL_26 | GPIO_SEL_27;

    gpio_config(&conf);

    gpio_set_level(GPIO_NUM_25, 0);
    gpio_set_level(GPIO_NUM_26, 0);
    gpio_set_level(GPIO_NUM_27, 0);
    gpio_set_level(GPIO_NUM_14, 0);
}

void PWM_Generator::Init()
{
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_NUM_25));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_NUM_26));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_NUM_27));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_NUM_14));

    mcpwm_config_t pwm_config;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    pwm_config.frequency = 1000;

    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config));
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config));

    ESP_ERROR_CHECK(mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0));
    ESP_ERROR_CHECK(mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1));
}

void PWM_Generator::Set_Pulse(Motor_Type motor, uint16_t us)
{
    if (us > 1000)
        return;
    
    if (motor & MOTOR_FL)
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, us));
    if (motor & MOTOR_BL)
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, us));
    if (motor & MOTOR_FR)
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, us));
    if (motor & MOTOR_BR)
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, us));
}

void PWM_Generator::Arm()
{
    // we set maximum pulse here
    this->Set_Pulse(MOTORS_ALL, 1000);

    vTaskDelay(1000 / portTICK_RATE_MS);

    // we set minimum pulse here
    this->Set_Pulse(MOTORS_ALL, 0);
}

}
