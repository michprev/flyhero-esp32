//
// Created by michal on 7.3.18.
//

#include <cstdint>
#include <soc/mcpwm_struct.h>
#include <esp_err.h>
#include <driver/mcpwm.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "OneShot125.h"

namespace flyhero
{

Motors_Protocol &OneShot125::Instance()
{
    static OneShot125 instance;

    return instance;
}

void OneShot125::Init()
{
    mcpwm_pin_config_t mcpwm_pin_config;
    mcpwm_pin_config.mcpwm0a_out_num = GPIO_NUM_25;
    mcpwm_pin_config.mcpwm0b_out_num = GPIO_NUM_26;
    mcpwm_pin_config.mcpwm1a_out_num = GPIO_NUM_27;
    mcpwm_pin_config.mcpwm1b_out_num = GPIO_NUM_14;
    mcpwm_pin_config.mcpwm2a_out_num = -1;
    mcpwm_pin_config.mcpwm2b_out_num = -1;
    mcpwm_pin_config.mcpwm_sync0_in_num = -1;
    mcpwm_pin_config.mcpwm_sync1_in_num = -1;
    mcpwm_pin_config.mcpwm_sync2_in_num = -1;
    mcpwm_pin_config.mcpwm_fault0_in_num = -1;
    mcpwm_pin_config.mcpwm_fault1_in_num = -1;
    mcpwm_pin_config.mcpwm_fault2_in_num = -1;
    mcpwm_pin_config.mcpwm_cap0_in_num = -1;
    mcpwm_pin_config.mcpwm_cap1_in_num = -1;
    mcpwm_pin_config.mcpwm_cap2_in_num = -1;

    ESP_ERROR_CHECK(mcpwm_set_pin(MCPWM_UNIT_0, &mcpwm_pin_config));

    // set unit clock to 16 MHz
    MCPWM0.clk_cfg.prescale = 9;

    // set timer clock to 8 MHz
    MCPWM0.timer[0].period.prescale = 1;

    // set timer period to 2000 ticks
    MCPWM0.timer[0].period.period = 2000;

    // set timer update method to immediate
    MCPWM0.timer[0].period.upmethod = 0;

    // set timer mode to count up
    MCPWM0.timer[0].mode.mode = 1;

    // stop timer
    MCPWM0.timer[0].mode.start = 0;

    MCPWM0.timer[0].sync.timer_phase = 0;

    // set both operators reference timer 0
    MCPWM0.timer_sel.operator0_sel = 0;
    MCPWM0.timer_sel.operator1_sel = 0;

    MCPWM0.channel[0].cmpr_cfg.a_upmethod = 0;
    MCPWM0.channel[0].cmpr_cfg.b_upmethod = 0;
    MCPWM0.channel[1].cmpr_cfg.a_upmethod = 0;
    MCPWM0.channel[1].cmpr_cfg.b_upmethod = 0;


    // 0 = no action, 1 = low, 2 = high
    // utep ticks = period
    // utez ticks = 0
    // utea ticks = a_cmpr
    // uteb ticks = b_cmpr
    for (uint8_t i = 0; i < 2; i++)
        for (uint8_t j = 0; j < 2; j++)
        {
            MCPWM0.channel[i].generator[j].utep = 0;
            MCPWM0.channel[i].generator[j].utez = 2;
            if (j == 0)
                MCPWM0.channel[i].generator[j].utea = 1;
            else
                MCPWM0.channel[i].generator[j].uteb = 1;
        }

    MCPWM0.update_cfg.global_up_en = 1;
    MCPWM0.update_cfg.global_force_up = 1;
    MCPWM0.update_cfg.global_force_up = 0;
}

void OneShot125::Arm()
{
    // set duty to 250 us = 100 %
    MCPWM0.channel[0].cmpr_value[0].cmpr_val = 2000;
    MCPWM0.channel[0].cmpr_value[1].cmpr_val = 2000;
    MCPWM0.channel[1].cmpr_value[0].cmpr_val = 2000;
    MCPWM0.channel[1].cmpr_value[1].cmpr_val = 2000;

    // free run mode
    MCPWM0.timer[0].mode.start = 2;

    vTaskDelay(1000 / portTICK_RATE_MS);

    // set duty to 125 us = 0 %
    MCPWM0.channel[0].cmpr_value[0].cmpr_val = 1000;
    MCPWM0.channel[0].cmpr_value[1].cmpr_val = 1000;
    MCPWM0.channel[1].cmpr_value[0].cmpr_val = 1000;
    MCPWM0.channel[1].cmpr_value[1].cmpr_val = 1000;

    vTaskDelay(1000 / portTICK_RATE_MS);
}

void OneShot125::Disarm()
{
    // stop timer
    MCPWM0.timer[0].mode.start = 0;

    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_25, 0));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_26, 0));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_27, 0));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_14, 0));
}

void OneShot125::Update(uint16_t motor_fl, uint16_t motor_bl, uint16_t motor_fr, uint16_t motor_br)
{
    MCPWM0.timer[0].mode.start = 4;
    MCPWM0.timer[0].sync.sync_sw = !MCPWM0.timer[0].sync.sync_sw;

    if (motor_fl <= 1000)
        MCPWM0.channel[0].cmpr_value[0].cmpr_val = motor_fl + 1000;
    if (motor_bl <= 1000)
        MCPWM0.channel[0].cmpr_value[1].cmpr_val = motor_bl + 1000;
    if (motor_fr <= 1000)
        MCPWM0.channel[1].cmpr_value[0].cmpr_val = motor_fr + 1000;
    if (motor_br <= 1000)
        MCPWM0.channel[1].cmpr_value[1].cmpr_val = motor_br + 1000;
}

}
