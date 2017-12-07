/*
 * HC_SR04.cpp
 *
 *  Created on: 19. 9. 2017
 *      Author: michp
 */

#include "HC_SR04.h"


namespace flyhero
{

HC_SR04::HC_SR04(gpio_num_t trigg, gpio_num_t echo, gpio_isr_t isr_handler) :
        trigg_pin(trigg), echo_pin(echo), echo_handler(isr_handler)
{
    this->level_high = false;
    this->distance = -1;
    this->distance_semaphore = xSemaphoreCreateBinary();
}

esp_err_t HC_SR04::trigg_init()
{
    gpio_config_t conf;
    conf.pin_bit_mask = (uint64_t) (((uint64_t) 1) << this->trigg_pin);
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;

    return gpio_config(&conf);
}

esp_err_t HC_SR04::echo_init()
{
    esp_err_t state;

    gpio_config_t conf;
    conf.pin_bit_mask = (uint64_t) (((uint64_t) 1) << this->echo_pin);
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    conf.intr_type = GPIO_INTR_ANYEDGE;

    if ((state = gpio_config(&conf)))
        return state;

    if ((state = gpio_install_isr_service(0)))
        return state;

    if ((state = gpio_isr_handler_add(this->echo_pin, this->echo_handler, (void *) this->echo_pin)))
        return state;

    return ESP_OK;
}

void HC_SR04::Init()
{
    xSemaphoreGive(this->distance_semaphore);
    ESP_ERROR_CHECK(this->trigg_init());
    ESP_ERROR_CHECK(this->echo_init());
}

void HC_SR04::Trigger()
{
    int64_t start, end;

    // we should not get errors here so no need to check
    gpio_set_level(this->trigg_pin, 1);
    start = esp_timer_get_time();

    uint32_t delta;

    do
    {
        end = esp_timer_get_time();
    } while (end - start < 10);

    gpio_set_level(this->trigg_pin, 0);
}

double HC_SR04::Get_Distance()
{
    while (xSemaphoreTake(this->distance_semaphore, 0) != pdTRUE);

    double ret = this->distance;

    xSemaphoreGive(this->distance_semaphore);

    return ret;
}

// TODO use float: https://www.esp32.com/viewtopic.php?t=1292
void HC_SR04::Echo_Callback()
{
    this->level_high = !this->level_high;

    if (this->level_high)
        gettimeofday(&this->start, NULL);
    else
    {
        timeval end;
        gettimeofday(&end, NULL);

        while (xSemaphoreTake(this->distance_semaphore, 0) != pdTRUE);

        this->distance = (end.tv_sec - this->start.tv_sec + (end.tv_usec - this->start.tv_usec) * 0.000001) * 334 * 0.5;

        xSemaphoreGive(this->distance_semaphore);
    }
}

} /* namespace flyhero */
