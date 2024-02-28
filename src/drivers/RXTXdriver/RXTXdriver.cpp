#include "RXTXdriver.hpp"
#include <chrono>


void RXTXdriver::measure_duty_cycle(float measured_duty_cycle[]) {
    // Count once for every 100 cycles the PWM B input is high
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_HIGH);
    pwm_config_set_clkdiv(&cfg, 100);

    float counting_rate = clock_get_hz(clk_sys) / 100;
    float max_possible_count = counting_rate * 0.01;

    // Only the PWM B pins can be used as inputs.
    for(int i = 0; i < 6; i++) {
        uint slice_num = pwm_gpio_to_slice_num(PWM_PINS[i]);
        pwm_init(slice_num, &cfg, false);
        gpio_set_function(PWM_PINS[i], GPIO_FUNC_PWM); 
        pwm_set_enabled(slice_num, true);
    }

    sleep_ms(20);

    for(int i = 0; i < 6; i++) {
        uint slice_num = pwm_gpio_to_slice_num(PWM_PINS[i]);
        pwm_set_enabled(slice_num, false);
        measured_duty_cycle[i] = pwm_get_counter(slice_num) / max_possible_count;
    }
}

void RXTXdriver::read_data(RXTXdata &container) {
    if(container.ch5 >= 1700) {
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_HIGH);
        pwm_config_set_clkdiv(&cfg, 100);

        float counting_rate = clock_get_hz(clk_sys) / 100;
        float max_possible_count = counting_rate * 0.01;

        uint slice_num = pwm_gpio_to_slice_num(PWM_PINS[4]);
        pwm_init(slice_num, &cfg, false);
        gpio_set_function(PWM_PINS[4], GPIO_FUNC_PWM); 
        pwm_set_enabled(slice_num, true);

        sleep_ms(20);

        pwm_set_enabled(slice_num, false);
        float rtValue = pwm_get_counter(slice_num) / max_possible_count;
        float pin_value = rtValue * 10000;
        container.ch5 = pin_value;
    } else {
        float measured_duty_cycle[6];
        measure_duty_cycle(measured_duty_cycle);

        for (int i = 0; i < 6; ++i) {
            float pin_value = measured_duty_cycle[i] * 10000;

            if (pin_value >= 1000 && pin_value <= 2000) {
                switch (i) {
                    case 0:
                        container.ch1 = pin_value;
                        break;
                    case 1:
                        container.ch2 = pin_value;
                        break;
                    case 2:
                        container.ch3 = pin_value;
                        break;
                    case 3:
                        container.ch4 = pin_value;
                        break;
                    case 4:
                        container.ch5 = pin_value;
                        break;
                    case 5:
                        container.ch6 = pin_value;
                        break;
                }
            }
        }
    }
}