#include "SERVOdriver.hpp"

int SERVOdriver::_improved_map(int value, int minIn, int maxIn, int minOut, int maxOut) {
    const int rangeIn = maxIn - minIn;
    const int rangeOut = maxOut - minOut;
    const int deltaIn = value - minIn;

    // fixed point math constants to improve accuracy of divide and rounding
    constexpr int fixedHalfDecimal = 1;
    constexpr int fixedDecimal = fixedHalfDecimal * 2;

    return ((deltaIn * rangeOut * fixedDecimal) / (rangeIn) + fixedHalfDecimal) / fixedDecimal + minOut;
}

bool SERVOdriver::pwmservo_init(uint pin) {
    // get slice number
    uint slicenum = pwm_gpio_to_slice_num(pin);

    // assign GPIO to pwm functionality
    gpio_set_function(pin, GPIO_FUNC_PWM);

    // set the clkdiv mode (this might not actually do anything)
    pwm_set_clkdiv_mode(slicenum, PWM_DIV_FREE_RUNNING);

    // disable phase correct (if enabled, frequency is halved and duty cycle is doubled)
    pwm_set_phase_correct(slicenum, false);

    // set clkdiv to system clock in Mhz (125Mhz clock = clkdiv of 125)
    // makes it so that our on/off threshold on each pwm channel is equal to the on time per cycle in microseconds
    pwm_set_clkdiv_int_frac(slicenum, clock_get_hz(clk_sys)/1000000, 0);

    // set wrap to 20000 to get a duty cycle of 50hz for standard servos
    pwm_set_wrap(slicenum, 20000);

    // enable PWM slice
    pwm_set_enabled(slicenum, true);
    pwm_set_gpio_level(pin, PWMSERVO_NEUTRAL);

    return true;
}