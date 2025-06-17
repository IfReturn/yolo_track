#pragma once
#include <string>

class SysfsControl {
public:
    // GPIO
    static bool export_gpio(int gpio_num);
    static bool unexport_gpio(int gpio_num);
    static bool set_gpio_direction(int gpio_num, const std::string& direction); // "in" or "out"
    static bool write_gpio(int gpio_num, int value); // 0 or 1
    static int read_gpio(int gpio_num);

    // PWM
    static bool export_pwm(int chip, int channel);
    static bool unexport_pwm(int chip, int channel);
    static bool set_pwm_period(int chip, int channel, int period_ns);
    static bool set_pwm_duty_cycle(int chip, int channel, int duty_ns);
    static bool enable_pwm(int chip, int channel, bool enable);

private:
    static bool write_sysfs(const std::string& path, const std::string& value);
    static std::string read_sysfs(const std::string& path);
};
