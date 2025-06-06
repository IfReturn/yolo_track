#pragma once
#include <string>

class SysfsControl {
public:
    // GPIO
    bool export_gpio(int gpio_num);
    bool unexport_gpio(int gpio_num);
    bool set_gpio_direction(int gpio_num, const std::string& direction); // "in" or "out"
    bool write_gpio(int gpio_num, int value); // 0 or 1
    int read_gpio(int gpio_num);

    // PWM
    bool export_pwm(int chip, int channel);
    bool unexport_pwm(int chip, int channel);
    bool set_pwm_period(int chip, int channel, int period_ns);
    bool set_pwm_duty_cycle(int chip, int channel, int duty_ns);
    bool enable_pwm(int chip, int channel, bool enable);

private:
    bool write_sysfs(const std::string& path, const std::string& value);
    std::string read_sysfs(const std::string& path);
};
