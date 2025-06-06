/**
 * @file sysfsControl.cpp
 * @brief Implementation of SysfsControl class for managing system files.
* This class provides methods to read and write to files under /sys  for controlling hardware components.
*/

#include <fstream>
#include <sstream>
#include <unistd.h>

// GPIO sysfs base: /sys/class/gpio
#define GPIO_PATH "/sys/class/gpio"
// PWM sysfs base: /sys/class/pwm/pwmchip{chip}/pwm{channel}
#define PWM_CHIP_PATH(chip) "/sys/class/pwm/pwmchip" + std::to_string(chip)
#define PWM_PATH(chip, channel) PWM_CHIP_PATH(chip) + "/pwm" + std::to_string(channel)

class SysfsControl {
public:
    bool export_gpio(int gpio_num);
    bool unexport_gpio(int gpio_num);
    bool set_gpio_direction(int gpio_num, const std::string& direction);
    bool write_gpio(int gpio_num, int value);
    int read_gpio(int gpio_num);
    bool export_pwm(int chip, int channel);
    bool unexport_pwm(int chip, int channel);
    bool set_pwm_period(int chip, int channel, int period_ns);
    bool set_pwm_duty_cycle(int chip, int channel, int duty_ns);
    bool enable_pwm(int chip, int channel, bool enable);

private:
    bool write_sysfs(const std::string& path, const std::string& value);
    std::string read_sysfs(const std::string& path);
};

bool SysfsControl::export_gpio(int gpio_num) {
    return write_sysfs(std::string(GPIO_PATH) + "/export", std::to_string(gpio_num));
}

bool SysfsControl::unexport_gpio(int gpio_num) {
    return write_sysfs(std::string(GPIO_PATH) + "/unexport", std::to_string(gpio_num));
}

bool SysfsControl::set_gpio_direction(int gpio_num, const std::string& direction) {
    return write_sysfs(std::string(GPIO_PATH) + "/gpio" + std::to_string(gpio_num) + "/direction", direction);
}

bool SysfsControl::write_gpio(int gpio_num, int value) {
    return write_sysfs(std::string(GPIO_PATH) + "/gpio" + std::to_string(gpio_num) + "/value", std::to_string(value));
}

int SysfsControl::read_gpio(int gpio_num) {
    std::string val = read_sysfs(std::string(GPIO_PATH) + "/gpio" + std::to_string(gpio_num) + "/value");
    return std::stoi(val);
}

bool SysfsControl::export_pwm(int chip, int channel) {
    return write_sysfs(PWM_CHIP_PATH(chip) + "/export", std::to_string(channel));
}

bool SysfsControl::unexport_pwm(int chip, int channel) {
    return write_sysfs(PWM_CHIP_PATH(chip) + "/unexport", std::to_string(channel));
}

bool SysfsControl::set_pwm_period(int chip, int channel, int period_ns) {
    return write_sysfs(PWM_PATH(chip, channel) + "/period", std::to_string(period_ns));
}

bool SysfsControl::set_pwm_duty_cycle(int chip, int channel, int duty_ns) {
    return write_sysfs(PWM_PATH(chip, channel) + "/duty_cycle", std::to_string(duty_ns));
}

bool SysfsControl::enable_pwm(int chip, int channel, bool enable) {
    return write_sysfs(PWM_PATH(chip, channel) + "/enable", enable ? "1" : "0");
}

bool SysfsControl::write_sysfs(const std::string& path, const std::string& value) {
    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;
    ofs << value;
    ofs.close();
    usleep(1000); // small delay for sysfs
    return true;
}

std::string SysfsControl::read_sysfs(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return "-1";
    std::stringstream ss;
    ss << ifs.rdbuf();
    return ss.str();
}
