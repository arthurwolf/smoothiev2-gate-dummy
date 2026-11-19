#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

// Model TM1638 serial display RAM and the button-scan response.
class V2TM1638 {
public:
    bool configure(const std::string &clock, const std::string &data, const std::string &strobe);
    bool set_buttons(unsigned value);
    bool read_register(unsigned address, unsigned &value) const;
private:
    void pin_changed(int pin, bool level);
    bool read_input(int pin, bool &value);
    int clock_ = -1, data_ = -1, strobe_ = -1;
    std::string data_name_, strobe_name_;
    std::vector<bool> bits_;
    std::array<uint8_t, 16> ram_{};
    unsigned buttons_ = 0, read_bit_ = 0;
    bool configured_ = false, read_mode_ = false, fixed_address_ = false;
};
