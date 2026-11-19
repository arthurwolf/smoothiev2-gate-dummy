#pragma once
#include <array>
#include <cstdint>
#include <string>

class V2ST7920 {
public:
    bool configure(const std::string &clock, const std::string &data);
    bool pixel(unsigned x, unsigned y, bool &value) const;
private:
    void pin_changed(int pin, bool level);
    void receive(uint8_t byte);
    int clock_ = -1;
    std::string data_;
    unsigned bits_ = 0, byte_ = 0;
    bool data_mode_ = false, half_ = false, have_y_ = false;
    unsigned high_ = 0, row_ = 0, column_ = 0;
    std::array<std::array<uint8_t, 32>, 32> ram_{};
};
