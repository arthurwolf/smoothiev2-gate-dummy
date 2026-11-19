#pragma once
#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

// Decode the firmware's bit-banged clock, data and chip-select lines.
class V2Max7219 {
public:
    bool configure(const std::string &clock, const std::string &data,
                   const std::vector<std::string> &selects, unsigned cascaded);
    bool read(const std::string &select, unsigned device, unsigned reg, unsigned &value) const;
private:
    struct Chain {
        std::string pin;
        int index;
        std::vector<bool> bits;
        std::vector<std::array<uint8_t, 16>> registers;
    };
    void pin_changed(int pin, bool level);
    int clock_ = -1;
    std::string data_;
    std::vector<Chain> chains_;
};
