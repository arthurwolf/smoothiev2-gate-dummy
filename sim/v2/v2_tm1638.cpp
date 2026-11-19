#include "v2_tm1638.h"
#include "Pin.h"

bool V2TM1638::configure(const std::string &clock, const std::string &data, const std::string &strobe)
{
    clock_ = Pin::external_pin_index(clock);
    data_ = Pin::external_pin_index(data);
    strobe_ = Pin::external_pin_index(strobe);
    if (clock_ < 0 || data_ < 0 || strobe_ < 0 || clock_ == data_ || clock_ == strobe_ || data_ == strobe_) return false;
    data_name_ = data;
    strobe_name_ = strobe;
    configured_ = true;
    Pin::add_observer([this](int pin, bool level) { pin_changed(pin, level); });
    Pin::set_input_provider([this](int pin, bool &value) { return read_input(pin, value); });
    return true;
}

void V2TM1638::pin_changed(int pin, bool level)
{
    if (pin == strobe_ && !level) { bits_.clear(); read_mode_ = false; read_bit_ = 0; }
    if (pin == clock_ && level && !read_mode_) {
        bool strobe = true, data = false;
        if (Pin::external_level(strobe_name_, strobe) && !strobe && Pin::external_level(data_name_, data) && bits_.size() < 136) {
            bits_.push_back(data);
            if (bits_.size() == 8) {
                unsigned command = 0;
                for (unsigned i = 0; i < 8; ++i) command |= unsigned(bits_[i]) << i;
                if (command == 0x42) read_mode_ = true;
            }
        }
    }
    if (pin != strobe_ || !level || read_mode_ || bits_.size() < 8 || bits_.size() % 8) return;
    std::vector<unsigned> bytes;
    for (size_t offset = 0; offset < bits_.size(); offset += 8) {
        unsigned byte = 0;
        for (unsigned bit = 0; bit < 8; ++bit) byte |= unsigned(bits_[offset + bit]) << bit;
        bytes.push_back(byte);
    }
    if (bytes[0] == 0x40 || bytes[0] == 0x44) fixed_address_ = bytes[0] == 0x44;
    if ((bytes[0] & 0xf0) != 0xc0) return;
    unsigned address = bytes[0] & 0x0f;
    for (size_t i = 1; i < bytes.size(); ++i) {
        ram_[address] = bytes[i];
        if (!fixed_address_) address = (address + 1) & 0x0f;
    }
}

bool V2TM1638::read_input(int pin, bool &value)
{
    if (pin != data_ || !read_mode_ || read_bit_ >= 32) return false;
    const unsigned byte = read_bit_ / 8, bit = read_bit_++ % 8;
    value = (bit == 0 && (buttons_ & (1u << byte))) || (bit == 4 && (buttons_ & (1u << (byte + 4))));
    return true;
}

bool V2TM1638::set_buttons(unsigned value)
{
    if (!configured_ || value > 255) return false;
    buttons_ = value;
    return true;
}

bool V2TM1638::read_register(unsigned address, unsigned &value) const
{
    if (!configured_ || address >= ram_.size()) return false;
    value = ram_[address];
    return true;
}
