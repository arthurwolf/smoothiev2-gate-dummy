#include "v2_st7920.h"
#include "Pin.h"
extern "C" double sim_v2_timer_now_us();
extern "C" void sim_v2_timer_advance_to(double);

double sim_v2_st7920_advance(unsigned us)
{
    const double end = sim_v2_timer_now_us() + us;
    sim_v2_timer_advance_to(end);
    return end;
}

bool V2ST7920::configure(const std::string &clock, const std::string &data)
{
    clock_ = Pin::external_pin_index(clock);
    const int data_index = Pin::external_pin_index(data);
    if (clock_ < 0 || data_index < 0 || clock_ == data_index) return false;
    data_ = data;
    Pin::add_observer([this](int pin, bool level) { pin_changed(pin, level); });
    return true;
}

void V2ST7920::pin_changed(int pin, bool level)
{
    if (pin != clock_ || !level) return;
    bool data = false;
    if (!Pin::external_level(data_, data)) return;
    byte_ = (byte_ << 1) | data;
    if (++bits_ == 8) {
        receive(static_cast<uint8_t>(byte_));
        bits_ = byte_ = 0;
    }
}

void V2ST7920::receive(uint8_t byte)
{
    if (byte == 0xf8 || byte == 0xfa) {
        data_mode_ = byte == 0xfa;
        half_ = false;
        if (!data_mode_) have_y_ = false;
        return;
    }
    if (!half_) { high_ = byte & 0xf0; half_ = true; return; }
    const unsigned value = high_ | (byte >> 4);
    half_ = false;
    if (data_mode_) {
        ram_[row_][column_ % 32] = value;
        column_ = (column_ + 1) % 32;
    } else if (value & 0x80) {
        if (!have_y_) { row_ = value & 31; have_y_ = true; }
        else { column_ = (value & 15) * 2; have_y_ = false; }
    }
}

bool V2ST7920::pixel(unsigned x, unsigned y, bool &value) const
{
    if (clock_ < 0 || x >= 128 || y >= 64) return false;
    value = (ram_[y % 32][(y / 32) * 16 + x / 8] & (0x80U >> (x % 8))) != 0;
    return true;
}
