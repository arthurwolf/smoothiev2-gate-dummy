#include "v2_max7219.h"
#include "Pin.h"
#include <set>

bool V2Max7219::configure(const std::string &clock, const std::string &data,
                          const std::vector<std::string> &selects, unsigned cascaded)
{
    clock_ = Pin::external_pin_index(clock);
    const int data_index = Pin::external_pin_index(data);
    if (clock_ < 0 || data_index < 0 || clock_ == data_index || selects.empty()) return false;
    data_ = data;
    std::set<int> used{clock_, data_index};
    for (const auto &select : selects) {
        const int index = Pin::external_pin_index(select);
        if (index < 0 || !used.insert(index).second) return false;
        Chain chain{select, index, {}, {}};
        chain.registers.resize(cascaded ? cascaded : 1);
        chains_.push_back(std::move(chain));
    }
    Pin::add_observer([this](int pin, bool level) { pin_changed(pin, level); });
    return true;
}

void V2Max7219::pin_changed(int pin, bool level)
{
    for (auto &chain : chains_) {
        if (pin == chain.index && !level) chain.bits.clear();
        if (pin == clock_ && level) {
            bool selected = true, data = false;
            if (chain.bits.size() <= chain.registers.size() * 16 && Pin::external_level(chain.pin, selected) && !selected && Pin::external_level(data_, data))
                chain.bits.push_back(data);
        }
        if (pin != chain.index || !level) continue;
        const size_t count = chain.registers.size();
        if (chain.bits.size() != count * 16) continue;
        for (size_t i = 0; i < count; ++i) {
            unsigned word = 0;
            for (size_t bit = 0; bit < 16; ++bit) word = (word << 1) | chain.bits[i * 16 + bit];
            const unsigned reg = (word >> 8) & 0xff;
            if (reg > 0 && reg < 16) chain.registers[count - 1 - i][reg] = word & 0xff;
        }
    }
}

bool V2Max7219::read(const std::string &select, unsigned device, unsigned reg, unsigned &value) const
{
    const int index = Pin::external_pin_index(select);
    for (const auto &chain : chains_) {
        if (chain.index == index && device < chain.registers.size() && reg > 0 && reg < 16) {
            value = chain.registers[device][reg];
            return true;
        }
    }
    return false;
}
