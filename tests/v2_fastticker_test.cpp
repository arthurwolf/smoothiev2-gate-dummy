#include "FastTicker.h"
#include <cassert>
#include <vector>
extern "C" int steptimer_setup(unsigned, unsigned, void *, void *);
extern "C" void sim_v2_timer_advance_to(double);
static std::vector<int> events;
static void step() { events.push_back(1); }
static void unstep() {}
int main()
{
    auto *ticker = FastTicker::getInstance();
    assert(ticker->attach(500, [] { events.push_back(2); }) >= 0);
    assert(steptimer_setup(1000, 10, (void *)step, (void *)unstep));
    assert(ticker->start());
    sim_v2_timer_advance_to(2000);
    assert((events == std::vector<int>{1, 1, 2}));
    assert(ticker->stop());
    sim_v2_timer_advance_to(4000);
    assert((events == std::vector<int>{1, 1, 2, 1, 1}));
}
