// Debounced active-low push button with long-press detection.
#pragma once
#include <stdint.h>

class Button {
public:
    explicit Button(uint8_t pin) : pin_(pin) {}
    void begin();
    void update(uint32_t now_ms);

    bool held() const { return down_; }
    uint32_t heldFor(uint32_t now_ms) const { return down_ ? now_ms - since_ : 0; }
    // True once per press, when the hold time first reaches ms.
    bool longPress(uint32_t now_ms, uint32_t ms);

private:
    uint8_t pin_;
    bool down_ = false, raw_ = false, fired_ = false;
    uint32_t since_ = 0, raw_since_ = 0;
};
