#include "button.h"

#include <Arduino.h>

void Button::begin() { pinMode(pin_, INPUT_PULLUP); }

void Button::update(uint32_t now) {
    bool r = digitalRead(pin_) == LOW;
    if (r != raw_) {
        raw_ = r;
        raw_since_ = now;
    }
    if (raw_ != down_ && now - raw_since_ >= 30) {
        down_ = raw_;
        since_ = now;
        fired_ = false;
    }
}

bool Button::longPress(uint32_t now, uint32_t ms) {
    if (down_ && !fired_ && now - since_ >= ms) {
        fired_ = true;
        return true;
    }
    return false;
}
