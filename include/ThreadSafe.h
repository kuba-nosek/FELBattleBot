#pragma once
#include <freertos/FreeRTOS.h>

template <typename T>
class ThreadSafe {
  private:
    T _value;
    mutable portMUX_TYPE _lock = portMUX_INITIALIZER_UNLOCKED;

  public:
    ThreadSafe() : _value(T{}) {}
    ThreadSafe(T initial_value) : _value(initial_value) {}

    ThreadSafe& operator=(const T& new_value) {
        portENTER_CRITICAL(&_lock);
        _value = new_value;
        portEXIT_CRITICAL(&_lock);
        return *this;
    }

    operator T() const {
        portENTER_CRITICAL(&_lock);
        T copy = _value;
        portEXIT_CRITICAL(&_lock);
        return copy;
    }

    void set(const T& new_value) { *this = new_value; }
    T get() const { return static_cast<T>(*this); }
};
