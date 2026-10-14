// RingBuffer — buffer circular de muestras IMU (docs/11 §7), C++ puro.
#pragma once

#include <cstddef>

namespace mcg {

template <typename T, size_t N>
class RingBuffer {
public:
    void push(const T& item) {
        m_buffer[m_head] = item;
        m_head = (m_head + 1) % N;
        if (m_count < N) ++m_count;
    }

    size_t size() const { return m_count; }
    size_t capacity() const { return N; }
    bool empty() const { return m_count == 0; }
    void clear() { m_head = 0; m_count = 0; }

    // Índice 0 = muestra más antigua.
    const T& operator[](size_t i) const {
        return m_buffer[(m_head + N - m_count + i) % N];
    }

    // Copia las muestras (más antigua → más nueva) en dst; devuelve cuántas copió.
    size_t copyTo(T* dst, size_t max) const {
        const size_t n = (m_count < max) ? m_count : max;
        for (size_t i = 0; i < n; ++i) {
            dst[i] = (*this)[i];
        }
        return n;
    }

private:
    T m_buffer[N] {};
    size_t m_head = 0;
    size_t m_count = 0;
};

} // namespace mcg
