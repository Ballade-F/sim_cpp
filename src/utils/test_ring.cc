#include "ring_buffer.hpp"
#include <iostream>

int main() {
    RingBuffer<int> ring(5);
    for (int i = 0; i < 10; i++) {
        ring.push(i);
        for (int j = 0; j < ring.size(); j++) {
            std::cout << ring[j] << " ";
        }
        std::cout << std::endl;
    }
    return 0;
}