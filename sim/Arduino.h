// Host stub of the bits of Arduino.h that topology.cpp touches.
#pragma once
#include <stdint.h>
#include <stddef.h>

extern uint32_t g_now_ms;
inline uint32_t millis() { return g_now_ms; }

// The firmware hashes the factory serial into a tile ID; the sim writes this.
struct SigRow { uint8_t SERNUM0, s1, s2, s3, s4, s5, s6, s7, s8, s9; };
extern SigRow SIGROW;
