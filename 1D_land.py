import matplotlib.pyplot as plt
import numpy as np
import math


# вариант 3


def landing(y, vy, time, m_sum, m_fuel):
    v_start = vy
    acceleration = (abs((vy * vy)) / (2 * abs(y - height_end)) + g_moon)
    acc = acceleration - g_moon
    if abs(acceleration - g_moon) > acceleration_max:
        return y, vy, time, -1000, acc
    while abs(vy) > 0.5:
        dt = 0.001
        time += dt
        y = y + vy * dt + (acceleration - g_moon) * dt * dt / 2
        vy = vy + (acceleration - g_moon) * dt
        if y <= height_end:
            break
    delta_v = abs(v_start - (np.sqrt(vy ** 2)))
    m1 = (m_sum + m_fuel) / np.exp(delta_v / gas_vel_out)
    m_fuel = m1 - m_sum
    return y, vy, time, m_fuel, acc


height_start = 410
v_start = -73
g_moon = 1.62
mass_constant = 2150
mass_fuel = 200
mass_fuel_st = mass_fuel
acceleration_max = 29.43
gas_vel_out = 3660
height_end = 0
time = 0
aoa = 0

y, vy, time, mass_fuel, acc = landing(height_start, v_start, time, mass_constant, mass_fuel)
dm_dt = (mass_fuel_st - mass_fuel) / time
print("AFTERBURN: H:", round(y, 1), "VEL:", round(vy, 1), "T+:", round(time, 1), "F_else:", round(mass_fuel, 1), "OVL:",
      round(acc, 1))
print("BURN:", "VEL:", round(v_start, 1), "H:", round(height_start, 1), "AoA:", aoa, "dM/dT:", round(dm_dt, 1),
      "BURN TIME:", round(time, 1))
