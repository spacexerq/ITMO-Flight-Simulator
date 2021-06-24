import matplotlib.pyplot as plt
import numpy as np
import math


def data_plots(x, y, vy, vx, t, a):
    t_graph.append(t)
    x_graph.append(x)
    y_graph.append(y)
    v_t_graph.append(np.sqrt(vx * vx + vy * vy))
    a_t_graph.append(a)
    return t_graph, x_graph, y_graph, v_t_graph, a_t_graph


def accel(x, y, vy, vx, time, v_end, m_sum, m_fuel, aoa):
    v_start = np.sqrt(vy ** 2 + vx ** 2)
    acc = np.sqrt(
        (acceleration_max * math.sin(math.radians(aoa)) - g_moon) ** 2 + acceleration_max * math.cos(math.radians(aoa)))
    while (vy * vy + vx * vx) ** 0.5 < v_end:
        dt = 0.01
        time += dt
        x = x + vx * dt + acceleration_max * math.cos(math.radians(aoa)) * dt * dt / 2
        y = y + vy * dt + (acceleration_max * math.sin(math.radians(aoa)) - g_moon) * dt * dt / 2
        vy = vy + (acceleration_max * math.sin(math.radians(aoa)) - g_moon) * dt
        vx = vx + acceleration_max * math.cos(math.radians(aoa)) * dt
        data_plots(x, y, vy, vx, time, acc)
    delta_v = abs(v_start - (np.sqrt(vy ** 2 + vx ** 2)))
    # (vy*vy+vx*vx)**0.5 - v_end = gas_vel_out * ln(m1/mo)
    m1 = (m_sum + m_fuel) / np.exp(delta_v / gas_vel_out)
    m_fuel = m1 - m_sum
    # print("LAST DATA: T+", time, " POS:", x, y, " VEL:", abs((vy * vy + vx * vx) ** 0.5))
    return x, y, vy, vx, time, m_fuel, acc


def vel_x_to_zero(x, y, vy, vx, time, m_sum, m_fuel):
    acceleration = (abs((vy * vy + vx * vx)) - abs(vy * vy)) / (2 * abs(x - space))
    acc = np.sqrt(acceleration ** 2 + g_moon ** 2)
    if np.sqrt(acceleration ** 2 + g_moon ** 2) > acceleration_max:
        return x, y, vy, vx, time, -1000, acc
    v_start = np.sqrt(vy ** 2 + vx ** 2)
    dt = 0.001
    while abs(vx) > 0.3:
        time += dt
        x = x + vx * dt - acceleration * dt * dt / 2
        y = y + vy * dt - g_moon * dt * dt / 2
        vy = vy - g_moon * dt
        vx = vx - acceleration * dt
        data_plots(x, y, vy, vx, time, acc)
    # (vy*vy+vx*vx)**0.5 - v_end = gas_vel_out * ln(m1/mo)
    delta_v = abs(v_start - (np.sqrt(vy ** 2 + vx ** 2)))
    m1 = (m_sum + m_fuel) / np.exp(delta_v / gas_vel_out)
    m_fuel = m1 - m_sum
    return x, y, vy, vx, time, m_fuel, acc


def landing(x, y, vy, vx, time, m_sum, m_fuel):
    v_start = vy
    acceleration = (abs((vy * vy + vx * vx)) - abs(vx * vx)) / (2 * abs(y - height_end)) + g_moon
    acc = acceleration - g_moon
    if abs(acceleration - g_moon) > acceleration_max:
        return x, y, vy, vx, time, -1000, acc
    while abs(vy) > 0.5:
        dt = 0.001
        time += dt
        x = x + vx * dt
        y = y + vy * dt + (acceleration - g_moon) * dt * dt / 2
        vy = vy + (acceleration - g_moon) * dt
        if y <= height_end:
            break
        data_plots(X, Y, velocity_up, velocity_hor, time, acc)
    delta_v = abs(v_start - (np.sqrt(vy ** 2 + vx ** 2)))
    m1 = (m_sum + m_fuel) / np.exp(delta_v / gas_vel_out)
    m_fuel = m1 - m_sum
    return x, y, vy, vx, time, m_fuel, acc


space = 250 * 1000
height_start = -5
height_end = 5

velocity_up = 0
velocity_hor = 0
aoa = 45
mass_constant = 2150
mass_fuel = 1000
acceleration_max = 29.43
gas_vel_out = 3660
g_moon = 1.62
time = 0
flight = 0
X = 0
Y = height_start
land = 0
t_graph = []
x_graph = []
y_graph = []
v_t_graph = []
a_t_graph = []

# я устанавливаю постоянным ускорение, которое должно действовать на корабль для совершения самых быстрых маневров
# это ускорение берется равным максимальному (acceleration_max)
# max_flight_lenght = v_start^2 * sin(2*alpha) / g_moon -> наиболее оптимальный угол для полета - alpha = 45
# m(t) * (acceleration_max - g_moon) = gas_vel_out * thrust
# предполагается использование одного стартового прожига, одного тормозного, выравнивающего и посадочного
# delta_v=gas_vel_out*ln(m1/m0) - характеристическая скорость маневра

print("LAUNCH INITIALIZATION")
print("START DATA: T+", time, " POS:", X, Y, " VEL:", abs((velocity_hor * velocity_up) ** 0.5), " FUEL:", mass_fuel)

for vel_end in range(550, 650):
    if land == 1:
        break
    for time_pause in range(490, 510):
        acc = 0
        t_graph = []
        x_graph = []
        y_graph = []
        v_t_graph = []
        a_t_graph = []
        Y = height_start
        velocity_up = 0
        velocity_hor = 0
        aoa = 45
        mass_constant = 2150
        mass_fuel = 1000
        X = 0
        time = 0
        mass_bef = mass_fuel
        time_bef = time
        dt = 0.1
        stages = [["V_x:", 0, "V_y:", 0, "X:", 0, "Y:", 0, "AoA:", 0, "dM/dT:", 0, "Burn time:", 0],
                  ["V_x:", 0, "V_y:", 0, "X:", 0, "Y:", 0, "AoA:", 0, "dM/dT:", 0, "Burn time:", 0],
                  ["V_x:", 0, "V_y:", 0, "X:", 0, "Y:", 0, "AoA:", 0, "dM/dT:", 0, "Burn time:", 0],
                  ["V_x:", 0, "V_y:", 0, "X:", 0, "Y:", 0, "AoA:", 0, "dM/dT:", 0, "Burn time:", 0]]
        stages[0][1] = velocity_hor
        stages[0][3] = velocity_up
        stages[0][5] = X
        stages[0][7] = Y
        stages[0][9] = aoa
        X, Y, velocity_up, velocity_hor, time, mass_fuel, acc = accel(X, Y, velocity_up, velocity_hor, time,
                                                                      vel_end, mass_constant, mass_fuel,
                                                                      aoa)
        if mass_fuel <= 0 or Y < height_end:
            continue
        delta_m = abs(mass_fuel - mass_bef) / abs(time - time_bef)
        stages[1][1] = velocity_hor
        stages[1][3] = velocity_up
        stages[1][5] = X
        stages[1][7] = Y
        stages[1][9] = 'NaN'
        stages[0][11] = delta_m
        stages[0][13] = abs(time - time_bef)
        while time < time_pause:
            time += dt
            X = X + velocity_hor * dt
            Y = Y + velocity_up * dt - g_moon * dt * dt / 2
            velocity_up = velocity_up - g_moon * dt
            data_plots(X, Y, velocity_up, velocity_hor, time, g_moon)
        aoa = 90
        stages[2][1] = velocity_hor
        stages[2][3] = velocity_up
        stages[2][5] = X
        stages[2][7] = Y
        stages[2][9] = aoa
        stages[1][11] = 0
        stages[1][13] = time_pause
        mass_bef = mass_fuel
        time_bef = time
        X, Y, velocity_up, velocity_hor, time, mass_fuel, acc = vel_x_to_zero(X, Y, velocity_up, velocity_hor,
                                                                              time,
                                                                              mass_constant, mass_fuel)
        if mass_fuel <= 0 or Y < height_end:
            continue
        delta_m = abs(mass_fuel - mass_bef) / abs(time - time_bef)
        stages[3][1] = velocity_hor
        stages[3][3] = velocity_up
        stages[3][5] = X
        stages[3][7] = Y
        stages[3][9] = aoa
        stages[2][11] = delta_m
        stages[2][13] = abs(time - time_bef)
        mass_bef = mass_fuel
        time_bef = time
        X, Y, velocity_up, velocity_hor, time, mass_fuel, acc = landing(X, Y, velocity_up, velocity_hor,
                                                                        time,
                                                                        mass_constant, mass_fuel)
        if mass_fuel <= 0 or Y < height_end:
            continue
        delta_m = abs(mass_fuel - mass_bef) / abs(time - time_bef)
        aoa = 0
        stages[3][11] = delta_m
        stages[3][13] = abs(time - time_bef)
        if abs(X - space) < 5 and abs(Y - height_end) <= 0.1 and abs(velocity_up) <= 3 and abs(velocity_up) <= 1:
            land = 1
            for i in range(len(stages)):
                for j in range(len(stages[0])):
                    if j % 2 != 0 and j != 9:
                        print(round(float(stages[i][j]), 1), end=' ')
                    else:
                        print(stages[i][j], end=' ')
                print()
            print("LANDING COMPLETE")
            print("T+", round(time, 1), "X:", X, "Y:", Y, "VEL_Y:", velocity_up, "VEL_X:", velocity_hor)
            break
fig1 = plt.figure()
fig2 = plt.figure()
fig3 = plt.figure()
pos = fig1.add_subplot()
pos.set_xlabel('X')
pos.set_ylabel('Y')
pos.set_title('y=f(x)')
pos.plot(x_graph, y_graph)
v = fig2.add_subplot()
v.set_xlabel('T')
v.set_ylabel('V')
v.set_title('v=f(t)')
v.plot(t_graph, v_t_graph)
a = fig3.add_subplot()
a.set_xlabel('T')
a.set_ylabel('A')
a.set_title('a=f(t)')
a.plot(t_graph, a_t_graph)

plt.show()
