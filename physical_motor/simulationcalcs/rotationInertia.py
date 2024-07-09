import math

radius = 0.085 / 2
length = 0.07
density = 7850

volume = math.pi * radius * radius * length
mass = volume * density

# full twinmotor measured to be 3.3753kg
print(mass)

izz = 0.5 * mass * radius * radius

print(izz)

current = 1
torque = current * 0.1
a = torque / izz


dvdx = 0.1421
dxdt = 0.004

dv = dvdx/dxdt


print(dv)
print(a)