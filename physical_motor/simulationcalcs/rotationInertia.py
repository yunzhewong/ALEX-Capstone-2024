import math

radius = 0.085 / 2
length = 0.07
density = 7850

volume = math.pi * radius * radius * length
mass = volume * density

izz = 0.5 * mass * radius * radius

print(izz)