#!/usr/bin/env python3
from time import *

up = 5
down = 3
snail = 0
distance = 10000
day = 0

start_time = time()
# while True:
#     day += 1
#     snail += up
#     if snail > distance:
#         break
#     else:
#         snail -= down
#     print(f"day:{day}")
day = (distance - down) // (up - down) + 1
end_time = time()
print(day)
print(f"time:{end_time - start_time}")
