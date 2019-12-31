import numpy as np
import matplotlib.pyplot as plt

ego_states = np.loadtxt("ego_states.txt")
front_states = np.loadtxt("front_states.txt")

ego_time, ego_x, ego_y, ego_vx, ego_vy = ego_states.T
front_time, front_x, front_y, front_vx, front_vy = front_states.T

plt.plot(ego_time, ego_x)
plt.plot(front_time, front_x)
plt.show()
