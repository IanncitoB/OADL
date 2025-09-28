from pyvrp import Model

import matplotlib.pyplot as plt

from pyvrp.stop import MaxRuntime
from pyvrp.plotting import plot_coordinates, plot_solution

COORDS = [
    (456, 320),  # location 0 - the depot
    (228, 0),    # location 1
    (912, 0),    # location 2
    (0, 80),     # location 3
    (114, 80),   # location 4
    (570, 160),  # location 5
    (798, 160),  # location 6
    (342, 240),  # location 7
    (684, 240),  # location 8
    (570, 400),  # location 9
    (912, 400),  # location 10
    (114, 480),  # location 11
    (228, 480),  # location 12
    (342, 560),  # location 13
    (684, 560),  # location 14
    (0, 640),    # location 15
    (798, 640),  # location 16
]
DEMANDS = [0, 1, 1, 2, 4, 2, 4, 8, 8, 1, 2, 1, 2, 4, 4, 8, 8]


m = Model()
m.add_vehicle_type(4, capacity=15)
depot = m.add_depot(x=COORDS[0][0], y=COORDS[0][1])
clients = [
    m.add_client(x=COORDS[idx][0], y=COORDS[idx][1], delivery=DEMANDS[idx])
    for idx in range(1, len(COORDS))
]

for frm in m.locations:
    for to in m.locations:
        distance = abs(frm.x - to.x) + abs(frm.y - to.y)  # Manhattan
        m.add_edge(frm, to, distance=distance)

# _, ax = plt.subplots(figsize=(8, 8))
# plot_coordinates(m.data(), ax=ax)
# plt.show()

res = m.solve(stop=MaxRuntime(1), display=True)  # one second
print(res)

# _, ax = plt.subplots(figsize=(8, 8))
# plot_solution(res.best, m.data(), ax=ax)
# plt.show()

# Group plots
fig, axs = plt.subplots(1, 2, figsize=(16, 8))
plot_coordinates(m.data(), ax=axs[0])
plot_solution(res.best, m.data(), ax=axs[1])
plt.show()