import csv

import matplotlib.pyplot as plt
import numpy as np

with open('path.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    time = []
    robot_x = []
    robot_y = []
    path_x = []
    path_y = []
    error = []
    for line in csvreader:
        if float(line[0]) > 22 and float(line[0]) < 32:
            time.append(float(line[0]) - 22)
            robot_x.append(float(line[1]))
            robot_y.append(float(line[2]))
            path_x.append(float(line[3]))
            path_y.append(float(line[4]))
            error.append(np.sqrt((float(line[3]) - float(line[1]))**2 + (float(line[4]) - float(line[2]))**2))

with open('tsocs_error_1.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    for t, e in zip(time, error):
        csvwriter.writerow([t, e])

my_dpi = 200
plt.figure(figsize=(1500/my_dpi, 1200/my_dpi), dpi=my_dpi)

plt.plot(time, error, 'C0', linewidth=3, label="Ошибка слежения")

plt.xlim((0, 10))
# plt.ylim((-0.6, 0.6))

plt.xlabel('t')
plt.ylabel('error')

plt.grid(color=(0,0,0), alpha=0.3, linestyle='-', linewidth=1)
plt.legend(loc='upper right')

plt.show()