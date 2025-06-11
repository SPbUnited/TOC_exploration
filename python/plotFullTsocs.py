import csv

import matplotlib.pyplot as plt
import numpy as np

with open('tsocs0.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    time = []
    tsocs0_x = []
    tsocs0_y = []
    for line in csvreader:
        tsocs0_x.append(float(line[0]))
        tsocs0_y.append(float(line[1]))

with open('tsocs1.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    time = []
    tsocs1_x = []
    tsocs1_y = []
    for line in csvreader:
        tsocs1_x.append(float(line[0]))
        tsocs1_y.append(float(line[1]))

with open('tsocs2.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    time = []
    tsocs2_x = []
    tsocs2_y = []
    for line in csvreader:
        tsocs2_x.append(float(line[0]))
        tsocs2_y.append(float(line[1]))

    my_dpi = 200
    plt.figure(figsize=(1500/my_dpi, 1200/my_dpi), dpi=my_dpi)

    plt.plot(tsocs0_x, tsocs0_y, 'C2--', linewidth=3, label="Первоначальное приближение")
    plt.plot(tsocs1_x, tsocs1_y, 'C0', linewidth=1, label="Первый этап оптимизации")
    plt.plot(tsocs2_x, tsocs2_y, 'C1', linewidth=3, label="Второй этап оптимизации")

    plt.arrow(0, 0, 0, -0.5, width=0.005, head_width=0.025, head_length=0.07, ec='r', color='r')
    plt.arrow(0.8, -1, 0, -0.5, width=0.005, head_width=0.025, head_length=0.07, ec='b', color='b')

    plt.xlim((-0.1, 1.1))
    plt.ylim((-1.6, 0.1))

    plt.xlabel('x')
    plt.ylabel('y')

    plt.grid(color=(0,0,0), alpha=0.3, linestyle='-', linewidth=1)
    plt.legend(loc='lower left')

    plt.show()