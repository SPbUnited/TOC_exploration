import csv

import matplotlib.pyplot as plt
import numpy as np

with open('bang.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    time = []
    bang_x = []
    bang_y = []
    for line in csvreader:
        bang_x.append(float(line[0]))
        bang_y.append(float(line[1]))

    my_dpi = 200
    plt.figure(figsize=(1500/my_dpi, 1200/my_dpi), dpi=my_dpi)

    plt.plot(bang_x, bang_y, 'C1', linewidth=3, label="Траектория")

    plt.arrow(0, 0, 0, -0.5, width=0.005, head_width=0.025, head_length=0.07, ec='r', color='r')
    plt.arrow(0.8, -1, 0, -0.5, width=0.005, head_width=0.025, head_length=0.07, ec='b', color='b')

    plt.xlim((-0.1, 0.9))
    plt.ylim((-1.6, 0.1))

    plt.xlabel('x')
    plt.ylabel('y')

    plt.grid(color=(0,0,0), alpha=0.3, linestyle='-', linewidth=1)
    plt.legend(loc='lower left')

    plt.show()