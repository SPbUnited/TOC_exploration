import csv

import matplotlib.pyplot as plt
import numpy as np

with open('path.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    time = []
    path_x = []
    path_y = []
    for line in csvreader:
        if float(line[0]) > 57.81 and float(line[0]) < 66.43:
            path_x.append(float(line[1]))
            path_y.append(float(line[2]))

with open('good_tsocs_2.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    time = []
    bang_x = []
    bang_y = []
    for line in csvreader:
        bang_x.append(float(line[0]))
        bang_y.append(float(line[1]))

    my_dpi = 200
    plt.figure(figsize=(1500/my_dpi, 1200/my_dpi), dpi=my_dpi)

    plt.plot(path_x, path_y, 'C1', linewidth=3, label="Траектория робота")
    plt.plot(bang_x, bang_y, 'C2--', linewidth=1.5, label="TSOCS")
    

    plt.arrow(0, 0, 0, 0.5, width=0.005, head_width=0.025, head_length=0.07, ec='C0', color='C0')
    plt.arrow(1, 0, 0.5, 0, width=0.005, head_width=0.025, head_length=0.07, ec='C0', color='C0')

    plt.xlim((-0.1, 1.6))
    plt.ylim((-0.6, 0.6))

    plt.xlabel('x')
    plt.ylabel('y')

    plt.grid(color=(0,0,0), alpha=0.3, linestyle='-', linewidth=1)
    plt.legend(loc='upper right')

    plt.show()