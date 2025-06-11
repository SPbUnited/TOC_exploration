import csv

import matplotlib.pyplot as plt
import numpy as np

with open('tsocs.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    time = []
    tsocs_x = []
    tsocs_y = []
    for line in csvreader:
        tsocs_x.append(float(line[0]))
        tsocs_y.append(float(line[1]))

with open('bang_s.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    time = []
    bang_s_x = []
    bang_s_y = []
    for line in csvreader:
        bang_s_x.append(float(line[0]))
        bang_s_y.append(float(line[1]))

with open('bang_f.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    time = []
    bang_f_x = []
    bang_f_y = []
    for line in csvreader:
        bang_f_x.append(float(line[0]))
        bang_f_y.append(float(line[1]))

    my_dpi = 200
    plt.figure(figsize=(1500/my_dpi, 1200/my_dpi), dpi=my_dpi)

    plt.plot(bang_s_x, bang_s_y, 'C2', linewidth=3, label="Bang-bang")#, Vmax=0.75")
    #plt.plot(bang_f_x, bang_f_y, 'C0', linewidth=3, label="Bang-bang, Vmax=99")
    plt.plot(tsocs_x, tsocs_y, 'C1--', linewidth=3, label="TSOCS")

    plt.arrow(0, 0, 0.25, 1, width=0.005, head_width=0.025, head_length=0.07, ec='r', color='r')
    plt.arrow(0.1, 0.4, -0.5, 0, width=0.005, head_width=0.025, head_length=0.07, ec='b', color='b')

    plt.xlim((-0.6, 0.4))
    plt.ylim((-0.1, 1.1))

    plt.xlabel('x')
    plt.ylabel('y')

    plt.grid(color=(0,0,0), alpha=0.3, linestyle='-', linewidth=1)
    plt.legend(loc='upper left')

    plt.show()