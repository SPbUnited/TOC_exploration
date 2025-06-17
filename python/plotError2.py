import csv

import matplotlib.pyplot as plt
import numpy as np

with open('tsocs_error_1.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    tsocs_time = []
    tsocs_error = []
    for line in csvreader:
        tsocs_time.append(float(line[0]))
        tsocs_error.append(float(line[1]))

with open('bang_error_1.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    bang_time = []
    bang_error = []
    for line in csvreader:
        bang_time.append(float(line[0]))
        bang_error.append(float(line[1]))

my_dpi = 200
plt.figure(figsize=(1500/my_dpi, 1200/my_dpi), dpi=my_dpi)

plt.plot(bang_time, bang_error, 'C1', linewidth=2, label="Ошибка слежения Bnag-Bang")
plt.plot(tsocs_time, tsocs_error, 'C0', linewidth=2, label="Ошибка слежения TSOCS")

plt.xlim((0, 10))
# plt.ylim((-0.6, 0.6))

plt.xlabel('t, с')
plt.ylabel('Ошибка, м')

plt.grid(color=(0,0,0), alpha=0.3, linestyle='-', linewidth=1)
plt.legend(loc='upper left')

plt.show()