import csv
import matplotlib.pyplot as plt

x = []
y = []
with open('lake_track_waypoints.csv', newline='') as f:
    reader = csv.reader(f, delimiter=',')
    for row in reader:
        tmp_x, tmp_y = row
        #print(tmp_x, tmp_y)

        if tmp_x!='x': # Remove first line of 'x', 'y' in csv file
            x.append(float(tmp_x))
            y.append(float(tmp_y))

plt.plot(x,y)
plt.show()

