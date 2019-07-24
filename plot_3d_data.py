from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

my_data = np.genfromtxt('house_data.csv', delimiter=',')
print(my_data.shape)
# exit()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
number=20
xline = my_data[0,][0::number]
yline = my_data[1,][0::number]
zline = my_data[2,][0::number]
print(xline.shape)

ax.scatter3D(xline, zline, yline, c=zline, cmap='hsv');
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()