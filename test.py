import numpy as np
import matlab.engine
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import csv
import pandas as pd

eng = matlab.engine.start_matlab()
# eng.cd(r'D:\MECH0020\MSS_Toolbox\VESSELS', nargout=0)
# x1 = matlab.double([200.0, 300.0, 400.0, 50.0, 60.0, 70.0, 80.0, 90.0])
# u = matlab.double([20, 30, 40])
# # listtest=[1,2,3]
# x = eng.tanker(x1, u)
# print(x)

eng.cd(r'D:\MECH0020\MSS_Toolbox\HYDRO\gdf', nargout=0)

table = eng.plot_wamitgdf(r'D:\MECH0020\MSS_Toolbox\HYDRO\vessels_wamit\tanker\tanker')
# np.savetxt('points.csv', table, delimiter=',')
input()
print(table)
# table = np.asarray(table)
# fig = plt.figure()
# ax = plt.axes(projection='3d')
# for i in table:
#     x = i[[0, 3, 6, 9]]
#     y = i[[1, 4, 7, 10]]
#     xx, yy = np.meshgrid(x, y)
#     z = i[[2, 5, 8, 11]]
#     zz = np.tile(z, (4, 1))
#     ax.plot_surface(xx, yy, zz)
# ax.set_aspect('equal')
# plt.show()


# table = np.asarray(table)
# fig = plt.figure()
# ax = plt.axes(projection='3d')
# i = table[0]
# x = i[[0,3,6,9]].reshape(2,2)
# y = i[[1,4,7,10]].reshape(2,2)
# z = i[[2,5,8,11]]
# zz = z.reshape(2,2)
# print(zz)
# ax.plot_surface(x,y,zz)
# ax.set_aspect('equal')
# plt.show()
