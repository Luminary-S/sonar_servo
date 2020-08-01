#!/usr/bin/python
# -*- coding: utf-8 -*-
# import sys
# sys.path.append("../")
# author : sgl
# time: 2020-06-26

import matplotlib.pyplot as plt 
import numpy as np
from scipy.optimize import curve_fit


# final result:  D(mm) = 0.1938 * x + 53.836
#  limitation: 47,1040 mm
# not synchronization, setup distance 25cm; synchronized：10cm ok.
"""
s1(ul): 
[ 0.19379818 50.71229465]
[   5.15996487 -261.65767125]
s2(ur):
[ 0.18776606 44.71181006]
[   5.32558705 -238.04358226]
s3(dl): 
[ 0.18651286 51.92410095]
[   5.36145324 -278.34761382]
s4(dr):
[ 0.19253614 45.97917378]
[   5.19363456 -238.72483811]
"""

fname = "/home/sgl/catkin_new/src/sonar_array/data/s4_dcm_fit.txt"

limitation = [47, 1040] #mm

def fit_func(x, a, b):
    return a*x + b

# sonic frequency: 111-44=67  ; 60hz
x_list=[]
y_list=[]
with open(fname) as f:
    # l = f.readline()
    for l in f:
        lt=l.split()
        # print(lt)
        x = int(lt[0])#*10  # here transferred to mm
        y = int(lt[1])
        # print(y)
        x_list.append(x)
        y_list.append(y)

p = np.polyfit(x_list, y_list, 1)
print(p)
params = curve_fit(fit_func, y_list, x_list)
print(params[0])
# t1=range(0,len(d1_list))
# t2=range(0,len(d2_list))
# t3=range(0,len(d3_list))
# t4=range(0,len(d4_list))
# f, ax1 = plt.subplots(1,1)
# # ax1 = fig1.add_subplot(111, projection='3d')
plt.figure()
plt.xlabel("s")
plt.ylabel("distance(mm)")
plt.plot(x_list, y_list,'o-')

plt.show()


