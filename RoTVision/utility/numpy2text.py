# Saves the numpy data into a human readable for in the folder ~/calibration/text/
import numpy as np
import os

path = 'calibration/'
files = []

for d, r, f  in os.walk(path):
    for file in f:
        print('\r\n'+file)
        print('=====================================')
        data = np.load(path+file)
        print(np.array2string(data,separator=',',formatter={'float_kind':lambda x: "%.10f" % x}))
        np.savetxt(path+'text/'+file+'.txt',data,delimiter=',',newline=']\r\n [')
