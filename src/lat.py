import pandas as pd
import matplotlib.pyplot as plt
#https://blog.csdn.net/zf_14159265358979/article/details/84865373 csdn


hw = pd.read_csv('track_log.csv', sep=',')
#plt.scatter(hw['usv.x'], hw['usv.y'],s=0.1,c='b')
#plt.scatter(hw['point.x'], hw['point.y'],s=10,c='g')
#plt.scatter(hw['x'], hw['y'],s=0.1,c='b')
plt.plot(hw['usv.x'], hw['usv.y'],color='b')
plt.plot(hw['point.x'],hw['point.y'],linewidth=2,color='r')

'''
#used_point
hd = pd.read_csv('track_set.csv', sep=',')
#plt.plot(hd['x'], hd['y'],linewidth=1,color='r')
plt.scatter(hd['x'], hd['y'],s=1,c='b')
'''
'''
#set_track
ht = pd.read_csv('track.csv', sep=',')
plt.plot(ht['x'], ht['y'])
plt.scatter(ht['x'], ht['y'],s=10,c='b')
'''



plt.show()
