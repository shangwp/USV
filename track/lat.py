import pandas as pd
import matplotlib.pyplot as plt
#https://blog.csdn.net/zf_14159265358979/article/details/84865373 csdn
hw = pd.read_csv('track1.1.1.csv', sep=',')

plt.scatter(hw['lon'], hw['lat'],s=0.1)
plt.show()
