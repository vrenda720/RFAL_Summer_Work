# import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt
# import os

# # path = "./IMU2CSV_quaternion.csv"
# path = r".\raw-imu-solution\2024_06_06_15_38_00_765781.csv"
# data = pd.read_csv(path)
# data.yaw.plot()
# # data.pitch.plot()
# #data.roll.plot()
# plt.show()

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

# Path to the CSV file
# path = r".\raw-imu-solution\2024_06_06_15_58_21_546428.csv"
# path = r".\raw-imu-solution\2024_06_06_15_59_36_754748.csv"
path = r".\raw-imu-solution\2024_06_06_16_55_50_458758.csv"


# Load the data
data = pd.read_csv(path)

# Plot yaw
plt.figure()
data['yaw'].plot()
plt.title('Yaw')
plt.xlabel('Sample Index')
plt.ylabel('Yaw (degrees)')
plt.grid(True)
plt.show()

# Plot pitch
plt.figure()
data['pitch'].plot()
plt.title('Pitch')
plt.xlabel('Sample Index')
plt.ylabel('Pitch (degrees)')
plt.grid(True)
plt.show()

# Plot roll
plt.figure()
data['roll'].plot()
plt.title('Roll')
plt.xlabel('Sample Index')
plt.ylabel('Roll (degrees)')
plt.grid(True)
plt.show()
