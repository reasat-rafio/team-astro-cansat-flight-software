import sqlite3
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Connect to the SQLite database
conn = sqlite3.connect('gps_data.db')
cursor = conn.cursor()

# Query to retrieve GPS data
cursor.execute("SELECT latitude, longitude, altitude FROM gps_data")
data = cursor.fetchall()

# Close the connection
conn.close()

# Extract data into lists
latitudes = [row[0] for row in data]
longitudes = [row[1] for row in data]
altitudes = [row[2] for row in data]

# Create a 3D scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(latitudes, longitudes, altitudes, c='r', marker='o')

ax.set_xlabel('Latitude')
ax.set_ylabel('Longitude')
ax.set_zlabel('Altitude')

plt.show()
