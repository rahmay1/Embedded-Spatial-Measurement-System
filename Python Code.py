import serial
import numpy as np
import open3d as o3d

# Opening the serial input and opening a new file to store points in
s = serial.Serial("COM6", 115200)
f = open("points2dx4proj.xyz","w")

# Printing the currently opened port stream
print("Opening: " + s.name)

# Initialized Variables
totalPlanes = int(input("Enter the number of data sets you would like to plot: ")) # Asking user for input and setting it to the var
planePts = 0

# Repeats until planes is done
for x in range(totalPlanes):
    # Initializing the readline for while loop
    x = s.readline()
    line = x.decode()
    # While loop which gets lines until "End" is found in a line (Second Points)
    while line.find("End") == -1:
        # Checks if it the nextline from stream does not contain a character 'r'
        if line.find("r") == -1:
            f.write(line)
            print(line)
            planePts = planePts+1;
        x = s.readline()
        line = x.decode()

# Closing inputstreams
f.close();
s.close();

# Initalized Vars
planePts = planePts/totalPlanes

# Grabs the points in the xyz file and stores into an array, also prints the points
pcd = o3d.io.read_point_cloud("points2dx4proj.xyz", format='xyz')
print(pcd)
print(np.asarray(pcd.points))

# Var which will contain lines between points
lines = []

# For loop that connects all points which line up in adjacent planes to each other (stores into lines array)
for i in range(totalPlanes-1):
    for j in range(int(planePts)):
        lines.append([j+planePts*i,j+planePts*(i+1)])

# For loop that connects all points in the same plane with its adjacent points (stores into lines array)
for i in range(totalPlanes):
    for j in range(int(planePts)):
        lines.append([j+i*planePts,(j+1)%planePts+i*planePts])

# Creates an array of lines from the line array using open3d library
lines_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))
o3d.visualization.draw_geometries([lines_set]) # Visualizes the array of lines using open3d library




