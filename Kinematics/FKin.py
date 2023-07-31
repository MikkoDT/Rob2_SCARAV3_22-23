import numpy as np

## Forward Kinematics ##
# link lengths in cm
a1 = float(input("a1 = ")) # For testing, 55 mm
a2 = float(input("a2 = ")) # For testing, 60 mm
a3 = float(input("a3 = ")) # For testing, 30 mm
a4 = float(input("a4 = ")) # For testing, 60 mm

d1 = float(input("d1 = ")) # For testing, 25 mm
T2 = float(input("T2 = ")) # For testing, 30 degrees
T3 = float(input("T3 = ")) # For testing, 60 degrees

T2 = (T2/180.0)*np.pi # Theta 2 in radians
T3 = (T3/180.0)*np.pi # Theta 3 in radians

PT = [[(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,0,a1+d1],
      [T2,(0.0/180.0)*np.pi,a2,0],
      [T3,(0.0/180.0)*np.pi,a4,a3]]

i = 0
H0_1 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]

i = 1
H1_2 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]

i = 2
H2_3 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]


H0_1 = np.matrix(H0_1) 
#print("H0_1=")
#print(H0_1)

H1_2 = np.matrix(H1_2) 
#print("H1_2=")
#print(H1_2)

H2_3 = np.matrix(H2_3)
#print("H2_3=")
#print(H2_3)

H0_2 = np.dot(H0_1,H1_2)
H0_3 = np.dot(H0_2,H2_3)
print("H0_3=")
print(np.matrix(np.around(H0_3,3)))

print("H0_2 = ")
print(np.matrix(H0_2))

## Position Vectors ##
X0_3 = H0_3[0,3]
print("X = ", np.around(X0_3,3))
Y0_3 =H0_3[1,3]
print("Y = ", np.around(Y0_3,3))
Z0_3 =H0_3[2,3]
print("Z = ", np.around(Z0_3,3))



