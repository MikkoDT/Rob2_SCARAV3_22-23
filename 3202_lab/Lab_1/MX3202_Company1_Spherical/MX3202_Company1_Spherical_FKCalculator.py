from tkinter import *
from tkinter import messagebox
from tkinter import PhotoImage
import numpy as np
import math
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import spatialmath
from spatialmath import SE3
import matplotlib 
matplotlib.use('TkAgg')

#Create a GUI window with a title 
mygui = Tk()
mygui.title("SPHERICAL Calculator")
mygui.resizable(True, True)

def reset():
    a1_E.delete(0, END)
    a2_E.delete(0, END)
    a3_E.delete(0, END)

    T1_E.delete(0, END)
    T2_E.delete(0, END)
    d3_E.delete(0, END)

    X_E.delete(0, END)
    Y_E.delete(0, END)
    Z_E.delete(0, END)

def f_k():
    # link lengths in cm
    a1 = float(a1_E.get())/100
    a2 = float(a2_E.get())/100
    a3 = float(a3_E.get())/100

    # joint variables: is cm if f, is degree if theta
    T1 = float(T1_E.get())
    T2 = float(T2_E.get())
    d3 = float(d3_E.get())/100

    # degree to radian
    T1 = (T1/180.0)*np.pi
    T2 = (T2/180.0)*np.pi

    # Parametric Table (Theta, alpha, r, d)
    PT = [[T1,(90.0/180.0)*np.pi,0,a1],
          [(90.0/180.0)*np.pi+T2,(90.0/180.0)*np.pi,0,0],
          [(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,0,a2+a3+d3]]
    
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
    H1_2 = np.matrix(H1_2)
    H2_3 = np.matrix(H2_3)

    H0_2 = np.dot(H0_1,H1_2)
    H0_3 = np.dot(H0_2,H2_3) # HTM of the SPHERICAL

    # Position Vectors
    X0_3 = H0_3[0,3]
    X_E.delete(0, END)
    X_E.insert(0, np.around(X0_3*100, 3))

    Y0_3 = H0_3[1,3]
    Y_E.delete(0, END)
    Y_E.insert(0, np.around(Y0_3*100, 3))

    Z0_3 = H0_3[2,3]
    Z_E.delete(0, END)
    Z_E.insert(0, np.around(Z0_3*100, 3))

    # Create links
    # [robot_variable]=DHRobot([RevoluteDH(d, r, alpha, offset)])
    SPHERICAL = DHRobot([
              RevoluteDH(a1, 0, (90.0/180.0)*np.pi, (0.0/180.0)*np.pi, qlim=[-np.pi/2, np.pi/2]),
              RevoluteDH(0, 0, (90.0/180.0)*np.pi, (90.0/180.0)*np.pi, qlim=[-np.pi/2, np.pi/2]),
              PrismaticDH(0, 0, (0.0/180.0)*np.pi, a2+a3, qlim=[0, 30/100]),
              ], name="SPHERICAL")

    # plot joint variables
    q1 = np.array([T1, T2, d3])

    # plot scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0.0
    z2 = 0.5

    # plot command
    SPHERICAL.plot(q1, limits=[x1,x2,y1,y2,z1,z2], block=True)

#Link lengths and Joint Variables Frame
FI = LabelFrame(mygui, text="Link Lengths and Joint Variables", font=(5), bg="#267365", fg="white")
FI.grid(row=0, column=0)

# label of link lengths
a1 = Label(FI, text=("a1 = "), font=(10), bg="#267365", fg="white")
a1_E = Entry(FI, width=5, font=(10))
cm1 = Label(FI, text=("cm"), font=(10), bg="#267365", fg="white")

a2 = Label(FI, text=("a2 = "), font=(10), bg="#267365", fg="white")
a2_E = Entry(FI, width=5, font=(10))
cm2 = Label(FI, text=("cm"), font=(10), bg="#267365", fg="white")

a3 = Label(FI, text=("a3 = "), font=(10), bg="#267365", fg="white")
a3_E = Entry(FI, width=5, font=(10))
cm3 = Label(FI, text=("cm"), font=(10), bg="#267365", fg="white")

a1.grid(row=0, column=0)
a1_E.grid(row=0, column=1)
cm1.grid(row=0, column=2)

a2.grid(row=1, column=0)
a2_E.grid(row=1, column=1)
cm2.grid(row=1, column=2)

a3.grid(row=2, column=0)
a3_E.grid(row=2, column=1)
cm3.grid(row=2, column=2)

# Joint variables label

T1 = Label(FI, text=("T1 = "), font=(10), bg="#267365", fg="white")
T1_E = Entry(FI, width=5, font=(10))
deg1 = Label(FI, text=("deg"), font=(10), bg="#267365", fg="white")

T2 = Label(FI, text=("T2 = "), font=(10), bg="#267365", fg="white")
T2_E = Entry(FI, width=5, font=(10))
deg2 = Label(FI, text=("deg"), font=(10), bg="#267365", fg="white")

d3 = Label(FI, text=("d3 = "), font=(10), bg="#267365", fg="white")
d3_E = Entry(FI, width=5, font=(10))
cm4 = Label(FI, text=("cm"), font=(10), bg="#267365", fg="white")

T1.grid(row=0, column=3)
T1_E.grid(row=0, column=4)
deg1.grid(row=0, column=5)

T2.grid(row=1, column=3)
T2_E.grid(row=1, column=4)
deg2.grid(row=1, column=5)

d3.grid(row=2, column=3)
d3_E.grid(row=2, column=4)
cm4.grid(row=2, column=5)

# Buttons Frame
BF = LabelFrame(mygui, text="Forward Kinematics", font=(5))
BF.grid(row=1, column=0)

# Buttons 
FK = Button(BF, text="Forward",font=(10), bg="green", fg="white", command=f_k)
rst = Button(BF, text="Reset",font=(10), bg="red", fg="white", command=reset)

FK.grid(row=0, column=0)
rst.grid(row=0, column=1)

# KPosition Vectors Frame
PV = LabelFrame(mygui, text="Position Vectors", font=(5), bg="#F29F05")
PV.grid(row=2, column=0)

# Position Vectors label
X = Label(PV, text=("X = "), font=(10), bg="#F29F05")
X_E = Entry(PV, width=5, font=(10))
cm6 = Label(PV, text=("cm"), font=(10), bg="#F29F05")

Y = Label(PV, text=("Y = "), font=(10), bg="#F29F05")
Y_E = Entry(PV, width=5, font=(10))
cm7 = Label(PV, text=("cm"), font=(10), bg="#F29F05")

Z = Label(PV, text=("Z = "), font=(10), bg="#F29F05")
Z_E = Entry(PV, width=5, font=(10))
cm8 = Label(PV, text=("cm"), font=(10), bg="#F29F05")

X.grid(row=0, column=0)
X_E.grid(row=0, column=1)
cm6.grid(row=0, column=2)

Y.grid(row=1, column=0)
Y_E.grid(row=1, column=1)
cm7.grid(row=1, column=2)

Z.grid(row=2, column=0)
Z_E.grid(row=2, column=1)
cm8.grid(row=2, column=2)

#display image

img = PhotoImage(file ="MX3202_Company1_Spherical_FKCalculator.png")
img = img.subsample(1,1)
PI = Label(mygui, image = img)
PI.grid(row = 3, column = 0)



mygui.mainloop()












