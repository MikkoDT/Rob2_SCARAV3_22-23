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

# Create a GUI window with a title
MX3201_Company7_Cartesian_FKCalculator = Tk()
MX3201_Company7_Cartesian_FKCalculator.title('Cartesian Calculator')
MX3201_Company7_Cartesian_FKCalculator.resizable(False, False)
MX3201_Company7_Cartesian_FKCalculator.configure(bg='pink')

def reset():
    a1_E.delete(0, END)
    a2_E.delete(0, END)
    a3_E.delete(0, END)
    a4_E.delete(0, END)

    d1_E.delete(0, END)
    d2_E.delete(0, END)
    d3_E.delete(0, END)

    X_E.delete(0, END)
    Y_E.delete(0, END)
    Z_E.delete(0, END)

def f_k():
    # link lengths in cm
    a1 = float(a1_E.get()) / 100
    a2 = float(a2_E.get()) / 100
    a3 = float(a3_E.get()) / 100
    a4 = float(a4_E.get()) / 100

    d1 = float(d1_E.get()) / 100
    d2 = float(d2_E.get()) / 100
    d3 = float(d3_E.get()) / 100

    # Parametric Table (Theta, Alpha, r, d)
    Cartesian_PT = [[0 / 180 * np.pi, 270 / 180 * np.pi, 0, a1],
                    [270 / 180 * np.pi, 270 / 180 * np.pi, 0, a2 + d1],
                    [90 / 180 * np.pi, 270 / 180 * np.pi, 0, a3 + d2],
                    [0 / 180 * np.pi, 0 / 180 * np.pi, 0, a4 + d3]]

    # HTM formulae
    i = 0
    H0_1 = [[1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, -1, 0, Cartesian_PT[i][3]],
            [0, 0, 0, 1]]

    i = 1
    H1_2 = [[0, 0, 1, 0],
            [-1, 0, 0, 0],
            [0, -1, 0, Cartesian_PT[i][3]],
            [0, 0, 0, 1]]

    i = 2
    H2_3 = [[0, 0, -1, 0],
            [1, 0, 0, 0],
            [0, -1, 0, Cartesian_PT[i][3]],
            [0, 0, 0, 1]]

    i = 3
    H3_4 = [[1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, Cartesian_PT[i][3]],
            [0, 0, 0, 1]]

    H0_1 = np.matrix(H0_1)
    H1_2 = np.matrix(H1_2)
    H2_3 = np.matrix(H2_3)
    H3_4 = np.matrix(H3_4)

    H0_2 = np.dot(H0_1, H1_2)
    H0_3 = np.dot(H0_2, H2_3)
    H0_4 = np.dot(H0_3, H3_4)

    X0_3 = H0_4[0, 3]
    X_E.delete(0, END)
    X_E.insert(0, np.around(X0_3 * 100, 3))

    Y0_3 = H0_4[1, 3]
    Y_E.delete(0, END)
    Y_E.insert(0, np.around(Y0_3 * 100, 3))

    Z0_3 = H0_4[2, 3]
    Z_E.delete(0, END)
    Z_E.insert(0, np.around(Z0_3 * 100, 3))

    # Create Links
    # [robot_variable] = DHRobot([RevoluteDH(d, r, alpha, offset)])
    Lab01 = DHRobot([
        PrismaticDH(0, 0, (270.0 / 180.0) * np.pi, a1, qlim=[0, 0]),
        RevoluteDH(a2, 0, (0.0 / 180.0) * np.pi, (270.0 / 180.0) * np.pi, qlim=[np.pi / 2, np.pi / 2]),
        PrismaticDH(0, 0, (270.0 / 180.0) * np.pi, 0, qlim=[0, (30 / 100)]),
        RevoluteDH(a3, 0, (0.0 / 180.0) * np.pi, (90.0 / 180.0) * np.pi, qlim=[np.pi / 2, np.pi / 2]),
        PrismaticDH(0, 0, (270.0 / 180.0) * np.pi, 0, qlim=[0, (30 / 100)]),
        PrismaticDH(0, 0, (0.0 / 180.0) * np.pi, a4, qlim=[0, (30 / 100)])
    ], name='Lab01')

    # plot joints
    q1 = np.array([0, 0, d1, 0, d2, d3])

    # plot scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0.0
    z2 = 0.5

    # plot command
    Lab01.plot(q1, limits=[x1, x2, y1, y2, z1, z2], block=True)

def i_k():
    # Inverse Kinematics Using Graphical Method

    # Link Lengths in cm
    a1 = float(a1_E.get())
    a2 = float(a2_E.get())
    a3 = float(a3_E.get())
    a4 = float(a4_E.get())

    # Position Vector in cm
    xe = float(X_E.get())
    ye = float(Y_E.get())
    ze = float(Z_E.get())

    # To solve for D2
    D2 = xe - a3  # 1

    # To solve for D3
    D3 = a1 - a4 - ze  # 2

    # To solve for D1
    D1 = ye - a2  # 3

    d1_E.delete(0, END)
    d1_E.insert(0, np.around(D1, 3))

    d2_E.delete(0, END)
    d2_E.insert(0, np.around(D2, 3))

    d3_E.delete(0, END)
    d3_E.insert(0, np.around(D3, 3))

    # Create Links
    # [robot_variable] = DHRobot([RevoluteDH(d, r, alpha, offset)])
    Lab01 = DHRobot([
        PrismaticDH(0, 0, (270.0 / 180.0) * np.pi, a1/100, qlim=[0, 0]),
        RevoluteDH(a2/100, 0, (0.0 / 180.0) * np.pi, (270.0 / 180.0) * np.pi, qlim=[np.pi / 2, np.pi / 2]),
        PrismaticDH(0, 0, (270.0 / 180.0) * np.pi, 0, qlim=[0, (30 / 100)]),
        RevoluteDH(a3/100, 0, (0.0 / 180.0) * np.pi, (90.0 / 180.0) * np.pi, qlim=[np.pi / 2, np.pi / 2]),
        PrismaticDH(0, 0, (270.0 / 180.0) * np.pi, 0, qlim=[0, (30 / 100)]),
        PrismaticDH(0, 0, (0.0 / 180.0) * np.pi, a4/100, qlim=[0, (30 / 100)])
    ], name='Lab01')

    # plot joints
    q1 = np.array([0, 0, D1/100, 0, D2/100, D3/100])

    # plot scale
    x1 = -0.55
    x2 = 0.55
    y1 = -0.55
    y2 = 0.55
    z1 = 0.05
    z2 = 0.55

    # plot command
    Lab01.plot(q1, limits=[x1, x2, y1, y2, z1, z2], block=True)

# Link Lengths and Joint Variables Frame
FI = LabelFrame(MX3201_Company7_Cartesian_FKCalculator, text='Link Lengths and Joint Variables', font=(5), bg='pink')
FI.grid(row=0, column=0)

# Link Length Label
a1 = Label(FI, text='a1 = ', font=(10), bg='pink', fg='black')
a1_E = Entry(FI, width=5, font=(10), bg='white', fg='red')
cm1 = Label(FI, text='cm', font=(10), bg='pink', fg='black')

a2 = Label(FI, text='a2 = ', font=(10), bg='pink', fg='black')
a2_E = Entry(FI, width=5, font=(10), bg='white', fg='red')
cm2 = Label(FI, text='cm', font=(10), bg='pink', fg='black')

a3 = Label(FI, text='a3 = ', font=(10), bg='pink', fg='black')
a3_E = Entry(FI, width=5, font=(10), bg='white', fg='red')
cm3 = Label(FI, text='cm', font=(10), bg='pink', fg='black')

a4 = Label(FI, text='a4 = ', font=(10), bg='pink', fg='black')
a4_E = Entry(FI, width=5, font=(10), bg='white', fg='red')
cm4 = Label(FI, text='cm', font=(10), bg='pink', fg='black')

a1.grid(row=0, column=0)
a1_E.grid(row=0, column=1)
cm1.grid(row=0, column=2)

a2.grid(row=1, column=0)
a2_E.grid(row=1, column=1)
cm2.grid(row=1, column=2)

a3.grid(row=2, column=0)
a3_E.grid(row=2, column=1)
cm3.grid(row=2, column=2)

a4.grid(row=3, column=0)
a4_E.grid(row=3, column=1)
cm4.grid(row=3, column=2)

# Joint Variable Label
d1 = Label(FI, text='d1 = ', font=(10), bg='pink', fg='black')
d1_E = Entry(FI, width=5, font=(10), bg='white', fg='red')
cm5 = Label(FI, text='cm', font=(10), bg='pink', fg='black')

d2 = Label(FI, text='d2 = ', font=(10), bg='pink', fg='black')
d2_E = Entry(FI, width=5, font=(10), bg='white', fg='red')
cm6 = Label(FI, text='cm', font=(10), bg='pink', fg='black')

d3 = Label(FI, text='d3 = ', font=(10), bg='pink', fg='black')
d3_E = Entry(FI, width=5, font=(10), bg='white', fg='red')
cm7 = Label(FI, text='cm', font=(10), bg='pink', fg='black')

d1.grid(row=0, column=3)
d1_E.grid(row=0, column=4)
cm5.grid(row=0, column=5)

d2.grid(row=1, column=3)
d2_E.grid(row=1, column=4)
cm6.grid(row=1, column=5)

d3.grid(row=2, column=3)
d3_E.grid(row=2, column=4)
cm7.grid(row=2, column=5)

# Button Frame
BF = LabelFrame(MX3201_Company7_Cartesian_FKCalculator, text='Forward Kinemetics', font=(5), bg='pink')
BF.grid(row=1, column=0)

# Buttons
FK = Button(BF, text='↓ FORWARD', font=(10), bg='pink', fg='white', command=f_k)
rst = Button(BF, text='RESET', font=(10), bg='pink', fg='white', command=reset)
IK = Button(BF, text='↑ INVERSE', font=(10), bg='pink', fg='white', command=f_k)

FK.grid(row=0, column=0)
rst.grid(row=0, column=1)
IK.grid(row=0, column=2)

# Position Vectors Frame
PV = LabelFrame(MX3201_Company7_Cartesian_FKCalculator, text='Position Vectors', font=(5), bg='pink')
PV.grid(row=2, column=0)

# Position Vectors Label
X = Label(PV, text='X = ', font=(10), bg='pink', fg='black')
X_E = Entry(PV, width=5, font=(10), bg='white', fg='red')
cm8 = Label(PV, text='cm', font=(10), bg='pink', fg='black')

Y = Label(PV, text='Y = ', font=(10), bg='pink', fg='black')
Y_E = Entry(PV, width=5, font=(10), bg='white', fg='red')
cm9 = Label(PV, text='cm', font=(10), bg='pink', fg='black')

Z = Label(PV, text='Z = ', font=(10), bg='pink', fg='black')
Z_E = Entry(PV, width=5, font=(10), bg='white', fg='red')
cm10 = Label(PV, text='cm', font=(10), bg='pink', fg='black')

X.grid(row=0, column=0)
X_E.grid(row=0, column=1)
cm8.grid(row=0, column=2)

Y.grid(row=1, column=0)
Y_E.grid(row=1, column=1)
cm9.grid(row=1, column=2)

Z.grid(row=2, column=0)
Z_E.grid(row=2, column=1)
cm10.grid(row=2, column=2)

# Display Image
img = PhotoImage(file="Cart2.png")
PI = Label(MX3201_Company7_Cartesian_FKCalculator, image=img)
PI.grid(row=3, column=0)

MX3201_Company7_Cartesian_FKCalculator.mainloop()