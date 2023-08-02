from tkinter import *
from tkinter import  messagebox
from tkinter import PhotoImage 
import numpy as np
import math
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import spatialmath
from spatialmath import SE3
import matplotlib
matplotlib.use("TkAgg")

# Create GUI window with title
mygui = Tk()
mygui.title("ARTICULATED Calculator")
mygui.resizable(False, False)
mygui.configure(bg = "black")

def reset():
    a1_e.delete(0, END)
    a2_e.delete(0, END)
    a3_e.delete(0, END)

    T1_e.delete(0, END)
    T2_e.delete(0, END)
    T3_e.delete(0, END)

    x_e.delete(0, END)
    y_e.delete(0, END)
    z_e.delete(0, END)

def f_k():
    # link lengths in mm
    a1 = float(a1_e.get()) / 100
    a2 = float(a2_e.get()) / 100
    a3 = float(a3_e.get()) / 100

    # joint variable: is mm if f, is degrees if theta
    T1 = float(T1_e.get()) 
    T2 = float(T2_e.get()) 
    T3 = float(T3_e.get()) 


    # degree to radian
    T1 = (T1/180.0) * np.pi
    T2 = (T2/180.0) * np.pi
    T3 = (T3/180.0) * np.pi

    # Parametric Table (theta, alpha, r, d)
    PT =   [[T1,(90.0/180.0) * np.pi, 0, a1], 
            [T2, (0.0/180.0) * np.pi, a2, 0], 
            [T3, (0.0/180.0) * np.pi, a3, 0]]

    # HTM formulae
    i = 0
    H0_1 = [[np.cos(PT[i][0]), -np.sin(PT[i][0]) * np.cos(PT[i][1]), np.sin(PT[i][0]) * np.sin(PT[i][1]), PT[i][2] * np.cos(PT[i][0])],
            [np.sin(PT[i][0]), np.cos(PT[i][0]) * np.cos(PT[i][1]), -np.cos(PT[i][0]) * np.sin(PT[i][1]), PT[i][2] * np.sin(PT[i][0])],
            [0, np.sin(PT[i][1]), np.cos(PT[i][1]), PT[i][3]],
            [0, 0, 0, 1]]

    i = 1
    H1_2 = [[np.cos(PT[i][0]), -np.sin(PT[i][0]) * np.cos(PT[i][1]), np.sin(PT[i][0]) * np.sin(PT[i][1]), PT[i][2] * np.cos(PT[i][0])],
            [np.sin(PT[i][0]), np.cos(PT[i][0]) * np.cos(PT[i][1]), -np.cos(PT[i][0]) * np.sin(PT[i][1]), PT[i][2] * np.sin(PT[i][0])],
            [0, np.sin(PT[i][1]), np.cos(PT[i][1]), PT[i][3]],
            [0, 0, 0, 1]]

    i = 2
    H2_3= [[np.cos(PT[i][0]), -np.sin(PT[i][0]) * np.cos(PT[i][1]), np.sin(PT[i][0]) * np.sin(PT[i][1]), PT[i][2] * np.cos(PT[i][0])],
            [np.sin(PT[i][0]), np.cos(PT[i][0]) * np.cos(PT[i][1]), -np.cos(PT[i][0]) * np.sin(PT[i][1]), PT[i][2] * np.sin(PT[i][0])],
            [0, np.sin(PT[i][1]), np.cos(PT[i][1]), PT[i][3]],
            [0, 0, 0, 1]]

    H0_1 = np.matrix(H0_1)
    H1_2 = np.matrix(H1_2)
    H2_3 = np.matrix(H2_3)

    H0_2 = np.dot(H0_1, H1_2)
    H0_3 = np.dot(H0_2, H2_3)

    X0_3 = H0_3[0, 3]
    x_e.delete(0, END)
    x_e.insert(0, np.around(X0_3 * 100, 3))

    Y0_3 = H0_3[1, 3]
    y_e.delete(0, END)
    y_e.insert(0, np.around(Y0_3 * 100, 3))

    Z0_3 = H0_3[2, 3]
    z_e.delete(0, END)
    z_e.insert(0, np.around(Z0_3 * 100, 3))

    # Create links
    # [robot_variable] = DHRobot([RevoluteDH(d, r, alpha, offset)])
    ARTICULATED = DHRobot([
        RevoluteDH(a1, 0, (90.0/180.0) * np.pi, (0.0/180.0)* np.pi, qlim = [-np.pi/2, np.pi/2]),
        RevoluteDH(0, a2, (0.0/180.0) * np.pi, (0.0/180.0) * np.pi, qlim = [-np.pi/2, np.pi/2]),
        RevoluteDH(0, a3, (0.0/180.0) * np.pi, (0.0/180.0) * np.pi, qlim = [-np.pi/2, np.pi/2]),
        ], name = "ARTICULATED")
    
    #Plot Joints
    q1 =  np.array([T1, T2, T3])

    #Plot Scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0.0
    z2 = 0.5

    #Plot Command
    ARTICULATED.plot(q1, limits = [x1, x2, y1, y2, z1, z2], block = True)

# Start of Inverse Kinematics
def i_k ():

    # Inverse Kinematics Using Graphical Method

    # Link lenghts in cm
        a1 = float(a1_e.get())
        a2 = float(a2_e.get())
        a3 = float(a3_e.get())

    # Position Vector in cm
        xe = float(x_e.get())
        ye = float(y_e.get())
        ze = float(z_e.get())

    # Solution for Theta 1, Theta 2, Theta 3
    # Try and Except
        try:
              Th1 = np.arctan(ye/xe)
        except:
              Th1 = -1 #NANERROR
              messagebox.showerror(title = "DivisionbyZero Error", message = "Undefined Solution if X = 0.")
            
        Th1 = np.arctan(ye/xe) #1
        r1 = np.sqrt(ye**2 + xe**2) #2
        r2 = ze - a1 #3
        phi1 = np.arctan(r2/r1) #4
        r3 = np.sqrt(r2**2 + r1**2) #5
        phi2 = np.arccos((a3**2 - a2**2 - r3**2)/(-2 * a2 * r3)) #6
        Th2 = phi1 + phi2 #7
        phi3 = np.arccos((r3**2 - a2**2 - a3**2)/(-2 * a2 * a3)) #8
        Th3 = phi3 - np.pi #9

        T1_e.delete(0, END)
        T1_e.insert(0,np.around(Th1*180/np.pi,3))

        T2_e.delete(0, END)
        T2_e.insert(0,np.around(Th2*180/np.pi,3))

        T3_e.delete(0, END)
        T3_e.insert(0,np.around(Th3*180/np.pi,3))

        # Create links
        # [robot_variable] = DHRobot([RevoluteDH(d, r, alpha, offset)])
        ARTICULATED = DHRobot([
                RevoluteDH(a1/100, 0, (90.0/180.0) * np.pi, (0.0/180.0)* np.pi, qlim = [-np.pi/2, np.pi/2]),
                RevoluteDH(0, a2/100, (0.0/180.0) * np.pi, (0.0/180.0) * np.pi, qlim = [-np.pi/2, np.pi/2]),
                RevoluteDH(0, a3/100, (0.0/180.0) * np.pi, (0.0/180.0) * np.pi, qlim = [-np.pi/2, np.pi/2]),
        ], name = "ARTICULATED")


# Plot joints
        q1 = np.array([Th1,Th2,Th2])


    # Plot scale
        x1 = -0.5
        x2 = 0.5
        y1 = -0.5
        y2 = 0.5
        z1 = 0.0
        z2 = 0.5

    #plot command
        ARTICULATED.plot(q1, limits=[x1,x2,y1,y2,z1,z2],block=True)

# End of Inverse Kinematics

# Link lengths and Joint variable Frame
FI = LabelFrame(mygui, text = "  Link Lengths and Joint Variables  ", font = ("Arial", 15), fg = "black")
FI.grid(row = 0, column = 0, padx = 15, pady = 15)

# Link lengths label
a1 = Label(FI, text = "a1 = ", font = ("Arial", 15))
a1_e = Entry(FI, width = 5, font = ("Arial", 15))
cm1 = Label(FI, text = "cm", font = ("Arial", 15))

a2 = Label(FI, text = "a2 = ", font = ("Arial", 15))
a2_e = Entry(FI, width = 5, font = ("Arial", 15))
cm2 = Label(FI, text = "cm", font = ("Arial", 15))

a3 = Label(FI, text = "a3 = ", font = ("Arial", 15))
a3_e = Entry(FI, width = 5, font = ("Arial", 15))
cm3 = Label(FI, text = "cm", font = ("Arial", 15))

a1.grid(row = 0, column = 0)
a1_e.grid(row = 0, column = 1)
cm1.grid(row = 0, column = 2)

a2.grid(row = 1, column = 0)
a2_e.grid(row = 1, column = 1)
cm2.grid(row = 1, column = 2)

a3.grid(row = 2, column = 0)
a3_e.grid(row = 2, column = 1)
cm3.grid(row = 2, column = 2)

# Joint variable
T1 = Label(FI, text = ("T1 = "), font = ("Arial", 15))
T1_e = Entry(FI, width = 5, font = ("Arial", 15))
deg1 = Label(FI, text = ("deg"), font = ("Arial", 15))

T2 = Label(FI, text = ("T2 = "), font = ("Arial", 15))
T2_e = Entry(FI, width = 5, font = ("Arial", 15))
deg2 = Label(FI, text = ("deg"), font = ("Arial", 15))

T3 = Label(FI, text = ("T3 = "), font = ("Arial", 15))
T3_e = Entry(FI, width = 5, font = ("Arial", 15))
deg3 = Label(FI, text = ("deg"), font = ("Arial", 15))

T1.grid(row = 0, column = 3)
T1_e.grid(row = 0, column = 4)
deg1.grid(row = 0, column = 5)

T2.grid(row = 1, column = 3)
T2_e.grid(row = 1, column = 4)
deg2.grid(row = 1, column = 5)

T3.grid(row = 2, column = 3)
T3_e.grid(row = 2, column = 4)
deg3.grid(row = 2, column = 5)

# Buttons Frame
BF = LabelFrame(mygui, text = "     Forward Kinematics ", font = ("Arial", 15))
BF.grid(row = 1, column = 0, padx = 5, pady = 5)

# Buttons
FK = Button(BF, text = "Forward", font = ("Arial", 15), bg = "blue", fg = "white", command = f_k)
FK.grid(row = 1, column = 1, padx = 15)

IK = Button(BF, text = "Inverse", font = ("Arial", 15), bg = "green", fg = "white", command = i_k)
IK.grid(row = 1, column = 2, padx = 15)

RST = Button(BF, text = "RESET", font = ("Arial", 15), bg = "red", fg = "white", command = reset)
RST.grid(row = 1, column = 2, padx = 20)

FK.grid(row = 0, column = 0)
RST.grid(row = 0, column = 1)
IK.grid(row = 0, column = 2)

# Position Vectors Frame
PV = LabelFrame(mygui, text = "  Position Vectors  ", font = ("Arial", 15))
PV.grid(row = 2, column = 0, padx = 10, pady = 10)

# Position Vectors label
x = Label(PV, text = ("x = "), font = ("Arial", 15))
x_e = Entry(PV, width = 5, font = ("Arial", 15))
cm6 = Label(PV, text = ("cm"), font = ("Arial", 15))

y = Label(PV, text = ("y = "), font = ("Arial", 15))
y_e = Entry(PV, width = 5, font = ("Arial", 15))
cm7 = Label(PV, text = ("cm"), font = ("Arial", 15))

z = Label(PV, text = ("z = "), font = ("Arial", 15))
z_e = Entry(PV, width = 5, font = ("Arial", 15))
cm8 = Label(PV, text = ("cm"), font = ("Arial", 15))

x.grid(row = 0, column = 0)
x_e.grid(row = 0, column = 1)
cm6.grid(row = 0, column = 2)

y.grid(row = 1, column = 0)
y_e.grid(row = 1, column = 1)
cm7.grid(row = 1, column = 2)

z.grid(row = 2, column = 0)
z_e.grid(row = 2, column = 1)
cm8.grid(row = 2, column = 2)

# insert image
img = PhotoImage(file="articulated_model.png")
img = img.subsample(2, 2)
PI = Label(mygui,image=img)
PI.grid(row = 3, column = 0, padx = 10, pady = 10)


mygui.mainloop()