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

# GUI Window with Title
scaragui = Tk()
scaragui.title("SCARA Calculator")
scaragui.resizable(False,False)
scaragui.configure(bg="darkolivegreen")

def reset():
    a1_E.delete(0, END)
    a2_E.delete(0, END)
    a3_E.delete(0, END)
    a4_E.delete(0, END)
    a5_E.delete(0, END)

    T1_E.delete(0, END)
    T1_E.delete(0, END)
    d3_E.delete(0, END)

    X_E.delete(0, END)
    Y_E.delete(0, END)
    Z_E.delete(0, END)

def forward():
    # Link Lengths in cm
    a1 = float(a1_E.get())/100
    a2 = float(a2_E.get())/100
    a3 = float(a3_E.get())/100
    a4 = float(a4_E.get())/100
    a5 = float(a5_E.get())/100

    T1 = float(T1_E.get())
    T2 = float(T2_E.get())
    d3 = float(d3_E.get())/100

    # Degrees to Radian
    T1 = (T1/180.0)*np.pi
    T2 = (T2/180.0)*np.pi

    # Parametric Table
    PT = [[T1,(0.0/180.0)*np.pi,a2,a1],
          [T2,(180.0/180.0)*np.pi,a4,a3],
          [(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,0,a5+d3]]
    
    # HTM Formulae
    # HTM formulae
    i = 0
    H0_1=[[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
          [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
          [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
          [0,0,0,1]]

    i = 1
    H1_2=[[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
          [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
          [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
          [0,0,0,1]]

    i = 2
    H2_3=[[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
         [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
         [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
         [0,0,0,1]]

    H0_1 = np.matrix(H0_1)
    H1_2 = np.matrix(H1_2)
    H2_3 = np.matrix(H2_3)

    H0_2 = np.dot(H0_1,H1_2)
    H0_3 = np.dot(H0_2,H2_3)

    X0_3 = H0_3[0,3]
    X_E.delete(0, END)

    X_E.insert(0,np.around(X0_3*100,3))

    Y0_3 = H0_3[1,3]
    Y_E.delete(0, END)

    Y_E.insert(0,np.around(Y0_3*100,3))

    Z0_3 = H0_3[2,3]
    Z_E.delete(0, END)

    Z_E.insert(0,np.around(Z0_3*100,3))

    # Creating Links
    # [robot_variable]=DHRobot([RevoluteDH(d,r,alpha,offset)])
    SCARA = DHRobot([
        RevoluteDH(a1,0,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        PrismaticDH(0,a2,0,0,qlim=[0,0]),
        RevoluteDH(a3,0,(180.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        PrismaticDH(0,a4,0,0,qlim=[0,0]),
        PrismaticDH(a5,0,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[0,(30/100)]),
        ], name="SCARA")
    
    # Plot Joints
    q1 = np.array([T1,0,T2,0,d3])

    # Plot Scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = -0.5
    z2 = 0.5

    # Plot Command
    SCARA.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)

# Link Lengths and Joint Variables Frame
LJF = LabelFrame(scaragui,text="    Link Lengths and Joint Variables    ",font=(15),bg="darkolivegreen",fg="ghostwhite")
LJF.grid(row=0,column=0)

# Link Lengths Label and Entry
a1 = Label(LJF,text="a1 = ",font="10",bg="darkolivegreen",fg="ghostwhite")
a1_E = Entry(LJF,width=6,font=(10))
cm1 = Label(LJF,text="cm  ",font="10",bg="darkolivegreen",fg="ghostwhite")

a2 = Label(LJF,text="a2 = ",font="10",bg="darkolivegreen",fg="ghostwhite")
a2_E = Entry(LJF,width=6,font=(10))
cm2 = Label(LJF,text="cm  ",font="10",bg="darkolivegreen",fg="ghostwhite")

a3 = Label(LJF,text="a3 = ",font="10",bg="darkolivegreen",fg="ghostwhite")
a3_E = Entry(LJF,width=6,font=(10))
cm3 = Label(LJF,text="cm  ",font="10",bg="darkolivegreen",fg="ghostwhite")

a4 = Label(LJF,text="a4 = ",font="10",bg="darkolivegreen",fg="ghostwhite")
a4_E = Entry(LJF,width=6,font=(10))
cm4 = Label(LJF,text="cm  ",font="10",bg="darkolivegreen",fg="ghostwhite")

a5 = Label(LJF,text="a5 = ",font="10",bg="darkolivegreen",fg="ghostwhite")
a5_E = Entry(LJF,width=6,font=(10))
cm5 = Label(LJF,text="cm  ",font="10",bg="darkolivegreen",fg="ghostwhite")

a1.grid(row=0,column=0)
a1_E.grid(row=0,column=1)
cm1.grid(row=0,column=2)

a2.grid(row=1,column=0)
a2_E.grid(row=1,column=1)
cm2.grid(row=1,column=2)

a3.grid(row=2,column=0)
a3_E.grid(row=2,column=1)
cm3.grid(row=2,column=2)

a4.grid(row=3,column=0)
a4_E.grid(row=3,column=1)
cm4.grid(row=3,column=2)

a5.grid(row=4,column=0)
a5_E.grid(row=4,column=1)
cm5.grid(row=4,column=2)

# Joint Variables Label and Entry
T1 = Label(LJF,text="T1 = ",font="10",bg="darkolivegreen",fg="ghostwhite")
T1_E = Entry(LJF,width=6,font=(10))
deg1 = Label(LJF,text="deg",font="10",bg="darkolivegreen",fg="ghostwhite")

T2 = Label(LJF,text="T2 = ",font="10",bg="darkolivegreen",fg="ghostwhite")
T2_E = Entry(LJF,width=6,font=(10))
deg2 = Label(LJF,text="deg",font="10",bg="darkolivegreen",fg="ghostwhite")

d3 = Label(LJF,text="d3 = ",font="10",bg="darkolivegreen",fg="ghostwhite")
d3_E = Entry(LJF,width=6,font=(10))
cm6 = Label(LJF,text="cm",font="10",bg="darkolivegreen",fg="ghostwhite")

T1.grid(row=0,column=3)
T1_E.grid(row=0,column=4)
deg1.grid(row=0,column=5)

T2.grid(row=1,column=3)
T2_E.grid(row=1,column=4)
deg2.grid(row=1,column=5)

d3.grid(row=2,column=3)
d3_E.grid(row=2,column=4)
cm6.grid(row=2,column=5)

# Button Frame
BF = LabelFrame(scaragui,text="Forward Kinematics",font=(15),bg="darkolivegreen",fg="ghostwhite")
BF.grid(row=1,column=0)

# Buttons
FK = Button(BF,text="Forward",font=(15),bg="steelblue",fg="ghostwhite",command=forward)
RST = Button(BF,text="Reset",font=(15),bg="gold",fg="black",command=reset)

FK.grid(row=0,column=0)
RST.grid(row=0,column=1)

# Position Vectors Frame
PV = LabelFrame(scaragui,text="Position Vectors",font=(15),bg="darkolivegreen",fg="ghostwhite")
PV.grid(row=2,column=0)

# Position Vectors Label and Entry
X = Label(PV,text="X = ",font="10",bg="darkolivegreen",fg="ghostwhite")
X_E = Entry(PV,width=6,font=(10))
cm7 = Label(PV,text="cm",font="10",bg="darkolivegreen",fg="ghostwhite")

Y = Label(PV,text="Y = ",font="10",bg="darkolivegreen",fg="ghostwhite")
Y_E = Entry(PV,width=6,font=(10))
cm8 = Label(PV,text="cm",font="10",bg="darkolivegreen",fg="ghostwhite")

Z = Label(PV,text="Z = ",font="10",bg="darkolivegreen",fg="ghostwhite")
Z_E = Entry(PV,width=6,font=(10))
cm9 = Label(PV,text="cm",font="10",bg="darkolivegreen",fg="ghostwhite")

X.grid(row=0,column=0)
X_E.grid(row=0,column=1)
cm7.grid(row=0,column=2)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
cm8.grid(row=1,column=2)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
cm9.grid(row=2,column=2)

# Display Image
img = PhotoImage(file="rsz_1scara.png")
PI = Label(scaragui,image=img)
PI.grid(row=3,column=0)

scaragui.mainloop()
