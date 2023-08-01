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

# GUI: Window and Title
mygui = Tk()
mygui.title("SPHERICAL CALCULATOR")
mygui.resizable(True,True)
mygui.configure(bg="#f5f5dc")

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
    # Link Lengths in cm
    a1 = float(a1_E.get())/100
    a2 = float(a2_E.get())/100
    a3 = float(a3_E.get())/100

    # Joint Variables: cm if d; degrees if theta
    T1 = float(T1_E.get())
    T2 = float(T2_E.get())
    d3 = float(d3_E.get())/100

    # Degree to Radian
    T1 = (T1/180.0)*np.pi
    T2 = (T2/180.0)*np.pi

    # Parametric Table (theta, alpha, r, d)
    PT = [[T1,(90.0/180.0)*np.pi,0,a1],
         [(90.0/180.0)*np.pi+T2,(90.0/180.0)*np.pi,0,0],
         [(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,0,a2+a3+d3]]

    # HTM Formula
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

    # Create Links
    # [robot_variable] = DHRobot([RevoluteDH(d,r,alpha,offset)]) ; If prismatic: (d=0, offset=d)
    SPHERICAL= DHRobot([
        RevoluteDH(a1,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,0,(90.0/180.0)*np.pi,(90.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        PrismaticDH(0,0,(0.0/180.0)*np.pi,a2+a3,qlim=[0,30/100])
        ], name="SPHERICAL")

    # Plot Joints
    q1 = np.array([T1,T2,d3])

    # Plot Scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0.0
    z2 = 0.5

    # Plot Command
    SPHERICAL.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)

def i_k():
    # Inverse Kinematics Using Graphical Method

    # Link Lengths in cm
    a1 = float(a1_E.get())
    a2 = float(a2_E.get())
    a3 = float(a3_E.get())

    # Position Vector in cm
    xe = float(X_E.get())
    ye = float(Y_E.get())
    ze = float(Z_E.get())

    # To solve Theta 1 or Th1 
    try:
        Th1 = np.arctan(ye/xe) #1
    except:
        Th1 = np.nan 
        if xe == 0 and ye != 0:
            messagebox.showerror(title = "DividedByZero Error", message = "Undefined Solution if x = 0")

    # To solve Theta 2 or Th2
    r1 = np.sqrt(xe**2 + ye**2) #2
    r2 = ze - a1 #3
    if r1 != 0:
        Th2 = np.arctan(r2/r1) #4
    else:
        Th2 = np.nan
        if xe == 0 and ye == 0:
            messagebox.showerror(title = "DividedByZero Error", message = "Undefined Solution if x = 0 and y = 0")

    # To solve D3
    D3 = np.sqrt(r1**2 + r2**2) - a2 - a3 #5

    T1_E.delete(0,END)
    T1_E.insert(0,np.around(Th1*180/np.pi,3))

    T2_E.delete(0,END)
    T2_E.insert(0,np.around(Th2*180/np.pi,3))

    d3_E.delete(0,END)
    d3_E.insert(0,np.around(D3,3))

    # Create Links
    # [robot_variable] = DHRobot([RevoluteDH(d,r,alpha,offset)]) ; If prismatic: (d=0, offset=d)
    SPHERICAL= DHRobot([
        RevoluteDH(a1/100,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,0,(90.0/180.0)*np.pi,(90.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        PrismaticDH(0,0,(0.0/180.0)*np.pi,(a2+a3)/100,qlim=[0,30/100])
        ], name="SPHERICAL")

    # Plot Joints
    q1 = np.array([Th1,Th2,D3/100])

    # Plot Scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0.0
    z2 = 0.5

    # Plot Command
    SPHERICAL.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)

# Link Lengths and Join Variables Frame
fontlabel = ("Gothic",12,"bold")
bgcolor = "#f5f5dc"

FI = LabelFrame(mygui,text="Link Lengths and Joint Variables",border=5,font=fontlabel,labelanchor="n",padx=10,pady=10,bg=bgcolor)
FI.grid(row=0,column=0,padx=5,pady=2)

# Link Lengths Labels
a1 = Label(FI,text=("a1 ="),font=(10),bg=bgcolor)
a1_E = Entry(FI,width=5,font=(10))
cm1 = Label(FI,text=("cm"),font=(10),bg=bgcolor)

a1.grid(row=0,column=0)
a1_E.grid(row=0,column=1)
cm1.grid(row=0,column=2)

a2 = Label(FI,text=("a2 ="),font=(10),bg=bgcolor)
a2_E = Entry(FI,width=5,font=(10))
cm2 = Label(FI,text=("cm"),font=(10),bg=bgcolor)

a2.grid(row=1,column=0)
a2_E.grid(row=1,column=1)
cm2.grid(row=1,column=2)

a3 = Label(FI,text=("a3 ="),font=(10),bg=bgcolor)
a3_E = Entry(FI,width=5,font=(10))
cm3 = Label(FI,text=("cm"),font=(10),bg=bgcolor)

a3.grid(row=2,column=0)
a3_E.grid(row=2,column=1)
cm3.grid(row=2,column=2)

# Joint Variables Labels
T1 = Label(FI,text=("  T1 ="),font=(10),bg=bgcolor)
T1_E = Entry(FI,width=5,font=(10))
deg1 = Label(FI,text=("deg"),font=(10),bg=bgcolor)

T1.grid(row=0,column=3)
T1_E.grid(row=0,column=4)
deg1.grid(row=0,column=5)

T2 = Label(FI,text=("  T2 ="),font=(10),bg=bgcolor)
T2_E = Entry(FI,width=5,font=(10))
deg2 = Label(FI,text=("deg"),font=(10),bg=bgcolor)

T2.grid(row=1,column=3)
T2_E.grid(row=1,column=4)
deg2.grid(row=1,column=5)

d3 = Label(FI,text=("  d3 ="),font=(10),bg=bgcolor)
d3_E = Entry(FI,width=5,font=(10))
cm4 = Label(FI,text=("cm"),font=(10),bg=bgcolor)

d3.grid(row=2,column=3)
d3_E.grid(row=2,column=4)
cm4.grid(row=2,column=5)

# Buttons Frame
BF = LabelFrame(mygui,text="Forward Kinematics",border=5,font=fontlabel,labelanchor="n",padx=10,pady=10,bg=bgcolor)
BF.grid(row=1,column=0,)

# Buttons
FK = Button(BF,text="↓ Forward",font=(10),bg="#388e3c",fg="white",border=3,command=f_k)
rst = Button(BF,text="RESET",font=(10),bg="#e64a19",fg="white",border=3,command=reset)
IK = Button(BF,text="↑ Inverse",font=(10),bg="#3b7ea1",fg="white",border=3,command=i_k)

FK.grid(row=0, column=0)
rst.grid(row=0, column=1)
IK.grid(row=0, column=2)

# Position Vector Frame
PV = LabelFrame(mygui,text="Position Vectors",border=5,font=fontlabel,labelanchor="n",padx=10,pady=10,bg=bgcolor)
PV.grid(row=2,column=0,pady=2)

# Position Vector Labels
X = Label(PV,text=("X ="),font=(10),bg=bgcolor)
X_E = Entry(PV,width=6,font=(10))
cm5 = Label(PV,text=("cm"),font=(10),bg=bgcolor)

X.grid(row=0,column=0)
X_E.grid(row=0,column=1)
cm5.grid(row=0,column=2)

Y = Label(PV,text=("Y ="),font=(10),bg=bgcolor)
Y_E = Entry(PV,width=6,font=(10))
cm6 = Label(PV,text=("cm"),font=(10),bg=bgcolor)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
cm6.grid(row=1,column=2)

Z = Label(PV,text=("Z ="),font=(10),bg=bgcolor)
Z_E = Entry(PV,width=6,font=(10))
cm7 = Label(PV,text=("cm"),font=(10),bg=bgcolor)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
cm7.grid(row=2,column=2)

# Kinematic Diagram Frame
KD = LabelFrame(mygui,text="Kinematic Diagram - Spherical Manipulator",border=5,font=fontlabel,labelanchor="n",padx=2,pady=5,bg=bgcolor)
KD.grid(row=3,column=0,padx=5,pady=5)

# Display Image
img = PhotoImage(file="MX3202_Company2_Spherical_FKCalculator.png")
img = img.subsample(1,1)
PI = Label(KD,image=img,bg=bgcolor)
PI.grid(row=1,column=0)

mygui.mainloop()
