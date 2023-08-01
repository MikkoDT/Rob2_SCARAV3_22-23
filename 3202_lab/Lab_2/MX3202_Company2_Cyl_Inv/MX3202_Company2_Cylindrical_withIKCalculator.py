from tkinter import *
from tkinter import messagebox
import numpy as np
import math
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import spatialmath
from spatialmath import SE3
import matplotlib
matplotlib.use ("TkAgg")

# Create a GUI window with a title
mygui = Tk()
mygui.title("CYLINDRICAL CALCULATOR")
mygui.resizable(False,False)
mygui.configure(bg='navajo white')

def reset():
    a1_E.delete(0, END)
    a2_E.delete(0, END)
    a3_E.delete(0, END)

    T1_E.delete(0, END)
    d2_E.delete(0, END)
    d3_E.delete(0, END)

    X_E.delete(0, END)
    Y_E.delete(0, END)
    Z_E.delete(0, END)

def f_k():
    #Link lengths in mm
    a1 = float(a1_E.get())/100
    a2 = float(a2_E.get())/100
    a3 = float(a3_E.get())/100

    # joint variables: if d mm, if Theta degrees
    T1 = float(T1_E.get())
    d2 = float(d2_E.get())/100
    d3 = float(d3_E.get())/100

    # degrees to radian
    T1 = (T1/180.0)*np.pi
    
    #Parametric Table (Theta, alpha, r, d)
    PT = [[T1,(0.0/180.0)*np.pi,0,a1],
      [(270.0/180.0)*np.pi,(270.0/180.0)*np.pi,0,a2+d2],
      [(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,0,a3+d3]]

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
    H0_3 = np.dot(H0_2,H2_3) # HTM of the CYLINDRICAL

    # Position Vectors
    X0_3 = H0_3[0,3]
    X_E.delete(0, END)
    X_E.insert(0,np.around(X0_3*100,3))

    Y0_3 = H0_3[1,3]
    Y_E.delete(0, END)
    Y_E.insert(0,np.around(Y0_3*100,3))

    Z0_3 = H0_3[2,3]
    Z_E.delete(0, END)
    Z_E.insert(0,np.around(Z0_3*100,3))

    # Create links
    # [robot_variable] = DHRobot([RevoluteDH(d,r,alpha,offset)])
    CYLINDRICAL = DHRobot([
        RevoluteDH(a1,0,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        PrismaticDH((270.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a2,qlim=[0,30/100]),
        PrismaticDH(0,0,(0.0/180.0)*np.pi,a3,qlim=[0,50/100])
    ], name="CYLINDRICAL")

    # Plot joints
    q1 = np.array([T1,d2,d3])

    # Plot scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0.0
    z2 = 0.5

    # Plot command
    CYLINDRICAL.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)

def i_k():

     #Link lengths in cm
    a1 = float(a1_E.get())
    a2 = float(a2_E.get())
    a3 = float(a3_E.get())

    # Position Vector in cm
    xe = float(X_E.get())
    ye = float(Y_E.get())
    ze = float(Z_E.get())

    # To solve for Theta 1 or Th1

    # try and except 
    try:
        Th1 =  np.arctan(ye/xe) #1
    except:
        Th1 = -1 #NAN error
        messagebox.showerror(title="DividedbyZero Error", message="Undefined Solution if X=0")

    # To Solve for Theta or Th1
    Th1 = np.arctan(ye/xe) #1
    

    # To Solve For D3
    D3 = np.sqrt(xe**2 + ye**2) - a3 #2 

    # To Solve for D2
    D2 = ze - a1 - a2 #3   

    # Joint Variables
    T1_E.delete(0, END)
    T1_E.insert(0,np.around(Th1*180/np.pi,3))
    
    d2_E.delete(0, END)
    d2_E.insert(0,np.around(D2,3))
    
    d3_E.delete(0, END)
    d3_E.insert(0,np.around(D3,3))


    # Create links
    # [robot_variable] = DHRobot([RevoluteDH(d,r,alpha,offset)])
    CYLINDRICAL = DHRobot([
        RevoluteDH(a1/100,0,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        PrismaticDH((270.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a2/100,qlim=[0,30/100]),
        PrismaticDH(0,0,(0.0/180.0)*np.pi,a3/100,qlim=[0,50/100])
    ], name="CYLINDRICAL")

    # Plot joints
    q1 = np.array([Th1,D2/100,D3/100])

    # Plot scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0.0
    z2 = 0.5

    # Plot command
    CYLINDRICAL.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)


# Link lengths and Joint Variables Frame
FI = LabelFrame(mygui,text="Link Lengths and Joint Variables",font=(5), bg="burlywood3",fg="black")
FI.grid(row=0,column=0)

# label of link lengths
a1 = Label(FI,text=("a1 = "),font=(10), bg="burlywood2",fg="black")
a1_E = Entry(FI,width=5,font=(10), bg="burlywood1",fg="black")
cm1 = Label(FI,text=("cm"),font=(10), bg="burlywood2",fg="black")

a2 = Label(FI,text=("a2 = "),font=(10), bg="burlywood2",fg="black")
a2_E = Entry(FI,width=5,font=(10), bg="burlywood1",fg="black")
cm2 = Label(FI,text=("cm"),font=(10), bg="burlywood2",fg="black")

a3 = Label(FI,text=("a3 = "),font=(10), bg="burlywood2",fg="black")
a3_E = Entry(FI,width=5,font=(10), bg="burlywood1",fg="black")
cm3 = Label(FI,text=("cm"),font=(10), bg="burlywood2",fg="black")

a1.grid(row=0,column=0)
a1_E.grid(row=0,column=1)
cm1.grid(row=0,column=2)

a2.grid(row=1,column=0)
a2_E.grid(row=1,column=1)
cm2.grid(row=1,column=2)

a3.grid(row=2,column=0)
a3_E.grid(row=2,column=1)
cm3.grid(row=2,column=2)

# Joint  variables label 
T1 = Label(FI,text=("T1 = "),font=(10), bg="burlywood2",fg="black")
T1_E = Entry(FI,width=5,font=(10), bg="burlywood1",fg="black")
deg1 = Label(FI,text=("deg"),font=(10), bg="burlywood2",fg="black")

d2 = Label(FI,text=("d2 = "),font=(10), bg="burlywood2",fg="black")
d2_E = Entry(FI,width=5,font=(10), bg="burlywood1",fg="black")
cm4 = Label(FI,text=("cm"),font=(10), bg="burlywood2",fg="black")

d3 = Label(FI,text=("d3 = "),font=(10), bg="burlywood2",fg="black")
d3_E = Entry(FI,width=5,font=(10), bg="burlywood1",fg="black")
cm5 = Label(FI,text=("cm"),font=(10), bg="burlywood2",fg="black")

T1.grid(row=0,column=3)
T1_E.grid(row=0,column=4)
deg1.grid(row=0,column=5)

d2.grid(row=1,column=3)
d2_E.grid(row=1,column=4)
cm4.grid(row=1,column=5)

d3.grid(row=2,column=3)
d3_E.grid(row=2,column=4)
cm5.grid(row=2,column=5)

# Button Frame
BF = LabelFrame(mygui,text="Forward Kinematics",font=(5),bg="rosybrown",fg="black")
BF.grid(row=1,column=0)

# Buttons
FK = Button(BF,text="Forward",font=(10),bg="rosybrown2",fg="black",command=f_k)
rst = Button(BF,text="Reset",font=(10),bg="rosybrown3",fg="black",command=reset)
IK = Button(BF,text="↑ Inverse",font=(10),bg="rosybrown1",fg="black",command=i_k)

FK.grid(row=0,column=0)
rst.grid(row=0,column=1)
IK.grid(row=0,column=2)

# KPosition Vectors Frame
PV = LabelFrame(mygui,text="Position Vectors",font=(5),bg="peachpuff3",fg="black")
PV.grid(row=2,column=0)

# Position Vectors Label
X = Label(PV,text=("X = "),font=(10),bg="peachpuff2",fg="black")
X_E = Entry(PV,width=5,font=(10),bg="peachpuff",fg="black")
cm6 = Label(PV,text=("cm"),font=(10),bg="peachpuff2",fg="black")

Y = Label(PV,text=("Y = "),font=(10),bg="peachpuff2",fg="black")
Y_E = Entry(PV,width=5,font=(10),bg="peachpuff",fg="black")
cm7 = Label(PV,text=("cm"),font=(10),bg="peachpuff2",fg="black")

Z = Label(PV,text=("Z = "),font=(10),bg="peachpuff2",fg="black")
Z_E = Entry(PV,width=5,font=(10),bg="peachpuff",fg="black")
cm8 = Label(PV,text=("cm"),font=(10),bg="peachpuff2",fg="black")

X.grid(row=0,column=0)
X_E.grid(row=0,column=1)
cm6.grid(row=0,column=2)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
cm7.grid(row=1,column=2)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
cm8.grid(row=2,column=2)

#display image
img = PhotoImage(file="MX3202_Company2_Cylindrical_Image.png")
img = img.subsample(1,1)
PI = Label(mygui,image=img, bg="bisque")
PI.grid(row=3,column=0)

mygui.mainloop()