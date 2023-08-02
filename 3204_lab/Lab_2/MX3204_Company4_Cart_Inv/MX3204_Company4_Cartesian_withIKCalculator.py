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

# Creating a GUI window with a title
mygui = Tk()
mygui.title("CARTESIAN_04 Calculator")
mygui.resizable(False, False)
mygui.configure(bg="gray")

def reset():
    a1_E.delete(0,END)
    a2_E.delete(0,END)
    a3_E.delete(0,END)
    a4_E.delete(0,END)

    d1_E.delete(0,END)
    d2_E.delete(0,END)
    d3_E.delete(0,END)

    X_E.delete(0,END)
    Y_E.delete(0,END)
    Z_E.delete(0,END)

def f_k():
    # link lengths in cm
    a1 = float(a1_E.get())/100
    a2 = float(a2_E.get())/100
    a3 = float(a3_E.get())/100
    a4 = float(a4_E.get())/100

    # joints variables: mm if d, degrees if theta
    d1 = float(d1_E.get())/100
    d2 = float(d2_E.get())/100
    d3 = float(d3_E.get())/100

    # Parametric Table (theta, alpha, r, d)
    PT = [[(0.0/180.0)*np.pi,(270.0/180.0)*np.pi,0,a1],
          [(270.0/180.0)*np.pi,(270.0/180.0)*np.pi,0,a2+d1],
          [(90.0/180.0)*np.pi,(270.0/180.0)*np.pi,0,a3+d2],
          [(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,0,a4+d3]]

    # HTM Formulae
    i = 0
    H0_1 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
         [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
         [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
         [0,0,0,1]] 

    i=1
    H1_2 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
         [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
         [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
         [0,0,0,1]] 

    i=2
    H2_3 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
         [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
         [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
         [0,0,0,1]]

    i=3
    H3_4 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
         [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
         [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
         [0,0,0,1]]  
     
      
    H0_1 = np.matrix(H0_1) 
    H1_2 = np.matrix(H1_2)
    H2_3 = np.matrix(H2_3) 
    H3_4 = np.matrix(H2_3)

    H0_2 = np.dot(H0_1,H1_2)
    H0_3 = np.dot(H0_2,H2_3)
    H0_4 = np.dot(H0_3,H3_4)

    X0_4 = H0_4[0,3]
    X_E.delete(0,END)
    X_E.insert(0,np.around(X0_4*100,3))

    Y0_4 = H0_4[1,3]
    Y_E.delete(0,END)
    Y_E.insert(0,np.around(Y0_4*100,3))

    Z0_4 = H0_4[2,3]
    Z_E.delete(0,END)
    Z_E.insert(0,np.around(Z0_4*100,3))

    # Create links
    # [robot variable]=DHRobot([PrismaticDH(d,r,alpha,offset)])
    CARTESIAN_04 = DHRobot([
        PrismaticDH(0,0,(270.0/180.0)*np.pi,a1,qlim=[0,0]),
        PrismaticDH((270.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a2,qlim=[0,(30/100)]),
        PrismaticDH((90.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a3,qlim=[0,(30/100)]),
        PrismaticDH(0,0,(0.0/180.0)*np.pi,a4,qlim=[0,(30/100)])
        ], name = 'CARTESIAN_04')

    # plot joints
    q1 = np.array([0,d1,d2,d3])

    # plot scale
    x1 = -0.5
    x2 =  0.5
    y1 = -0.5
    y2 =  0.5
    z1 = -0.0
    z2 =  0.5

    # plot command
    CARTESIAN_04.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)# link lengths in mm


def i_k():
    # Inverse Kinematics Using Graphical Method

    # link lengths in cm
    a1 = float(a1_E.get())
    a2 = float(a2_E.get())
    a3 = float(a3_E.get())
    a4 = float(a4_E.get())

    # position vector in cm
    xe = float (X_E.get())
    ye = float (Y_E.get())
    ze = float (Z_E.get())

    # To solve for D1
    D2 = xe - a3 #1

    # To solve for D2
    D3 = a1 - a4 - ze #2

    # To solve for D3
    D1 = ye - a2 #3

    d1_E.delete(0,END)
    d1_E.insert(0,np.around(D1,3))

    d2_E.delete(0,END)
    d2_E.insert(0,np.around(D2,3))

    d3_E.delete(0,END)
    d3_E.insert(0,np.around(D3,3))

    # Create links
    # [robot variable]=DHRobot([PrismaticDH(d,r,alpha,offset)])
    CARTESIAN_04 = DHRobot([
        PrismaticDH(0,0,(270.0/180.0)*np.pi,a1/100,qlim=[0,0]),
        PrismaticDH((270.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a2/100,qlim=[0,(30/100)]),
        PrismaticDH((90.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a3/100,qlim=[0,(30/100)]),
        PrismaticDH(0,0,(0.0/180.0)*np.pi,a4/100,qlim=[0,(30/100)])
        ], name = 'CARTESIAN_04')

    # plot joints
    q1 = np.array([0,D1/100,D2/100,D3/100])

    # plot scale
    x1 = -0.5
    x2 =  0.5
    y1 = -0.5
    y2 =  0.5
    z1 =  0.0
    z2 =  0.5

     # plot command
    CARTESIAN_04.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)# link lengths in cm


#Link lengths and Joint variables Frame
FI = LabelFrame(mygui,text="Link Lengths and Joint Variables",font=(5), bg="gray")
FI.grid(row=0,column=0)

#Link lengths label
a1 = Label(FI,text=("a1 = "), font=(10),bg="light blue")
a1_E = Entry(FI,width=5,font=(10),bg="white")
cm1 = Label(FI,text=("cm "), font=(10),bg="light blue")

a2 = Label(FI,text=("a2 = "), font=(10),bg="light blue")
a2_E = Entry(FI,width=5,font=(10),bg="white")
cm2 = Label(FI,text=("cm "), font=(10),bg="light blue")

a3 = Label(FI,text=("a3 = "), font=(10),bg="light blue")
a3_E = Entry(FI,width=5,font=(10),bg="white")
cm3 = Label(FI,text=("cm "), font=(10),bg="light blue")

a4 = Label(FI,text=("a4 = "), font=(10),bg="light blue")
a4_E = Entry(FI,width=5,font=(10),bg="white")
cm4 = Label(FI,text=("cm "), font=(10),bg="light blue")

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

# Joint variable labels
d1 = Label(FI,text=("   d1 = "), font=(10),bg="light blue")
d1_E = Entry(FI,width=5,font=(10),bg="white")
cm5 = Label(FI,text=("cm "), font=(10),bg="light blue")

d2 = Label(FI,text=("   d2 = "), font=(10),bg="light blue")
d2_E = Entry(FI,width=5,font=(10),bg="white")
cm6 = Label(FI,text=("cm "), font=(10),bg="light blue")

d3 = Label(FI,text=("   d3 = "), font=(10),bg="light blue")
d3_E = Entry(FI,width=5,font=(10),bg="white")
cm7 = Label(FI,text=("cm "), font=(10),bg="light blue")

d1.grid(row=0,column=3)
d1_E.grid(row=0,column=4)
cm5.grid(row=0,column=5)

d2.grid(row=1,column=3)
d2_E.grid(row=1,column=4)
cm6.grid(row=1,column=5)

d3.grid(row=2,column=3)
d3_E.grid(row=2,column=4)
cm7.grid(row=2,column=5)

# Button Frame
BF = LabelFrame(mygui, text="    Forward and Inverse Kinematics", font=(5),bg="light blue")
BF.grid(row=1, column=0)

#Buttons
FK = Button(BF,text="↓FORWARD",font=(10),bg="blue",fg="white",command=f_k)
rst= Button(BF,text="RESET",font=(10),bg="red",fg="white",command=reset)
IK = Button(BF,text="↑INVERSE",font=(10),bg="blue",fg="white", command=i_k)

FK.grid(row=0,column=0)
rst.grid(row=0,column=1)
IK.grid(row=0,column=2)


# Position Vector Frame
PV = LabelFrame(mygui, text="Position Vectors", font=(5),bg="light blue")
PV.grid(row=2, column=0)

#Position Vector Labels
X = Label(PV,text=("X = "), font=(10),bg="light blue")
X_E = Entry(PV,width=5,font=(10),bg="white")
cm6 = Label(PV,text=("cm "), font=(10),bg="light blue")

Y = Label(PV,text=("Y = "), font=(10),bg="light blue")
Y_E = Entry(PV,width=5,font=(10),bg="white")
cm7 = Label(PV,text=("cm "), font=(10),bg="light blue")

Z = Label(PV,text=("Z = "), font=(10),bg="light blue")
Z_E = Entry(PV,width=5,font=(10),bg="white")
cm8 = Label(PV,text=("cm "), font=(10),bg="light blue")

X.grid(row=0,column=0,)
X_E.grid(row=0,column=1)
cm6.grid(row=0,column=2)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
cm7.grid(row=1,column=2)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
cm8.grid(row=2,column=2)

# display image
img = PhotoImage(file='Cartesian_Image.png')
img = img.subsample(2,2)
PI = Label(mygui,image=img)
PI.grid(row=3,column=0)

mygui.mainloop() 