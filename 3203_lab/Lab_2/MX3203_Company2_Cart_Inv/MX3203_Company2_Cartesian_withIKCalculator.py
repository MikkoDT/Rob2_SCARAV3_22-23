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

# Create GUI window title
mygui = Tk()
mygui.title("CARTESIANC2_03 CALCULATOR")
mygui.resizable(True,True)
mygui.configure(bg="pink")

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
    a1 = float(a1_E.get())/100
    a2 = float(a2_E.get())/100
    a3 = float(a3_E.get())/100
    a4 = float(a4_E.get())/100

    # joint variables: if d mm, if Theta degrees
    d1 = float(d1_E.get())/100
    d2 = float(d2_E.get())/100
    d3 = float(d3_E.get())/100

    # Parametric Table (Theta, alpha, r, d)
    PT = [[(0.0/180) * np.pi, (270.0/180.0) * np.pi ,0, a1],
            [(270.0/180.0) * np.pi, (270.0/180) * np.pi, 0, a2+d1],
            [(90.0/180.0) * np.pi, (270.0/180) * np.pi, 0, a3+d2],
            [(0.0/180) * np.pi, (0.0/180) * np.pi , 0, a4+d3]]
    
    # HTM formula
    i = 0
    H0_1 = [[np.cos(PT[i][0]), -np.sin(PT[i][0])*np.cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.cos(PT[i][0])], 
            [np.sin(PT[i][0]), np.cos(PT[i][0])*np.cos(PT[i][1]), -np.cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
            [0, np.sin(PT[i][1]),np.cos(PT[i][1]), PT[i][3]],
            [0,0,0,1]]

    i = 1
    H1_2 = [[np.cos(PT[i][0]), -np.sin(PT[i][0])*np.cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.cos(PT[i][0])], 
            [np.sin(PT[i][0]), np.cos(PT[i][0])*np.cos(PT[i][1]), -np.cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
            [0, np.sin(PT[i][1]),np.cos(PT[i][1]), PT[i][3]],
            [0,0,0,1]]

    i = 2
    H2_3 = [[np.cos(PT[i][0]), -np.sin(PT[i][0])*np.cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.cos(PT[i][0])], 
            [np.sin(PT[i][0]), np.cos(PT[i][0])*np.cos(PT[i][1]), -np.cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
            [0, np.sin(PT[i][1]),np.cos(PT[i][1]), PT[i][3]],
            [0,0,0,1]]

    i = 3
    H3_4 = [[np.cos(PT[i][0]), -np.sin(PT[i][0])*np.cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.cos(PT[i][0])], 
            [np.sin(PT[i][0]), np.cos(PT[i][0])*np.cos(PT[i][1]), -np.cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
            [0, np.sin(PT[i][1]),np.cos(PT[i][1]), PT[i][3]],
            [0,0,0,1]]

    H0_1 = np.matrix(H0_1)
    H1_2 = np.matrix(H1_2)
   
    H2_3 = np.matrix(H2_3)
    H3_4 = np.matrix(H3_4)

    H0_2 = np.dot(H0_1,H1_2)
    H0_3 = np.dot(H0_2,H2_3)
    H0_4 = np.dot(H0_3,H3_4)


    X_04 = H0_4[0,3]
    X_E.delete(0,END)
    X_E.insert(0,np.around(X_04*100,3))

    Y_04 = H0_4[1,3]
    Y_E.delete(0,END)
    Y_E.insert(0,np.around(Y_04*100,3))

    Z_04 = H0_4[2,3]
    Z_E.delete(0,END)
    Z_E.insert(0,np.around(Z_04*100,3))


# create links
# [robot_variable]=DHRobot(d, r, alpha, offset)])
    CARTESIANC2_03 = DHRobot ([ 
        PrismaticDH(0,0,(270.0/180.0) * np.pi,a1,qlim=[0,0]),
        PrismaticDH((270.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a2,qlim=[0, d1]),
        PrismaticDH((90.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a3,qlim=[0, d2]),
        PrismaticDH(0,0,(0.0/180.0)*np.pi,a4,qlim=[0, d3])],
        name= 'CARTESIANC2_03' )
    
# plot joints
    q1 = np.array([0,d1,d2,d3])

# plot scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0
    z2 = 0.5

# plot command
    CARTESIANC2_03.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)

def i_k():
    #Inverse Kinematics Using Graphical method

    #Link lengths in cm
    a1 = float(a1_E.get())
    a2 = float(a2_E.get())
    a3 = float(a3_E.get())
    a4 = float(a4_E.get())

    #Position Vector
    xe = float(X_E.get())
    ye = float(Y_E.get())
    ze = float(Z_E.get())

    #To solve for d1, d2, d3
    
    #solving for d2
    d2 = xe - a3  #1

    #solving for d3
    d3 = a1 - a4 -ze  #2

    #solving for d1
    d1 = ye - a2  #3

    d1_E.delete(0, END)
    d1_E.insert(0, np.around(d1, 3))

    d2_E.delete(0, END)
    d2_E.insert(0, np.around(d2, 3))

    d3_E.delete(0, END)
    d3_E.insert(0, np.around(d3, 3))

    # [robot_variable]=DHRobot(d, r, alpha, offset)])
    CARTESIANC2_03 = DHRobot ([ 
    PrismaticDH(0,0,(270.0/180.0) * np.pi,a1/100,qlim=[0,0]),
    PrismaticDH((270.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a2/100,qlim=[0, d1]),
    PrismaticDH((90.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a3/100,qlim=[0, d2]),
    PrismaticDH(0,0,(0.0/180.0)*np.pi,a4/100,qlim=[0, d3])],
    name= 'CARTESIANC2_03' )

    # plot joints
    q1 = np.array([0,d1/100,d2/100,d3/100])

    # plot scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0
    z2 = 0.5

    #plot command
    CARTESIANC2_03.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)

# Link lengths and Joint variable frame
FI = LabelFrame(mygui,text="Link Lengths and Joint Variables",font=(5),fg="black", bg="gray")
FI.grid(row=0,column=0)

# Link lenghts label
a1 = Label(FI,text=("a1 = "),font=(10))
a1_E = Entry(FI,width=5,font=(10))
cm1 = Label(FI,text=("cm "),font=(10))

a2 = Label(FI,text=("a2 = "),font=(10))
a2_E = Entry(FI,width=5,font=(10))
cm2 = Label(FI,text=("cm "),font=(10))

a3 = Label(FI,text=("a3 = "),font=(10))
a3_E = Entry(FI,width=5,font=(10))
cm3 = Label(FI,text=("cm "),font=(10))

a4 = Label(FI,text=("a4 = "),font=(10))
a4_E = Entry(FI,width=5,font=(10))
cm4 = Label(FI,text=("cm "),font=(10))

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
d1 = Label(FI,text=("d1 = "),font=(10))
d1_E = Entry(FI,width=5,font=(10))
cm5 = Label(FI,text=("cm "),font=(10))

d2 = Label(FI,text=("d2 = "),font=(10))
d2_E = Entry(FI,width=5,font=(10))
cm6 = Label(FI,text=("cm "),font=(10))

d3 = Label(FI,text=("d3 = "),font=(10))
d3_E = Entry(FI,width=5,font=(10))
cm7 = Label(FI,text=("cm "),font=(10))

d1.grid(row=0,column=3)
d1_E.grid(row=0,column=4)
cm5.grid(row=0,column=5)

d2.grid(row=1,column=3)
d2_E.grid(row=1,column=4)
cm6.grid(row=1,column=5)

d3.grid(row=2,column=3)
d3_E.grid(row=2,column=4)
cm7.grid(row=2,column=5)

# Buttons Frame
BF = LabelFrame(mygui,text="Forward and Inverse Kinematics",font=(5), fg="black", bg="gray")
BF.grid(row=1,column=0)

# Buttons
FK = Button(BF,text="↑ Forward",font=(10),bg="red", fg="black",command=f_k)
rst = Button(BF,text="↻ RESET",font=(10),bg="black",fg="white",command=reset)
IK = Button(BF,text="↓ Inverse",font=(10),bg="gold", fg="black",command=i_k)

FK.grid(row=0, column=0)
rst.grid(row=0, column=1)
IK.grid(row=0, column=2)

# Position Vectors Frame
PV = LabelFrame(mygui,text="Position Vector", font=(5), bg="teal")
PV.grid(row=2,column=0)

# Position Vector Label
X = Label(PV,text=("X = "),font=(10))
X_E = Entry(PV,width=5, font=(10))
cm8 = Label(PV,text=("cm "),font=(10))

Y = Label(PV,text=("Y = "),font=(10))
Y_E = Entry(PV,width=5, font=(10))
cm9 = Label(PV,text=("cm "),font=(10))

Z = Label(PV,text=("Z = "),font=(10))
Z_E = Entry(PV,width=5, font=(10))
cm10 = Label(PV,text=("cm "),font=(10))

X.grid(row=0,column=0)
X_E.grid(row=0,column=1)
cm8.grid(row=0,column=2)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
cm9.grid(row=1,column=2)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
cm10.grid(row=2,column=2)

# Insert Image
img = PhotoImage(file="cartesian.png")
img = img.subsample(3,4)
PI = Label(mygui,image=img)
PI.grid(row=3, column=0)

mygui.mainloop()



