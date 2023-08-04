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
matplotlib .use('TkAgg')

#Window and Title
mygui = Tk()
mygui.title("Articulated Calculator")
mygui.resizable(False,False)
mygui.configure(bg="Maroon")

def reset():
    a1_E.delete(0, END)
    a2_E.delete(0, END)
    a3_E.delete(0, END)

    T1_E.delete(0, END)
    T2_E.delete(0, END)
    T3_E.delete(0, END)

    X_E.delete(0, END)
    Y_E.delete(0, END)
    Z_E.delete(0, END)

def f_k():
    # link lengths in cm
    a1 = float(a1_E.get())/100
    a2 = float(a2_E.get())/100
    a3 = float(a3_E.get())/100

    # joint variables: is cm if f, is degrees if theta
    T1 = float(T1_E.get())
    T2 = float(T2_E.get())
    T3 = float(T3_E.get())

    # degree to radian
    T1 = (float(T1)/180.0)*np.pi
    T2 = (float(T2)/180.0)*np.pi
    T3 = (float(T3)/180.0)*np.pi

    # Parametric table (theta,alpha,r,d)
    PT = [[float(T1),(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,0,a1],
          [float(T2),(0.0/180.0)*np.pi,a2,0],
          [float(T3),(0.0/180.0)*np.pi,a3,0]]
    
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
    X_E.delete(0,END)
    X_E.insert(0,np.around(X0_3*100,3))

    Y0_3 = H0_3[1,3]
    Y_E.delete(0,END)
    Y_E.insert(0,np.around(Y0_3*100,3))

    Z0_3 = H0_3[2,3]
    Z_E.delete(0,END)
    Z_E.insert(0,np.around(Z0_3*100,3))

    # Create links
    # [robot_variable]=DHRobot([RevoluteDH(d,r,alpha,offset)])
    Articulated = DHRobot([
        RevoluteDH(a1,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,a2,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,a3,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2])
        ], name="Articulated")

    #plot joints
    q1 = np.array([T1,T2,T3])

    #plot scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0.0
    z2 = 0.5

    #Plot command
    Articulated.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)

def i_k():
    #Inverse Kinematics Using Graphical Method

    #link lengths in cm
    a1 = float(a1_E.get())
    a2 = float(a2_E.get())
    a3 = float(a3_E.get())

    #Position Vector in cm
    xe = float(X_E.get())
    ye = float(Y_E.get())
    ze = float(Z_E.get())

    #Try and Except
    try:
        phi1 = np.arctan((r2)/(r1)) #4
    except:
        phi1 = -1 # NAN error
        messagebox.showerror(title="DevidedbyZero Error",message="Undefined solution if X=0")
    
    try:
        phi2 = np.arccos((a3**2 - a2**2 -r3**2)/(-2 * a2 * r3)) #6
    except:
        phi2 = -1 # NAN error
        messagebox.showerror(title="DevidedbyZero Error",message="Undefined solution if X=0")
    
    try:
        phi3 = np.arccos((a3**2 - a2**2 - r3**2)/(-2 * a2 * a3)) #8
    except:
        phi3 = -1 # NAN error
        messagebox.showerror(title="DevidedbyZero Error",message="Undefined solution if X=0")

    # To solve for Theta 1 or Th1
    Th1 = np.arctan(ye/xe) #1
    r1 = np.sqrt(ye**2 + xe**2) #2
    r2 = (ze - a1) #3 
    phi1 = np.arctan((r2)/(r1)) #4
    r3 = np.sqrt (r2**2 + r1**2) #5

    # To solve for Theta 2 or Th2
    phi2 = np.arccos((a3**2 - a2**2 -r3**2)/(-2 * a2 * r3)) #6
    Th2 = phi1 + phi2 #7

    # To solve for Theta 3 or Th3 
    phi3 = np.arccos((a3**2 - a2**2 - r3**2)/(-2 * a2 * a3)) #8
    Th3 = phi3 - np.pi #9

    T1_E.delete(0,END)
    T1_E.insert(0,np.around(Th1*180/np.pi,3))

    T2_E.delete(0,END)
    T2_E.insert(0,np.around(Th2*180/np.pi,3))

    T3_E.delete(0,END)
    T3_E.insert(0,np.around(Th3*180/np.pi,3))

    # Create links
    # [robot_variable]=DHRobot([RevoluteDH(d,r,alpha,offset)])
    Articulated = DHRobot([
        RevoluteDH(a1/100,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,a2/100,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,a3/100,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2])
        ], name="Articulated")

    #plot joints
    q1 = np.array([Th1,Th2,Th3])

    #plot scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0.0
    z2 = 0.5

    #Plot command
    Articulated.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)


#Link lengths and Joint Variable Frame
FI = LabelFrame(mygui,text="Link Lengths and Joint Variables",font=(5))
FI.grid(row=0,column=0)

#Link lengths label
a1 = Label(FI,text=("a1 = "),font=(10))
a1_E = Entry(FI,width=5,font=(10))
cm1 = Label(FI,text=("cm"),font=(10))

a2 = Label(FI,text=("a2 = "),font=(10))
a2_E = Entry(FI,width=5,font=(10))
cm2 = Label(FI,text=("cm"),font=(10))

a3 = Label(FI,text=("a3 = "),font=(10))
a3_E = Entry(FI,width=5,font=(10))
cm3 = Label(FI,text=("cm"),font=(10))

a1.grid(row=0,column=0)
a1_E.grid(row=0,column=1)
cm1.grid(row=0,column=2)

a2.grid(row=1,column=0)
a2_E.grid(row=1,column=1)
cm2.grid(row=1,column=2)

a3.grid(row=2,column=0)
a3_E.grid(row=2,column=1)
cm3.grid(row=2,column=2)

#Joint Variable labels
T1 = Label(FI,text=(" T1 = "),font=(10))
T1_E = Entry(FI,width=5,font=(10))
deg1 = Label(FI,text=("deg"),font=(10))

T2 = Label(FI,text=(" T2 = "),font=(10))
T2_E = Entry(FI,width=5,font=(10))
deg2 = Label(FI,text=("deg"),font=(10))

T3 = Label(FI,text=(" T3 = "),font=(10))
T3_E = Entry(FI,width=5,font=(10))
deg3 = Label(FI,text=("deg"),font=(10))

T1.grid(row=0,column=3)
T1_E.grid(row=0,column=4)
deg1.grid(row=0,column=5)

T2.grid(row=1,column=3)
T2_E.grid(row=1,column=4)
deg2.grid(row=1,column=5)

T3.grid(row=2,column=3)
T3_E.grid(row=2,column=4)
deg3.grid(row=2,column=5)

# Buttons Frame
BF = LabelFrame(mygui,text="Forward Kinematics",font=(5))
BF.grid(row=1,column=0)

#Buttons
FK = Button(BF,text="↓ Forward",font=(10),bg="orange",fg="black",command=f_k)
rst = Button(BF,text="Reset",font=(10),bg="red",fg="black",command=reset)
IK = Button(BF,text="↑ Inverse",font=(10),bg="green",fg="black",command=i_k)

FK.grid(row=0,column=0)
rst.grid(row=0,column=1)
IK.grid(row=0,column=2)

# Position Vector Frame
PV = LabelFrame(mygui,text="Position Vectors",font=(5))
PV.grid(row=2,column=0)

#Position Vector labels
X = Label(PV,text=("X = "),font=(10))
X_E = Entry(PV,width=5,font=(10))
deg4 = Label(PV,text=("cm"),font=(10))

Y = Label(PV,text=("Y = "),font=(10))
Y_E = Entry(PV,width=5,font=(10))
deg5 = Label(PV,text=("cm"),font=(10))

Z = Label(PV,text=("Z = "),font=(10))
Z_E = Entry(PV,width=5,font=(10))
deg6 = Label(PV,text=("cm"),font=(10))

X.grid(row=0,column=0)
X_E.grid(row=0,column=1)
deg4.grid(row=0,column=2)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
deg5.grid(row=1,column=2)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
deg6.grid(row=2,column=2)

#display image
img = PhotoImage(file="Articulated.png")
img = img.subsample(3,3)
PI = Label(mygui,image=img)
PI.grid(row=3,column=0)

mygui.mainloop()