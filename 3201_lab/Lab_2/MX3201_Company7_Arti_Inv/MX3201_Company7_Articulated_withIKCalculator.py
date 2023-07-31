from tkinter import *
from tkinter import messagebox
from tkinter import PhotoImage
import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import spatialmath
from spatialmath import SE3
import matplotlib
matplotlib.use('TkAgg')

# Create a GUI window with a title 
MX3201_Company7_Articulated_IKCalculator = Tk()
MX3201_Company7_Articulated_IKCalculator.title("Articulated Calculator")
MX3201_Company7_Articulated_IKCalculator.resizable(False,False)
MX3201_Company7_Articulated_IKCalculator.configure(bg="thistle")

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
    # Link Lengths in mm
    a1 = float(a1_E.get())/100 
    a2 = float(a2_E.get())/100  
    a3 = float(a3_E.get())/100 

    T1 = float(T1_E.get())
    T2 = float(T2_E.get())
    T3 = float(T3_E.get())
   
   # degrees to radian
    T1 = (T1/180.0)*np.pi
    T2 = (T2/180.0)*np.pi
    T3 = (T3/180.0)*np.pi

    # Parametric table (Theta, alpha, r, d)
    PT = [[T1,(90.0/180.0)*np.pi,0,a1],
      [T2,(0.0/180.0)*np.pi,a2,0],
      [T3,(0.0/180.0)*np.pi,a3,0]]

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

    # Create links
    # [robot_variable]=DHRobot([RevoluteDH(d,r,alpha,offset)])
    MX3201_Company7_Articulated_IKCalculator = DHRobot([
        RevoluteDH(a1,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,a2,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,a3,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        ], name="Articulated")
    
    #plot joints
    q1 = np.array([T1,T2,T3])

    #plot scale
    X1 = -0.5
    X2 = 0.5
    Y1 = -0.5
    Y2 = 0.5
    Z1 = 0.0
    Z2 = 0.5

    #plot command
    MX3201_Company7_Articulated_IKCalculator.plot(q1,limits=[X1,X2,Y1,Y2,Z1,Z2],block=True)

def i_k():
    # Inverse Kinematics using Graphical Method

    # Link Lengths in cm
    a1 = float(a1_E.get()) 
    a2 = float(a2_E.get()) 
    a3 = float(a3_E.get()) 

    # Position Vector in cm
    xe = float(X_E.get()) 
    ye = float(Y_E.get()) 
    ze = float(Z_E.get()) 

    # To solve for theta 1 or th1
    if xe == 0:
      Th1 = np.pi/2 if ye > 0 else -np.pi/2
    else:
      Th1 = np.arctan(ye/xe) #1

    # To solve for theta 2 or th2
    r1 = np.sqrt(ye**2 + xe**2) #2
    r2 = ze - a1 #3

    if r1 == 0:
      phi1 = np.pi/2 if r2 > 0 else -np.pi/2
    else:
      phi1 = np.arctan(r2/r1) #4


    r3 = np.sqrt(r2**2 + r1**2) #5
    phi2 = np.arccos(np.clip((a3**2 - a2**2 - r3**2)/(-2*a2*r3), -1, 1)) #6
    Th2 = phi1 + phi2 #7

    # To solve for theta 3 or th3
    phi3 = np.arccos(np.clip((r3**2 - a2**2 - a3**2)/(-2*a2*a3), -1, 1)) #8
    Th3 = phi3 - np.pi #9

    T1_E.delete(0,END)
    T1_E.insert(0,np.around(Th1*180/np.pi,3))

    T2_E.delete(0,END)
    T2_E.insert(0,np.around(Th2*180/np.pi,3))

    T3_E.delete(0,END)
    T3_E.insert(0,np.around(Th3*180/np.pi,3))

    # Create links
    # [robot_variable]=DHRobot([RevoluteDH(d,r,alpha,offset)])
    MX3201_Company7_Articulated_IKCalculator = DHRobot([
        RevoluteDH(a1/100,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,a2/100,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,a3/100,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        ], name="Articulated")
    
    #plot joints
    q1 = np.array([Th1,Th2,Th3])

    #plot scale
    X1 = -0.5
    X2 = 0.5
    Y1 = -0.5
    Y2 = 0.5
    Z1 = 0.0
    Z2 = 0.5

    #plot command
    MX3201_Company7_Articulated_IKCalculator.plot(q1,limits=[X1,X2,Y1,Y2,Z1,Z2],block=True)

# Link lengths and Joint Variables Frame
FI = LabelFrame(MX3201_Company7_Articulated_IKCalculator,text="Link Lengths and Joint Variables",font=(5),bg="#313866",fg="#FF7BBF")
FI.grid(row=0,column=0)

# Link lenghts label
a1 = Label(FI,text="a1 = ",font=(10),bg="#964EC2",fg="black")
a1_E = Entry(FI,width=5,font=(10),bg="white")
cm1 = Label(FI,text="cm",font=(10),bg="#964EC2",fg="black")

a2 = Label(FI,text="a2 = ",font=(10),bg="#964EC2",fg="black")
a2_E = Entry(FI,width=5,font=(10),bg="white")
cm2 = Label(FI,text="cm",font=(10),bg="#964EC2",fg="black")

a3 = Label(FI,text="a3 = ",font=(10),bg="#964EC2",fg="black")
a3_E = Entry(FI,width=5,font=(10),bg="white")
cm3 = Label(FI,text="cm",font=(10),bg="#964EC2",fg="black")

a1.grid(row=0,column=0)
a1_E.grid(row=0,column=1)
cm1.grid(row=0,column=2)

a2.grid(row=1,column=0)
a2_E.grid(row=1,column=1)
cm2.grid(row=1,column=2)

a3.grid(row=2,column=0)
a3_E.grid(row=2,column=1)
cm3.grid(row=2,column=2)

# Joint Variable label
T1 = Label(FI,text="T1 = ",font=(10),bg="#964EC2",fg="black")
T1_E = Entry(FI,width=5,font=(10),bg="white")
deg1 = Label(FI,text="deg",font=(10),bg="#964EC2",fg="black")

T2 = Label(FI,text="T2 = ",font=(10),bg="#964EC2",fg="black")
T2_E = Entry(FI,width=5,font=(10),bg="white")
deg2 = Label(FI,text="deg",font=(10),bg="#964EC2",fg="black")

T3 = Label(FI,text="T3 = ",font=(10),bg="#964EC2",fg="black")
T3_E = Entry(FI,width=5,font=(10),bg="white")
deg3 = Label(FI,text="deg",font=(10),bg="#964EC2",fg="black")

T1.grid(row=0,column=3)
T1_E.grid(row=0,column=4)
deg1.grid(row=0,column=5)

T2.grid(row=1,column=3)
T2_E.grid(row=1,column=4)
deg2.grid(row=1,column=5)

T3.grid(row=2,column=3)
T3_E.grid(row=2,column=4)
deg3.grid(row=2,column=5)

# Button Frame
BF = LabelFrame(MX3201_Company7_Articulated_IKCalculator,text="Forward Kinematics",font=(5),bg="#313866",fg="#FF7BBF")
BF.grid(row=1,column=0)

# Buttons
FK = Button(BF,text="↓Forward",font=(10),bg="blue",fg="black",command=f_k)
rst = Button(BF,text="RESET",font=(10),bg="red",fg="black",command=reset)
IK = Button(BF,text="↑Inverse",font=(10),bg="blue",fg="black",command=i_k)


FK.grid(row=0,column=0)
rst.grid(row=0,column=1)
IK.grid(row=0,column=2)

# Position Vectors Frame
PV = LabelFrame(MX3201_Company7_Articulated_IKCalculator,text="Position Vectors",font=(5),bg="#313866",fg="#FF7BBF")
PV.grid(row=2,column=0)

# Position Vectos label
X = Label(PV,text="X = ",font=(10),fg="black",bg="#964EC2")
X_E = Entry(PV,width=5,font=(10))
cm4 = Label(PV,text="cm",font=(10),fg="black",bg="#964EC2")

Y = Label(PV,text="Y = ",font=(10),fg="black",bg="#964EC2")
Y_E = Entry(PV,width=5,font=(10))
cm5 = Label(PV,text="cm",font=(10),fg="black",bg="#964EC2")

Z = Label(PV,text="Z = ",font=(10),fg="black",bg="#964EC2")
Z_E = Entry(PV,width=5,font=(10))
cm6 = Label(PV,text="cm",font=(10),fg="black",bg="#964EC2")

X.grid(row=0,column=0)
X_E.grid(row=0,column=1)
cm4.grid(row=0,column=2)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
cm5.grid(row=1,column=2)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
cm6.grid(row=2,column=2)

#display image
img = PhotoImage(file="image.png")
img = img.subsample(2,2)
PI = Label(MX3201_Company7_Articulated_IKCalculator, image=img,bg="thistle")
PI.grid(row=3,column=0)


MX3201_Company7_Articulated_IKCalculator.mainloop()