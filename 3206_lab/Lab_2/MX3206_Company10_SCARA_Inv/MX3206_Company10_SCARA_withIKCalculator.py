from smtplib import LMTP, LMTP_PORT
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
mygui.title("LAB2_SCARA_CALCULATOR")
mygui.resizable(False,False)
mygui.configure(bg="#000000")

def reset():
    a1_E.delete(0, END)
    a2_E.delete(0, END)
    a3_E.delete(0, END)
    a4_E.delete(0, END)  
    a5_E.delete(0, END)

    T1_E.delete(0, END)
    T2_E.delete(0, END)
    d3_E.delete(0, END)

    X_E.delete(0, END)
    Y_E.delete(0, END)
    Z_E.delete(0, END)
    
def f_k():
    #  link length in mm
    a1 = float(a1_E.get())/100
    a2 = float(a2_E.get())/100
    a3 = float(a3_E.get())/100
    a4 = float(a4_E.get())/100
    a5 = float(a5_E.get())/100

    #joint variables: mm if d, degrees if theta
    T1 = float(T1_E.get())
    T2 = float(T2_E.get())
    d3 = float(d3_E.get())/100

    # degrees to radian
    T1 = (T1/180.0)*np.pi
    T2 = (T2/180.0)*np.pi

    # Parametric Table (theta,alpha,r,d)
    PT = [
      [T1,(0/180.0)*np.pi, a2, a1],
      [T2,(180.0/180.0)*np.pi, a4, a3],
      [0, 0, 0, a5+d3]]
    
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

    X0_E = H0_3[0,3]
    X_E.delete(0, END)
    X_E.insert(0,np.around(X0_E*100,3))

    Y0_E = H0_3[1,3]
    Y_E.delete(0, END)
    Y_E.insert(0,np.around(Y0_E*100,3))

    Z0_E = H0_3[2,3]
    Z_E.delete(0, END)
    Z_E.insert(0,np.around(Z0_E*100,3))

  # Create Links
    #   [robot_variable]=DHRobot([RevoluteDH(d,r,alpha,offset)])
    SCARA = DHRobot([
     PrismaticDH(0,0,(0/180.0)*np.pi,a1,qlim=[0,0]), 
     RevoluteDH(0,a2,(0/180.0)*np.pi,(0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
     PrismaticDH(0,0,(0/180.0)*np.pi,a3,qlim=[0,0]),
     RevoluteDH(0,a4,(180/180.0)*np.pi,(0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
     PrismaticDH(0,0,(0/180.0)*np.pi,a5,qlim=[0,(30/100)]),
    ], name="SCARA")

    #plot joints
    q1 = np.array([0,T1,0,T2,d3])
    
    #plot scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0
    z2 = 0.5

    #plot command
    SCARA.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)
    
def i_k():
    # Inverse Kinematics solution using graphical method

     #  link length in mm
     a1 = float(a1_E.get())
     a2 = float(a2_E.get())
     a3 = float(a3_E.get())
     a4 = float(a4_E.get())
     a5 = float(a5_E.get())

     # position vector in mm
     xe = float(X_E.get ())
     ye = float(Y_E.get ())
     ze = float(Z_E.get ())

     # to solve for Theta 2 or Th2.

     # Try and Except
     try:
          phi1 = np.arccos((a4**2 - r1**2 - a2**2)/(-2 * r1 * a2)) #3
     except:
          phi1 = -1 # NAN error
          messagebox.showerror(title="DevidedbyZero Error",message="Undefined solution if X=0")
     
     try:
          phi2 = np.arctan(ye/xe) #1
     except:
          phi2 = -1  # NAN error
          messagebox.showerror(title="DevidedbyZero Error",message="Undefined solution if X=0")

     try:
          phi3 = np.arccos((r1**2 - a2**2 - a4**2)/(-2 * a2 * a4)) #5
     except:
          phi3 = -1 # NAN error
          messagebox.showerror(title="DevidedbyZero Error",message="Undefined solution if X=0")

     phi2 = np.arctan(ye/xe) #1
     r1 = np.sqrt(ye**2 + xe**2) #2
     phi1 = np.arccos ((a4**2 - r1**2 - a2**2)/(-2 * r1 * a2)) #3
     Th1 = phi2 - phi1  #4

     # To solve for Theta2 or Th2
     phi3 = np.arccos((r1**2 - a2**2 - a4**2)/(-2 * a2 * a4)) #5
     Th2 = np.pi - phi3 #6

     # To solve for D3 or d sub 3
     D3 = a1 + a3 - a5 - ze #7

     T1_E.delete(0, END)
     T1_E.insert(0,np.around(Th1*180/np.pi,3))

     T2_E.delete(0, END)
     T2_E.insert(0,np.around(Th2*180/np.pi,3))

     d3_E.delete(0, END)
     d3_E.insert(0,np.around(D3,3))

    # Create Links
    #   [robot_variable]=DHRobot([RevoluteDH(d,r,alpha,offset)])
     SCARA = DHRobot([
          PrismaticDH(0,0,(0/180.0)*np.pi,a1/100,qlim=[0,0]), 
          RevoluteDH(0,a2/100,(0/180.0)*np.pi,(0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
          PrismaticDH(0,0,(0/180.0)*np.pi,a3/100,qlim=[0,0]),
          RevoluteDH(0,a4/100,(180/180.0)*np.pi,(0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
          PrismaticDH(0,0,(0/180.0)*np.pi,a5/100,qlim=[0,(30/100)]),
          ], name="SCARA")

    #plot joints
     q1 = np.array([0,Th1,0,Th2,D3/100])

    #plot scale
     x1 = -0.5
     x2 = 0.5
     y1 = -0.5
     y2 = 0.5
     z1 = 0
     z2 = 0.5

    #plot command
     SCARA.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)

#Link lenghts and Joint variables Frame
FI = LabelFrame(mygui,tex=" Link Lengths and Joint Variables", font=(1),bg="#14213D",fg="white")
FI.grid(row=0,column=0)

#Link lengths label
a1 = Label(FI,text=("a1 = "),font=(10),bg="#14213D",fg="white")
a1_E = Entry(FI,width=5,font=(10),bg="#FCA311",fg="black")
cm1 = Label(FI,text=("cm "),font=(10),bg="#14213D",fg="white")

a2 = Label(FI,text=("a2 = "),font=(10),bg="#14213D",fg="white")
a2_E = Entry(FI,width=5,font=(10),bg="#FCA311",fg="black")
cm2 = Label(FI,text=("cm "),font=(10),bg="#14213D",fg="white")

a3 = Label(FI,text=("a3 = "),font=(10),bg="#14213D",fg="white")
a3_E = Entry(FI,width=5,font=(10),bg="#FCA311",fg="black")
cm3 = Label(FI,text=("cm "),font=(10),bg="#14213D",fg="white")

a4 = Label(FI,text=("a4 = "),font=(10),bg="#14213D",fg="white")
a4_E = Entry(FI,width=5,font=(10),bg="#FCA311",fg="black")
cm4 = Label(FI,text=("cm "),font=(10),bg="#14213D",fg="white")

a5 = Label(FI,text=("a5 = "),font=(10),bg="#14213D",fg="white")
a5_E = Entry(FI,width=5,font=(10),bg="#FCA311",fg="black")
cm5 = Label(FI,text=("cm "),font=(10),bg="#14213D",fg="white")

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

#Joint Variables label
T1 = Label(FI,text=("T1 = "),font=(10),bg="#14213D",fg="white")
T1_E = Entry(FI,width=5,font=(10),bg="#FCA311",fg="black")
deg1 = Label(FI,text=("deg"),font=(10),bg="#14213D",fg="white")

T2 = Label(FI,text=("T2 = "),font=(10),bg="#14213D",fg="white")
T2_E = Entry(FI,width=5,font=(10),bg="#FCA311",fg="black")
deg2 = Label(FI,text=("deg"),font=(10),bg="#14213D",fg="white")

d3 = Label(FI,text=("d3 = "),font=(10),bg="#14213D",fg="white")
d3_E = Entry(FI,width=5,font=(10),bg="#FCA311",fg="black")
cm5 = Label(FI,text=("cm "),font=(10),bg="#14213D",fg="white")

T1.grid(row=0,column=3)
T1_E.grid(row=0,column=4)
deg1.grid(row=0,column=5)

T2.grid(row=1,column=3)
T2_E.grid(row=1,column=4)
deg2.grid(row=1,column=5)

d3.grid(row=2,column=3)
d3_E.grid(row=2,column=4)
cm5.grid(row=2,column=5)

#Button Frames
BF = LabelFrame(mygui,text="      Forward and Inverse Kinematics",font=(1),bg="#FCA311",fg="black")
BF.grid(row=1,column=0)

#Buttons
FK = Button(BF,text="↓ FORWARD",font=(10),bg="green",fg="white",command=f_k)
rst = Button(BF,text="RESET",font=(10),bg="red",fg="white",command=reset)
IK = Button(BF,text="↑ INVERSE",font=(10),bg="blue",fg="white",command=i_k)

FK.grid(row=0,column=0)
rst.grid(row=0,column=1)
IK.grid(row=0,column=2)

#Position Vectors Frame
PV = LabelFrame(mygui,text="Position Vector",font=(5),bg="#14213D",fg="white")
PV.grid(row=2,column=0)


#Position Vectors Label
X = Label(PV,text=("X = "),font=(10),bg="#14213D",fg="white")
X_E = Entry(PV,width=5,font=(10),bg="#FCA311",fg="black")
cm6 = Label(PV,text=("cm "),font=(10),bg="#14213D",fg="white")

Y = Label(PV,text=("Y = "),font=(10),bg="#14213D",fg="white")
Y_E = Entry(PV,width=5,font=(10),bg="#FCA311",fg="black")
cm7 = Label(PV,text=("cm "),font=(10),bg="#14213D",fg="white")

Z = Label(PV,text=("Z = "),font=(10),bg="#14213D",fg="white")
Z_E = Entry(PV,width=5,font=(10),bg="#FCA311",fg="black")
cm8 = Label(PV,text=("cm "),font=(10),bg="#14213D",fg="white")

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
img = PhotoImage(file="KINEMATIC DIAGRAM.png")
img = img.subsample(4,3)
PI = Label(mygui,image=img)
PI.grid(row=3,column=0)

mygui.mainloop()
