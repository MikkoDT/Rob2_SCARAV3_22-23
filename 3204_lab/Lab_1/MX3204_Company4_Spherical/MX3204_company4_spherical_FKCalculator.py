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

#creating a GUI window with a title
mygui = Tk()
mygui.title("SCARAV3 Calculator")
mygui.resizable(True, True) #(width,hieght)
mygui.configure(bg="pink")

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
    # link lengths in cm
    a1 = float(a1_E.get())/100
    a2 = float(a2_E.get())/100
    a3 = float(a3_E.get())/100

    # joint variables: mm if d, degrees if theta
    T1 = float(T1_E.get())
    T2 = float(T2_E.get())
    d3 = float(d3_E.get())/100

    #degrees to radian 
    T1 = (T1/180.0)*np.pi
    T2 = (T2/180.0)*np.pi

    # parametric Table (Theta, alpha , R , d)
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
    X_E.delete(0,END)
    X_E.insert(0,np.around(X0_3*100,3))

    Y0_3 = H0_3[1,3]
    Y_E.delete(0,END)
    Y_E.insert(0,np.around(Y0_3*100,3))

    Z0_3 = H0_3[2,3]
    Z_E.delete(0,END)
    Z_E.insert(0,np.around(Z0_3*100,3))

        #Create links
    # [robot variable] = DHRobot([RevoluteDH(d,r,alpha,offset)])
    Spherical = DHRobot([
          
          RevoluteDH(a1,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
          RevoluteDH(0,0,(90.0/180.0)*np.pi,(90.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
          PrismaticDH(0,0,(0.0/180)*np.pi,a2+a3,qlim=[0,30/100])
          ],name='Spherical')

    #plot joints
    q1 = np.array([T1,T2,d3])

    #plot scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0.0
    z2 = 0.5

    #plot command 
    Spherical.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)

    #Link lenghts and joint variables Frame
FI = LabelFrame(mygui,text="Link Lengths and Joint Variables", font=(5),bg="#8A2BE2")
FI.grid(row=0, column=0)

#link lengths label 
a1 = Label(FI,text=("a1 = "),font=(10),bg="orange")
a1_E = Entry(FI,width=5,font=(10))
cm1 = Label(FI,text=("cm "),font=(10),bg="orange")

a2 = Label(FI,text=("a2 = "),font=(10),bg="orange")
a2_E = Entry(FI,width=5,font=(10))
cm2 = Label(FI,text=("cm "),font=(10),bg="orange")

a3 = Label(FI,text=("a3 = "),font=(10),bg="orange")
a3_E = Entry(FI,width=5,font=(10))
cm3 = Label(FI,text=("cm "),font=(10),bg="orange")



a1.grid(row=0,column=0)
a1_E.grid(row=0,column=1)
cm1.grid(row=0,column=2)

a2.grid(row=1,column=0)
a2_E.grid(row=1,column=1)
cm2.grid(row=1,column=2)

a3.grid(row=2,column=0)
a3_E.grid(row=2,column=1)
cm3.grid(row=2,column=2)



#joint variable labels 
T1 = Label(FI,text=("T1 = "),font=(10),bg="light blue")
T1_E = Entry(FI,width=5,font=(10))
deg1 = Label(FI,text=("deg "),font=(10),bg="light blue")

T2 = Label(FI,text=("T2 = "),font=(10),bg="light blue")
T2_E = Entry(FI,width=5,font=(10))
deg2 = Label(FI,text=("deg "),font=(10),bg="light blue")

d3 = Label(FI,text=("d1 = "),font=(10),bg="light blue")
d3_E = Entry(FI,width=5,font=(10))
cm4 = Label(FI,text=("cm "),font=(10),bg="light blue")

T1.grid(row=0,column=3)
T1_E.grid(row=0,column=4)
deg1.grid(row=0,column=5)

T2.grid(row=1,column=3)
T2_E.grid(row=1,column=4)
deg2.grid(row=1,column=5)

d3.grid(row=2,column=3)
d3_E.grid(row=2,column=4)
cm4.grid(row=2,column=5)

#button frame
BF = LabelFrame(mygui,text="Forward Kinematics", font=(5),bg="cyan")
BF.grid(row=1, column=0)

#buttons
FK = Button(BF, text="Forward",font=(10),bg="blue",fg="white",command=f_k) #bg is for bacground color, fg is for font color
rst = Button(BF, text="Reset",font=(10),bg="red",fg="white",command=reset)

FK.grid(row=0,column=0)
rst.grid(row=0,column=1)

#position vector frame
PV = LabelFrame(mygui,text="Position Vectors", font=(5),bg="cyan")
PV.grid(row=2, column=0)

#position vector label
X = Label(PV,text=("X = "),font=(10),bg="light green")
X_E = Entry(PV,width=5,font=(10))
cm5 = Label(PV,text=("cm "),font=(10),bg="light green")

Y = Label(PV,text=("Y = "),font=(10),bg="light green")
Y_E = Entry(PV,width=5,font=(10))
cm6 = Label(PV,text=("cm "),font=(10),bg="light green")

Z = Label(PV,text=("Z = "),font=(10),bg="light green")
Z_E = Entry(PV,width=5,font=(10))
cm7 = Label(PV,text=("cm "),font=(10),bg="light green")

X.grid(row=0,column=0)
X_E.grid(row=0,column=1)
cm5.grid(row=0,column=2)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
cm6.grid(row=1,column=2)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
cm7.grid(row=2,column=2)

#image display
img = PhotoImage(file="Spherical.png")
img = img.subsample(1,2)
PI = Label(mygui,image=img)
PI.grid(row=3,column=0)

mygui.mainloop()