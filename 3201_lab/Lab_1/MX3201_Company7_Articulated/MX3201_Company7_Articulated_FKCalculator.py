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
MX3201_Company7_Articulated_FKCalculator = Tk()
MX3201_Company7_Articulated_FKCalculator.title("Articulated Calculator")
MX3201_Company7_Articulated_FKCalculator.resizable(False,False)
MX3201_Company7_Articulated_FKCalculator.configure(bg="light gray")

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
    MX3201_Company7_Articulated_FKCalculator = DHRobot([
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
    MX3201_Company7_Articulated_FKCalculator.plot(q1,limits=[X1,X2,Y1,Y2,Z1,Z2],block=True)


# Link lengths and Joint Variables Frame
FI = LabelFrame(MX3201_Company7_Articulated_FKCalculator,text="Link Lengths and Joint Variables",font=(5),bg="gray",fg="light blue")
FI.grid(row=0,column=0)

# Link lenghts label
a1 = Label(FI,text="a1 = ",font=(10),bg="light gray",fg="blue")
a1_E = Entry(FI,width=5,font=(10),bg="white")
cm1 = Label(FI,text="cm",font=(10),bg="white",fg="blue")

a2 = Label(FI,text="a2 = ",font=(10),bg="light gray",fg="blue")
a2_E = Entry(FI,width=5,font=(10),bg="white")
cm2 = Label(FI,text="cm",font=(10),bg="white",fg="blue")

a3 = Label(FI,text="a3 = ",font=(10),bg="light gray",fg="blue")
a3_E = Entry(FI,width=5,font=(10),bg="white")
cm3 = Label(FI,text="cm",font=(10),bg="white",fg="blue")

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
T1 = Label(FI,text="T1 = ",font=(10),bg="light gray",fg="blue")
T1_E = Entry(FI,width=5,font=(10),bg="white")
deg1 = Label(FI,text="deg",font=(10),bg="white",fg="blue")

T2 = Label(FI,text="T2 = ",font=(10),bg="light gray",fg="blue")
T2_E = Entry(FI,width=5,font=(10),bg="white")
deg2 = Label(FI,text="deg",font=(10),bg="white",fg="blue")

T3 = Label(FI,text="T3 = ",font=(10),bg="light gray",fg="blue")
T3_E = Entry(FI,width=5,font=(10),bg="white")
deg3 = Label(FI,text="deg",font=(10),bg="white",fg="blue")

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
BF = LabelFrame(MX3201_Company7_Articulated_FKCalculator,text="Forward Kinematics",font=(5),bg="gray",fg="light blue")
BF.grid(row=1,column=0)

# Buttons
FK = Button(BF,text="Forward",font=(10),bg="blue",fg="white",command=f_k)
rst = Button(BF,text="RESET",font=(10),bg="red",fg="white",command=reset)

FK.grid(row=0,column=0)
rst.grid(row=0,column=1)

# Position Vectors Frame
PV = LabelFrame(MX3201_Company7_Articulated_FKCalculator,text="Position Vectors",font=(5),bg="gray",fg="light blue")
PV.grid(row=2,column=0)

# Position Vectos label
X = Label(PV,text="X = ",font=(10),fg="blue",bg="light gray")
X_E = Entry(PV,width=5,font=(10))
cm4 = Label(PV,text="cm",font=(10),fg="blue",bg="white")

Y = Label(PV,text="Y = ",font=(10),fg="blue",bg="light gray")
Y_E = Entry(PV,width=5,font=(10))
cm5 = Label(PV,text="cm",font=(10),fg="blue",bg="white")

Z = Label(PV,text="Z = ",font=(10),fg="blue",bg="light gray")
Z_E = Entry(PV,width=5,font=(10))
cm6 = Label(PV,text="cm",font=(10),fg="blue",bg="white")

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
img = PhotoImage(file="image.PNG")
img = img.subsample(1,1)
PI = Label(MX3201_Company7_Articulated_FKCalculator, image=img,bg="gray")
PI.grid(row=3,column=0)


MX3201_Company7_Articulated_FKCalculator.mainloop()