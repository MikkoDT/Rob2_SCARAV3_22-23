from tkinter import * #* to access the library of tkinter
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
mygui.resizable(True, True) #(width,height)
mygui.configure(bg="black")

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
    # link lengths in cm
    a1 = float(a1_E.get())/100
    a2 = float(a2_E.get())/100
    a3 = float(a3_E.get())/100
    a4 = float(a4_E.get())/100
    a5 = float(a5_E.get())/100

    # joint variables: mm if d, degrees if theta
    T1 = float(T1_E.get())
    T2 = float(T2_E.get())
    d3 = float(d3_E.get())/100

    # parametric Table (Theta, alpha , R , d)
    PT =[[T1,(0.0/180.0)*np.pi,a2,a1],
        [T2,(180.0/180.0)*np.pi,a4,a3],
        [(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,0,a5+d3]]
    
    
    # HTM Formulae
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
    #[robot variable] = DHRobot([RevoluteDH(d,r,alpha,offset)])
    SCARA = DHRobot([
        RevoluteDH(a1,0,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        PrismaticDH(0,a2,0,0,qlim=[0,0]),
        RevoluteDH(a3,0,(180.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        PrismaticDH(0,a4,0,0,qlim=[0,0]),
        PrismaticDH((0.0/180.0)*np.pi,0,(0.0/180.0)*np.pi,a5,qlim=[0,30/100])
          ],name='SCARA_V3')

    #plot joints
    q1 = np.array([T1,0,T2,0,d3])

    #plot scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0.0
    z2 = 0.5

    # plot command
    SCARA.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block = True)
# Link lengths and joint variables Frame
FI = LabelFrame(mygui,text="Link Lengths and Joint Variables")
FI.grid(row=0, column=0)

# Link lengths label
a1 = Label(FI,text=("a1= "),font=(10),bg="white")
a1_E = Entry(FI,width=5,font=(10))
cm1 = Label(FI,text=("cm"),font=(10),bg="white")

a2 = Label(FI,text=("a2= "),font=(10),bg="white")
a2_E = Entry(FI,width=5,font=(10))
cm2 = Label(FI,text=("cm"),font=(10),bg="white")

a3 = Label(FI,text=("a3= "),font=(10),bg="white")
a3_E = Entry(FI,width=5,font=(10))
cm3 = Label(FI,text=("cm"),font=(10),bg="white")

a4 = Label(FI,text=("a4= "),font=(10),bg="white")
a4_E = Entry(FI,width=5,font=(10))
cm4 = Label(FI,text=("cm"),font=(10),bg="white")

a5 = Label(FI,text=("a5= "),font=(10),bg="white")
a5_E = Entry(FI,width=5,font=(10))
cm5 = Label(FI,text=("cm"),font=(10),bg="white")

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

# Joint Variable Labels
T1 = Label(FI,text=("T1="),font=(10))
T1_E = Entry(FI,width=5, font=(10))
deg1 = Label(FI,text=("deg"), font=(10))

T2 = Label(FI,text=("T2="),font=(10))
T2_E = Entry(FI,width=5, font=(10))
deg2 = Label(FI,text=("deg"), font=(10))

d3 = Label(FI,text=("d3= "),font=(10))
d3_E = Entry(FI,width=5, font=(10))
cm6 = Label(FI,text=("cm"), font=(10))

T1.grid(row=0,column=3)
T1_E.grid(row=0,column=4)
deg1.grid(row=0,column=5)

T2.grid(row=1,column=3)
T2_E.grid(row=1,column=4)
deg2.grid(row=1,column=5)

d3.grid(row=2,column=3)
d3_E.grid(row=2,column=4)
cm6.grid(row=2,column=5)

#button frame
BF = LabelFrame(mygui,text="Forward Kinematics",font=(5))
BF.grid(row=1,column=0)

# Buttons
FK = Button(BF,text="Forward",font=(10),bg="gray",fg="red",command=f_k) #bg is for background color and fg is for font color
rst = Button(BF,text="Reset",font=(10),bg="gray",fg="pink",command=reset)

FK.grid(row=0,column=0)
rst.grid(row=0,column=1)

# Position vector frame
PV = LabelFrame(mygui,text="Position Vectors",font=(5))
PV.grid(row=2,column=0)

# Position vector label
X = Label(PV,text=("X="),font=(10))
X_E = Entry(PV,width=5,font=(10))
cm7 = Label(PV,text=("cm"),font=(10))

Y = Label(PV,text=("Y= "),font=(10))
Y_E = Entry(PV,width=5,font=(10))
cm8 = Label(PV,text=("cm"),font=(10))

Z = Label(PV,text=("Z="),font=(10))
Z_E = Entry(PV,width=5,font=(10))
cm9 = Label(PV,text=("cm"),font=(10))

X.grid(row=0,column=0)
X_E.grid(row=0,column=1)
cm7.grid(row=0,column=2)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
cm8.grid(row=1,column=2)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
cm9.grid(row=2,column=2)

# Image display
img = PhotoImage(file='SCARA.png')
img = img.subsample(7,8)
PI = Label(mygui,image=img)
PI.grid(row=3,column=0)

mygui.mainloop()










