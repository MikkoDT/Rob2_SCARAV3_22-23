from tkinter import*
from tkinter import PhotoImage
from tkinter import messagebox
import numpy as np
import math
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, PrismaticDH
import spatialmath 
from spatialmath import SE3
import matplotlib
matplotlib.use('TkAgg')

#Create a GUI window with a title
mygui = Tk()
mygui.title("Cartesian calculator")
mygui.resizable (False,False)

def reset():
    a1_E.delete(0,END)
    a2_E.delete(0,END)
    a3_E.delete(0, END)
    a4_E.delete(0, END)

    d1_E.delete(0, END)
    d2_E.delete(0, END)
    d3_E.delete(0, END)

    X_E.delete(0, END)
    Y_E.delete(0, END)
    Z_E.delete(0, END)

    lm1_E.delete(0, END)

def forward():
    a1 = float(a1_E.get())/100
    a2 = float(a2_E.get())/100
    a3 = float(a3_E.get())/100
    a4 = float(a4_E.get())/100

    d1 = float(d1_E.get())/100
    d2 = float(d2_E.get())/100
    d3 = float(d3_E.get())/100
    
    lm1 = float (lm1_E.get())/100

    PT = [[(0.0/180.0)*np.pi,(270.0/180.0)*np.pi,0,a1],
      [(270.0/180.0)*np.pi,(270.0/180.0)*np.pi,0,a2+d1],
      [(90.0/180.0)*np.pi,(270.0/180.0)*np.pi,0,a3+d2],
      [(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,0,a4+d3]]
    
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
    i = 3
    H3_4 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])], 
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]
    
    H0_1 = np.matrix(H0_1)
    H1_2 = np.matrix(H1_2)
    H2_3 = np.matrix(H2_3)
    H3_4 = np.matrix(H3_4)
    H0_2 = np.dot(H0_1,H1_2)
    H0_3 = np.dot(H0_2,H2_3)
    H0_4 = np.dot(H0_3,H3_4)

    X0_3 = H0_4[0,3]
    X_E.delete(0, END)
    X_E.insert(0,np.around(X0_3*100,3))
    Y0_3 = H0_4[1,3]
    Y_E.delete(0, END)
    Y_E.insert(0,np.around(Y0_3*100,3))
    Z0_3 = H0_4[2,3]
    Z_E.delete(0, END)
    Z_E.insert(0,np.around(Z0_3*100,3))
    
    

    CARTESIAN = DHRobot([
        PrismaticDH(0,0,(270.0/180.0)*np.pi,a1,qlim=[0,0]),
        PrismaticDH((270.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a2,qlim=[0,lm1]),
        PrismaticDH((90.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a3,qlim=[0,lm1]),
        PrismaticDH(0,0,(0.0/180.0)*np.pi,a4,qlim=[0,lm1]),
        ],name='Cartesian') 
    
    #plot joints
    q1 = np.array([0, d1, d2, d3])

    #plot Scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0
    z2 = 0.5

    #plot command  
    #CARTESIAN.teach(jointlabels=5)9
    CARTESIAN.plot(q1, limits=[x1,x2,y1,y2,z1,z2],block=True)
   

#Link lengths and Joint Variable Frames
FI = LabelFrame(mygui,text="Link Lengths and Joint Variables", font=(5))
FI.grid(row=0,column=0)

# Label of Link lengths
a1 = Label(FI,text=("a1= "),font=(10))
a1_E = Entry(FI,width=5,font=(10))  
cm1 = Label(FI,text=("cm"),font=(10))

a2 = Label(FI,text=("a2= "),font=(10))
a2_E = Entry(FI,width=5,font=(10)) 
cm2 = Label(FI,text=("cm"),font=(10))

a3 = Label(FI,text=("a3= "),font=(10))
a3_E = Entry(FI,width=5,font=(10)) 
cm3 = Label(FI,text=("cm"),font=(10))

a4 = Label(FI,text=("a4= "),font=(10))
a4_E = Entry(FI,width=5,font=(10)) 
cm4 = Label(FI,text=("cm"),font=(10))

d1 = Label(FI,text=("d1= "),font=(10))
d1_E = Entry(FI,width=5,font=(10)) 
cm5 = Label(FI,text=("cm"),font=(10))

d2 = Label(FI,text=("d2= "),font=(10))
d2_E = Entry(FI,width=5,font=(10)) 
cm6 = Label(FI,text=("cm"),font=(10))

d3 = Label(FI,text=("d3= "),font=(10))
d3_E = Entry(FI,width=5,font=(10)) 
cm7 = Label(FI,text=("cm"),font=(10))

lm1 = Label(FI,text=("lm1= "),font=(10))
lm1_E = Entry(FI,width=5,font=(10)) 
cm8 = Label(FI,text=("cm"),font=(10))

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

d1.grid(row=0,column=3)
d1_E.grid(row=0,column=4)
cm5.grid(row=0,column=5)

d2.grid(row=1,column=3)
d2_E.grid(row=1,column=4)
cm6.grid(row=1,column=5)

d3.grid(row=2,column=3)
d3_E.grid(row=2,column=4)
cm7.grid(row=2,column=5)

lm1.grid(row=3,column=3)
lm1_E.grid(row=3,column=4)
cm8.grid(row=3,column=5)

#Button frames
BF = LabelFrame(mygui,text="forward Kinematics",font=(5))
BF.grid(row=1,column=0)

#button label
FK =Button(BF,text="Forward",font=(10),bg="green", fg="black",command=forward)
rst =Button(BF,text="Reset",font=(10),bg="red",fg="black",command=reset)

FK.grid(row=0,column=0)
rst.grid(row=0,column=1)

PV = LabelFrame(mygui,text="Position Vectors", font=(5))
PV.grid(row=2,column=0)

#Position vectors Labels
X = Label(PV,text=("X= "),font=(10))
X_E = Entry(PV,width=5,font=(10)) 
cm6 = Label(PV,text=("cm"),font=(10))

Y = Label(PV,text=("Y= "),font=(10))
Y_E = Entry(PV,width=5,font=(10)) 
cm7 = Label(PV,text=("cm"),font=(10))

Z = Label(PV,text=("Z= "),font=(10))
Z_E = Entry(PV,width=5,font=(10)) 
cm8 = Label(PV,text=("cm"),font=(10))

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
img= PhotoImage(file="he.png")
img = img.subsample(1,2)
PI = Label(mygui,image=img)
PI.grid(row=3,column=0)

mygui.mainloop()

