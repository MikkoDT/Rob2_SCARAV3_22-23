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
mygui.title("SCARA RRP Calculator")
#mygui.resizable(True, True) #(width,height) 
mygui.geometry("880x500")
mygui.configure(bg = 'gray13')


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

    #degrees to radian 
    T1 = (T1/180.0)*np.pi
    T2 = (T2/180.0)*np.pi

    # parametric Table (Theta, alpha , R , d)
    PT = [[T1,(0.0/180.0)*np.pi,a2,a1],
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
    # [robot variable] = DHRobot([RevoluteDH(d,r,alpha,offset)])
    SCARAV3 = DHRobot([
        RevoluteDH(a1,0,(0/180.0)*np.pi,(0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        PrismaticDH(0,a2,0,0,qlim=[0,0]),
        RevoluteDH(a3,0,(180/180.0)*np.pi,(0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        PrismaticDH(0,a4,0,0,qlim=[0,0]),
        PrismaticDH(0,0,(0/180.0)*np.pi,a5,qlim=[0,30/100])
        ], name = "SCARAV1" )

    #Plot Joints
    q1 = np.array([T1, 0, T2, 0, d3])

    #Plot Scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0
    z2 = 0.5

    #Plot Command
    SCARAV3.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)



#Link lenghts and joint variables Frame
FI = LabelFrame(mygui,text="", borderwidth=(0),  font=(5), bg = "gray13", fg="white", padx=(25), pady=(25))
FI.grid(row=0, column=0)


#link lengths label 
lljv = Label(FI,text=("Link Lengths and Joint Variables"), font=(10), bg = "gray13", fg="white")
lljv.grid(row=0,column=1)


a1 = Label(FI,text=("a1 = "), font=(10), bg = "gray13", fg="white")
a1_E = Entry(FI,width=15,font=(10), bg ="gray13", fg="white", highlightthickness=(1))
a1_E.config(highlightbackground= "aquamarine2", highlightcolor="white")
cm1 = Label(FI,text=("cm"),font=(10), bg = "gray13", fg="gray85")

a2 = Label(FI,text=("a2 = "),font=(10), bg = "gray13", fg="white")
a2_E = Entry(FI,width=15,font=(10), bg ="gray13", fg="white", highlightthickness=(1))
a2_E.config(highlightbackground= "aquamarine2", highlightcolor="white")
cm2 = Label(FI,text=("cm"),font=(10), bg = "gray13", fg="gray85")

a3 = Label(FI,text=("a3 = "),font=(10), bg = "gray13", fg="white")
a3_E = Entry(FI,width=15,font=(10), bg ="gray13", fg="white", highlightthickness=(1))
a3_E.config(highlightbackground= "aquamarine2", highlightcolor="white")
cm3 = Label(FI,text=("cm"),font=(10), bg = "gray13", fg="gray85")

a4 = Label(FI,text=("a4 = "),font=(10), bg = "gray13", fg="white")
a4_E = Entry(FI,width=15,font=(10), bg ="gray13", fg="white", highlightthickness=(1))
a4_E.config(highlightbackground= "aquamarine2", highlightcolor="white")
cm4 = Label(FI,text=("cm"),font=(10), bg = "gray13", fg="gray85")

a5 = Label(FI,text=("a5 = "),font=(10), bg = "gray13", fg="white")
a5_E = Entry(FI,width=15,font=(10), bg ="gray13", fg="white", highlightthickness=(1))
a5_E.config(highlightbackground= "aquamarine2", highlightcolor="white")
cm5 = Label(FI,text=("cm"),font=(10), bg = "gray13", fg="gray85")

a1.grid(row=1,column=0)
a1_E.grid(row=1,column=1)
cm1.grid(row=1,column=2)

a2.grid(row=2,column=0)
a2_E.grid(row=2,column=1)
cm2.grid(row=2,column=2)

a3.grid(row=3,column=0)
a3_E.grid(row=3,column=1)
cm3.grid(row=3,column=2)

a4.grid(row=4,column=0)
a4_E.grid(row=4,column=1)
cm4.grid(row=4,column=2)

a5.grid(row=5,column=0)
a5_E.grid(row=5,column=1)
cm5.grid(row=5,column=2)

#joint variable labels 
T1 = Label(FI,text=("θ1 = "),font=(10), bg = "gray13", fg="white")
T1_E = Entry(FI,width=15,font=(10), bg ="gray13", fg="white", highlightthickness=(1))
T1_E.config(highlightbackground= "aquamarine2", highlightcolor="white")
cm5 = Label(FI,text=("deg"),font=(10), bg = "gray13", fg="gray85")

T2 = Label(FI,text=("θ2 = "),font=(10), bg = "gray13", fg="white")
T2_E = Entry(FI,width=15,font=(10), bg ="gray13", fg="white", highlightthickness=(1))
T2_E.config(highlightbackground= "aquamarine2", highlightcolor="white")
deg1 = Label(FI,text=("deg"),font=(10), bg = "gray13", fg="gray85")

d3 = Label(FI,text=("d3 = "),font=(10), bg = "gray13", fg="white")
d3_E = Entry(FI,width=15,font=(10), bg ="gray13", fg="white", highlightthickness=(1))
d3_E.config(highlightbackground= "aquamarine2", highlightcolor="white")
deg2 = Label(FI,text=("cm"),font=(10), bg = "gray13", fg="gray85")

space1 = Label(FI,text=(""),bg = "gray13")

T1.grid(row=6,column=0)
T1_E.grid(row=6,column=1)
cm5.grid(row=6,column=2)

T2.grid(row=7,column=0)
T2_E.grid(row=7,column=1)
deg1.grid(row=7,column=2)

d3.grid(row=8,column=0)
d3_E.grid(row=8,column=1)
deg2.grid(row=8,column=2)

space1.grid(row=9,column=0)

fdkm = Label(FI,text=("Forward Kinematics"),font=(10), bg = "gray13", fg="white")
fdkm.grid(row=10,column=1)


#BF.pack(fill="both", expand="yes")
#BF.grid(padx = 50, pady = 50) 


#buttons
#bg is for bacground color, fg is for font color
#to change color when mouse is hovered in the button >>> activeforeground="black", activebackground="white"
FK = Button(FI, text="CALCULATE",font=(10), bg="mediumseagreen", width=(15), fg="white", highlightthickness=(1), command=f_k) 
FK.config(highlightbackground= "gray13", highlightcolor="white")
rst = Button(FI, text="RESET",font=(10),bg="firebrick1", width=(15), fg="white", highlightthickness=(1), command=reset)
rst.config(highlightbackground= "gray13", highlightcolor="white")

space2 = Label(FI, text="", bg="gray13")


FK.grid(row=11,column=1)
rst.grid(row=12,column=1)
space2.grid(row=13,column=0)

#position vector frame
pnvr = Label(FI,text="Position Vectors", font=(5), bg = "gray13", fg="white")
pnvr.grid(row=14, column=1)

#position vector label

X = Label(FI,text=("X = "),font=(10), bg = "gray13", fg="white")
X_E = Entry(FI,width=15,font=(10), bg = "gray13", fg="white", highlightthickness=(1))
X_E.config(highlightbackground= "aquamarine2", highlightcolor="white")
cm6 = Label(FI,text=("cm "),font=(10), bg = "gray13", fg="gray85")

Y = Label(FI,text=("Y = "),font=(10), bg = "gray13", fg="white")
Y_E = Entry(FI,width=15,font=(10), bg = "gray13", fg="white", highlightthickness=(1))
Y_E.config(highlightbackground= "aquamarine2", highlightcolor="white")
cm7 = Label(FI,text=("cm "),font=(10), bg = "gray13", fg="gray85")

Z = Label(FI,text=("Z = "),font=(10), bg = "gray13", fg="white")
Z_E = Entry(FI,width=15,font=(10), bg = "gray13", fg="white", highlightthickness=(1))
Z_E.config(highlightbackground= "aquamarine2", highlightcolor="white")
cm8 = Label(FI,text=("cm "),font=(10), bg = "gray13", fg="gray85")

X.grid(row=15,column=0)
X_E.grid(row=15,column=1)
cm6.grid(row=15,column=2)

Y.grid(row=16,column=0)
Y_E.grid(row=16,column=1)
cm7.grid(row=16,column=2)

Z.grid(row=17,column=0)
Z_E.grid(row=17,column=1)
cm8.grid(row=17,column=2)

#image display
img = PhotoImage(file="scara1.png")
img = img.subsample(2,2)
PI = Label(mygui,image=img)
PI.grid(row=0,column=3)


mygui.mainloop()