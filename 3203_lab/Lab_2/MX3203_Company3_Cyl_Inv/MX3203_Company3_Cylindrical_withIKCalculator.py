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

# Create GUI window with title
mygui = Tk()
mygui.title("Cylindrical Manipulator")
mygui.resizable(False,False)
mygui.configure(bg="#45423c")

def reset():
    a1_E.delete(0,END)
    a2_E.delete(0,END)
    a3_E.delete(0,END)

    t1_E.delete(0,END)
    d2_E.delete(0,END)
    d3_E.delete(0,END)

    X_E.delete(0,END)
    Y_E.delete(0,END)
    Z_E.delete(0,END)

def f_k():
    test1 = a1_E.get()
    test2 = a2_E.get()
    test3 = a3_E.get()
    test4 = d2_E.get()
    test5 = d3_E.get()
    try:
        if float(test1) >= 0:
            if float(test2) >= 0:
                if float(test3) >= 0:
                    if float(test4) >= 0:
                        if float(test5) >= 0:
                            # Link lengths in cm
                            a1 = float(a1_E.get())/100
                            a2 = float(a2_E.get())/100
                            a3 = float(a3_E.get())/100

                            # Joint Variables: if d in cm; if theta in deg
                            t1 = float(t1_E.get())
                            d2 = float(d2_E.get())/100
                            d3 = float(d3_E.get())/100

                            # degrees to radian
                            t1 = (t1/180.0)*np.pi

                            # parametric table
                            pt = [[t1,(0.0/180.0)*np.pi,0,a1],
                                  [(270.0/180.0)*np.pi,(270.0/180.0)*np.pi,0,a2+d2],
                                  [(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,0,a3+d3]]
                            
                            # Homogeneous Transformation Matrix Formulae
                            i = 0
                            h0_1 = [[np.cos(pt[i][0]),-np.sin(pt[i][0])*np.cos(pt[i][1]),np.sin(pt[i][0])*np.sin(pt[i][1]),pt[i][2]*np.cos(pt[i][0])],
                                    [np.sin(pt[i][0]),np.cos(pt[i][0])*np.cos(pt[i][1]),-np.cos(pt[i][0])*np.sin(pt[i][1]),pt[i][2]*np.sin(pt[i][0])],
                                    [0,np.sin(pt[i][1]),np.cos(pt[i][1]),pt[i][3]],
                                    [0,0,0,1]]

                            i = 1
                            h1_2 = [[np.cos(pt[i][0]),-np.sin(pt[i][0])*np.cos(pt[i][1]),np.sin(pt[i][0])*np.sin(pt[i][1]),pt[i][2]*np.cos(pt[i][0])],
                                    [np.sin(pt[i][0]),np.cos(pt[i][0])*np.cos(pt[i][1]),-np.cos(pt[i][0])*np.sin(pt[i][1]),pt[i][2]*np.sin(pt[i][0])],
                                    [0,np.sin(pt[i][1]),np.cos(pt[i][1]),pt[i][3]],
                                    [0,0,0,1]]

                            i = 2
                            h2_3 = [[np.cos(pt[i][0]),-np.sin(pt[i][0])*np.cos(pt[i][1]),np.sin(pt[i][0])*np.sin(pt[i][1]),pt[i][2]*np.cos(pt[i][0])],
                                    [np.sin(pt[i][0]),np.cos(pt[i][0])*np.cos(pt[i][1]),-np.cos(pt[i][0])*np.sin(pt[i][1]),pt[i][2]*np.sin(pt[i][0])],
                                    [0,np.sin(pt[i][1]),np.cos(pt[i][1]),pt[i][3]],
                                    [0,0,0,1]]
                            
                            h0_1 = np.matrix(h0_1)

                            h1_2 = np.matrix(h1_2)

                            h2_3 = np.matrix(h2_3)

                            h0_2 = np.dot(h0_1,h1_2)

                            h0_3 = np.dot(h0_2,h2_3)

                            X0_3 = h0_3[0,3]
                            X_E.delete(0,END)
                            X_E.insert(0, np.around(X0_3*100,3))

                            Y0_3 = h0_3[1,3]
                            Y_E.delete(0,END)
                            Y_E.insert(0, np.around(Y0_3*100,3))

                            Z0_3 = h0_3[2,3]
                            Z_E.delete(0,END)
                            Z_E.insert(0, np.around(Z0_3*100,3))

                            # Create links
                            Cylind = DHRobot([
                                RevoluteDH(a1,0,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[0,2*np.pi]),
                                PrismaticDH((270.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a2,qlim=[0,50/100]),
                                PrismaticDH((0.0/180.0)*np.pi,0,(0.0/180.0)*np.pi,a3,qlim=[0,50/100])
                                ], name='Cylindrical Manipulator')
                            
                            # plot joints
                            q1 = np.array([t1,d2,d3])
                            
                            # plot scale
                            x0 = -1
                            x1 = 1
                            y0 = -1
                            y1 = 1
                            z0 = 0 
                            z1 = 1

                            # plot command
                            Cylind.plot(q1,limits=[x0,x1,y0,y1,z0,z1],block=True)
                        else:
                            messagebox.showerror("Error!", "Negative Input")
                    else:
                        messagebox.showerror("Error!", "Negative Input")
                else:
                    messagebox.showerror("Error!", "Negative Input")
            else:
                messagebox.showerror("Error!", "Negative Input")
        else:
            messagebox.showerror("Error!", "Negative Input")
    except ValueError:
        messagebox.showerror("Error!", "Invalid Input: Please enter proper link lengths and joint variables value")

def i_k():
    try:
        test1 = a1_E.get()
        test2 = a2_E.get()
        test3 = a3_E.get()
        if float(test1) >= 0:
            if float(test2) >= 0:
                if float(test3) >= 0:
                    # Link lengths in cm
                    a1 = float(a1_E.get())
                    a2 = float(a2_E.get())
                    a3 = float(a3_E.get())

                    # Position Vectors in mm
                    xe = float(X_E.get())
                    ye = float(Y_E.get())
                    ze = float(Z_E.get())

                    # To solve for theta 1 or th1:
                    # Try and Except
                    try:
                        th1 = np.arctan(ye/xe) # 1
                    except:
                        th1 = -1 # NAN
                        messagebox.showerror(title="DividedbyZero Error",message="Undefined solution if X = 0")
                    th1 = np.arctan(ye/xe) # 1

                    # To solve for d3
                    D3 = np.sqrt(xe**2 + ye**2) - a3 # 2

                    # To solve for d2
                    D2 = ze - a1 - a2 # 3

                    t1_E.delete(0, END)
                    t1_E.insert(0, np.around(th1*(180/np.pi),3))

                    d2_E.delete(0, END)
                    d2_E.insert(0, np.around(D2,3))

                    d3_E.delete(0, END)
                    d3_E.insert(0, np.around(D3,3))

                    # Create links
                    Cylind = DHRobot([
                        RevoluteDH(a1/100,0,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[0,2*np.pi]),
                        PrismaticDH((270.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a2/100,qlim=[0,50/100]),
                        PrismaticDH((0.0/180.0)*np.pi,0,(0.0/180.0)*np.pi,a3/100,qlim=[0,50/100])
                        ], name='Cylindrical Manipulator')
                    
                    # plot joints
                    q1 = np.array([th1,D2/100,D3/100])
                    
                    # plot scale
                    x0 = -1
                    x1 = 1
                    y0 = -1
                    y1 = 1
                    z0 = 0 
                    z1 = 1

                    # plot command
                    Cylind.plot(q1,limits=[x0,x1,y0,y1,z0,z1],block=True)
                else:
                    messagebox.showerror("Error!", "Negative Input")
            else:
                messagebox.showerror("Error!", "Negative Input")
        else:
            messagebox.showerror("Error!", "Negative Input")
    except ValueError:
        messagebox.showerror("Error!", "Invalid Input: Please enter proper link lengths and position vectors value")

# Link lengths and joint variables label
FI = LabelFrame(mygui, text="Link Lengths and Joint Variables", font=(5), bg="#302f2a", fg="white", relief="flat", labelanchor="n")
FI.grid(row=0,column=0,padx=10,pady=10,rowspan=2)

# Link lengths label
a1 = Label(FI, text="   a1 = ", font=10, bg="#302f2a", fg="white")
a1_E = Entry(FI, width=10, font=10, bg="#45423c", fg="white", highlightbackground="#302f2a", highlightcolor="#199123", highlightthickness=3, relief="flat")
cm1 = Label(FI, text=" cm  ", font=10, bg="#302f2a", fg="white")

a2 = Label(FI, text="   a2 = ", font=10, bg="#302f2a", fg="white")
a2_E = Entry(FI, width=10, font=10, bg="#45423c", fg="white", highlightbackground="#302f2a", highlightcolor="#199123", highlightthickness=3, relief="flat")
cm2 = Label(FI, text=" cm  ", font=10, bg="#302f2a", fg="white")

a3 = Label(FI, text="   a3 = ", font=10, bg="#302f2a", fg="white")
a3_E = Entry(FI, width=10, font=10, bg="#45423c", fg="white", highlightbackground="#302f2a", highlightcolor="#199123", highlightthickness=3, relief="flat")
cm3 = Label(FI, text=" cm  ", font=10, bg="#302f2a", fg="white")

a1.grid(row=0,column=0,padx=5,pady=5)
a1_E.grid(row=0,column=1,padx=5,pady=5)
cm1.grid(row=0,column=2,padx=5,pady=5)

a2.grid(row=1,column=0,padx=5,pady=5)
a2_E.grid(row=1,column=1,padx=5,pady=5)
cm2.grid(row=1,column=2,padx=5,pady=5)

a3.grid(row=2,column=0,padx=5,pady=5)
a3_E.grid(row=2,column=1,padx=5,pady=5)
cm3.grid(row=2,column=2,padx=5,pady=5)

# Joint Variables label
t1 = Label(FI, text="   θ1 = ", font=10, bg="#302f2a", fg="white")
t1_E = Entry(FI, width=10, font=10, bg="#45423c", fg="white", highlightbackground="#302f2a", highlightcolor="#199123", highlightthickness=3, relief="flat")
deg1 = Label(FI, text=" deg  ", font=10, bg="#302f2a", fg="white")

d2 = Label(FI, text="   d2 = ", font=10, bg="#302f2a", fg="white")
d2_E = Entry(FI, width=10, font=10, bg="#45423c", fg="white", highlightbackground="#302f2a", highlightcolor="#199123", highlightthickness=3, relief="flat")
cm4 = Label(FI, text=" cm  ", font=10, bg="#302f2a", fg="white")

d3 = Label(FI, text="   d3 = ", font=10, bg="#302f2a", fg="white")
d3_E = Entry(FI, width=10, font=10, bg="#45423c", fg="white", highlightbackground="#302f2a", highlightcolor="#199123", highlightthickness=3, relief="flat")
cm5 = Label(FI, text=" cm  ", font=10, bg="#302f2a", fg="white")

t1.grid(row=3,column=0,padx=5,pady=5)
t1_E.grid(row=3,column=1,padx=5,pady=5)
deg1.grid(row=3,column=2,padx=5,pady=5)

d2.grid(row=4,column=0,padx=5,pady=5)
d2_E.grid(row=4,column=1,padx=5,pady=5)
cm4.grid(row=4,column=2,padx=5,pady=5)

d3.grid(row=5,column=0,padx=5,pady=5)
d3_E.grid(row=5,column=1,padx=5,pady=5)
cm5.grid(row=5,column=2,padx=5,pady=5)

# Buttons Frame
BF = LabelFrame(mygui, text="Forward Kinematics", font=(5), bg="#302f2a", fg="white", relief="flat", labelanchor="n")
BF.grid(row=0,column=1,padx=10,pady=5)

# Buttons
FK = Button(BF, text="Forward", font=10, bg="#199123", fg="white", highlightbackground="#302f2a", highlightcolor="#302f2a", highlightthickness=3, activebackground="#2fba3a", activeforeground="white", width=7, relief="flat", command=f_k)
RST = Button(BF, text="Reset", font=10, bg="#45423c", fg="white", highlightbackground="#302f2a", highlightcolor="#302f2a", highlightthickness=3, activebackground="#e83535", activeforeground="white", width=19, relief="flat", command=reset)
IK = Button(BF, text="Inverse", font=10, bg="#ba761c", fg="white", highlightbackground="#302f2a", highlightcolor="#302f2a", highlightthickness=3, activebackground="#f7aa45", activeforeground="white", width=7, relief="flat", command=i_k)

FK.grid(row=0,column=0,padx=9,pady=3)
RST.grid(row=1,column=0,padx=9,pady=3,columnspan=2)
IK.grid(row=0,column=1,padx=9,pady=3)

# Position Vectors Frame
PV = LabelFrame(mygui, text="Position Vectors", font=(5), bg="#302f2a", fg="white", relief="flat", labelanchor="n")
PV.grid(row=1,column=1,padx=10,pady=5)

# Position Vectors label
X = Label(PV, text="   X = ", font=10, bg="#302f2a", fg="white")
X_E = Entry(PV, width=10, font=10, bg="#45423c", fg="white", highlightbackground="#302f2a", highlightcolor="#e83535", highlightthickness=3, relief="flat")
cm6 = Label(PV, text="cm   ", font=10, bg="#302f2a", fg="white")

Y = Label(PV, text="   Y = ", font=10, bg="#302f2a", fg="white")
Y_E = Entry(PV, width=10, font=10, bg="#45423c", fg="white", highlightbackground="#302f2a", highlightcolor="#e83535", highlightthickness=3, relief="flat")
cm7 = Label(PV, text="cm   ", font=10, bg="#302f2a", fg="white")

Z = Label(PV, text="   Z = ", font=10, bg="#302f2a", fg="white")
Z_E = Entry(PV, width=10, font=10, bg="#45423c", fg="white", highlightbackground="#302f2a", highlightcolor="#e83535", highlightthickness=3, relief="flat")
cm8 = Label(PV, text="cm   ", font=10, bg="#302f2a", fg="white")

X.grid(row=0,column=0,padx=6,pady=3)
X_E.grid(row=0,column=1,padx=6,pady=3)
cm6.grid(row=0,column=2,padx=6,pady=3)

Y.grid(row=1,column=0,padx=6,pady=3)
Y_E.grid(row=1,column=1,padx=6,pady=3)
cm7.grid(row=1,column=2,padx=6,pady=3)

Z.grid(row=2,column=0,padx=6,pady=3)
Z_E.grid(row=2,column=1,padx=6,pady=3)
cm8.grid(row=2,column=2,padx=6,pady=3)

# Insert Image
img = PhotoImage(file="KD.png")
img = img.subsample(1,1)
PI = Label(mygui, image=img)
PI.grid(row=2,column=0,padx=20,pady=20,columnspan=2)

mygui.mainloop()