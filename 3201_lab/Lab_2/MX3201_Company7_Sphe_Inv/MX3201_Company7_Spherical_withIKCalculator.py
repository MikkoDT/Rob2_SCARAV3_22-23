import tkinter as tk
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
from PIL import Image, ImageTk

#Initialize app
w1 = Tk()
x = w1.winfo_screenwidth() // 3
y = int(w1.winfo_screenheight() * 0.1)
w1.geometry('525x660+' + str(x) + "+" + str(y))
w1.title('SPHERICAL CALCULATOR')


def open():
    w2 = Toplevel()
    w2.title('SPHERICAL CALCULATOR')
    x = w2.winfo_screenwidth() // 3
    y = int(w2.winfo_screenheight() * 0.1)
    w2.geometry('525x660+' + str(x) + "+" + str(y))
    lbl=Label(w2, text="GOOD DAY MASTER, I'M GOOD TO GO!").pack(pady=10)
    

       #Assigning Frames
    options_frame = tk.Frame(w2, bg='#8A360F', highlightbackground='black', highlightthickness=1)

    options_frame.pack(side=tk.BOTTOM)
    options_frame.pack_propagate(False)
    options_frame.configure(width=500, height=50)
     


    main_frame = tk.Frame(w2, bg='black', highlightbackground='black', highlightthickness=3)

    main_frame.pack(side=tk.BOTTOM)
    main_frame.pack_propagate(False)
    main_frame.configure(width=500, height=600)

    #Defining Frames
    
    close_btn = tk.Button(options_frame, 
                        text='CLOSE',
                        font=('Bold', 10),
                        fg='orange', 
                        cursor="hand2", 
                        bg='black', 
                        activebackground="black", 
                        activeforeground="white",
                        width= 15,
                        height=2, 
                        command=close)
    close_btn.pack(padx=10, pady=10)
    
    

    
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

          # joint variables: is cm if f, is degree if theta
          T1 = float(T1_E.get())
          T2 = float(T2_E.get())
          d3 = float(d3_E.get())/100

          # degree to radian
          T1 = (T1/180.0)*np.pi
          T2 = (T2/180.0)*np.pi

          # Parametric Table (Theta, alpha, r, d)
          PT = [[T1,(90.0/180.0)*np.pi,0,a1],
                    [(90.0/180.0)*np.pi+T2,(90.0/180.0)*np.pi,0,0],
                    [(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,0,a2+a3+d3]]
          
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
          H0_3 = np.dot(H0_2,H2_3) # HTM of the SPHERICAL

          # Position Vectors
          X0_3 = H0_3[0,3]
          X_E.delete(0, END)
          X_E.insert(0, np.around(X0_3*100, 3))

          Y0_3 = H0_3[1,3]
          Y_E.delete(0, END)
          Y_E.insert(0, np.around(Y0_3*100, 3))

          Z0_3 = H0_3[2,3]
          Z_E.delete(0, END)
          Z_E.insert(0, np.around(Z0_3*100, 3))

          # Create links
          # [robot_variable]=DHRobot([RevoluteDH(d, r, alpha, offset)])
          SPHERICAL = DHRobot([
                    RevoluteDH(a1, 0, (90.0/180.0)*np.pi, (0.0/180.0)*np.pi, qlim=[-np.pi/2, np.pi/2]),
                    RevoluteDH(0, 0, (90.0/180.0)*np.pi, (90.0/180.0)*np.pi, qlim=[-np.pi/2, np.pi/2]),
                    PrismaticDH(0, 0, (0.0/180.0)*np.pi, a2+a3, qlim=[0, 30/100]),
                    ], name="SPHERICAL")

          # plot joint variables
          q1 = np.array([T1, T2, d3])

          # plot scale
          x1 = -0.5
          x2 = 0.5
          y1 = -0.5
          y2 = 0.5
          z1 = 0.0
          z2 = 0.5

          # plot command
          SPHERICAL.plot(q1, limits=[x1,x2,y1,y2,z1,z2], block=True)
    def i_k():
        # Inverse Kinematics using graphical method

                    # link lengths in mm
                    a1 = float(a1_E.get())
                    a2 = float(a2_E.get())
                    a3 = float(a3_E.get())

                    # Position vectors in cm
                    xe = float(X_E.get())
                    ye = float(Y_E.get())
                    ze = float(Z_E.get())

                    # try and except
                    try:
                         th1 = np.arctan(ye/xe) #1
                    except:
                         th1 = -1 #NAN error
                         messagebox.showerror(title = "DividedbyZero Error", message = "Undefined Solution if X = 0") 

                    # to solve for theta 1 or th1
                    th1 = np.arctan(ye/xe) #1
                    r1 = np.sqrt(xe**2 + ye**2) #2

                    # to solve for theta 2 or th2
                    r2 = ze - a1 #3
                    th2 = np.arctan(r2 / r1) #4

                    # to solve for D3
                    D3 = np.sqrt(r1**2 + r2**2) - a2 - a3

                    d3_E.delete(0, END)
                    d3_E.insert(0, np.around(D3, 3))

                    T1_E.delete(0, END)
                    T1_E.insert(0, np.around(th1*180/np.pi, 3))

                    T2_E.delete(0, END)
                    T2_E.insert(0, np.around(th2*180/np.pi, 3))

                    # Create links
                    # [robot_variable]=DHRobot([RevoluteDH(d, r, alpha, offset)])
                    SPHERICAL = DHRobot([
                         RevoluteDH(a1/100, 0, (90.0/180.0)*np.pi, (0.0/180.0)*np.pi, qlim=[-np.pi/2, np.pi/2]),
                         RevoluteDH(0, 0, (90.0/180.0)*np.pi, (90.0/180.0)*np.pi, qlim=[-np.pi/2, np.pi/2]),
                         PrismaticDH(0, 0, (0.0/180.0)*np.pi, a2/100+ a3/100, qlim=[0, 30/100]),
                         ], name="SPHERICAL")

                    # plot joint variables
                    q1 = np.array([th1, th2, D3/100])

                    # plot scale
                    x1 = -0.5
                    x2 = 0.5
                    y1 = -0.5
                    y2 = 0.5
                    z1 = 0.0
                    z2 = 0.5

                    # plot command
                    SPHERICAL.plot(q1, limits=[x1,x2,y1,y2,z1,z2], block=True)            
          



    #Link lengths and Joint Variables
    FI = LabelFrame(main_frame, text="Link Lengths and Joint Variables", 
                    font=(5), bg="#808A87", fg="black")
    FI.grid(row=0, column=0)

    # label of link lengths
    a1 = Label(FI, text=("a1 = "), 
               font=(10),
            bg="black",
            fg="orange")
    a1_E = Entry(FI, width=5, font=(10))
    cm1 = Label(FI, text=("cm"), font=(10), 
                bg="black",
                fg="orange")

    a2 = Label(FI, text=("a2 = "), 
               font=(10),
               bg="black", 
               fg="orange")
    a2_E = Entry(FI, width=5, font=(10))
    cm2 = Label(FI, text=("cm"), font=(10), 
                bg="black",
                fg="orange")

    a3 = Label(FI, text=("a3 = "), font=(10), 
               bg="black",
               fg="orange")
    a3_E = Entry(FI, width=5, font=(10))
    cm3 = Label(FI, text=("cm"), font=(10), 
                bg="black",
                fg="orange")

    a1.grid(row=0, column=0)
    a1_E.grid(row=0, column=1)
    cm1.grid(row=0, column=2)

    a2.grid(row=1, column=0)
    a2_E.grid(row=1, column=1)
    cm2.grid(row=1, column=2)

    a3.grid(row=2, column=0)
    a3_E.grid(row=2, column=1)
    cm3.grid(row=2, column=2)

    # Joint variables label

    T1 = Label(FI, text=("T1 = "), font=(10), 
               bg="black",
               fg="orange")
    T1_E = Entry(FI, width=5, font=(10))
    deg1 = Label(FI, text=("deg"), font=(10), 
                 bg="black",
                 fg="orange")

    T2 = Label(FI, text=("T2 = "), font=(10), 
               bg="black",
               fg="orange")
    T2_E = Entry(FI, width=5, font=(10))
    deg2 = Label(FI, text=("deg"), font=(10), 
                 bg="black",
                 fg="orange")

    d3 = Label(FI, text=("d3 = "), font=(10), 
               bg="black",
               fg="orange")
    d3_E = Entry(FI, width=5, font=(10))
    cm4 = Label(FI, text=("cm"), font=(10), 
                bg="black",
                fg="orange")

    T1.grid(row=0, column=3)
    T1_E.grid(row=0, column=4)
    deg1.grid(row=0, column=5)

    T2.grid(row=1, column=3)
    T2_E.grid(row=1, column=4)
    deg2.grid(row=1, column=5)

    d3.grid(row=2, column=3)
    d3_E.grid(row=2, column=4)
    cm4.grid(row=2, column=5)

    # Buttons Frame
    BF = LabelFrame(main_frame, text="Forward Kinematics", 
                    font=(5), bg="#808A87")
    BF.grid(row=1, column=0)

    # Buttons 
    FK = Button(BF, text="Forward",font=(10), bg="#8A360F", 
                fg="white", command=f_k)
    rst = Button(BF, text="Reset",font=(10), bg="#8A360F",
                fg="white", command=reset)
    IK = Button(BF, text="Inverse",font=(10), bg="#8A360F", fg="white", command=i_k)
    
    FK.grid(row=0, column=0)
    rst.grid(row=0, column=1)
    IK.grid(row=0, column=2)

    # KPosition Vectors Frame
    PV = LabelFrame(main_frame, text="Position Vectors", font=(5), 
                    bg="#808A87")
    PV.grid(row=2, column=0)

    # Position Vectors label
    X = Label(PV, text=("X = "), font=(10), bg="black",
                fg="orange")
    X_E = Entry(PV, width=5, font=(10))
    cm6 = Label(PV, text=("cm"), font=(10),bg="black",
                fg="orange")

    Y = Label(PV, text=("Y = "), font=(10), bg="black",
                fg="orange")
    Y_E = Entry(PV, width=5, font=(10))
    cm7 = Label(PV, text=("cm"), font=(10), bg="black",
                fg="orange")

    Z = Label(PV, text=("Z = "), font=(10), bg="black",
                fg="orange")
    Z_E = Entry(PV, width=5, font=(10))
    cm8 = Label(PV, text=("cm"), font=(10), bg="black",
                fg="orange")

    X.grid(row=0, column=0)
    X_E.grid(row=0, column=1)
    cm6.grid(row=0, column=2)

    Y.grid(row=1, column=0)
    Y_E.grid(row=1, column=1)
    cm7.grid(row=1, column=2)

    Z.grid(row=2, column=0)
    Z_E.grid(row=2, column=1)
    cm8.grid(row=2, column=2)





    
    
#Quit Window
def close():
    w1.quit()
    open.quit() 

#Frames
frame1 = tk.Frame(w1, width=525, height=600, bg="black")
frame1.grid(row=0, column=0)                                      
frame1.pack_propagate(False)
    
tk.Label(frame1,
         text="ROBO CALC",
         fg="#D2691E",
         bg="black",
         font=("Helvetica", 15)).pack(ipadx=400, ipady=20)


#Frame widget
logo_img = tk.PhotoImage(file="spheri.png")
logo_widget = tk.Label(w1, image=logo_img, bg="#ED9121")
logo_widget.img = logo_img
logo_widget.grid(row=0, column=0)



#Button
open_btn = tk.Button(w1, text='OPEN', 
                     font=('Bold', 10),
                     fg='orange', cursor="hand2", 
                     bg='black', 
                     activebackground="black", 
                     activeforeground="white",
                     width= 15,
                     height=2,
                     command=open)
open_btn.grid(row=1, column=0)

                                         


#Run App
w1.mainloop()