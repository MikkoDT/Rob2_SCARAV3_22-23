from tkinter import *
from tkinter import PhotoImage
from tkinter import messagebox
import numpy as np
import math
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import spatialmath
from spatialmath import SE3
import matplotlib
matplotlib.use('TkAgg')

class SplashScreen:
    def __init__(self, master):
        self.master = master

        # Remove window borders
        master.overrideredirect(True) 

        #splash screen centered
        screen_width = master.winfo_screenwidth()
        screen_height = master.winfo_screenheight()
        x = (screen_width / 2) - (400 / 2)
        y = (screen_height / 2) - (300 / 2)

        master.geometry('%dx%d+%d+%d' % (350, 400, x, y))
        master.configure(background='#FFF8DC')

        # image
        self.splash_image = PhotoImage(file='cartesian.png')
        self.image_label = Label(master, image=self.splash_image)
        self.image_label.pack(pady=30)

        Label(master, text='Cartesian Calculator', font=('Garamond', 18), bg = '#DEB887', fg = 'black').pack(pady=14)

    def transition(self):
        #remove splash screen
        self.master.destroy()

        # transition to main window
        MainWindow()

class MainWindow:
    def __init__(self):
        #GUI main window title
        mygui = Tk()
        mygui.title("Cartesian_Manipulator_Calculator")
        mygui.resizable(False,False)

        screen_width2 = mygui.winfo_screenwidth()
        screen_height2 = mygui.winfo_screenheight()
        x2 = (screen_width2 / 2) - (1000 / 2)
        y2 = (screen_height2 / 2) - (590 / 2)

        mygui.geometry('%dx%d+%d+%d' % (1000, 590, x2, y2))
        mygui.configure(bg="#FFF8DC")


        #matrix label
        global Matrix_label

        def reset():
            #delete entry and result
            a1_E.delete(0, END)
            a2_E.delete(0, END)
            a3_E.delete(0, END)
            a4_E.delete(0, END)
            d1_E.delete(0, END)
            d2_E.delete(0, END)
            d3_E.delete(0, END)
            X_E.delete(0, END)
            Y_E.delete(0, END)
            Z_E.delete(0, END)
    
            try:
                Matrix_label.config(text="")
            except Exception as e:
                pass  # do nothing if the label does not exist


        
     
        def f_k():
            global Matrix_label

            # link lengths in cm
            a1 = float(a1_E.get())/100
            a2 = float(a2_E.get())/100
            a3 = float(a3_E.get())/100
            a4 = float(a4_E.get())/100

            # joint variables: if d cm, if Theta degrees
            d1 = float(d1_E.get())/100
            d2 = float(d2_E.get())/100
            d3 = float(d3_E.get())/100

            # Parametric Table (Theta, alpha, r, d)
            pt = [[(0.0/180) * np.pi, (270.0/180.0) * np.pi ,0, a1],
            [(270.0/180.0) * np.pi, (270.0/180) * np.pi, 0, a2+d1],
            [(90.0/180.0) * np.pi, (270.0/180) * np.pi, 0, a3+d2],
            [(0.0/180) * np.pi, (0.0/180) * np.pi , 0, a4+d3]]

            # Homogenous Transformation Matrix Formulae
            i = 0
            H0_1 = [[np.cos(pt[i][0]),-np.sin(pt[i][0])*np.cos(pt[i][1]),np.sin(pt[i][0])*np.sin(pt[i][1]),pt[i][2]*np.cos(pt[i][0])],
            [np.sin(pt[i][0]),np.cos(pt[i][0])*np.cos(pt[i][1]),-np.cos(pt[i][0])*np.sin(pt[i][1]),pt[i][2]*np.sin(pt[i][0])],
            [0,np.sin(pt[i][1]),np.cos(pt[i][1]),pt[i][3]],
            [0,0,0,1]]

            i = 1
            H1_2 = [[np.cos(pt[i][0]),-np.sin(pt[i][0])*np.cos(pt[i][1]),np.sin(pt[i][0])*np.sin(pt[i][1]),pt[i][2]*np.cos(pt[i][0])],
            [np.sin(pt[i][0]),np.cos(pt[i][0])*np.cos(pt[i][1]),-np.cos(pt[i][0])*np.sin(pt[i][1]),pt[i][2]*np.sin(pt[i][0])],
            [0,np.sin(pt[i][1]),np.cos(pt[i][1]),pt[i][3]],
            [0,0,0,1]]

            i = 2
            H2_3 = [[np.cos(pt[i][0]),-np.sin(pt[i][0])*np.cos(pt[i][1]),np.sin(pt[i][0])*np.sin(pt[i][1]),pt[i][2]*np.cos(pt[i][0])],
            [np.sin(pt[i][0]),np.cos(pt[i][0])*np.cos(pt[i][1]),-np.cos(pt[i][0])*np.sin(pt[i][1]),pt[i][2]*np.sin(pt[i][0])],
            [0,np.sin(pt[i][1]),np.cos(pt[i][1]),pt[i][3]],
            [0,0,0,1]]

            i = 3
            H3_4 = [[np.cos(pt[i][0]),-np.sin(pt[i][0])*np.cos(pt[i][1]),np.sin(pt[i][0])*np.sin(pt[i][1]),pt[i][2]*np.cos(pt[i][0])],
            [np.sin(pt[i][0]),np.cos(pt[i][0])*np.cos(pt[i][1]),-np.cos(pt[i][0])*np.sin(pt[i][1]),pt[i][2]*np.sin(pt[i][0])],
            [0,np.sin(pt[i][1]),np.cos(pt[i][1]),pt[i][3]],
            [0,0,0,1]]
            H0_1 = np.matrix(H0_1)
            H1_2 = np.matrix(H1_2)

            H2_3 = np.matrix(H2_3)
            H3_4 = np.matrix(H3_4)

            H0_2 = np.dot(H0_1,H1_2)
            H0_3 = np.dot(H0_2, H2_3)
            H0_4 = np.dot(H0_3, H3_4)

            Matrix_label = Label(Matrix_frame, height = "17", width = "54", bg = '#FFFFFF', fg = 'black', font = 50,
            text=f"H0_4:\n{np.array2string(H0_4, formatter={'float_kind':lambda x: '%.2f' % x}, separator=' ')}")
            Matrix_label.pack()


            X_04 = H0_4[0,3]
            X_E.delete(0, END)
            X_E.insert(0, np.around(X_04*100,3))

            Y_04 = H0_4[1,3]
            Y_E.delete(0, END) 
            Y_E.insert(0, np.around(Y_04*100,3))
            Z_04 = H0_4[2,3]
            Z_E.delete(0, END)
            Z_E.insert(0, np.around(Z_04*100,3))

            # create links
            # [robot_variable]=DHRobot([RevoluteDH(d, r, alpha, offset)])
            Cartesian_Ground = DHRobot([
            PrismaticDH(0,0,(270.0/180.0) * np.pi,a1,qlim=[0,0]),
            PrismaticDH((270.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a2,qlim=[0, d1]),
            PrismaticDH((90.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a3,qlim=[0, d2]),
            PrismaticDH(0,0,(0.0/180.0)*np.pi,a4,qlim=[0, d3])
            ], name='Cartesian_Ground' )

            # plot joints
            q1 = np.array([0,d1,d2,d3])

            # plot scale
            x1 = -0.5
            x2 = 0.5
            y1 = -0.5
            y2 = 0.5
            z1 = 0
            z2 = 0.5

            # plot command
            Cartesian_Ground.plot(q1, limits = [x1,x2,y1,y2,z1,z2], block = True)
        
        def  i_k():

            # link lengths in cm
            a1 = float(a1_E.get())
            a2 = float(a2_E.get())
            a3 = float(a3_E.get())
            a4 = float(a4_E.get())

            #Position vector
            xe = float(X_E.get())
            ye = float(Y_E.get())
            ze = float(Z_E.get())


            #solving for d2
            d2 = xe - a3

            #solving for d3
            d3 =  a1 - a4 - ze

            #solving for d1
            d1 = ye - a2

            d1_E.delete(0, END)
            d1_E.insert(0, np.around(d1, 3))

            d2_E.delete(0, END)
            d2_E.insert(0, np.around(d2, 3))

            d3_E.delete(0, END)
            d3_E.insert(0, np.around(d3, 3))

            # create links
            # [robot_variable]=DHRobot([RevoluteDH(d, r, alpha, offset)])
            Cartesian_Ground = DHRobot([
            PrismaticDH(0,0,(270.0/180.0) * np.pi,a1/100,qlim=[0,0]),
            PrismaticDH((270.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a2/100,qlim=[0, d1]),
            PrismaticDH((90.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a3/100,qlim=[0, d2]),
            PrismaticDH(0,0,(0.0/180.0)*np.pi,a4/100,qlim=[0, d3])
            ], name='Cartesian_Ground' )

            # plot joints
            q1 = np.array([0,d1/100,d2/100,d3/100])

            # plot scale
            x1 = -0.5
            x2 = 0.5
            y1 = -0.5
            y2 = 0.5
            z1 = 0
            z2 = 0.5

            # plot command
            Cartesian_Ground.plot(q1, limits = [x1,x2,y1,y2,z1,z2], block = True)


        # Link lengths and Joint Variable Frame
        Title = Label(mygui, text='CARTESIAN CALCULATOR', font=('Garamond', 24), bg = '#9A7B4F', fg = 'white')
        Title.place(x = 25, y = 20)

        FI = LabelFrame(mygui, bg = '#FFFFFF', width = '440', height = '480')
        FI.place(x = 25, y = 73)

        LJ_title = Label(FI, text="Link Lengths and Joint Variables", font=('Garamond', 18), bg = '#FFFFFF', fg = 'black')
        LJ_title.place(x = 10, y = 10)

        #Link Lengths label
        a1 = Label(FI, text="a1 = ", font=(10), bg = '#F3EAD3', fg = 'black')
        a1_E = Entry(FI, width=5, font=(10))
        cm1 = Label(FI, text=("cm "), font=(10), bg = '#F3EAD3', fg = 'black')

        a2 = Label(FI, text="a2 = ", font=(10), bg = '#F3EAD3', fg = 'black')
        a2_E = Entry(FI, width=5, font=(10))
        cm2 = Label(FI, text=("cm "), font=(10), bg = '#F3EAD3', fg = 'black')

        a3 = Label(FI, text="a3 = ", font=(10), bg = '#F3EAD3', fg = 'black')
        a3_E = Entry(FI, width=5, font=(10))
        cm3 = Label(FI, text=("cm "), font=(10), bg = '#F3EAD3', fg = 'black')

        a4 = Label(FI, text="a4 = ", font=(10), bg = '#F3EAD3', fg = 'black')
        a4_E = Entry(FI, width=5, font=(10))
        cm4 = Label(FI, text=("cm "), font=(10), bg = '#F3EAD3', fg = 'black')

        a1.place(x = 20, y = 45)
        a1_E.place(x = 65, y = 45)
        cm1.place(x = 125, y = 45)

        a2.place(x = 20, y = 70)
        a2_E.place(x = 65, y = 70)
        cm2.place(x = 125, y = 70)

        a3.place(x = 20, y = 95)
        a3_E.place(x = 65, y = 95)
        cm3.place(x = 125, y = 95)

        a4.place(x = 20, y = 120)
        a4_E.place(x = 65, y = 120)
        cm4.place(x = 125, y = 120)
    
        #Joint Variable label
        d1 = Label(FI, text="d1 = ", font=(10), bg = '#F3EAD3', fg = 'black')
        d1_E = Entry(FI, width=5, font=(10))
        cm5 = Label(FI, text=("cm "), font=(10), bg = '#F3EAD3', fg = 'black')

        d2 = Label(FI, text="d2 = ", font=(10), bg = '#F3EAD3', fg = 'black')
        d2_E = Entry(FI, width=5, font=(10))
        cm6 = Label(FI, text=("cm "), font=(10), bg = '#F3EAD3', fg = 'black')

        d3 = Label(FI, text="d3 = ", font=(10), bg = '#F3EAD3', fg = 'black')
        d3_E = Entry(FI, width=5, font=(10))
        cm7 = Label(FI, text=("cm "), font=(10), bg = '#F3EAD3', fg = 'black')

        d1.place(x = 180, y = 45)
        d1_E.place(x = 230, y = 45)
        cm5.place(x = 290, y = 45)

        d2.place(x = 180, y = 70)
        d2_E.place(x = 230, y = 70)
        cm6.place(x = 290, y = 70)

        d3.place(x = 180, y = 95)
        d3_E.place(x = 230, y = 95)
        cm7.place(x = 290, y = 95)

        #Buttons Frame
        BF = LabelFrame(mygui, text=("Forward Kinematics"), padx = 20, pady =10, font=(26), labelanchor = 'n', bg = '#FFFFFF', fg = 'black')
        BF.place(x = 100, y = 420)

        #Buttons
        FK = Button(BF, text=("↓ FORWARD"), font=(14), bg="#D2B48C", fg=("black"), command=f_k) #arrow down
        rst = Button(BF, text=("↻ RESET"), font=(14), bg="#FFE4C4", fg=("black"), command=reset)
        IK = Button(BF, text=("↑ INVERSE"), font=(14), bg="#906E3E", fg=("black"), command=i_k) #arrow down
        

        

        FK.grid(row=0, column=0)
        rst.grid(row=0, column=1)
        IK.grid(row=1, column=0)

        
        

        #position VEctor Frame
        PV = LabelFrame(mygui, text="Position Vectors", padx = 20, pady = 20, labelanchor = 'n', font=(26), bg = '#FFFFFF', fg = 'black')
        PV.place(x = 140, y = 240)

        #Position Vector Label
        X = Label(PV, text="X = ", font=(14), bg = '#F3EAD3', fg = 'black')
        X_E = Entry(PV, width=5, font=(14))
        cm6 = Label(PV, text=("cm "), font=(14), bg = '#F3EAD3', fg = 'black')

        Y = Label(PV, text="Y = ", font=(14), bg = '#F3EAD3', fg = 'black')
        Y_E = Entry(PV, width=5, font=(14))
        cm7 = Label(PV, text=("cm "), font=(14), bg = '#F3EAD3', fg = 'black')

        Z = Label(PV, text="Z = ", font=(14), bg = '#F3EAD3', fg = 'black')
        Z_E = Entry(PV, width=5, font=(14))
        cm8 = Label(PV, text=("cm "), font=(14), bg = '#F3EAD3', fg = 'black')

        X.grid(row=0, column=0)
        X_E.grid(row=0, column=1)
        cm6.grid(row=0, column=2)

        Y.grid(row=1, column=0)
        Y_E.grid(row=1, column=1)
        cm7.grid(row=1, column=2)

        Z.grid(row=2, column=0)
        Z_E.grid(row=2, column=1)
        cm8.grid(row=2, column=2)

        # Homogeneous Transformation Matrix/ Computation result
        Matrix_frame = LabelFrame(mygui, text = "Homogeneous Transformation Matrix", padx =20, pady = 20,
        labelanchor = 'n', height = 220, width = 450, font = 44, bg = '#9A7B4F', fg = 'white')
        Matrix_frame.place(x = 500, y = 170)
        Matrix_frame.pack_propagate(0)


root = Tk()
app = SplashScreen(root)
root.after(5000, app.transition)
root.mainloop()