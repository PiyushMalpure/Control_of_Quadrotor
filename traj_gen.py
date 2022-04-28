from turtle import position
from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


class QuinticTraj():
    def __init__(self, tspan, pts_init, pts_final, vel_init, vel_final, acc_init, acc_final) :
        self.t0 = tspan[0]
        self.tf = tspan[1]

        self.x0 = pts_init[0]
        self.y0 = pts_init[1]
        self.z0 = pts_init[2]
        
        self.xf = pts_final[0]
        self.yf = pts_final[1]
        self.zf = pts_final[2]
        

        self.dx0 = vel_init[0]
        self.dy0 = vel_init[1]
        self.dz0 = vel_init[2]
        
        self.dxf = vel_final[0]
        self.dyf = vel_final[1]
        self.dzf = vel_final[2]


        self.ddx0 = acc_init[0]
        self.ddy0 = acc_init[1]
        self.ddz0 = acc_init[2]
        
        self.ddxf = acc_final[0]
        self.ddyf = acc_final[1]
        self.ddzf = acc_final[2]
        #self.xf = 

    def GenerateQuinticTraj(self):
        A = [[1, self.t0, self.t0**2, self.t0**3, self.t0**4, self.t0**5],
             [0, 1, 2*self.t0, 3*self.t0**2, 4*self.t0**3, 5*self.t0**4],
             [0, 0, 2, 6*self.t0, 12*self.t0**2, 20*self.t0**3],
             [1, self.tf, self.tf**2, self.tf**3, self.tf**4, self.tf**5],
             [0, 1, 2*self.tf, 3*self.tf**2, 4*self.tf**3, 5*self.tf**4],
             [0, 0, 2, 6*self.tf, 12*self.tf**2, 20*self.tf**3]]
        
        B_x = [[self.x0],[self.dx0],[self.ddx0], [self.xf],[self.dxf],[self.ddxf]]
        B_y = [[self.y0],[self.dy0],[self.ddy0], [self.yf],[self.dyf],[self.ddyf]]
        B_z = [[self.z0],[self.dz0],[self.ddz0], [self.zf],[self.dzf],[self.ddzf]]
        
        self.trajectory_x = np.dot(np.linalg.inv(np.asarray(A)), np.asarray(B_x))
        # print(self.trajectory_x)
        self.trajectory_y = np.dot(np.linalg.inv(np.asarray(A)), np.asarray(B_y))
        self.trajectory_z = np.dot(np.linalg.inv(np.asarray(A)), np.asarray(B_z))
        return [self.trajectory_x, self.trajectory_y, self.trajectory_z]

    def calculate_position(self, t, coeff):
        position = coeff[0] + coeff[1]*t + coeff[2]*t**2 + coeff[3] * t**3 + coeff[4] * t**4 + coeff[5] * t**5 
        return position
    
    def calculate_velocity(self, t, coeff):
        velocity = 0 + coeff[1] + 2 * coeff[2]*t + 3 * coeff[3] * t**2 + 4 * coeff[4] * t**3 + 5 * coeff[5] * t**4    
        return velocity

    def calculate_acceleration(self, t, coeff):
        acceleration = 0 + 0 + 2 * coeff[2] + 6 * coeff[3] * t + 12 * coeff[4] * t**2 + 20 * coeff[5] * t**3
        return acceleration
    
    def TrajPlot(self):
    
        tym = np.linspace(self.t0,self.tf, 50)
        x_pos = []
        y_pos = []
        z_pos = []

        x_vel = []
        y_vel = []
        z_vel = []

        x_acc = []
        y_acc = []
        z_acc = []
        norm_acc = []
        # print(tym)
        for i in tym:
            # print(i)
            # print(type(self.trajectory_x))
            x_pos.append(self.calculate_position(i,self.trajectory_x))
            y_pos.append(self.calculate_position(i, self.trajectory_y))
            z_pos.append(self.calculate_position(i, self.trajectory_z))

            x_vel.append(self.calculate_velocity(i,self.trajectory_x))
            y_vel.append(self.calculate_velocity(i, self.trajectory_y))
            z_vel.append(self.calculate_velocity(i, self.trajectory_z))

            x_acc.append(self.calculate_acceleration(i,self.trajectory_x))
            y_acc.append(self.calculate_acceleration(i, self.trajectory_y))
            z_acc.append(self.calculate_acceleration(i, self.trajectory_z))

            norm_acc.append(np.linalg.norm([x_acc,y_acc,z_acc]))
        fig = plt.figure()
        ax = Axes3D(fig)
        #ax = plt.axes(projection='3d')
        ax.scatter(x_pos,y_pos,z_pos, c=tym)

        # fig1 = plt.figure()
        # ax1 = Axes3D(fig1)
        # #ax = plt.axes(projection='3d')
        # ax1.scatter(x_vel,y_vel,z_vel, c=tym)

        # fig3 = plt.figure()
        # ax3 = Axes3D(fig3)
        # #ax = plt.axes(projection='3d')
        # ax3.scatter(x_acc,y_acc,z_acc,c=tym)
        fig4 = plt.figure()
        plt.plot(tym, z_pos)
        plt.plot(tym, y_pos)
        plt.plot(tym, x_pos)

        fig4 = plt.figure()
        plt.plot(tym, z_acc)
        plt.plot(tym, y_acc)
        plt.plot(tym, x_acc)
        

        fig5 = plt.figure()
        plt.plot(tym, z_vel)
        plt.plot(tym, y_vel)
        plt.plot(tym, x_vel)
        plt.show()
v0 = [0,0,0]
vf = v0
acc0 = v0
accf = v0
points = [[0,0,0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1], [0, 0, 1]]
traj = []
tspan = [[0, 5], [5, 20], [20, 35], [35, 50], [50, 65]]
for i in range(len(points)-1):
    traj_obj = QuinticTraj(tspan[i], points[i], points[i+1], v0, vf, acc0, accf)
    traj.append(traj_obj.GenerateQuinticTraj())
    traj_obj.TrajPlot()
# traj0 = QuinticTraj([0,5], [1,0,0], [0,2,1], v0, vf, v0,v0)
# coeff = traj0.GenerateQuinticTraj()
# traj0.TrajPlot(coeff)
# print(traj0)
# traj0 = QuinticTraj(tspan=[0,5], x0=np.array([0, 0, 0]), xf = np.array([0, 0, 1]).T).GenerateQuinticTraj()
# traj1 = QuinticTraj(tspan=[5,20], x0=np.array([0, 0, 1]), xf=np.array([1, 0, 1]).T).GenerateQuinticTraj()
# traj2 = QuinticTraj(tspan=[20,35], x0=np.array([1, 0, 1]), xf=np.array([1, 1, 1]).T).GenerateQuinticTraj()
# traj3 = QuinticTraj(tspan=[35,50], x0=np.array([1, 1, 1]), xf=np.array([0, 1, 1]).T).GenerateQuinticTraj()
# traj4 = QuinticTraj(tspan=[50,65], x0=np.array([0, 1, 1]), xf=np.array([0, 0, 1]).T).GenerateQuinticTraj()

   