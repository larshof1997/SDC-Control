import csv
import numpy as np

from math import radians
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import os

from libs import CarDescription, KinematicBicycleModel, generate_cubic_spline
from stanley_controller import StanleyController
from longitudinal_control import LongitudinalController

# Get path to waypoints.csv
with open(os.getcwd() + '/control/data/polygon.csv', newline='') as f:
    rows = list(csv.reader(f, delimiter=','))

# Assuming 'rows' is defined somewhere above this code
polygon_x,polygon_y = [[float(i) for i in row] for row in zip(*rows[1:])]



class Simulation:

    def __init__(self):

        fps = 20.0

        self.dt = 1/fps
        self.map_size_x = 60
        self.map_size_y = 25
        self.frames = 1400
        self.loop = False

class Path:

    def __init__(self):

        # Get path to waypoints.csv
        with open(os.getcwd() + '/control/data/waypoints.csv', newline='') as f:
            rows = list(csv.reader(f, delimiter=','))

        ds = 0.1
        # Assuming 'rows' is defined somewhere above this code
        x,y = [[float(i) for i in row] for row in zip(*rows[1:])]

        # Get path to waypoints.csv
        with open(os.getcwd() + '/control/data/yaw_angles.csv', newline='') as f:
            rows = list(csv.reader(f, delimiter=','))

        # Assuming 'rows' is defined somewhere above this code
        # print(rows)
        yaw = [float(row[0]) for row in rows[1:]]


        self.px, self.py, self.pyaw, _ = generate_cubic_spline(x, y, ds)
        print(self.px)
        print(np.array(x))

        # print(self.pyaw)

        # self.px = np.array(x)
        # self.py = np.array(y)
        # self.pyaw = yaw

class Car:

    def __init__(self, init_x, init_y, init_yaw, sim_params, path_params):

        # Model parameters
        self.x = init_x
        self.y = init_y
        self.yaw = init_yaw
        self.v = 0 #Find a way to implement lateral control
        self.goal_v = 4.0
        self.delta = 0.0
        self.wheelbase = 1.687
        self.max_steer = radians(35) #Stock is 35
        self.dt = sim_params.dt
        self.c_r = 0.01
        self.c_a = 2.0 #Check this

        # Tracker parameters
        self.px = path_params.px
        self.py = path_params.py
        self.pyaw = path_params.pyaw
        self.k = 3.5
        self.ksoft = 15.0
        self.kyaw = 0.0
        self.ksteer = 0.5
        self.crosstrack_error = None
        self.target_id = None

        # Description parameters
        self.overall_length = 2.339 
        self.overall_width = 1.612
        self.tyre_diameter = 0.53
        self.tyre_width = 0.125
        self.axle_track = 1.112
        self.rear_overhang = (self.overall_length - self.wheelbase) / 2

        #Longitudinal control parameters
        self.kp = 0.8
        self.ki = 0.01
        self.kd = 0
    

        self.tracker = StanleyController(self.k, self.ksoft, self.kyaw, self.ksteer, self.max_steer, self.wheelbase, self.px, self.py, self.pyaw)
        self.kbm = KinematicBicycleModel(self.wheelbase, self.dt)
        self.longitudinal = LongitudinalController(self.dt, self.kp, self.ki, self.kd)

    def drive(self,t):
        
        if t > 10: 
            self.goal_v = 8.0
        if t > 20: 
            self.goal_v = 2.0
        if t > 30: 
            self.goal_v = 6.0

        
        self.v = self.longitudinal.update_speed(v_desired=self.goal_v)
        self.delta, self.target_id, self.crosstrack_error = self.tracker.stanley_control(self.x, self.y, self.yaw, self.v, self.delta)
        self.x, self.y, self.yaw = self.kbm.kinematic_model(self.x, self.y, self.yaw, self.v, self.delta)
class Fargs:

    def __init__(self, ax, sim, path, car, desc, outline, fr, rr, fl, rl, rear_axle, annotation, target, yaw_arr, yaw_data, crosstrack_arr, crosstrack_data, velocity_arr, velocity_data):
        
        self. ax = ax
        self.sim = sim
        self.path = path
        self.car = car
        self.desc = desc
        self.outline = outline
        self.fr = fr
        self.rr = rr
        self.fl = fl
        self.rl = rl
        self.rear_axle = rear_axle
        self.annotation = annotation
        self.target = target
        self.yaw_arr = yaw_arr
        self.yaw_data = yaw_data
        self.crosstrack_arr = crosstrack_arr
        self.crosstrack_data = crosstrack_data
        self.velocity_arr = velocity_arr
        self.velocity_data = velocity_data
def animate(frame, fargs):

    ax = fargs.ax
    sim = fargs.sim
    path = fargs.path
    car = fargs.car
    desc = fargs.desc
    outline = fargs.outline
    fr = fargs.fr
    rr = fargs.rr
    fl = fargs.fl
    rl = fargs.rl
    rear_axle = fargs.rear_axle
    annotation = fargs.annotation
    target = fargs.target
    yaw_arr = fargs.yaw_arr
    yaw_data = fargs.yaw_data
    crosstrack_arr = fargs.crosstrack_arr
    crosstrack_data = fargs.crosstrack_data
    velocity_arr = fargs.velocity_arr
    velocity_data = fargs.velocity_data

    ax[0].set_title(f'{sim.dt*frame:.2f}s', loc='right')

    # Camera tracks car
    ax[0].set_xlim(car.x - sim.map_size_x, car.x + sim.map_size_x)
    ax[0].set_ylim(car.y - sim.map_size_y, car.y + sim.map_size_y)

    # Drive and draw car
    car.drive(t=sim.dt*frame)
    outline_plot, fr_plot, rr_plot, fl_plot, rl_plot = desc.plot_car(car.x, car.y, car.yaw, car.delta)
    outline.set_data(outline_plot[0], outline_plot[1])
    fr.set_data(*fr_plot)
    rr.set_data(*rr_plot)
    fl.set_data(*fl_plot)
    rl.set_data(*rl_plot)
    rear_axle.set_data(car.x, car.y)

    # Show car's target
    target.set_data(path.px[car.target_id], path.py[car.target_id])

    # Annotate car's coordinate above car
    annotation.set_text(f"Crosstrack error: {car.crosstrack_error:.5f}")
    annotation.set_position((car.x - 10, car.y + 5))

    # Animate yaw
    yaw_arr.append(car.yaw)
    yaw_data.set_data(np.arange(frame + 1), yaw_arr)
    ax[1].set_ylim(yaw_arr[-1] - 5, yaw_arr[-1] + 5)

    # Animate crosstrack error
    crosstrack_arr.append(car.crosstrack_error)
    crosstrack_data.set_data(np.arange(frame + 1), crosstrack_arr)
    ax[2].set_ylim(crosstrack_arr[-1] - 1, crosstrack_arr[-1] + 1)

    # Animate velocity
    velocity_arr.append(car.v)
    velocity_data.set_data(np.arange(frame + 1), velocity_arr)
    ax[3].set_ylim(0,max(velocity_arr)+2.0)

    return outline, fr, rr, fl, rl, rear_axle, target, yaw_data, crosstrack_data, velocity_data

def main():
    
    sim = Simulation()
    path = Path()
    car = Car(path.px[0], path.py[0], path.pyaw[0], sim, path)
    desc = CarDescription(car.overall_length, car.overall_width, car.rear_overhang, car.tyre_diameter, car.tyre_width, car.axle_track, car.wheelbase)

    interval = sim.dt * 10**3

    fig, ax = plt.subplots(4, 1)

    ax[0].set_aspect('equal')
    ax[0].plot(path.px, path.py, '--', color='gold')
    ax[0].plot(polygon_x, polygon_y, color='black')
    

    annotation = ax[0].annotate(f"Crosstrack error: {float('inf')}", xy=(car.x - 10, car.y + 5), color='black', annotation_clip=False)
    target, = ax[0].plot([], [], '+r')

    outline, = ax[0].plot([], [], color='black')
    fr, = ax[0].plot([], [], color='black')
    rr, = ax[0].plot([], [], color='black')
    fl, = ax[0].plot([], [], color='black')
    rl, = ax[0].plot([], [], color='black')
    rear_axle, = ax[0].plot(car.x, car.y, '+', color='black', markersize=1)

    yaw_arr = []
    yaw_data, = ax[1].plot([], [])
    ax[1].set_xlim(0, sim.frames)
    ax[1].set_ylabel("Yaw")
    ax[1].grid()

    crosstrack_arr = []
    crosstrack_data, = ax[2].plot([], [])
    ax[2].set_xlim(0, sim.frames)
    ax[2].set_ylabel("Crosstrack error")
    ax[2].grid()

    velocity_arr = []
    velocity_data, = ax[3].plot([], [])
    ax[3].set_xlim(0, sim.frames)
    ax[3].set_ylabel("Velocity (m/s)")
    ax[3].grid()

    fargs = [
        Fargs(
            ax=ax,
            sim=sim,
            path=path,
            car=car,
            desc=desc,
            outline=outline,
            fr=fr,
            rr=rr,
            fl=fl,
            rl=rl,
            rear_axle=rear_axle,
            annotation=annotation,
            target=target,
            yaw_arr=yaw_arr,
            yaw_data=yaw_data,
            crosstrack_arr=crosstrack_arr,
            crosstrack_data=crosstrack_data,
            velocity_arr=velocity_arr,
            velocity_data=velocity_data
        )
    ]

    _ = FuncAnimation(fig, animate, frames=sim.frames, init_func=lambda: None ,fargs=fargs, interval=interval, repeat=sim.loop)
    # anim.save('animation.gif', writer='imagemagick', fps=50)
    plt.show()

    print(f"Mean yaw: {np.mean(yaw_arr)}")
    print(f"Mean crosstrack error: {np.mean(crosstrack_arr)}")
    print(f"Mean velocity: {np.mean(velocity_arr)}")

if __name__ == '__main__':
    main()
