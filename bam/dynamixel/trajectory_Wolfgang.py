import numpy as np
import placo

from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

from .trajectory_generator_node import TrajectoryGeneratorNode

#TODO: Mit quintic_walk koppeln (ROS package)

class Trajectory_2R:
    init_duration = None
    traj_duration = None
    init_pos = None

    def __init__(self) -> None:    
        # Load the robot
        self.robot = placo.RobotWrapper('Wolfgang/wolfgang_assets', placo.Flags.ignore_collisions)

        # Set initial configuration
        #TODO: Aus dem Urdf die Gelenknamen auslesen

        for joint in self.robot.joint_names:
            self.robot.set_joint(joint, 1e-5)
            # self.robot.set_joint("R1", 1e-5)
            # self.robot.set_joint("R2", 1e-5)
        self.robot.update_kinematics()  # Damit der Roboter die Kinematik aktualisiert (wegen der gesetzten Gelenkwerte)
        self.solver = self.robot.make_solver()  

        # TODO: Überprfüfe, ob so richtig ist -> Hängt davon ab, wie der Roboter im URDF definiert ist
        # Base is fixed
        T_world_base = self.robot.get_T_world_frame("base")
        base_task = self.solver.add_frame_task("base", T_world_base)
        base_task.configure("base", "hard", 1., 1.)

        # Creating end task 
        self.y_offset = self.robot.get_T_world_frame("end")[1, 3]
        self.x_init = self.robot.get_T_world_frame("end")[0, 3]
        self.z_init = self.robot.get_T_world_frame("end")[2, 3]
        self.end_task = self.solver.add_position_task("end", np.array([self.x_init, self.y_offset, self.z_init]))
        self.end_task.configure("end", "soft", 1.0)

    def __call__(self, t: float):
        """
        Retrieve (r1, r2) at time t
        """
        raise NotImplementedError
    
class QuinticWalk(Trajectory_2R):
    init_duration = 2.0 #TODO: Brauche ich das?
    traj_duration = 10.0 #TODO: Brauche ich das?

    def __init__(self) -> None:
        super().__init__()
        self.traj_generator = TrajectoryGeneratorNode()

        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = 0.0

        self.clock = Clock()
        self.clock.clock.sec = 0
        self.clock.clock.nanosec = 0


    def __call__(self, t: float):
        self.clock.clock.sec = int(t)
        self.clock.clock.nanosec = int((t - self.clock.clock.sec) * 1_000_000_000)

        if t < self.init_duration:
            self.cmd_vel.linear.x = 0.0
            joint_states = self.robot.get_joint_states_message() #TODO: Auf die richtige Weise joint_states erfassen
            msg = self.traj_generator.calculate_trajectory(self.clock, self.cmd_vel, joint_states)

        else:
            self.cmd_vel.linear.x = 0.3  # Gehe vorwärts mit 0.3 m/s
            joint_states = self.robot.get_joint_states_message() #TODO: Auf die richtige Weise joint_states erfassen
            msg = self.traj_generator.calculate_trajectory(self.clock, self.cmd_vel, joint_states)

        joint_names = list(msg.joint_names)
        positions = list(msg.positions)
        velocities = list(msg.velocities)

        for i, name in enumerate(joint_names):
            self.robot.set_joint(name, float(positions[i]))
            if velocities[i] != -1.0:
                self.robot.set_joint_velocity(name, float(velocities[i]))
            else:
                pass #TODO: Maximale Geschwindigkeit setzen
        
        self.robot.update_kinematics()

        joints = []
        for joint_name in self.robot.joint_names:
            joints.append(self.robot.get_joint(joint_name))
        return joints
        
"""

class Square(Trajectory_2R): # Vererbung von Trajectory_2R
    init_duration = 6.0
    traj_duration = 8.0

    def __init__(self) -> None:
        super().__init__()  

        # Init spline 
        self.init_spline = placo.CubicSpline3D()
        self.init_pos = np.array([0.05, self.y_offset, 0.35])

        # Kontrollpunkte mit Zeit, Position und Geschwindigkeit werden eingefügt -> Erzeugt eine sanfte Initialbewegung
        self.init_spline.add_point(0, np.array([self.x_init, self.y_offset, self.z_init]), np.array([0.0, 0.0, 0.0]))
        self.init_spline.add_point(1.5, self.init_pos + np.array([0.0, 0.0, -0.03]), np.array([0.0, 0.0, 0.0]))
        self.init_spline.add_point(2.5, self.init_pos, np.array([0.0, 0.0, 0.0]))
        self.init_spline.add_point(6, self.init_pos, np.array([0.0, 0.0, 0.0]))

        # Trajetory spline to add to init_pos -> Erzeugt die Rechteckbewegung
        self.traj_spline = placo.CubicSpline3D()
        self.traj_spline.add_point(0, np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))
        self.traj_spline.add_point(2, np.array([0.0, 0.0, -0.1]), np.array([0.0, 0.0, 0.0]))
        self.traj_spline.add_point(4, np.array([-0.1, 0.0, -0.1]), np.array([0.0, 0.0, 0.0]))
        self.traj_spline.add_point(6, np.array([-0.1, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))
        self.traj_spline.add_point(8, np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))

    # Ausführung der Trajektorie für einen gegebenen Zeitpunkt t
    def __call__(self, t: float):
        if t < self.init_duration:
            self.end_task.target_world = self.init_spline.pos(t)

        else:
            self.end_task.target_world = self.init_pos + self.traj_spline.pos(t - self.init_duration)
        
        self.solver.solve(True)
        self.robot.update_kinematics()
        return self.robot.get_joint("R1"), self.robot.get_joint("R2")


class SquareWave(Trajectory_2R):
    init_duration = 6.0
    seg_duration = 1.5

    def __init__(self, nb_waves=9) -> None:
        super().__init__()

        self.traj_duration = 2*self.seg_duration*nb_waves

        # Init spline 
        self.init_spline = placo.CubicSpline3D()
        self.init_pos = np.array([0.2, self.y_offset, 0.3])

        self.init_spline.add_point(0, np.array([self.x_init, self.y_offset, self.z_init]), np.array([0.0, 0.0, 0.0]))
        self.init_spline.add_point(1.5, self.init_pos + np.array([0.0, 0.0, -0.03]), np.array([0.0, 0.0, 0.0]))
        self.init_spline.add_point(2.5, self.init_pos, np.array([0.0, 0.0, 0.0]))
        self.init_spline.add_point(6, self.init_pos, np.array([0.0, 0.0, 0.0]))

        # Trajetory spline to add to init_pos 
        self.traj_spline = placo.CubicSpline3D()
        self.traj_spline.add_point(0, np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))
        for i in range(nb_waves):
            if i % 2 == 0:
                self.traj_spline.add_point(self.seg_duration*(2*i+1), np.array([-i*0.4/nb_waves, 0.0, -0.05]), np.array([0.0, 0.0, 0.0]))
                self.traj_spline.add_point(self.seg_duration*(2*i+2), np.array([-(i+1)*0.4/nb_waves, 0.0, -0.05]), np.array([0.0, 0.0, 0.0]))
            else:
                self.traj_spline.add_point(self.seg_duration*(2*i+1), np.array([-i*0.4/nb_waves, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))
                self.traj_spline.add_point(self.seg_duration*(2*i+2), np.array([-(i+1)*0.4/nb_waves, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))
        if nb_waves % 2 == 1:
            self.traj_spline.add_point(self.seg_duration*(2*nb_waves+1), np.array([-0.4, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))
            self.traj_duration += 2

    def __call__(self, t: float):
        if t < self.init_duration:
            self.end_task.target_world = self.init_spline.pos(t)

        else:
            self.end_task.target_world = self.init_pos + self.traj_spline.pos(t - self.init_duration)
        
        self.solver.solve(True)
        self.robot.update_kinematics()
        return self.robot.get_joint("R1"), self.robot.get_joint("R2")


class TriangularWave(Trajectory_2R):
    init_duration = 6.0
    seg_duration = 1.5

    def __init__(self, nb_waves=5) -> None:
        super().__init__()

        self.traj_duration = 2*self.seg_duration*nb_waves

        # Init spline 
        self.init_spline = placo.CubicSpline3D()
        self.init_pos = np.array([0.2, self.y_offset, 0.3])

        self.init_spline.add_point(0, np.array([self.x_init, self.y_offset, self.z_init]), np.array([0.0, 0.0, 0.0]))
        self.init_spline.add_point(1.5, self.init_pos + np.array([0.0, 0.0, -0.03]), np.array([0.0, 0.0, 0.0]))
        self.init_spline.add_point(2.5, self.init_pos, np.array([0.0, 0.0, 0.0]))
        self.init_spline.add_point(6, self.init_pos, np.array([0.0, 0.0, 0.0]))

        # Trajetory spline to add to init_pos
        self.traj_spline = placo.CubicSpline3D()
        self.traj_spline.add_point(0, np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))
        for i in range(nb_waves):
            self.traj_spline.add_point(self.seg_duration*(2*i+1), np.array([-(2*i+1)*0.2/nb_waves, 0.0, -0.05]), np.array([0.0, 0.0, 0.0]))
            self.traj_spline.add_point(self.seg_duration*(2*i+2), np.array([-(2*i+2)*0.2/nb_waves, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))
        

    def __call__(self, t: float):
        if t < self.init_duration:
            self.end_task.target_world = self.init_spline.pos(t)

        else:
            self.end_task.target_world = self.init_pos + self.traj_spline.pos(t - self.init_duration)
        
        self.solver.solve(True)
        self.robot.update_kinematics()
        return self.robot.get_joint("R1"), self.robot.get_joint("R2")


class Circle(Trajectory_2R):
    init_duration = 6.0
    traj_duration = 8.0
    radius = 0.05

    def __init__(self) -> None:
        super().__init__()

        # Init spline 
        self.init_spline = placo.CubicSpline3D()
        self.init_pos = np.array([0.0, self.y_offset, 0.35])

        self.init_spline.add_point(0, np.array([self.x_init, self.y_offset, self.z_init]), np.array([0.0, 0.0, 0.0]))
        self.init_spline.add_point(1.5, self.init_pos + np.array([0.0, 0.0, -0.03]), np.array([0.0, 0.0, 0.0]))
        self.init_spline.add_point(2.5, self.init_pos, np.array([0.0, 0.0, 0.0]))
        self.init_spline.add_point(6, self.init_pos, np.array([0.0, 0.0, 0.0]))

    def __call__(self, t: float):
        if t < self.init_duration:
            self.end_task.target_world = self.init_spline.pos(t)

        else:
            x = self.radius * np.sin(2 * np.pi * (t - self.init_duration) / self.traj_duration)
            z = self.radius * np.cos(2 * np.pi * (t - self.init_duration) / self.traj_duration)
            self.end_task.target_world = self.init_pos + np.array([x, 0.0, z - self.radius])
        
        self.solver.solve(True)
        self.robot.update_kinematics()
        return self.robot.get_joint("R1"), self.robot.get_joint("R2")

"""

trajectories = {
    #"square": Square(),
    #"circle": Circle(),
    #"square_wave": SquareWave(),
    #"triangular_wave": TriangularWave(),
    "quintic_walk": QuinticWalk()
}

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # trajectory = Square()
    # trajectory = Circle()
    # trajectory = SquareWave()
    # trajectory = TriangularWave()
    trajectory = QuinticWalk()

    print("Duration:", trajectory.traj_duration)

    ts = np.linspace(0.0, trajectory.init_duration + trajectory.traj_duration, 1000)
    rs = [trajectory(t) for t in ts]
    rs = np.array(rs)

    for i in range(rs.shape[1]):
        plt.plot(ts, rs[:, i], label=f"dxl_{i+1}")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (rad)")
    plt.legend()
    plt.grid()
    plt.show()

    truncated_rs = rs[ts > trajectory.init_duration]

    x = []
    z = []
    for r in truncated_rs:
        for joint_name in trajectory.robot.joint_names:
            trajectory.robot.set_joint(f"dxl_{i+1}", r[i])
        trajectory.robot.update_kinematics()
        end = trajectory.robot.get_T_world_frame("end")
        x.append(end[0, 3])
        z.append(end[2, 3])

    plt.plot(x, z)
    plt.axis('equal')
    plt.show()
