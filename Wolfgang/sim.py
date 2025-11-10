import os
import numpy as np
import time
import json
import mujoco
import mujoco.viewer
import placo
import pandas
from placo_utils.tf import tf
from bam.model import load_model
from bam.mujoco import MujocoController

NUMBER_DYNAMIXELS = 20

class MujocoSimulation2R:
    def __init__(self, testbench: str):
        """
        Loading the Wolfgang simulation
        """
        this_directory = os.path.dirname(os.path.realpath(__file__))

        #TODO: Anpassen auf mein Modell
        self.model: mujoco.MjModel = mujoco.MjModel.from_xml_path(
            f"{this_directory}/wolfgang_assets/wolfgang_scene.xml"
        )
        self.data: mujoco.MjData = mujoco.MjData(self.model)
        self.testbench = testbench

        # Placo robot
        self.robot = None

        self.viewer = None
        self.viewer_start = None
        self.t: float = 0
        self.dt: float = self.model.opt.timestep
        self.frame: int = 0

    def step(self, controllers: list = []) -> None:
        # Drehmomente und Reibungen aktualisieren
        for controller in controllers:
            controller.update()

        self.t = self.frame * self.dt
        mujoco.mj_step(self.model, self.data) # Simulationsschritt ausführen
        self.frame += 1

    def reset(self):
        self.t = 0
        self.frame = 0
        self.viewer_start = time.time()

    def render(self, realtime: bool = True):
        """
        Renders the visualization of the simulation.

        Args:
            realtime (bool, optional): if True, render will sleep to ensure real time viewing. Defaults to True.
        """
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer_start = time.time()

        if realtime:
            current_ts = self.viewer_start + self.frame * self.dt
            to_sleep = current_ts - time.time()
            if to_sleep > 0:
                time.sleep(to_sleep)

        self.viewer.sync()

    def simulate_log(
        self, data: dict, params: str, replay: bool = False, render: bool = False
    ):
        if self.robot is None:
            this_directory = os.path.dirname(os.path.realpath(__file__))
            self.robot = placo.RobotWrapper(
                this_directory + f"/2r_{self.testbench}/robot.urdf",   #TODO: URDF für mein Modell finden
                placo.Flags.ignore_collisions,
            )
            if self.testbench in ["mx"]:
                self.robot.set_T_world_frame(
                    "base", tf.rotation_matrix(np.pi, [1, 0, 0])
                )

        # Updating actuator KP
        model_dic = {}
        if "," in params:
            for index, param in enumerate(params.split(",")):
                model_dic[f"param_{index + 1}"] = load_model(param) # Laden der Reibungsmodelle (model.py)
        else:
            for i in range(self.robot.nq):
                model_dic[f"param_{index + 1}"] = load_model(params) # Laden der Reibungsmodelle (model.py)

        if type(data["kp"]) is list:
            for index in range(self.robot.nq):
                model_dic[f"param_{index + 1}"].actuator.kp = data["kp"][index]
        else:
            for index in range(self.robot.nq):
                model_dic[f"param_{index + 1}"].actuator.kp = data["kp"]

        #TODO: Fix names of joints -> entries dxl_1, dxl_2, ...

        # Creating bam controllers
        if not replay:
            dxl_dic = {}
            for index in range(self.robot.nq):
                dxl_dic[f"dxl_{index + 1}"] = MujocoController(
                    model_dic[f"param_{index + 1}"], f"dxl_{index+1}",self.model, self.data
                ) # MujocoModelle, auf denenn Drehmomente und Reibungen angewendet werden (simuliert)

        #TODO: Code Refactoring beenden -> record und trajectory vorher refactorn

        # Setting initial configuration
        for index in range(NUMBER_DYNAMIXELS):
            self.data.joint(f"dxl_{index + 1}").qpos[0] = data["entries"][0][f"dxl_{index + 1}"]["position"] # Startposition setzen

        log_t0 = data["entries"][0]["timestamp"]
        self.reset()
        entry_index = 0
        running = True

        while running:
            entry = data["entries"][entry_index]

            if not replay:
                self.step(list(dxl_dic.values())) # Simulationsschritt wird ausgeführt
            else:
                self.step() # Simulationsschritt wird ausgeführt

            if render:
                self.render()

            if replay:
                # If it's a replay, simply jump to the read position
                for index in range(NUMBER_DYNAMIXELS):
                    self.data.joint(f"dxl_{index + 1}").qpos[0] = entry[f"dxl_{index + 1}"]["position"] # Aktuelle Position wird auf die im Log gespeicherte Position gesetzt

            else:
                for index in range(NUMBER_DYNAMIXELS):
                    dxl_dic[f"dxl_{index + 1}"].set_q_target(f"dxl_{index + 1}", entry[f"dxl_{index + 1}"]["goal_position"]) # Zielposition für den jeweiligen Freiheitsgrad setzen

            while running and (log_t0 + self.t >= entry["timestamp"]):
                entry = data["entries"][entry_index]
                entry_index += 1
                for index in range(NUMBER_DYNAMIXELS):
                    entry[f"dxl_{index + 1}"]["sim_position"] = self.data.joint(f"dxl_{index + 1}").qpos[0] # Simulierte Position wird im Log gespeichert

                #TODO: Überprüfen, ob Endeffektoren relevant sind
                entry["end_effector"] = {}
                for position in "position", "goal_position", "sim_position":
                    for index in range(NUMBER_DYNAMIXELS):
                        self.robot.set_joint(f"dxl_{index + 1}", entry[f"dxl_{index + 1}"][position]) # Gelenkwinkel für den jeweiligen Freiheitsgrad setzen
                    self.robot.update_kinematics() # Kinematik des Roboters aktualisieren -> Wie?
                    pos = self.robot.get_T_world_frame("end")[:3, 3] # Position des Endeffektors im Weltkoordinatensystem abrufen
                    entry["end_effector"][position] = pos

                if entry_index == len(data["entries"]):
                    running = False


if __name__ == "__main__":
    import argparse
    import matplotlib.pyplot as plt

    args_parser = argparse.ArgumentParser()
    args_parser.add_argument("--log", type=str, default="Wolfgang/log.json", nargs="+")
    args_parser.add_argument("--params", type=str, default=[], nargs="+") # Modell
    args_parser.add_argument("--testbench", type=str, required=True)
    args_parser.add_argument("--replay", action="store_true")
    args_parser.add_argument("--render", action="store_true")
    args_parser.add_argument("--plot", action="store_true")
    args_parser.add_argument("--plot_joint", action="store_true")
    args_parser.add_argument("--vertical", action="store_true")
    args_parser.add_argument("--mae", action="store_true")
    args = args_parser.parse_args()

    # Loading bam model
    sim = MujocoSimulation2R(testbench=args.testbench)
    maes = {}

    for log in args.log:
        # Loading log
        data = json.load(open(log)) # Laden der Simulationsdaten (Positionen, Geschwindigkeiten, Steuerwerte, Zeitstempel) (?)
        maes[log] = {}
        n = len(args.params)

        if args.plot:
            # Creating n horizontal subplots
            if args.vertical:
                f, axs = plt.subplots(n, 1, sharex=True)
            else:
                f, axs = plt.subplots(1, n, sharey=True)

            if n == 1:
                axs = [axs]
            # Setting figure size
            f.set_size_inches(12, 4)
        else:
            axs = [None] * n

        for params, ax in zip(args.params, axs):
            # Simulation wird ausgeführt
            sim.simulate_log(data, params, args.replay, args.render)

            # MAE berechnen und darstellen
            mae = 0
            for index in range(NUMBER_DYNAMIXELS):
                dof = f"dxl_{index + 1}"
                errors = [
                    entry[dof]["position"] - entry[dof]["sim_position"]
                    for entry in data["entries"]
                ]
                mae += np.mean(np.abs(errors))
            mae /= 2 # Durchschnittlicher MAE über beide Freiheitsgrade
            maes[log][params] = mae

            # Zeigt den Verlauf der Endeffektorposition an -> #TODO: Anpassen auf mein Modell
            if args.plot:
                for position in "position", "goal_position", "sim_position":
                    ax.plot(
                        [
                            entry["end_effector"][position][0]
                            for entry in data["entries"]
                        ],
                        [
                            entry["end_effector"][position][2]
                            for entry in data["entries"]
                        ],
                        label=position,
                        ls="--" if position == "goal_position" else "-",
                    )
                ax.legend()
                ax.grid()
                ax.set_aspect("equal", adjustable="box")
                ax.set_title(f"{os.path.basename(log)}, {params}")

            # Visualisierung der Gelenkbewegungen und die Differenz zwischen gemessenen und simulierten Werten 
            if args.plot_joint:
                for index in range(NUMBER_DYNAMIXELS):
                    dof = f"dxl_{index + 1}"
                    # Creating two subplots axises
                    f, (ax1, ax2) = plt.subplots(2, sharex=True)

                    goal_positions = [
                        entry[dof]["goal_position"] for entry in data["entries"]
                    ]
                    positions = [entry[dof]["position"] for entry in data["entries"]]
                    sim_positions = [
                        entry[dof]["sim_position"] for entry in data["entries"]
                    ]

                    ax1.plot(goal_positions, label=f"{dof} goal", color="red")
                    ax1.plot(positions, label=f"{dof} read", color="blue")
                    ax1.plot(sim_positions, label=f"{dof} sim", color="green")
                    ax1.grid()
                    ax1.legend()

                    errors = [read - sim for read, sim in zip(positions, sim_positions)]
                    mae = np.mean(np.abs(errors))
                    print("MAE: ", mae)
                    ax2.plot(errors, color="black", label="Simulation error")
                    ax2.set_ylim(-0.05, 0.05)
                    ax2.grid()

                    plt.title(f"{log}, {params}")
                    plt.show()

        if args.plot:
            plt.tight_layout() # Plots überlappen sich nicht
            plt.show()

    # Durschnittlichen MAE pro Modell über alle Logs darstellen (?)
    if args.mae:
        total_mae = {params: [] for params in args.params}
        for log in maes:
            print(f"Log: {log}")
            for params in maes[log]:
                print(f"  {params}: {maes[log][params]}")
                total_mae[params] += [maes[log][params]]

        labels = [os.path.basename(log) for log in maes]
        df = pandas.DataFrame(total_mae, index=labels)
        df.plot(kind="bar")
        plt.grid(axis="y")
        # Setting x label with 45°, keeping the top aligned
        plt.xticks(rotation=45, ha="right")
        plt.title("MAE per log")
        plt.tight_layout()
        plt.show()

        for params in total_mae:
            print(f"Total MAE for {params}: {np.mean(total_mae[params])}")
