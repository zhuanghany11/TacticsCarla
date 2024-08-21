import random
import sys
import time

import pygame

sys.path.append('/TacticsCarla/tactics2d')

from typing import Any, Tuple, List, Dict
import numpy as np
import tactics2d
import xml.etree.ElementTree as ET
from tactics2d.map.parser import OSMParser, XODRParser
from tactics2d.traffic import ScenarioDisplay
from tactics2d.map.element import Map
from tactics2d.map.generator import RacingTrackGenerator
from tactics2d.participant.element import Vehicle, Pedestrian
from tactics2d.participant.trajectory import State
from tactics2d.physics import SingleTrackKinematics
from tactics2d.sensor import RenderManager, TopDownCamera, CarlaSensorBase
from tactics2d.traffic import ScenarioManager, ScenarioStatus, TrafficStatus
from tactics2d.traffic.event_detection import NoAction, OffLane, OutBound, TimeExceed

from .traffic_generator import TrafficGenerator
from .vehicle_controller import Controller

class TacticsCarlaManager:
    def __init__(self, args):
        """Initialize the TacticsCarlaManager.

        Args:
            args: The arguments passed to the script.
        """

        self.number_of_vehicles = args.number_of_vehicles
        self.number_of_walkers = args.number_of_walkers
        self.args = args
        self.step_size = 1
        self.max_steer = 0.75
        self.max_accel = 2.0
        self.vehicles_list = []
        self.walkers_list = []
        self.participants = {}
        self.map_ = self.get_carla_map(args.map_path)
        self.max_steps = 1000
        self.render_fps = 60
        self.off_screen = False
        width = (self.map_.boundary[1] - self.map_.boundary[0]) / 2
        height = (self.map_.boundary[3] - self.map_.boundary[2]) / 2
        window_width = 800
        window_height = height / width * window_width
        self._window_size = (window_width, window_height)

        self.traffic_generator = TrafficGenerator(self.args)

        self.ego_vehicle = None
        self.carla_sensors = dict()


    def carla_setup(self):
        """
        Set up the CARLA simulation environment using the TrafficGenerator.
        """
        self.traffic_generator.setup()

    def spawn_ego_vehicle(self, location=None, rotation=None):
        """Spawn the ego vehicle in the CARLA simulation and create a corresponding tactics2d Vehicle instance.


        Args:
            location (list of list of float, optional): The list of [x, y, z] coordinates for vehicle spawns.
            rotation (list of list of float, optional): The list of [yaw in radians] for vehicle orientations.

        """

        # spawn carla ego vehicle
        if location is not None:
            transform = self.traffic_generator.get_transform(location, rotation)
        else:
            transform = None
        self.carla_ego_vehicle = self.traffic_generator.spawn_ego_vehicle(transform)


        # spawn tactics2d ego vehicle
        ego_vehicle = Vehicle(id_=self.carla_ego_vehicle.id)
        ego_vehicle.load_from_template("medium_car")
        ego_vehicle.physics_model = SingleTrackKinematics(
            lf=ego_vehicle.length / 2 - ego_vehicle.front_overhang,
            lr=ego_vehicle.length / 2 - ego_vehicle.rear_overhang,
            steer_range=(-self.max_steer, self.max_steer),
            speed_range=ego_vehicle.speed_range,
            accel_range=(-self.max_accel, self.max_accel),
            interval=self.step_size,
        )
        self.vehicles_list.append(ego_vehicle)
        self.participants.update({ego_vehicle.id_: ego_vehicle})
        self.ego_vehicle_id = [ego_vehicle.id_]
        return ego_vehicle


    def spawn_vehicles(self, location=None, rotation=None):
        """Spawn vehicles in the CARLA simulation and create corresponding tactics2d Vehicle instances.

        Args:
            location (list of list of float, optional): The list of [x, y, z] coordinates for vehicle spawns.
            rotation (list of list of float, optional): The list of [yaw in radians] for vehicle orientations.
        """

        # spawn carla vehicles
        if location is not None:
            number_of_vehicles = len(location)
            transform = self.traffic_generator.get_transform(location, rotation)
        else:
            number_of_vehicles = None
            transform = None
        self.vehicles_list_id = self.traffic_generator.spawn_vehicles(number_of_vehicles, transform)

        # spawn tactics2d vehicles
        vehicles_num = len(self.vehicles_list_id)
        for i in range(vehicles_num):
            vehicle = Vehicle(id_=self.vehicles_list_id[i])
            vehicle.load_from_template("medium_car")
            vehicle.physics_model = SingleTrackKinematics(
                lf=vehicle.length / 2 - vehicle.front_overhang,
                lr=vehicle.length / 2 - vehicle.rear_overhang,
                steer_range=(-self.max_steer, self.max_steer),
                speed_range=vehicle.speed_range,
                accel_range=(-self.max_accel, self.max_accel),
                interval=self.step_size,
            )
            self.vehicles_list.append(vehicle)
            self.participants.update({vehicle.id_: vehicle})

    def spawn_walkers(self):
        """
        Spawn walkers in the CARLA simulation and create corresponding tactics2d Pedestrian instances.
        """

        # spawn carla walkers
        walkers_list = self.traffic_generator.spawn_walkers()
        self.walkers_list_id = [walker['id'] for walker in walkers_list]
        # spawn tactics2d walkers
        walkers_num = len(self.walkers_list_id)
        for i in range(walkers_num):
            walker = Pedestrian(id_=self.walkers_list_id[i])
            walker.load_from_template("adult_male")
            # Todo: add pedestrian physics model
            self.walkers_list.append(walker)
            self.participants.update({walker.id_: walker})


    def get_participants(self):
        """Get the dictionary of participants in the simulation.

        Returns:
            dict: A dictionary with participant IDs as keys and participant objects as values.
        """

        return self.participants

    def set_speed_difference_percentage(self, percentage):
        """Set the global speed difference percentage for all vehicles in the simulation.

       Args:
           percentage (float): The speed difference percentage to apply.
       """
        self.traffic_generator.set_speed_difference_percentage(percentage)

    def get_carla_map(self,map_path):
        """Parse an OpenDRIVE or OSM map file and return a tactics2d Map object.

        Args:
            map_path (str): The file path to the map XML file.

        Returns:
            tactics2d.map.element.Map: A Map object representing the parsed map.
        """

        map_root = ET.parse(map_path).getroot()
        map_parser = XODRParser()
        map_ = map_parser.parse(map_root)
        return map_

    def render(self):
        """
        Update and render the simulation state using the RenderManager.
        """

        self.render_manager.update(self.participants, self.vehicles_list_id + self.walkers_list_id +self.ego_vehicle_id)
        for id, sensor in self.carla_sensors.items():
            self.render_manager._sensors[id]._surface = sensor.render()

        self.render_manager.render()


    def add_carla_sensors(self,id, window_size, sensor_type, location, rotation, sensor_options):
        """Adds carla sensors  in tactics2d.

            Args:
                id (int): A unique identifier for the sensor.
                window_size (tuple): The size of the sensor's window, as a (width, height) pair.
                sensor_type (str): The type of sensor to be added, e.g., 'RGBCamera', 'LiDAR', 'SemanticLiDAR'.
                location (tuple): The location of the sensor relative to the ego vehicle, as a (x, y, z) tuple.
                rotation (tuple): The rotation of the sensor relative to the ego vehicle, as a (pitch, yaw, roll) tuple.
                sensor_options (dict): Additional options for the sensor, such as attributes or post-processing settings.

            Returns:
                None
        """

        sensorbase = CarlaSensorBase(
            id_=id,
            map_=self.map_,
            window_size=window_size,
            off_screen=self.off_screen,
        )
        self.render_manager.add_sensor(sensorbase,main_sensor=False)

        self.carla_sensors[id] = self.traffic_generator.add_sensor(window_size, sensor_type, location, rotation, sensor_options)




    def update_participants(self, actors_info: List[Dict], frame: int = None) -> List[State]:
        """Update the state of each participant based on the provided actors information.

        Args:
            actors_info (list of dict): A list containing information about each actor's state.
            frame (int, optional): The frame number to update the participant state to.
        """

        for i in range(len(actors_info)):
            x, y, _ = actors_info[i]['location']
            _, yaw_deg, _ = actors_info[i]['rotation']
            yaw_rad = np.radians(yaw_deg)
            vx, vy = actors_info[i]['velocity']
            ax, ay = actors_info[i]['acceleration']
            if self.participants[actors_info[i]['id']].current_state is not None:
                frame = frame if frame is not None else self.step_size
                next_state = State(self.participants[actors_info[i]['id']].current_state.frame + frame, x, -y, -yaw_rad,
                                   vx, -vy, speed=None, ax=ax, ay=-ay, accel=None)
                self.participants[actors_info[i]['id']].add_state(next_state)
            else:
                next_state = State(0, x, y, -yaw_rad, vx, -vy, speed=None, ax=ax,
                                   ay=-ay, accel=None)
                self.participants[actors_info[i]['id']].reset(next_state)


    def run(self, step_num = None):
        """Run the simulation for a specified number of steps.

        Args:
            step_num (int, optional): The number of simulation steps to run. Defaults to self.max_steps.
        """

        if step_num is None:
            step_num = self.max_steps

        t1 = time.time()
        clock = pygame.time.Clock()

        for i in range(step_num):
            if not self.args.asynch and self.traffic_generator.synchronous_master:
                self.traffic_generator.world.tick()
            else:
                self.traffic_generator.world.wait_for_tick()
            self.vehicle_agent.update(clock)
            actors_info = self.traffic_generator.get_actors_info()
            self.update_participants(actors_info)
            self.render()

            # time.sleep(0.1)
        t2 = time.time()
        fps = f"The average fps is {self.max_steps / (t2 - t1): .2f} Hz."
        print(fps)

        self.traffic_generator.cleanup()
        print('\ndone.')

    def reset(self):
        """
        Reset the simulation to its initial state, clearing all participants and reinitializing the environment.
        """
        self.vehicles_list = []
        self.walkers_list = []
        self.vehicles_list_id = []
        self.walkers_list_id = []
        self.participants = {}
        self.ego_vehicle = self.spawn_ego_vehicle()
        self.render_manager = RenderManager(
            fps=self.render_fps, windows_size=self._window_size, layout_style = "block", off_screen=self.off_screen
        )


        camera = TopDownCamera(
            id_=0,
            map_=self.map_,
            # perception_range=(30, 30, 50, 10),
            window_size=self._window_size,
            off_screen=self.off_screen,
        )
        self.render_manager.add_sensor(camera,main_sensor=True)

        self.add_carla_sensors(1, self._window_size, 'RGBCamera', [0, 0, 2.4], [0, -90, 0], {})
        self.add_carla_sensors(2, self._window_size, 'RGBCamera', [0, 0, 2.4], [0, +00, 0], {})
        self.add_carla_sensors(3, self._window_size, 'RGBCamera', [0, 0, 2.4], [0, +90, 0], {})
        self.add_carla_sensors(4, self._window_size, 'RGBCamera', [0, 0, 2.4], [0, 180, 0], {})
        self.add_carla_sensors(5, self._window_size, 'LiDAR', [0, 0, 2.4],  [0, 0, 0], {'channels' : '64', 'range' : '100',  'points_per_second': '250000', 'rotation_frequency': '20'})
        self.add_carla_sensors(6, self._window_size, 'SemanticLiDAR', [0, 0, 2.4], [0, 0, 0],
                               {'channels' : '64', 'range' : '100', 'points_per_second': '100000', 'rotation_frequency': '20'})


        self.vehicle_agent = Controller(control_type='carla', vehicle=self.carla_ego_vehicle)



    def cleanup(self):
        """
        Clean up the simulation by destroying all actors and removing all sensors.
        """
        self.traffic_generator.cleanup()
        for id, sensor in self.carla_sensors.items():
            sensor.destroy()






