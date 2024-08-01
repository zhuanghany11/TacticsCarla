import glob
import os
import sys
import time
import numpy as np
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
from carla import VehicleLightState as vls
import argparse
import logging
from numpy import random

class TrafficGenerator:
    def __init__(self, args):
        """Initialize the TrafficGenerator.

        Args:
            args: The arguments passed to the script.
        """
        self.args = args
        self.world = None
        self.vehicles_list = []
        self.walkers_list = []
        self.all_id = []
        self.synchronous_master = False
        random.seed(args.seed if args.seed is not None else int(time.time()))

        # @todo cannot import these directly.
        self.SpawnActor = carla.command.SpawnActor
        self.SetAutopilot = carla.command.SetAutopilot
        self.SetVehicleLightState = carla.command.SetVehicleLightState
        self.FutureActor = carla.command.FutureActor

    def setup(self):
        """
        Set up the simulation environment, including the traffic manager and world settings.
        """

        # 加载特定的地图，例如 'Town03'
        self.client = carla.Client(self.args.host, self.args.port)
        self.client.set_timeout(10.0)
        self.client.load_world(self.args.map)
        self.world = self.client.get_world()
        current_map = self.world.get_map().name
        print(f"Current loaded map is:: {current_map}")
        self._setup_traffic_manager()
        self._setup_world_settings()
        return self.world


    def _setup_traffic_manager(self):
        """
        Set traffic manager parameters.
        """
        self.traffic_manager = self.client.get_trafficmanager(self.args.tm_port)
        self.traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        if self.args.respawn:
            self.traffic_manager.set_respawn_dormant_vehicles(True)
        if self.args.hybrid:
            self.traffic_manager.set_hybrid_physics_mode(True)
            self.traffic_manager.set_hybrid_physics_radius(70.0)
        if self.args.seed is not None:
            self.traffic_manager.set_random_device_seed(self.args.seed)

    def _setup_world_settings(self):
        """
        Set CARLA world parameters, such as sync mode and render mode
        """
        self.settings = self.world.get_settings()
        if not self.args.asynch:
            self.traffic_manager.set_synchronous_mode(True)
            if not self.settings.synchronous_mode:
                self.synchronous_master = True
                self.settings.synchronous_mode = True
                self.settings.fixed_delta_seconds = 0.05
            else:
                self.synchronous_master = False
        else:
            print("You are currently in asynchronous mode. If this is a traffic simulation, \
            you could experience some issues. If it's not working correctly, switch to synchronous \
            mode by using traffic_manager.set_synchronous_mode(True)")

        if self.args.no_rendering:
            self.settings.no_rendering_mode = True
        self.world.apply_settings(self.settings)

    def get_transform(self, location, rotation):
        """Convert location and rotation data into a list of Carla Transform objects.

        Args:
            location (list of list of float): A list of [x, y, z] coordinates for each actor.
            rotation (list of list of float): A list of [yaw in radians] for each actor's orientation.

        Returns:
            list of carla.Transform: A list of transforms with location and rotation set.
        """

        transform = []
        for i in range(len(location)):
            transform.append(carla.Transform(carla.Location(x=location[i][0], y=location[i][1],z=location[i][2]),
             carla.Rotation(yaw=np.rad2deg(rotation[i][0]))))

        return transform

    def get_spawn_points(self):
        """Retrieve the spawn points available in the current world map.

        Returns:
            list of carla.Transform: A list of spawn points where actors can be spawned.
        """

        return self.world.get_map().get_spawn_points()

    def spawn_vehicles(self,number_of_vehicles = None, transform = None):
        """Spawn vehicles in CARLA.

        Args:
            number_of_vehicles (int, optional): The number of vehicles to spawn.
            transform (list of carla.Transform, optional): The transforms to spawn the vehicles at.

        Returns:
            list: A list of spawned vehicle actor IDs.
        """

        if number_of_vehicles is None:
            number_of_vehicles = self.args.number_of_vehicles

        blueprints = self.get_actor_blueprints(self.args.filterv, self.args.generationv)
        if self.args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
            blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('t2')]
            blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
            blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        if transform is None:
            spawn_points = self.get_spawn_points()
        else:
            spawn_points = transform
        number_of_spawn_points = len(spawn_points)

        if number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, number_of_vehicles, number_of_spawn_points)
            number_of_vehicles = number_of_spawn_points

        print(f"Attempting to spawn {number_of_vehicles} vehicles.")

        batch = []
        hero = self.args.hero
        for n, transform in enumerate(spawn_points):
            if n >= number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            if hero:
                blueprint.set_attribute('role_name', 'hero')
                hero = False
            else:
                blueprint.set_attribute('role_name', 'autopilot')

            # prepare the light state of the cars to spawn
            light_state = vls.NONE
            if self.args.car_lights_on:
                light_state = vls.Position | vls.LowBeam | vls.LowBeam

            # spawn the cars and set their autopilot and light state all together
            batch.append(self.SpawnActor(blueprint, transform)
                .then(self.SetAutopilot(self.FutureActor, True, self.traffic_manager.get_port()))
                .then(self.SetVehicleLightState(self.FutureActor, light_state)))

        for response in self.client.apply_batch_sync(batch, self.synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                self.vehicles_list.append(response.actor_id)

        print(f"Successfully spawned {len(self.vehicles_list)} vehicles.")
        return self.vehicles_list

    def spawn_walkers(self):
        """Spawn walkers in CARLA.

        Returns:
            list: A list of spawned walker actor IDs.
        """

        print(f"Attempting to spawn {self.args.number_of_walkers} vehicles.")
        blueprintsWalkers = self.get_actor_blueprints(self.args.filterw, self.args.generationw)
        percentagePedestriansRunning = 0.0  # how many pedestrians will run
        percentagePedestriansCrossing = 0.0  # how many pedestrians will walk through the road
        # take all the random locations to spawn
        walkers_spawned = 0
        walker_speeds = []
        while walkers_spawned < self.args.number_of_walkers:
            spawn_point = carla.Transform()
            loc = self.world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc

            # spawn the walker object
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed = walker_bp.get_attribute('speed').recommended_values[1]
                else:
                    # running
                    walker_speed = walker_bp.get_attribute('speed').recommended_values[2]
            else:
                print("Walker has no speed")
                walker_speed = 0.0

            res = self.SpawnActor(walker_bp, spawn_point)
            results = self.client.apply_batch_sync([res], True)
            if results[0].error:
                logging.error(results[0].error)
            else:
                self.walkers_list.append({"id": results[0].actor_id})
                walker_speeds.append(walker_speed)
                walkers_spawned += 1


        # spawn the walker controller
        batch = []
        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(self.walkers_list)):
            batch.append(self.SpawnActor(walker_controller_bp, carla.Transform(), self.walkers_list[i]["id"]))
        results = self.client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                self.walkers_list[i]["con"] = results[i].actor_id
        # put together the walkers and controllers id to get the objects from their id
        for i in range(len(self.walkers_list)):
            self.all_id.append(self.walkers_list[i]["con"])
            self.all_id.append(self.walkers_list[i]["id"])
        self.all_actors = self.world.get_actors(self.all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if self.args.asynch or not self.synchronous_master:
            self.world.wait_for_tick()
        else:
            self.world.tick()

        # initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        self.world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(self.all_id), 2):
            # start walker
            self.all_actors[i].start()
            # set walk to random point
            self.all_actors[i].go_to_location(self.world.get_random_location_from_navigation())
            # max speed
            self.all_actors[i].set_max_speed(float(walker_speeds[int(i / 2)]))

        print(f"Successfully spawned {len(walker_speeds)} walkers.")
        return self.walkers_list



    def set_speed_difference_percentage(self,percentage):
        """Set the global speed percentage difference for all vehicles in the simulation.

        Args:
            percentage (float): The speed difference percentage.
        """
        self.traffic_manager.global_percentage_speed_difference(percentage)

    #TODO
    def get_actors_info(self):
        """Retrieve the positions, velocities, and accelerations of all vehicles and walkers in the simulation.

        Returns:
            list of dict: A list containing information about each actor.
        """

        vehicles = self.world.get_actors().filter('vehicle.*')
        walkers = self.world.get_actors().filter('walker.pedestrian.*')

        actors_info = []
        for vehicle in vehicles:
            transform = vehicle.get_transform()
            location = transform.location
            rotation = transform.rotation
            velocity = vehicle.get_velocity()
            acceleration = vehicle.get_acceleration()
            actor_info = {
                'id': vehicle.id,
                'location': (location.x, location.y, location.z),
                'rotation': (rotation.pitch, rotation.yaw, rotation.roll),
                'velocity': (velocity.x, velocity.y),
                'acceleration': (acceleration.x, acceleration.y)
            }
            actors_info.append(actor_info)

        for walker in walkers:
            transform = walker.get_transform()
            location = transform.location
            rotation = transform.rotation
            velocity = walker.get_velocity()
            acceleration = walker.get_acceleration()
            actor_info = {
                'id': walker.id,
                'location': (location.x, location.y, location.z),
                'rotation': (rotation.pitch, rotation.yaw, rotation.roll),
                'velocity': (velocity.x, velocity.y),
                'acceleration': (acceleration.x, acceleration.y)
            }
            actors_info.append(actor_info)

        return actors_info



    def cleanup(self):
        """
        Clean up the simulation by destroying all spawned vehicles and walkers.
        """

        if not self.args.asynch and self.synchronous_master:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(self.vehicles_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(self.all_id), 2):
            self.all_actors[i].stop()

        print('\ndestroying %d walkers' % len(self.walkers_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.all_id])

        time.sleep(0.1)

    def get_actor_blueprints(self, filter, generation):
        """Get a list of actor blueprints filtered by the given criteria.

        Args:
            filter (str): The filter string to search for actor blueprints.
            generation (str): The generation of the actor blueprints to retrieve.

        Returns:
            list: A list of filtered actor blueprints.
        """

        bps = self.world.get_blueprint_library().filter(filter)

        if generation.lower() == "all":
            return bps

        # If the filter returns only one bp, we assume that this one needed
        # and therefore, we ignore the generation
        if len(bps) == 1:
            return bps

        try:
            int_generation = int(generation)
            # Check if generation is in available generations
            if int_generation in [1, 2]:
                bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
                return bps
            else:
                print("   Warning! Actor Generation is not valid. No actor will be spawned.")
                return []
        except:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
