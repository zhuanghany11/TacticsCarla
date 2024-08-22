import carla
import pygame
import numpy as np



# Abstract vehicle controller interface
class VehicleController:
    def apply_control(self, vehicle, clock, control_input):
        raise NotImplementedError("This method should be overridden by subclasses.")


# Carla-based autopilot controller
class CarlaController(VehicleController):
    def apply_control(self, vehicle, clock, control_input=None):
        # Use CARLA autopilot
        vehicle.set_autopilot(True)


# Keyboard input controller
class KeyboardController(VehicleController):
    def __init__(self):
        self.control = carla.VehicleControl()
        self.keys = {
            pygame.K_UP: 'throttle',
            pygame.K_DOWN: 'brake',
            pygame.K_LEFT: 'steer_left',
            pygame.K_RIGHT: 'steer_right'
        }
        self._steer_cache = 0

    def apply_control(self, vehicle, clock,control_input=None):
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN or event.type == pygame.KEYUP:
                self._update_control(event.key, clock.get_time(), event.type)

        vehicle.apply_control(self.control)

    def _update_control(self, key, milliseconds, event_type):
        # Check if the key is pressed or released
        is_pressed = event_type == pygame.KEYDOWN

        if self.keys.get(key) == 'throttle':
            self.control.throttle = 0.4 if is_pressed else 0.0
        elif self.keys.get(key) == 'brake':
            self.control.brake = 0.4 if is_pressed else 0.0
        elif self.keys.get(key) == 'steer_left':
            self.control.steer = -0.3 if is_pressed else 0.0
        elif self.keys.get(key) == 'steer_right':
            self.control.steer = 0.3 if is_pressed else 0.0


# Customized controller for autonomous driving
class CustomizedController(VehicleController):
    def apply_control(self, vehicle, clock, control_input):
        # Apply control using custom algorithm
        vehicle.apply_control(control_input)


# Main agent class to manage vehicle and controller
class Controller:
    def __init__(self, control_type, vehicle):
        self.vehicle = vehicle
        self.controller = self._get_controller(control_type)

    def _get_controller(self, control_type):
        if control_type == 'carla':
            return CarlaController()
        elif control_type == 'keyboard':
            return KeyboardController()
        elif control_type == 'customized':
            return CustomizedController()
        else:
            raise ValueError(f"Unknown control type: {control_type}")

    def update(self, clock, control_input=None):
        self.controller.apply_control(self.vehicle, clock, control_input)

