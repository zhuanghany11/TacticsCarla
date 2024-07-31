from config import get_config
from tacticscarla_interface import TacticsCarlaManager

parser = get_config()
args = parser.parse_args()

tactics_carla_manager = TacticsCarlaManager(args)

tactics_carla_manager.reset()

# tactics_carla_manager.spawn_vehicles([[335.49,273.74,0.30],[299.40, 133.24, 0.30]],[[1.8],[0.9]])
tactics_carla_manager.spawn_vehicles()
tactics_carla_manager.spawn_walkers()
tactics_carla_manager.set_speed_difference_percentage(30.00)

try:
    tactics_carla_manager.run(step_num = 1000)

except KeyboardInterrupt:
    pass

finally:
    tactics_carla_manager.cleanup()