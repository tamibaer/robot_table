from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch

# setup assistant (wichtige umgebungsvariablen, damit es funktioniert): 
# export DISPLAY=:0
# export QT_QPA_PLATFORM=xcb
# export LIBGL_ALWAYS_SOFTWARE=1
# export WAYLAND_DISPLAY=wayland-0

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot_table", package_name="robot_table_moveit_config").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
