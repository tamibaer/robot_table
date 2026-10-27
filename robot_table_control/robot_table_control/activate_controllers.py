import sys

from controller_manager_msgs.msg import ControllerManagerActivity
from controller_manager_msgs.srv import SwitchController
from lifecycle_msgs.msg import State
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
LOADED_STATES = (State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_ACTIVE)


class ActivateControllers(Node):
    def __init__(self, controllers):
        super().__init__('activate_controllers')
        if not controllers:
            raise ValueError('Keine Controller angegeben')
        self.controllers = list(controllers)
        self.wait_for_program = self.declare_parameter('wait_for_program', True).value
        self.settle_time = self.declare_parameter('settle_time', 1.0).value
        self.service_timeout = self.declare_parameter('service_timeout', 10.0).value

        self.controller_states = {}
        self.program_running = False
        self.exit_code = None

        self.create_subscription(ControllerManagerActivity, 'controller_manager/activity', self._on_activity, LATCHED)
        self.create_subscription(Bool, 'io_and_status_controller/robot_program_running', self._on_program_running, LATCHED)

        self.zero_client = self.create_client(Trigger, 'io_and_status_controller/zero_ftsensor')
        self.switch_client = self.create_client(SwitchController, 'controller_manager/switch_controller')

        self.phase = 'waiting'
        self.phase_since = self.now()
        self.pending = None
        self.last_wait_log = None
        self.create_timer(0.1, self._step)
        self.get_logger().info(f'Warte auf {self.controllers}, io_and_status_controller' + (' und laufendes External-Control-Programm' if self.wait_for_program else ''))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _set_phase(self, phase):
        self.phase = phase
        self.phase_since = self.now()

    def _finish(self, ok, msg):
        (self.get_logger().info if ok else self.get_logger().error)(msg)
        self.exit_code = 0 if ok else 1

    def _on_activity(self, msg):
        self.controller_states = {c.name: c.state.id for c in msg.controllers}

    def _on_program_running(self, msg):
        if msg.data != self.program_running:
            self.get_logger().info('External Control ' + ('laeuft' if msg.data else 'gestoppt'))
        self.program_running = msg.data

    def _ready(self):
        loaded = all(self.controller_states.get(c) in LOADED_STATES for c in self.controllers)
        io_active = self.controller_states.get('io_and_status_controller') == State.PRIMARY_STATE_ACTIVE
        program_ok = self.program_running or not self.wait_for_program
        return loaded and io_active and program_ok

    def _already_active(self):
        return all(self.controller_states.get(c) == State.PRIMARY_STATE_ACTIVE for c in self.controllers)

    def _step(self):
        if self.exit_code is not None:
            return

        if self._already_active():
            self._finish(True, f'Bereits aktiv: {self.controllers}')
            return

        if self.phase in ('settling', 'zeroing') and not self._ready():
            self.get_logger().warn('Bedingungen nicht mehr erfuellt, warte erneut')
            self.pending = None
            self._set_phase('waiting')

        if self.phase == 'waiting':
            if self._ready():
                self._set_phase('settling')
            elif self.last_wait_log is None or self.now() - self.last_wait_log > 10.0:
                self.last_wait_log = self.now()
                missing = [c for c in self.controllers + ['io_and_status_controller']
                           if c not in self.controller_states]
                self.get_logger().info(
                    'Warte noch ... '
                    + (f'nicht geladen: {missing} ' if missing else '')
                    + ('' if self.program_running or not self.wait_for_program else 'Programm laeuft nicht'))

        elif self.phase == 'settling':
            if self.now() - self.phase_since >= self.settle_time:
                self._start_zeroing()

        elif self.phase in ('zeroing', 'switching'):
            if self.pending is not None and self.now() - self.phase_since > self.service_timeout:
                self.pending = None
                self._finish(False, f'Timeout in Phase {self.phase}, Admittanz-Kette nicht aktiviert')

    def _start_zeroing(self):
        if not self.zero_client.service_is_ready():
            return  # naechster Timer-Tick versucht es erneut
        self._set_phase('zeroing')
        self.pending = self.zero_client.call_async(Trigger.Request())
        self.pending.add_done_callback(self._on_zero_done)

    def _on_zero_done(self, future):
        if future is not self.pending or self.phase != 'zeroing':
            return  # veraltete Antwort
        self.pending = None
        result = future.result()
        if not (result and result.success):
            self._finish(False, 'F/T-Sensor konnte nicht genullt werden, Admittanz-Kette nicht aktiviert')
            return
        self.get_logger().info('F/T-Sensor genullt')
        self._start_switching()

    def _start_switching(self):
        if not self.switch_client.service_is_ready():
            self._finish(False, 'switch_controller nicht erreichbar')
            return
        self._set_phase('switching')
        request = SwitchController.Request(
            activate_controllers=self.controllers,
            strictness=SwitchController.Request.STRICT,
            activate_asap=True,
        )
        self.pending = self.switch_client.call_async(request)
        self.pending.add_done_callback(self._on_switch_done)

    def _on_switch_done(self, future):
        if future is not self.pending:
            return
        self.pending = None
        result = future.result()
        if result and result.ok:
            self._finish(True, f'Aktiv: {self.controllers}')
        else:
            self._finish(False, f'Aktivieren fehlgeschlagen: {self.controllers}')


def main():
    rclpy.init()
    node = ActivateControllers(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.exit_code = 1
    exit_code = node.exit_code if node.exit_code is not None else 1
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()