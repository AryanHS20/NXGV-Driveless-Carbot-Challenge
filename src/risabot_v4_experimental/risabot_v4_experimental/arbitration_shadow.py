"""Stage 7 diagnostic arbitration; publishes JSON and no motion messages."""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .arbitration_core import select_diagnostic_intent


class ArbitrationShadow(Node):
    def __init__(self) -> None:
        super().__init__('v4_arbitration_shadow')
        self.declare_parameter('enabled', False)
        gate_names = ('integration_reviewed', 'command_contract_validated',
                      'stop_preemption_validated', 'timeout_validated',
                      'physical_trials_validated')
        for name in gate_names:
            self.declare_parameter(name, False)
        self.declare_parameter('input_timeout_sec', 0.35)
        self.declare_parameter('lane_only', False)
        self._enabled = bool(self.get_parameter('enabled').value)
        self._lane_only = bool(self.get_parameter('lane_only').value)
        self._gates = {name: bool(self.get_parameter(name).value) for name in gate_names}
        self._timeout = float(self.get_parameter('input_timeout_sec').value)
        self._values = {'state': '', 'permit': False, 'trajectory': {},
                        'parking': {}, 'recovery': {}}
        self._seen = {name: 0.0 for name in self._values}
        self._required_inputs = (
            ('state', 'permit', 'trajectory')
            if self._lane_only
            else tuple(self._values)
        )
        self._decision = None
        self._last_error = ''
        self._status_pub = self.create_publisher(String, '/v4_experimental/arbitration/status', 10)
        self._proposal_pub = self.create_publisher(String, '/v4_experimental/arbitration/proposed_request', 2)
        self.create_subscription(String, '/dashboard_state', lambda msg: self._set('state', msg.data), 10)
        self.create_subscription(Bool, '/motion_permitted', lambda msg: self._set('permit', bool(msg.data)), 10)
        self.create_subscription(String, '/v4_experimental/trajectory/status', lambda msg: self._json('trajectory', msg), 10)
        if not self._lane_only:
            self.create_subscription(String, '/v4_experimental/parking/status', lambda msg: self._json('parking', msg), 10)
            self.create_subscription(String, '/v4_experimental/recovery/status', lambda msg: self._json('recovery', msg), 10)
        self.create_timer(0.1, self._evaluate)
        self.create_timer(0.5, self._publish_status)

    def _set(self, name, value):
        self._values[name], self._seen[name] = value, time.monotonic()

    def _json(self, name, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError(f'{name} status must be an object')
            self._set(name, value)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self._last_error = str(exc)

    def _blockers(self, include_physical_trials=True):
        now = time.monotonic()
        labels = {'integration_reviewed': 'integration review is incomplete',
                  'command_contract_validated': 'command contract is not validated',
                  'stop_preemption_validated': 'stop preemption is not validated',
                  'timeout_validated': 'timeout handling is not validated',
                  'physical_trials_validated': 'physical trials are incomplete'}
        blockers = [label for gate, label in labels.items()
                    if not self._gates[gate]
                    and (include_physical_trials or gate != 'physical_trials_validated')]
        blockers.extend(f'{name} input is stale' for name in self._required_inputs
                        for stamp in (self._seen[name],)
                        if not stamp or now - stamp > self._timeout)
        return blockers

    def _evaluate(self):
        if not self._enabled:
            return
        self._decision = select_diagnostic_intent(
            self._values['state'], self._values['permit'], self._values['trajectory'],
            self._values['parking'], self._values['recovery'])
        # A diagnostic proposal is required to perform the physical trial.
        # It remains non-executable here; Stage 8 owns explicit motion authority.
        blockers = self._blockers(include_physical_trials=False)
        payload = {'algorithm_stage': 7, 'diagnostic_only': True,
                   'can_execute': False, 'source': self._decision.source,
                   'action': self._decision.action, 'reason': self._decision.reason,
                   'reference': self._decision.reference, 'blockers': blockers}
        # Even a diagnostic path proposal is withheld until all review gates pass.
        if not blockers:
            self._proposal_pub.publish(String(data=json.dumps(payload, separators=(',', ':'), sort_keys=True)))

    def _publish_status(self):
        payload = {'algorithm_stage': 7, 'mode': 'shadow', 'enabled': self._enabled,
                   'motion_authority': False, 'can_publish_motion': False,
                   'lane_only': self._lane_only,
                   'can_execute_proposed_request': False, 'validation_gates': self._gates,
                   'blockers': ['node disabled'] if not self._enabled else self._blockers(),
                   'selected_diagnostic_only': None if self._decision is None else {
                       'source': self._decision.source, 'action': self._decision.action,
                       'reason': self._decision.reason, 'reference': self._decision.reference},
                   'last_error': self._last_error}
        self._status_pub.publish(String(data=json.dumps(payload, separators=(',', ':'), sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArbitrationShadow()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
