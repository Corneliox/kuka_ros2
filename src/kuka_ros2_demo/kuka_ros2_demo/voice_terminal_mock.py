#!/usr/bin/env python3
"""
Voice Terminal Mock
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Menu-driven stand-in for real speech input. Prints a numbered list of
every valid target -- colors (color-detection path) and screw classes
(YOLO-detection path) -- pulled live from hardware_database.py, so the
menu can never offer something pick_place_coordinator.py would reject.
Publishes the chosen target to /voice_command (std_msgs/String), exactly
as voice_ai_node.py does.

Phase upgrade path:
  - Replace this entire node with a real STT node that publishes
    to the same /voice_command topic. Zero changes elsewhere.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from kuka_ros2_demo.hardware_database import KNOWN_COLORS, KNOWN_SCREW_CLASSES


class VoiceTerminalMock(Node):

    def __init__(self):
        super().__init__('voice_terminal_mock')
        self._pub = self.create_publisher(String, '/voice_command', 10)

        # Build the menu once at startup: colors first, then screw classes,
        # both alphabetized so the numbering is stable run to run.
        self._menu = [(c, 'color') for c in sorted(KNOWN_COLORS)] + \
                     [(s, 'screw') for s in sorted(KNOWN_SCREW_CLASSES)]

        self.get_logger().info('Voice terminal (menu mode) ready.')

    def _print_menu(self):
        print('\n' + '─' * 46)
        print('  Select a target:')
        for i, (name, kind) in enumerate(self._menu, start=1):
            label = 'color' if kind == 'color' else 'screw'
            print(f'   {i:>2}) {name}   [{label}]')
        print('    q) quit')
        print('─' * 46)

    def loop(self):
        self._print_menu()
        while rclpy.ok():
            try:
                raw = input('\n[Surgeon] Selection: ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if raw in ('q', 'quit', 'exit'):
                break
            if not raw:
                continue

            if not raw.isdigit():
                print(f'"{raw}" is not a valid selection -- enter a menu number or "q".')
                continue

            idx = int(raw)
            if idx < 1 or idx > len(self._menu):
                print(f'{idx} is out of range (1-{len(self._menu)}).')
                continue

            cmd, _kind = self._menu[idx - 1]
            msg = String()
            msg.data = cmd
            self._pub.publish(msg)
            self.get_logger().info(f'Published: "{cmd}"')

            self._print_menu()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceTerminalMock()
    try:
        node.loop()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()