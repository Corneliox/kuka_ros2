#!/usr/bin/env python3
"""
Voice Terminal Mock
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Menu-driven stand-in for real speech input. Prints a numbered list of
every valid target -- colors (color-detection path) and screw classes
(YOLO-detection path) -- pulled live from hardware_database.py, so the
menu can never offer something pick_place_coordinator.py would reject.
Publishes the chosen target(s) to /voice_command (std_msgs/String), one
message per target, exactly as voice_ai_node.py does.

MULTI-TARGET (this revision): each turn now asks single or multi first.
  - single: same as before -- pick one menu number, publish it.
  - multi: enter a comma-separated list of menu numbers, e.g. "1,3,4".
    All are published back-to-back, in the order typed (FIFO). Ordering
    on the receiving end depends on pick_place_coordinator.py actually
    queuing commands that arrive while busy instead of dropping them --
    this node does NOT wait for completion between publishes, it just
    fires them in order and trusts the coordinator's queue.

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

    def _publish(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self._pub.publish(msg)
        self.get_logger().info(f'Published: "{cmd}"')

    def _resolve_index(self, raw: str):
        """raw is one menu-number token, already stripped. Returns the
        matching command string, or None (with a printed reason) if it's
        not a valid selection."""
        if not raw.isdigit():
            print(f'"{raw}" is not a valid selection -- must be a menu number.')
            return None
        idx = int(raw)
        if idx < 1 or idx > len(self._menu):
            print(f'{idx} is out of range (1-{len(self._menu)}).')
            return None
        cmd, _kind = self._menu[idx - 1]
        return cmd

    def _select_one(self):
        raw = input('[Surgeon] Selection: ').strip().lower()
        if not raw:
            return None
        return self._resolve_index(raw)

    def _select_many(self):
        """Comma-separated menu numbers, e.g. '1, 3,4'. Returns a list of
        command strings in the order typed (FIFO order for dispatch) --
        any invalid token aborts the whole entry rather than publishing a
        partial set, so a typo can't silently drop a target."""
        raw = input('[Surgeon] Selections (comma-separated, e.g. 1,3,4): ').strip().lower()
        if not raw:
            return []
        tokens = [t.strip() for t in raw.split(',') if t.strip()]
        cmds = []
        for t in tokens:
            cmd = self._resolve_index(t)
            if cmd is None:
                print('Aborting this multi-selection -- fix the invalid entry and retry.')
                return []
            cmds.append(cmd)
        return cmds

    def loop(self):
        self._print_menu()
        while rclpy.ok():
            try:
                mode = input('\n[Surgeon] Single or multi target? (s/m/q): ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if mode in ('q', 'quit', 'exit'):
                break
            if not mode:
                continue
            if mode not in ('s', 'm'):
                print(f'"{mode}" not recognized -- enter s, m, or q.')
                continue

            if mode == 's':
                cmd = self._select_one()
                if cmd is not None:
                    self._publish(cmd)
            else:
                cmds = self._select_many()
                for cmd in cmds:  # FIFO -- published in the order typed
                    self._publish(cmd)

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