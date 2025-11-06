#!/usr/bin/env python3
import os
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

import tf2_ros

# Headless-safe plotting
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


class TFTracer(Node):
    def __init__(self):
        super().__init__(
            'tf_tracer',
            automatically_declare_parameters_from_overrides=True
        )
        # (you can remove the explicit declare of use_sim_time entirely)

        # Params (override with --ros-args -p base_frame:=... etc.)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame',   'ee_link')
        self.declare_parameter('rate_hz',    20.0)
        self.declare_parameter('outfile',    os.path.expanduser('~/ee_trace'))

        self.base = self.get_parameter('base_frame').get_parameter_value().string_value
        self.ee   = self.get_parameter('ee_frame').get_parameter_value().string_value
        hz        = float(self.get_parameter('rate_hz').get_parameter_value().double_value)
        out_param = self.get_parameter('outfile').get_parameter_value().string_value
        self.out  = os.path.expanduser(out_param)

        # Ensure parent directory exists
        out_dir = os.path.dirname(self.out)
        if out_dir and not os.path.isdir(out_dir):
            os.makedirs(out_dir, exist_ok=True)

        # TF
        self.buf = tf2_ros.Buffer()
        self.lst = tf2_ros.TransformListener(self.buf, self)

        # Publishers
        self.path_pub = self.create_publisher(Path, 'ee_path', 10)
        self.mark_pub = self.create_publisher(Marker, 'ee_trace_marker', 10)

        # Accumulators
        self.path = Path()
        self.path.header.frame_id = self.base
        self.samples = []  # list of (x, y, z)
        self.tick_cnt = 0
        self.last_warn = self.get_clock().now()

        # Timers
        self.create_timer(1.0 / max(hz, 1.0), self.tick)  # sample
        self.create_timer(5.0, self.autosave)             # periodic CSV autosave

        self.get_logger().info(
            f'Tracing TF {self.base} -> {self.ee} at {hz:.1f} Hz; saving to {self.out}.*'
        )

    def tick(self):
        # Try to get the latest transform with a small timeout
        try:
            tf = self.buf.lookup_transform(
                self.base, self.ee,
                Time(),  # latest
                timeout=Duration(seconds=0.2)
            )
        except Exception as e:
            now = self.get_clock().now()
            if (now - self.last_warn).nanoseconds > 5e9:
                self.get_logger().warn(f'No transform {self.base}->{self.ee} yet: {e}')
                self.last_warn = now
            return

        t = self.get_clock().now().to_msg()
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        z = tf.transform.translation.z
        self.samples.append((x, y, z))

        if len(self.samples) % 50 == 0:
            self.get_logger().info(f'Collected {len(self.samples)} samples')

        # Publish Path
        ps = PoseStamped()
        ps.header.stamp = t
        ps.header.frame_id = self.base
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0

        self.path.header.stamp = t
        self.path.poses.append(ps)
        self.path_pub.publish(self.path)

        # Publish Marker (LINE_STRIP)
        m = Marker()
        m.header.frame_id = self.base
        m.header.stamp = t
        m.ns = 'ee_trace'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.01
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        m.points = [Point(x=a, y=b, z=c) for (a, b, c) in self.samples]
        self.mark_pub.publish(m)

    def autosave(self):
        """Lightweight periodic CSV save so we don't rely on clean shutdown."""
        try:
            if len(self.samples) == 0:
                return
            arr = np.array(self.samples)
            np.savetxt(f'{self.out}.csv', arr, delimiter=',', header='x,y,z', comments='')
            with open(f'{self.out}.ok', 'w') as f:
                f.write(str(len(self.samples)))
        except Exception as e:
            self.get_logger().error(f'autosave failed: {e}')

    def _save_plots(self):
        if len(self.samples) == 0:
            return
        arr = np.array(self.samples)

        # 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(arr[:, 0], arr[:, 1], arr[:, 2])
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_zlabel('z [m]')
        ax.set_title('End-Effector 3D Trajectory')

        def span(v):
            r = float(np.max(v) - np.min(v))
            return r if r > 0 else 1.0
        ax.set_box_aspect((span(arr[:, 0]), span(arr[:, 1]), span(arr[:, 2])))
        fig.savefig(f'{self.out}_3d.png', dpi=200, bbox_inches='tight')
        plt.close(fig)

        # 2D projections
        def proj(x, y, name, xl, yl, title):
            fig2 = plt.figure()
            plt.plot(x, y)
            plt.axis('equal'); plt.grid(True)
            plt.xlabel(xl); plt.ylabel(yl); plt.title(title)
            fig2.savefig(f'{self.out}_{name}.png', dpi=200, bbox_inches='tight')
            plt.close(fig2)

        proj(arr[:, 0], arr[:, 1], 'xy', 'x [m]', 'y [m]', 'XY Projection')
        proj(arr[:, 0], arr[:, 2], 'xz', 'x [m]', 'z [m]', 'XZ Projection')
        proj(arr[:, 1], arr[:, 2], 'yz', 'y [m]', 'z [m]', 'YZ Projection')

    def destroy_node(self):
        # Final plots on shutdown (CSV already autosaved)
        try:
            self._save_plots()
            self.get_logger().info(
                f"Saved: {self.out}.csv (autosave), plus _3d/_xy/_xz/_yz.png"
            )
        except Exception as e:
            self.get_logger().error(f'final save failed: {e}')
        return super().destroy_node()


def main():
    rclpy.init()
    node = TFTracer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
