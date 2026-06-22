# MIT License
#
# blaze_slam map_autosaver
#
# Watches the SLAM Toolbox pose graph and saves the current map whenever a
# new loop closure is detected. Also exposes a manual save service.
#
# Loop closure detection:
#   SLAM Toolbox does not publish a discrete "loop closed" event. It does
#   publish its pose graph on /slam_toolbox/graph_visualization as a
#   MarkerArray (nodes as spheres, edges as line lists). In a single
#   trajectory the graph is a chain, so edges == nodes - 1. Every loop
#   closure adds one extra constraint edge, so:
#       loop_closures = edges - (nodes - 1)
#   When that count increases, a loop closure just happened and we save.
#
# Output format matches a standard nav2_map_server map (png + yaml,
# mode: trinary), which is what the F1Tenth gym sim and Pure Pursuit expect.

import os

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.srv import SaveMap
from std_srvs.srv import Trigger

# slam_toolbox python service bindings may not be present in every install.
# The map (png/yaml) save does not need them; only the optional posegraph
# serialization does, so import defensively.
try:
    from slam_toolbox.srv import SerializePoseGraph
    HAVE_SLAM_SRV = True
except Exception:
    SerializePoseGraph = None
    HAVE_SLAM_SRV = False


class MapAutoSaver(Node):

    def __init__(self):
        super().__init__('map_autosaver')

        # Where and how to save
        self.declare_parameter(
            'maps_dir',
            os.path.join(os.path.expanduser('~'), 'blaze_racer', 'maps'))
        self.declare_parameter('map_name', 'track')
        # 'overwrite' replaces an existing map of the same name.
        # 'rename'    keeps the existing map and writes track_1, track_2, ...
        self.declare_parameter('save_policy', 'rename')

        self.declare_parameter('image_format', 'png')
        self.declare_parameter('map_mode', 'trinary')
        self.declare_parameter('free_thresh', 0.25)
        self.declare_parameter('occupied_thresh', 0.65)
        self.declare_parameter('map_topic', '/map')

        # Loop closure autosave behavior
        self.declare_parameter('autosave_on_loop_closure', True)
        self.declare_parameter('min_save_interval_sec', 10.0)
        self.declare_parameter('serialize_posegraph', True)
        self.declare_parameter('graph_topic', '/slam_toolbox/graph_visualization')

        self.maps_dir = os.path.expanduser(
            self.get_parameter('maps_dir').value)
        self.map_name = self.get_parameter('map_name').value
        self.save_policy = self.get_parameter('save_policy').value
        if self.save_policy not in ('overwrite', 'rename'):
            self.get_logger().warn(
                "save_policy '%s' invalid, falling back to 'rename'"
                % self.save_policy)
            self.save_policy = 'rename'

        self.image_format = self.get_parameter('image_format').value
        self.map_mode = self.get_parameter('map_mode').value
        self.free_thresh = float(self.get_parameter('free_thresh').value)
        self.occupied_thresh = float(
            self.get_parameter('occupied_thresh').value)
        self.map_topic = self.get_parameter('map_topic').value

        self.autosave = self.get_parameter('autosave_on_loop_closure').value
        self.min_interval = float(
            self.get_parameter('min_save_interval_sec').value)
        self.serialize = self.get_parameter('serialize_posegraph').value
        graph_topic = self.get_parameter('graph_topic').value

        os.makedirs(self.maps_dir, exist_ok=True)

        # State
        self._loop_closures_seen = 0
        self._save_requested = False
        self._force_save = False
        self._save_reason = ''
        self._last_save_time = None
        self._futures = []  # keep references so futures are not GC'd

        # Service clients
        self._save_client = self.create_client(SaveMap, '/map_saver/save_map')
        self._serialize_client = None
        if self.serialize and HAVE_SLAM_SRV:
            self._serialize_client = self.create_client(
                SerializePoseGraph, '/slam_toolbox/serialize_map')
        elif self.serialize and not HAVE_SLAM_SRV:
            self.get_logger().warn(
                'serialize_posegraph requested but slam_toolbox srv bindings '
                'are unavailable; posegraph will not be written')

        # Pose graph subscription drives loop closure detection
        self.create_subscription(
            MarkerArray, graph_topic, self._graph_cb, 10)

        # Manual save: ros2 service call /map_autosaver/save_map std_srvs/srv/Trigger
        self.create_service(Trigger, '~/save_map', self._manual_cb)

        # Worker tick keeps service calls out of the subscription callback
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            'map_autosaver ready. maps_dir=%s map_name=%s policy=%s '
            'autosave_on_loop_closure=%s'
            % (self.maps_dir, self.map_name, self.save_policy, self.autosave))

    # ------------------------------------------------------------------ #
    # Loop closure detection
    # ------------------------------------------------------------------ #
    def _graph_cb(self, msg):
        nodes = 0
        edges = 0
        for m in msg.markers:
            if m.action != Marker.ADD:
                continue
            if m.type == Marker.SPHERE:
                nodes += 1
            elif m.type == Marker.SPHERE_LIST or m.type == Marker.POINTS:
                nodes += len(m.points)
            elif m.type == Marker.LINE_LIST:
                edges += len(m.points) // 2
            elif m.type == Marker.LINE_STRIP:
                edges += max(0, len(m.points) - 1)

        if nodes == 0:
            return

        loop_closures = max(0, edges - (nodes - 1))
        if loop_closures > self._loop_closures_seen:
            self._loop_closures_seen = loop_closures
            self.get_logger().info(
                'Loop closure detected (total=%d)' % loop_closures)
            if self.autosave:
                self._save_requested = True
                self._save_reason = 'loop closure #%d' % loop_closures

    # ------------------------------------------------------------------ #
    # Manual trigger
    # ------------------------------------------------------------------ #
    def _manual_cb(self, request, response):
        self._save_requested = True
        self._force_save = True  # manual saves ignore the debounce interval
        self._save_reason = 'manual request'
        response.success = True
        response.message = 'map save scheduled'
        return response

    # ------------------------------------------------------------------ #
    # Worker
    # ------------------------------------------------------------------ #
    def _tick(self):
        if not self._save_requested:
            return
        if not self._force_save and self._last_save_time is not None:
            elapsed = (self.get_clock().now()
                       - self._last_save_time).nanoseconds * 1e-9
            if elapsed < self.min_interval:
                return
        self._save_requested = False
        self._force_save = False
        self._do_save()

    def _resolve_map_url(self):
        base = os.path.join(self.maps_dir, self.map_name)
        if self.save_policy == 'overwrite':
            return base
        if not os.path.exists(base + '.yaml'):
            return base
        i = 1
        while os.path.exists('%s_%d.yaml' % (base, i)):
            i += 1
        return '%s_%d' % (base, i)

    def _do_save(self):
        os.makedirs(self.maps_dir, exist_ok=True)
        if not self._save_client.service_is_ready():
            self.get_logger().warn(
                'map_saver service not available yet; will retry on next event')
            self._save_requested = True  # try again later
            return

        map_url = self._resolve_map_url()
        req = SaveMap.Request()
        req.map_topic = self.map_topic
        req.map_url = map_url
        req.image_format = self.image_format
        req.map_mode = self.map_mode
        req.free_thresh = self.free_thresh
        req.occupied_thresh = self.occupied_thresh

        self.get_logger().info(
            'Saving map -> %s.%s (%s)'
            % (map_url, self.image_format, self._save_reason))
        future = self._save_client.call_async(req)
        self._futures.append(future)
        future.add_done_callback(
            lambda f, url=map_url: self._on_save_done(f, url))
        self._last_save_time = self.get_clock().now()

    def _on_save_done(self, future, map_url):
        try:
            result = future.result()
            ok = getattr(result, 'result', True)
        except Exception as exc:
            self.get_logger().error('map save failed: %s' % exc)
            return
        if not ok:
            self.get_logger().error('map_saver reported failure')
            return
        self.get_logger().info(
            'Map saved: %s.%s + %s.yaml'
            % (map_url, self.image_format, map_url))

        if self._serialize_client is not None:
            if self._serialize_client.service_is_ready():
                sreq = SerializePoseGraph.Request()
                sreq.filename = map_url
                sfuture = self._serialize_client.call_async(sreq)
                self._futures.append(sfuture)
                sfuture.add_done_callback(
                    lambda f, url=map_url: self._on_serialize_done(f, url))
            else:
                self.get_logger().warn(
                    'serialize_map service not ready; skipping posegraph')

    def _on_serialize_done(self, future, map_url):
        try:
            future.result()
            self.get_logger().info(
                'Pose graph saved: %s.posegraph + %s.data'
                % (map_url, map_url))
        except Exception as exc:
            self.get_logger().error('posegraph serialize failed: %s' % exc)


def main():
    rclpy.init()
    node = MapAutoSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
