#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import functools
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback,
)
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class InteractiveMarkerEFF(Node):
    def __init__(self):
        super().__init__('multi_ee_goal_markers')

        self.declare_parameter('fixed_frame', 'pelvis')
        self.declare_parameter('spawn_rate_hz', 5.0)

        self.declare_parameter('right_tf_frame', 'right_hand_point_contact')
        self.declare_parameter('right_topic', '/g1pilot/hand_goal/right')
        self.declare_parameter('right_scale', 0.05)

        self.declare_parameter('left_tf_frame', 'left_hand_point_contact')
        self.declare_parameter('left_topic', '/g1pilot/hand_goal/left')
        self.declare_parameter('left_scale', 0.05)

        self.declare_parameter('publish_enabled_default', False)
        # Leader-Follower: solange ein Marker NICHT gezogen wird, folgt er der
        # echten Hand (TF). So bleibt er nach externen Befehlen (Homing etc.)
        # synchron und ein Antippen erzeugt nur eine kleine relative Bewegung
        # statt eines Sprungs. Schaltbar per Param und per Topic.
        self.declare_parameter('marker_follow_ee', True)
        self.declare_parameter('follow_rate_hz', 20.0)

        self.fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value
        self.spawn_dt = 1.0 / float(self.get_parameter('spawn_rate_hz').value)

        self.right_tf = self.get_parameter('right_tf_frame').get_parameter_value().string_value
        self.right_topic = self.get_parameter('right_topic').get_parameter_value().string_value
        self.right_scale = float(self.get_parameter('right_scale').value)

        self.left_tf = self.get_parameter('left_tf_frame').get_parameter_value().string_value
        self.left_topic = self.get_parameter('left_topic').get_parameter_value().string_value
        self.left_scale = float(self.get_parameter('left_scale').value)

        self.publish_default = bool(self.get_parameter('publish_enabled_default').value)
        self.follow_ee = bool(self.get_parameter('marker_follow_ee').value)
        self.follow_dt = 1.0 / float(self.get_parameter('follow_rate_hz').value)

        # Pro Seite merken, ob der Marker gerade mit der Maus gezogen wird, damit
        # der Follow-Update das Ziehen nicht ueberschreibt.
        self.dragging = {"right": False, "left": False}
        # Nach dem Loslassen: Follow pausieren, bis der Arm das Ziel erreicht hat
        # bzw. steht. Sonst springt der Marker auf die noch hinterherhinkende Hand
        # zurueck (Arm bleibt dann mitten im Raum stehen).
        self._await_arrival = {"right": False, "left": False}
        self._target_pose = {"right": None, "left": None}
        self._still_count = {"right": 0, "left": 0}
        self._prev_hand = {"right": None, "left": None}

        self.server = InteractiveMarkerServer(self, "g1_ee_goal_markers")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.qos = QoSProfile(depth=10)

        self.ee_publishers = {
            "right": self.create_publisher(PoseStamped, self.right_topic, self.qos),
            "left":  self.create_publisher(PoseStamped,  self.left_topic,  self.qos),
        }

        self.publish_enabled = {"right": self.publish_default, "left": self.publish_default}

        self.current_pose = {"right": None, "left": None}

        self.menu_handlers = {
            "right": self._build_menu_handler(),
            "left":  self._build_menu_handler(),
        }
        self.menu_entry_ids = {
            "right": {},
            "left":  {},
        }

        self.marker_spawned = {"right": False, "left": False}
        self.timer = self.create_timer(self.spawn_dt, self._try_spawn_missing)

        # Laufzeit-Umschalten des Follow-Verhaltens (z.B. vom Streamdeck-Button).
        self.create_subscription(Bool, '/g1pilot/marker_follow_ee', self._set_follow, 10)
        self.follow_timer = self.create_timer(self.follow_dt, self._follow_update)
        self.get_logger().info(f"[marker] follow_ee={self.follow_ee} (Leader-Follower)")

    def _set_follow(self, msg: Bool):
        if bool(msg.data) != self.follow_ee:
            self.follow_ee = bool(msg.data)
            self.get_logger().info(f"[marker] follow_ee -> {self.follow_ee}")

    @staticmethod
    def _pose_close(a: Pose, b: Pose, pos_tol=0.004, ori_tol=0.999):
        """True, wenn zwei Posen praktisch gleich sind (Position + Orientierung)."""
        dx = a.position.x - b.position.x
        dy = a.position.y - b.position.y
        dz = a.position.z - b.position.z
        if (dx * dx + dy * dy + dz * dz) ** 0.5 > pos_tol:
            return False
        dot = (a.orientation.x * b.orientation.x + a.orientation.y * b.orientation.y +
               a.orientation.z * b.orientation.z + a.orientation.w * b.orientation.w)
        return abs(dot) >= ori_tol

    def _follow_update(self):
        """Marker auf die aktuelle Hand-TF nachfuehren -- aber:
          * nicht waehrend des Ziehens,
          * nach dem Loslassen erst, wenn der Arm das Ziel erreicht hat ODER steht,
          * nur bei merklicher Aenderung (kein Dauer-Flackern im Stillstand)."""
        if not self.follow_ee:
            return
        # Anzahl ruhiger Ticks, ab denen der Arm als "steht" gilt (~0.5 s).
        still_needed = max(1, int(0.5 / self.follow_dt))
        changed = False
        for side, tf_frame in (("right", self.right_tf), ("left", self.left_tf)):
            if not self.marker_spawned[side] or self.dragging.get(side, False):
                continue
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.fixed_frame, tf_frame, rclpy.time.Time())
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue
            hand = Pose()
            hand.position.x = trans.transform.translation.x
            hand.position.y = trans.transform.translation.y
            hand.position.z = trans.transform.translation.z
            hand.orientation = trans.transform.rotation

            # Nach dem Loslassen: warten, bis der Arm angekommen ist / steht.
            if self._await_arrival.get(side, False):
                target = self._target_pose.get(side)
                reached = target is not None and self._pose_close(
                    hand, target, pos_tol=0.03, ori_tol=0.99)
                prev = self._prev_hand.get(side)
                if prev is not None and self._pose_close(hand, prev, pos_tol=0.002):
                    self._still_count[side] += 1
                else:
                    self._still_count[side] = 0
                self._prev_hand[side] = hand
                stopped = self._still_count[side] >= still_needed
                if reached or stopped:
                    self._await_arrival[side] = False  # ab jetzt normal folgen
                # Marker bleibt derweil auf der Ziel-Pose stehen -> nicht bewegen.
                continue

            cur = self.current_pose.get(side)
            if cur is not None and self._pose_close(cur, hand):
                continue  # Hand steht praktisch still -> nichts tun
            self.current_pose[side] = hand
            self.server.setPose(f"{side}_hand_goal", hand)
            changed = True
        if changed:
            self.server.applyChanges()

    def _build_menu_handler(self) -> MenuHandler:
        mh = MenuHandler()
        toggle_id = mh.insert("Enable publishing", callback=self._menu_cb)
        reset_id  = mh.insert("Reset to TF pose", callback=self._menu_cb)
        mh.setCheckState(toggle_id, MenuHandler.CHECKED if self.publish_default else MenuHandler.UNCHECKED)
        mh._toggle_id = toggle_id
        mh._reset_id = reset_id
        return mh

    def _try_spawn_missing(self):
        if not self.marker_spawned["right"]:
            self._try_spawn_one("right", self.right_tf, self.right_scale)

        if not self.marker_spawned["left"]:
            self._try_spawn_one("left", self.left_tf, self.left_scale)

    def _try_spawn_one(self, side, tf_frame, scale):
        try:
            trans = self.tf_buffer.lookup_transform(self.fixed_frame, tf_frame, rclpy.time.Time())

            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            self.current_pose[side] = pose

            self._spawn_marker(side, scale, pose)
            self.marker_spawned[side] = True
            self.get_logger().info(f"Spawned {side} hand marker from TF '{tf_frame}' in '{self.fixed_frame}'")
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info(f"Waiting for TF: {self.fixed_frame} -> {tf_frame}")

    def _spawn_marker(self, side, cube_size, pose: Pose):
        name = f"{side}_hand_goal"

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.fixed_frame
        int_marker.name = name
        int_marker.description = f"{side.title()} Hand Goal"
        int_marker.scale = 0.2
        int_marker.pose = pose

        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = cube_size
        box_marker.scale.y = cube_size
        box_marker.scale.z = cube_size

        if self.publish_enabled[side]:
            if side == "right":
                box_marker.color.r, box_marker.color.g, box_marker.color.b, box_marker.color.a = (0.2, 0.8, 0.2, 1.0)
            else:
                box_marker.color.r, box_marker.color.g, box_marker.color.b, box_marker.color.a = (0.2, 0.2, 0.8, 1.0)
        else:
            box_marker.color.r, box_marker.color.g, box_marker.color.b, box_marker.color.a = (0.5, 0.5, 0.5, 1.0)

        always = InteractiveMarkerControl()
        always.always_visible = True
        always.markers.append(box_marker)
        int_marker.controls.append(always)

        menu_ctrl = InteractiveMarkerControl()
        menu_ctrl.interaction_mode = InteractiveMarkerControl.MENU
        menu_ctrl.name = "menu"
        int_marker.controls.append(menu_ctrl)

        self._add_6dof_controls(int_marker)

        self.server.insert(int_marker)
        cb = functools.partial(self._feedback_cb, ee_name=side)
        self.server.setCallback(int_marker.name, cb)

        mh = self.menu_handlers[side]
        mh.apply(self.server, int_marker.name)
        self.menu_entry_ids[side] = {
            "toggle": mh._toggle_id,
            "reset":  mh._reset_id,
        }

        self.server.applyChanges()

    def _add_6dof_controls(self, int_marker: InteractiveMarker):
        axes = {'x': (1.0, 0.0, 0.0, 1.0),
                'y': (0.0, 1.0, 0.0, 1.0),
                'z': (0.0, 0.0, 1.0, 1.0)}
        for axis, (ox, oy, oz, ow) in axes.items():
            for mode in (InteractiveMarkerControl.ROTATE_AXIS, InteractiveMarkerControl.MOVE_AXIS):
                c = InteractiveMarkerControl()
                c.orientation.x = float(ox)
                c.orientation.y = float(oy)
                c.orientation.z = float(oz)
                c.orientation.w = float(ow)
                c.name = f"{'ROT' if mode==InteractiveMarkerControl.ROTATE_AXIS else 'MOV'}_{axis}"
                c.interaction_mode = mode
                int_marker.controls.append(c)

    def _feedback_cb(self, feedback, ee_name: str):
        et = feedback.event_type
        if et == InteractiveMarkerFeedback.MOUSE_DOWN:
            self.dragging[ee_name] = True

        self.current_pose[ee_name] = feedback.pose

        # NUR waehrend aktivem Ziehen (bzw. beim Loslassen die finale Pose)
        # publizieren. Sonst wuerde das Follow-Update (setPose) ueber das
        # Feedback ein Ziel ausloesen -> Rueckkopplung (Arm faehrt -> Hand bewegt
        # sich -> Marker folgt -> Feedback -> ...) = Flackern/Springen.
        is_drag = self.dragging.get(ee_name, False) or et == InteractiveMarkerFeedback.MOUSE_UP
        if self.publish_enabled.get(ee_name, True) and is_drag:
            pub = self.ee_publishers.get(ee_name)
            if pub is not None:
                pose = PoseStamped()
                pose.header = feedback.header
                pose.header.frame_id = self.fixed_frame
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose = feedback.pose
                pub.publish(pose)

        if et == InteractiveMarkerFeedback.MOUSE_UP:
            self.dragging[ee_name] = False
            # Marker bleibt auf dieser Ziel-Pose, bis der Arm sie erreicht/steht.
            self._await_arrival[ee_name] = True
            self._target_pose[ee_name] = feedback.pose
            self._still_count[ee_name] = 0
            self._prev_hand[ee_name] = None

    def _menu_cb(self, feedback):
        marker_name = feedback.marker_name or ""
        if marker_name.startswith("right_"):
            side = "right"
        elif marker_name.startswith("left_"):
            side = "left"
        else:
            self.get_logger().warn(f"Menu callback with unknown marker '{marker_name}'")
            return

        entry_id = feedback.menu_entry_id
        ids = self.menu_entry_ids[side]
        mh = self.menu_handlers[side]

        if entry_id == ids["toggle"]:
            new_state = not self.publish_enabled[side]
            self.publish_enabled[side] = new_state
            mh.setCheckState(ids["toggle"], MenuHandler.CHECKED if new_state else MenuHandler.UNCHECKED)
            mh.reApply(self.server)
            self._recolor_marker(side)
            self.server.applyChanges()

            self.get_logger().info(f"[{side}] Publishing {'ENABLED' if new_state else 'DISABLED'}")

        elif entry_id == ids["reset"]:
            tf_frame = self.right_tf if side == "right" else self.left_tf
            self._reset_marker_to_tf(side, tf_frame)
            self.server.applyChanges()
            self.get_logger().info(f"[{side}] Reset to TF '{tf_frame}'")

    def _recolor_marker(self, side: str):
        pose = self.current_pose.get(side)
        if pose is None:
            return
        name = f"{side}_hand_goal"
        try:
            self.server.erase(name)
        except Exception:
            pass
        cube_size = self.right_scale if side == "right" else self.left_scale
        self._spawn_marker(side, cube_size, pose)

    def _reset_marker_to_tf(self, side: str, tf_frame: str):
        try:
            trans = self.tf_buffer.lookup_transform(self.fixed_frame, tf_frame, rclpy.time.Time())
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            self.current_pose[side] = pose
            self.server.setPose(f"{side}_hand_goal", pose)
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn(f"Cannot reset {side}: TF {self.fixed_frame}->{tf_frame} not available yet.")

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveMarkerEFF()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
