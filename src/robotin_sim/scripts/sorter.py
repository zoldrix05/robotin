#!/usr/bin/env python3
"""Follow individual fruit, schedule the diverter and observe the result."""

from collections import deque
from dataclasses import dataclass
import json
from math import degrees, hypot, radians

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String


ANGLES = (radians(43), 0.0, radians(-43))
IR_GATE = 0.185
CAMERA_AXIS = 0.25
BELT_EDGE = 0.50
CLASSIFICATION_DELAY = 0.210
ANGLE_TOLERANCE = radians(2)
FRAME_WINDOW = 0.15


class ImageClassifier:
    def __init__(self, model):
        try:
            from ai_edge_litert.interpreter import Interpreter
        except ImportError as exc:
            raise RuntimeError("Instala ai-edge-litert en el entorno de la simulación") from exc
        import cv2
        import numpy as np
        from cv_bridge import CvBridge

        self.cv2 = cv2
        self.np = np
        self.bridge = CvBridge()
        self.interpreter = Interpreter(model_path=model)
        self.interpreter.allocate_tensors()
        self.input, = self.interpreter.get_input_details()
        self.output, = self.interpreter.get_output_details()
        if (tuple(self.input["shape"]) != (1, 160, 160, 3)
                or tuple(self.output["shape"]) != (1, 3)
                or self.input["dtype"] != np.float32 or self.output["dtype"] != np.float32):
            raise ValueError("Se espera el modelo híbrido del informe con entrada y salida float32")

    def predict(self, image):
        pixels = self.bridge.imgmsg_to_cv2(image, "rgb8")
        resized = self.cv2.resize(pixels, (160, 160), interpolation=self.cv2.INTER_LINEAR)
        self.interpreter.set_tensor(self.input["index"], resized[None].astype(self.np.float32))
        self.interpreter.invoke()
        scores = self.interpreter.get_tensor(self.output["index"])[0].tolist()
        if not self.np.isfinite(scores).all() or any(p < 0 or p > 1 for p in scores):
            raise ValueError("El modelo debe producir tres probabilidades")
        if abs(sum(scores) - 1) > .02:
            raise ValueError("Las probabilidades del modelo no suman uno")
        green, ripe, discard = scores
        risks = (ripe + 4.5 * discard, green + 1.6 * discard, 4.5 * green + 1.4 * ripe)
        return min(range(3), key=risks.__getitem__), scores


@dataclass
class Fruit:
    expected: int
    detected: float | None = None
    capture: float | None = None
    frame_time: float | None = None
    decision: int | None = None
    scores: list[float] | None = None
    arrival: float | None = None
    aligned: float | None = None
    on_target: bool = False
    clear: float | None = None
    stable_since: float | None = None
    near_floor: bool = False
    rebound: float = 0.0
    last_pose: list[float] | None = None
    actual: int | None = None
    error: str = ""


class Sorter(Node):
    def __init__(self):
        super().__init__("robotin_sorter")
        self.classes = self.declare_parameter("classes", [0, 2, 1]).value
        if not self.classes or any(label not in (0, 1, 2) for label in self.classes):
            raise ValueError("Las clases deben estar entre 0 y 2")
        self.camera = self.declare_parameter("camera", True).value
        self.scope = self.declare_parameter("scope", "/robotin").value
        self.radius = self.declare_parameter("diameter_mm", 64.0).value / 2000
        model = self.declare_parameter("model", "").value
        self.classifier = ImageClassifier(model) if model else None
        self.frames = deque(maxlen=20)
        self.run = None
        self.reset_run()
        snapshot = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.captures = self.output_publishers(Image, "/camera/capture", snapshot)
        self.status = self.output_publishers(String, "/status", snapshot)
        self.results = self.output_publishers(String, "/result", snapshot)
        self.live_image = None
        if self.camera:
            if self.scope != "/robotin":
                self.live_image = self.create_publisher(
                    Image, "/robotin/camera/image", 5
                )
            self.create_subscription(
                Image, f"{self.scope}/camera/image", self.receive_frame, qos_profile_sensor_data
            )
        self.command = self.create_publisher(Float64, f"{self.scope}/diverter/command", 10)
        self.create_subscription(String, f"{self.scope}/plant", self.update_plant, 1)

    def output_publishers(self, message_type, suffix, qos):
        topics = dict.fromkeys((f"{self.scope}{suffix}", f"/robotin{suffix}"))
        return [self.create_publisher(message_type, topic, qos) for topic in topics]

    @staticmethod
    def publish(publishers, message):
        for publisher in publishers:
            publisher.publish(message)

    def reset_run(self):
        self.fruits = [Fruit(label) for label in self.classes]
        self.active = 0
        self.finished = False
        self.target = 0.0
        self.last_status = -1.0
        self.last_update = -1.0
        self.paused = None
        self.last_command = None
        self.command_at = -1.0
        self.started_at = None
        self.outcome = ""
        self.frames.clear()

    def receive_frame(self, image):
        if self.live_image is not None and self.live_image.get_subscription_count():
            self.live_image.publish(image)
        stamp = image.header.stamp
        time = stamp.sec + stamp.nanosec / 1e9
        if self.frames and time < self.frames[-1][0]:
            self.frames.clear()
        self.frames.append((time, image))

    def update_plant(self, message):
        data = json.loads(message.data)
        time = data["time"]
        angle = data["angle"]
        servo = data["servo"]
        if self.run is not None and data["run"] != self.run:
            if data["run"] < self.run:
                return
            self.reset_run()
        self.run = data["run"]
        pause_changed = data["paused"] != self.paused
        if time < self.last_update or (time == self.last_update and not pause_changed):
            return
        self.paused = data["paused"]
        self.last_update = time
        if self.paused:
            self.publish_status(data)
            return
        if self.started_at is None:
            self.started_at = time

        for i, (fruit, pose) in enumerate(zip(self.fruits, data["fruit"], strict=True)):
            x, y, z = pose
            if fruit.detected is None and x >= IR_GATE:
                fruit.detected = time
                self.get_logger().info(f"Palta {i + 1} cruza la barrera en t={time:.2f} s")
            if fruit.detected is not None and fruit.capture is None and x >= CAMERA_AXIS:
                fruit.capture = time
            self.update_decision(i, fruit, time)
            if fruit.arrival is None and x >= BELT_EDGE:
                fruit.arrival = time
                if fruit.decision is None:
                    fruit.error = fruit.error or "Decisión ausente al llegar al borde"
                elif any(previous.clear is None for previous in self.fruits[:i]):
                    fruit.error = "Canal ocupado al llegar"
                else:
                    fruit.on_target = self.is_aligned(fruit.decision, angle, servo)
                    if not fruit.on_target:
                        fruit.error = "Canal fuera de posición al llegar"
                self.get_logger().info(
                    f"Palta {i + 1} al borde en t={time:.2f} s, "
                    f"canal={degrees(angle):+.1f}°, servo={degrees(servo):+.1f}°, "
                    f"margen={self.margin(fruit)}, lista={fruit.on_target}"
                )
            if fruit.arrival is not None:
                if fruit.clear is None and (hypot(x - BELT_EDGE, y) >= 0.250 or z < 0.235):
                    fruit.clear = time
                self.check_collection(i, fruit, pose, time)

        while self.active < len(self.fruits) and self.fruits[self.active].clear is not None:
            self.active += 1
        if self.active < len(self.fruits):
            fruit = self.fruits[self.active]
            if fruit.decision is not None:
                self.target = ANGLES[fruit.decision]
                if fruit.aligned is None and self.is_aligned(fruit.decision, angle, servo):
                    fruit.aligned = time
        if not self.camera or self.frames:
            if self.target != self.last_command or time - self.command_at >= .12:
                self.command.publish(Float64(data=self.target))
                self.last_command = self.target
                self.command_at = time

        deadline = (0.25 + .15 * (len(self.fruits) - 1)) / data["belt_speed"] + 6
        first = self.fruits[0].capture
        no_stream = self.camera and not self.frames and time - self.started_at > 6
        if not self.finished and (all(fruit.actual is not None for fruit in self.fruits)
                                  or no_stream or (first is not None and time - first > deadline)):
            self.finish(no_stream)
        status_tick = int((time + 0.000001) / .05)
        if pause_changed or status_tick != self.last_status:
            self.last_status = status_tick
            self.publish_status(data)

    @staticmethod
    def is_aligned(decision, angle, servo):
        return abs(angle - ANGLES[decision]) <= ANGLE_TOLERANCE and abs(servo - angle) <= ANGLE_TOLERANCE

    def update_decision(self, i, fruit, time):
        if fruit.capture is None or fruit.decision is not None or fruit.error:
            return
        if self.camera and fruit.frame_time is None:
            for stamp, image in self.frames:
                if fruit.capture <= stamp <= fruit.capture + FRAME_WINDOW:
                    fruit.frame_time = stamp
                    self.publish(self.captures, image)
                    if self.classifier:
                        try:
                            fruit.decision, fruit.scores = self.classifier.predict(image)
                        except (RuntimeError, ValueError) as exc:
                            fruit.error = f"Inferencia fallida. {exc}"
                            self.get_logger().error(fruit.error)
                    self.get_logger().info(
                        f"Palta {i + 1} capturada en t={stamp:.2f} s ({image.width}×{image.height})"
                    )
                    break
            if fruit.frame_time is None and time - fruit.capture > FRAME_WINDOW + .2:
                fruit.error = "Sin fotograma en la ventana de captura"
        if (not self.classifier and time >= fruit.capture + CLASSIFICATION_DELAY
                and (not self.camera or fruit.frame_time is not None)):
            fruit.decision = fruit.expected

    @staticmethod
    def margin(fruit):
        if fruit.arrival is not None and fruit.aligned is not None:
            return round(fruit.arrival - fruit.aligned, 3)
        return None

    def check_collection(self, i, fruit, pose, time):
        if fruit.actual is not None:
            return
        x, y, z = pose
        height = .158 + self.radius
        half_width = .255 if x < .791 else .225
        in_footprint = .595 <= x <= 1.045 and abs(y) < half_width
        inside = in_footprint and height - .013 <= z <= height + .022
        if in_footprint and height - .009 <= z <= height + .007:
            fruit.near_floor = True
        if fruit.near_floor and in_footprint:
            fruit.rebound = max(fruit.rebound, z - height)
        moving = fruit.last_pose is None or max(
            abs(a - b) for a, b in zip(pose, fruit.last_pose, strict=True)
        ) > .0004
        fruit.last_pose = pose
        if not inside or moving:
            fruit.stable_since = None
            return
        if fruit.stable_since is None:
            fruit.stable_since = time
        if time - fruit.stable_since >= .25:
            fruit.actual = 0 if y > .075 else 2 if y < -.075 else 1
            if fruit.actual != fruit.expected:
                fruit.error = fruit.error or "Tolva distinta de la esperada"
            self.get_logger().info(f"Palta {i + 1} en tolva {fruit.actual}, esperada {fruit.expected}")

    def finish(self, no_stream):
        for fruit in self.fruits:
            if fruit.actual is None and not fruit.error:
                if no_stream:
                    fruit.error = "Cámara sin señal"
                elif fruit.arrival is not None and fruit.clear is None:
                    fruit.error = "Posible atasco en el canal"
                else:
                    fruit.error = "Sin llegada confirmada a una tolva"
        self.finished = True
        passed = all(fruit.on_target and fruit.actual == fruit.expected and not fruit.error
                     for fruit in self.fruits)
        self.outcome = "PASS" if passed else "FAIL"
        self.publish(self.results, String(data=json.dumps({
            "outcome": self.outcome,
            "expected": self.classes,
            "actual": [fruit.actual for fruit in self.fruits],
            "on_target": [fruit.on_target for fruit in self.fruits],
            "frames": [fruit.frame_time for fruit in self.fruits],
            "clear": [fruit.clear for fruit in self.fruits],
            "margin": [self.margin(fruit) for fruit in self.fruits],
            "rebound_mm": [round(fruit.rebound * 1000, 1) for fruit in self.fruits],
            "rest_pose": [fruit.last_pose for fruit in self.fruits],
            "errors": [fruit.error for fruit in self.fruits],
        })))
        log = self.get_logger().info if passed else self.get_logger().error
        log(f"RESULTADO ROBOTIN {self.outcome}")
        for i, fruit in enumerate(self.fruits):
            if fruit.error:
                self.get_logger().error(f"Palta {i + 1} · {fruit.error} · posición {fruit.last_pose}")

    def publish_status(self, data):
        self.publish(self.status, String(data=json.dumps({
            "run": self.run, "time": data["time"], "speed": data["belt_speed"] * 1000,
            "paused": self.paused,
            "angle": degrees(data["angle"]), "target": degrees(self.target),
            "mode": "Modelo TFLite" if self.classifier else "Clases de prueba",
            "outcome": self.outcome,
            "fruits": [{
                "decision": fruit.decision, "actual": fruit.actual, "error": fruit.error,
                "stage": self.stage(fruit), "margin": self.margin(fruit),
                "captured": fruit.frame_time, "scores": fruit.scores,
            } for fruit in self.fruits],
        })))

    @staticmethod
    def stage(fruit):
        if fruit.error:
            return fruit.error
        if fruit.actual is not None:
            return "En tolva"
        if fruit.clear is not None:
            return "Salió del canal"
        if fruit.arrival is not None:
            return "En el canal"
        if fruit.decision is not None:
            return "Destino asignado"
        if fruit.capture is not None:
            return "Captura"
        if fruit.detected is not None:
            return "Detectada"
        return "En espera"


def main():
    with rclpy.init():
        node = Sorter()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            node.destroy_node()


if __name__ == "__main__":
    main()
