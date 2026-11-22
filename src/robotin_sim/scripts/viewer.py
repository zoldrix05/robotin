#!/usr/bin/env python3
"""Inspection controls and ROS telemetry for the Robotin demonstration."""

import json
from math import cos, sin
import signal
import sys

from python_qt_binding import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


DESTINATIONS = ("Almacén", "Consumo", "Descarte")
COLORS = ("#72ce9c", "#ebc372", "#ef9994")
VIEWS = {
    "General": ((1.05, -.89, .88), .53, 2.09),
    "Descarga": ((.95, -.54, .54), .44, 2.10),
    "Superior": ((.54, 0, 1.34), 1.5707, 0.0),
    "Perfil": ((.50, -1.04, .47), .12, 1.5708),
}


class InspectionWindow(QtWidgets.QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.camera = node.declare_parameter("camera", True).value
        scope = node.declare_parameter("scope", "/robotin").value
        self.cover_open = False
        self.control_process = QtCore.QProcess(self)
        self.control_process.finished.connect(self.control_finished)
        self.command_ok = None
        self.command_error = ""
        self.setWindowTitle("Robotin · Inspección")
        self.resize(450, 795)
        self.setStyleSheet("""
            QWidget { background: #142027; color: #eff4f4; font-size: 14px; }
            QLabel#caption { color: #a7bbc0; }
            QLabel#title { font-size: 24px; font-weight: 650; }
            QLabel#metric { color: #d5e2e4; font-weight: 600; }
            QPushButton { background: #263942; border: 1px solid #45606b; border-radius: 6px;
                          padding: 7px 8px; }
            QPushButton:hover { background: #344e59; }
            QPushButton:focus { border-color: #f0b879; }
            QPushButton#play { background: #edb270; border-color: #edb270; color: #253037;
                               font-weight: 650; }
            QFrame#entry { background: #1b2a32; border: 1px solid #304550; border-radius: 7px; }
            QFrame#entry QLabel { background: transparent; }
            QLabel#video { background: #0b1419; border: 1px solid #304550; border-radius: 6px; }
        """)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)
        layout.addWidget(self.label("Inspección de clasificación", "title"))
        self.mode = self.label("Esperando a Gazebo", "caption")
        layout.addWidget(self.mode)

        playback = QtWidgets.QHBoxLayout()
        playback.addWidget(self.button("Reproducir", lambda: self.control(False), "play"))
        playback.addWidget(self.button("Pausar", lambda: self.control(True)))
        playback.addWidget(self.button("Paso", lambda: self.control(True, steps=25)))
        playback.addWidget(self.button("Reiniciar", lambda: self.control(True, reset=True)))
        layout.addLayout(playback)

        viewpoints = QtWidgets.QHBoxLayout()
        for name in VIEWS:
            viewpoints.addWidget(self.button(name, lambda name=name: self.set_view(name)))
        layout.addLayout(viewpoints)

        camera_tools = QtWidgets.QHBoxLayout()
        self.cover_button = self.button("Ocultar tapa", self.toggle_cover)
        camera_tools.addWidget(self.cover_button)
        self.live_button = self.button("En vivo", self.redraw_image)
        self.live_button.setCheckable(True)
        camera_tools.addWidget(self.live_button)
        layout.addLayout(camera_tools)
        self.image_label = self.label("Cámara en espera" if self.camera else "Cámara desactivada", "video")
        self.image_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumHeight(205)
        layout.addWidget(self.image_label)
        self.image_detail = self.label("", "caption")
        layout.addWidget(self.image_detail)

        self.metrics = self.label("Tiempo —  ·  Faja —  ·  Canal —", "metric")
        layout.addWidget(self.metrics)
        self.entries = QtWidgets.QVBoxLayout()
        self.entries.setSpacing(6)
        self.rows = []
        layout.addLayout(self.entries)
        self.feedback = self.label("Pulsa Reproducir para iniciar la secuencia.", "caption")
        self.feedback.setWordWrap(True)
        layout.addWidget(self.feedback)
        layout.addStretch()

        self.live_image = None
        self.captured_image = None
        self.capture_stamp = None
        self.last_run = None
        self.capture_fruit = None
        snapshot = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, f"{scope}/status", self.update_status, snapshot)
        if self.camera:
            node.create_subscription(Image, f"{scope}/camera/image", self.update_live,
                                     qos_profile_sensor_data)
            node.create_subscription(Image, f"{scope}/camera/capture", self.update_capture, snapshot)

    @staticmethod
    def label(text, name=""):
        label = QtWidgets.QLabel(text)
        label.setObjectName(name)
        label.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        return label

    def button(self, text, callback, name=""):
        button = QtWidgets.QPushButton(text, self)
        button.setObjectName(name)
        button.clicked.connect(lambda: callback())
        return button

    def control(self, paused, steps=0, reset=False):
        request = f"pause: {str(paused).lower()}"
        if steps:
            request += f" multi_step: {steps}"
        if reset:
            request += " reset: {all: true}"
        accepted = "Secuencia reiniciada" if reset else "Paso de 50 ms" if steps else (
            "Simulación en pausa" if paused else "Simulación en marcha"
        )
        self.send_command(
            "/world/robotin/control", "gz.msgs.WorldControl", request,
            lambda: self.world_reply(accepted, reset), "Gazebo aún no recibe órdenes",
        )

    def world_reply(self, accepted, reset=False):
        if reset:
            self.live_image = self.captured_image = self.capture_stamp = self.capture_fruit = None
            self.cover_open = False
            self.cover_button.setText("Ocultar tapa")
            self.image_label.clear()
            self.redraw_image()
            for title, detail in self.rows:
                title.setText("En espera")
                detail.clear()
        self.feedback.setText(accepted)

    def toggle_cover(self):
        opened = not self.cover_open
        height = -2.0 if opened else 0.0
        request = f'name: "inspection_cover" position: {{z: {height}}} orientation: {{w: 1}}'

        def done():
            self.cover_open = opened
            self.cover_button.setText("Mostrar tapa" if opened else "Ocultar tapa")

        self.send_command("/world/robotin/set_pose", "gz.msgs.Pose", request, done,
                          "Gazebo no pudo mover la tapa")

    def set_view(self, name):
        (x, y, z), pitch, yaw = VIEWS[name]
        cp, sp = cos(pitch / 2), sin(pitch / 2)
        cy, sy = cos(yaw / 2), sin(yaw / 2)
        pose = (f"pose: {{position: {{x: {x} y: {y} z: {z}}} "
                f"orientation: {{x: {-sp * sy} y: {sp * cy} z: {cp * sy} w: {cp * cy}}}}}")
        self.send_command("/gui/move_to/pose", "gz.msgs.GUICamera", pose, None,
                          "La vista 3D aún no recibe órdenes")

    def send_command(self, service, request_type, request, success, error):
        if self.control_process.state() != QtCore.QProcess.ProcessState.NotRunning:
            return
        self.command_ok = success
        self.command_error = error
        self.control_process.start("gz", [
            "service", "-s", service, "--reqtype", request_type,
            "--reptype", "gz.msgs.Boolean", "--timeout", "2500", "--req", request,
        ])

    def control_finished(self, code, _status):
        reply = bytes(self.control_process.readAllStandardOutput()).decode()
        if code != 0 or "data: true" not in reply:
            self.feedback.setText(self.command_error)
        elif self.command_ok:
            self.command_ok()

    @staticmethod
    def qimage(image):
        return QtGui.QImage(bytes(image.data), image.width, image.height, image.step,
                            QtGui.QImage.Format.Format_RGB888).copy()

    def update_live(self, image):
        self.live_image = self.qimage(image)
        if self.captured_image is None or self.live_button.isChecked():
            self.redraw_image()

    def update_capture(self, image):
        self.captured_image = self.qimage(image)
        stamp = image.header.stamp
        self.capture_stamp = stamp.sec + stamp.nanosec / 1e9
        self.redraw_image()

    def redraw_image(self):
        live = self.live_button.isChecked() or self.captured_image is None
        frame = self.live_image if live else self.captured_image
        if frame is None:
            self.image_label.setText("Cámara en espera" if self.camera else "Cámara desactivada")
            self.image_detail.clear()
            return
        pixmap = QtGui.QPixmap.fromImage(frame).scaled(
            200, 200, QtCore.Qt.AspectRatioMode.KeepAspectRatio,
            QtCore.Qt.TransformationMode.SmoothTransformation,
        )
        self.image_label.setPixmap(pixmap)
        if live:
            self.image_detail.setText("Cámara cenital · En vivo")
        else:
            fruit = f"Palta {self.capture_fruit} · " if self.capture_fruit else ""
            self.image_detail.setText(f"{fruit}Captura a {self.capture_stamp:.2f} s")

    def update_status(self, message):
        data = json.loads(message.data)
        if self.last_run is not None and self.last_run != data["run"]:
            self.captured_image = self.capture_stamp = self.capture_fruit = None
            self.redraw_image()
        self.last_run = data["run"]
        self.mode.setText(f"{data['mode']}  ·  {'En pausa' if data['paused'] else 'En marcha'}")
        self.metrics.setText(
            f"{data['time']:.2f} s  ·  Faja {data['speed']:.0f} mm/s  ·  Canal {data['angle']:+.1f}°"
        )
        while len(self.rows) < len(data["fruits"]):
            entry = QtWidgets.QFrame(self)
            entry.setObjectName("entry")
            layout = QtWidgets.QVBoxLayout(entry)
            layout.setContentsMargins(9, 6, 9, 6)
            layout.setSpacing(2)
            title, detail = self.label("", "metric"), self.label("", "caption")
            detail.setWordWrap(True)
            layout.addWidget(title)
            layout.addWidget(detail)
            self.entries.addWidget(entry)
            self.rows.append((title, detail))

        for i, (fruit, (title, detail)) in enumerate(zip(data["fruits"], self.rows, strict=True), 1):
            decision = fruit["decision"]
            destination = DESTINATIONS[decision] if decision is not None else "Sin destino"
            title.setText(f"Palta {i}   ·   {destination}")
            title.setStyleSheet(f"color: {COLORS[decision] if decision is not None else '#eff4f4'}")
            text = fruit["stage"]
            if fruit["actual"] is not None:
                text += f" · {DESTINATIONS[fruit['actual']]}"
            if fruit["margin"] is not None:
                text += f" · Margen {fruit['margin']:.2f} s"
            if fruit["scores"] is not None:
                text += "\n" + " · ".join(
                    f"{label} {score:.0%}" for label, score in zip(DESTINATIONS, fruit["scores"])
                )
            detail.setText(text)
            detail.setStyleSheet(f"color: {'#ffada7' if fruit['error'] else '#a7bbc0'}")
            if fruit["captured"] is not None and self.capture_stamp == fruit["captured"]:
                self.capture_fruit = i

        if self.capture_fruit and not self.live_button.isChecked():
            self.redraw_image()
        if data["outcome"]:
            self.feedback.setText(
                "Secuencia completada" if data["outcome"] == "PASS" else "Secuencia con errores"
            )


def main():
    app = QtWidgets.QApplication([sys.argv[0]])
    with rclpy.init(args=sys.argv):
        node = Node("robotin_inspection")
        window = InspectionWindow(node)
        timer = QtCore.QTimer()
        def spin():
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0)
            else:
                app.quit()
        timer.timeout.connect(spin)
        timer.start(12)
        signal.signal(signal.SIGINT, lambda *_: app.quit())
        signal.signal(signal.SIGTERM, lambda *_: app.quit())
        window.show()
        app.exec()
        timer.stop()
        node.destroy_node()


if __name__ == "__main__":
    main()
