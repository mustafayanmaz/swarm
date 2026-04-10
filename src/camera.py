"""
Gazebo kamera görüntüsü yakalama modülü.
gz-transport üzerinden drone kameralarına abone olur,
OpenCV frame olarak döner.
"""
import os
import sys
import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# gz-transport protobuf uyumluluğu
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

import logging

log = logging.getLogger("swarm.camera")

# Gazebo pixel format → OpenCV dönüşüm
_GZ_FMT = {
    1: cv2.COLOR_RGB2BGR,   # RGB_INT8
    2: cv2.COLOR_RGBA2BGR,  # RGBA_INT8  (PixelFormatType)
    6: cv2.COLOR_RGB2BGR,   # RGB_INT8 alternatif
}


class GzCamera:
    """Tek bir drone'un aşağı bakan kamerasından frame alır."""

    def __init__(self, drone_index: int, world_name: str = "default"):
        self._node = Node()
        self._frame = None
        self._new_frame = False

        # PX4 SITL multi-vehicle topic formatı:
        # /world/{world}/model/{model}_{i}/link/camera_link/sensor/camera/image
        model_name = f"x500_mono_cam_down_{drone_index}"

        self.topic = (
            f"/world/{world_name}/model/{model_name}"
            f"/link/camera_link/sensor/camera/image"
        )
        log.info(f"[Cam {drone_index}] Topic: {self.topic}")

        ok = self._node.subscribe(Image, self.topic, self._on_image)
        if not ok:
            log.error(f"[Cam {drone_index}] Subscribe BAŞARISIZ: {self.topic}")
        else:
            log.info(f"[Cam {drone_index}] Subscribe OK")

    def _on_image(self, msg):
        """Gazebo image callback → numpy BGR frame."""
        w = msg.width
        h = msg.height
        fmt = msg.pixel_format_type

        data = np.frombuffer(msg.data, dtype=np.uint8)
        expected = h * w

        if data.size == expected * 3:
            data = data.reshape(h, w, 3)
            self._frame = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
        elif data.size == expected * 4:
            data = data.reshape(h, w, 4)
            self._frame = cv2.cvtColor(data, cv2.COLOR_RGBA2BGR)
        elif data.size == expected:
            self._frame = data.reshape(h, w)
        else:
            log.warning(f"Beklenmeyen veri boyutu: {data.size}, fmt={fmt}, {w}x{h}")
            return

        self._new_frame = True

    def get_frame(self):
        """Son frame'i döndür. Yoksa None."""
        if self._frame is not None:
            self._new_frame = False
            return self._frame.copy()
        return None

    def has_new_frame(self) -> bool:
        return self._new_frame


class SwarmCameras:
    """Tüm droneların kameralarını yönetir."""

    def __init__(self, num_drones: int = 3, world_name: str = "default"):
        self.num_drones = num_drones
        self.cameras = {}
        for i in range(num_drones):
            self.cameras[i] = GzCamera(i, world_name)
        log.info(f"{num_drones} kamera başlatıldı.")

    def get_frame(self, drone_id: int):
        """Belirtilen drone'un kamera frame'ini al."""
        cam = self.cameras.get(drone_id)
        if cam:
            return cam.get_frame()
        return None

    def get_all_frames(self):
        """Tüm droneların frame'lerini döndür."""
        return {did: cam.get_frame() for did, cam in self.cameras.items()}

    def show_frames(self):
        """Tüm kamera görüntülerini OpenCV pencerelerinde göster. Non-blocking."""
        for did, cam in self.cameras.items():
            frame = cam.get_frame()
            if frame is not None:
                # Küçült (performans için)
                small = cv2.resize(frame, (640, 480))
                cv2.imshow(f"Drone {did} Kamera", small)
        cv2.waitKey(1)  # Non-blocking
