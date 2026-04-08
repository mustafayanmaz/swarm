"""
Sürü Kontrolcüsü — Fiziksiz (Ursina tabanlı)
Drone'lar hedefe yumuşak interpolasyonla gider. Gerçek PID/RPM yok.
"""
import numpy as np
from src.formation import (
    FormationType,
    get_offsets_2d,
    compute_world_positions,
    heading_to_target,
    FORMATION_NAME_MAP,
)


class SwarmSim:
    """
    N drone'luk sürüyü fiziksiz kontrol eder.
    Her update'te formasyon pozisyonlarını hesaplar, drone entity'lere gönderir.
    """

    def __init__(self, num_drones: int):
        self.num_drones = num_drones

        # Her drone'un mevcut pozisyonu (sim koordinatları: x=ileri, y=sağ, z=yükseklik)
        self.positions = [np.array([0.0, 0.0, 0.0]) for _ in range(num_drones)]

        # Formasyon
        self.formation_type = FormationType.LINE
        self.spacing = 3.0
        self.centroid = np.array([0.0, 0.0, 0.0])
        self.heading = 0.0  # rad
        self.target_altitude = 0.0
        self.pitch_angle = 0.0
        self.roll_angle = 0.0

        # Aktif drone'lar
        self.active_drones = list(range(num_drones))

        # Navigasyon
        self._nav_target = None  # (x, y)
        self._nav_speed = 3.0  # m/s

        # Kalkış/iniş
        self._taking_off = False
        self._landing = False

    # ── Formasyon ─────────────────────────────────────────────

    def set_formation(self, formation_type: FormationType, spacing: float = None):
        self.formation_type = formation_type
        if spacing is not None:
            self.spacing = spacing

    def set_heading(self, heading: float):
        self.heading = heading

    def set_pitch_maneuver(self, angle_deg: float):
        self.pitch_angle = np.radians(angle_deg)

    def set_roll_maneuver(self, angle_deg: float):
        self.roll_angle = np.radians(angle_deg)

    def clear_maneuvers(self):
        self.pitch_angle = 0.0
        self.roll_angle = 0.0

    def set_altitude(self, alt: float):
        self.target_altitude = alt
        self.centroid[2] = alt

    # ── Sürü Birey Yönetimi ──────────────────────────────────

    def remove_drone(self, drone_id: int):
        if drone_id in self.active_drones:
            self.active_drones.remove(drone_id)

    def add_drone(self, drone_id: int):
        if drone_id not in self.active_drones:
            self.active_drones.append(drone_id)
            self.active_drones.sort()

    # ── Kalkış / İniş ────────────────────────────────────────

    def takeoff(self, altitude: float):
        self.target_altitude = altitude
        self.centroid[2] = altitude
        self._taking_off = True
        self._landing = False

    def land(self):
        self.target_altitude = 0.0
        self.centroid[2] = 0.0
        self._landing = True
        self._taking_off = False

    # ── Navigasyon ────────────────────────────────────────────

    def navigate_to(self, target_xy: np.ndarray, auto_heading: bool = True):
        self._nav_target = np.array(target_xy[:2], dtype=float)
        if auto_heading:
            self.heading = heading_to_target(self.centroid[:2], self._nav_target)

    def is_at_target(self, threshold: float = 0.5) -> bool:
        if self._nav_target is None:
            return True
        dist = np.linalg.norm(self.centroid[:2] - self._nav_target)
        return dist < threshold

    def _update_navigation(self, dt: float):
        if self._nav_target is None:
            return
        diff = self._nav_target - self.centroid[:2]
        dist = np.linalg.norm(diff)
        if dist < 0.1:
            self.centroid[0] = self._nav_target[0]
            self.centroid[1] = self._nav_target[1]
            self._nav_target = None
            return
        direction = diff / dist
        step_size = min(self._nav_speed * dt, dist)
        self.centroid[0] += direction[0] * step_size
        self.centroid[1] += direction[1] * step_size

    # ── Hareket (Yarı Otonom) ─────────────────────────────────

    def move_forward(self, speed: float, dt: float):
        self.centroid[0] += speed * dt * np.cos(self.heading)
        self.centroid[1] += speed * dt * np.sin(self.heading)

    def move_right(self, speed: float, dt: float):
        right = self.heading - np.pi / 2
        self.centroid[0] += speed * dt * np.cos(right)
        self.centroid[1] += speed * dt * np.sin(right)

    def yaw_rotate(self, rate: float, dt: float):
        self.heading += rate * dt

    def change_altitude(self, rate: float, dt: float):
        self.target_altitude += rate * dt
        self.target_altitude = max(0.0, self.target_altitude)
        self.centroid[2] = self.target_altitude

    # ── Ana Güncelleme ────────────────────────────────────────

    def update(self, dt: float) -> list[np.ndarray]:
        """
        Her frame çağır. Drone hedef pozisyonlarını döndür.
        Returns: [np.array([x, y, z]), ...] — her drone için hedef
        """
        self._update_navigation(dt)
        self.centroid[2] = self.target_altitude

        # Aktif drone'lar için formasyon pozisyonları
        n_active = len(self.active_drones)
        offsets = get_offsets_2d(self.formation_type, n_active, self.spacing)
        targets_active = compute_world_positions(
            offsets, self.centroid, self.heading,
            self.pitch_angle, self.roll_angle,
        )

        # Greedy atama (drone'ların mevcut pozisyonlarına göre)
        assigned = self._assign_targets_greedy(targets_active)

        # Tüm drone'lar için hedefler
        all_targets = []
        for i in range(self.num_drones):
            if i in self.active_drones:
                idx = self.active_drones.index(i)
                all_targets.append(assigned[idx])
            else:
                # Pasif drone: yerde bekle
                all_targets.append(np.array([
                    self.positions[i][0],
                    self.positions[i][1],
                    0.0,
                ]))
        return all_targets

    def _assign_targets_greedy(self, target_positions: list) -> list:
        n = len(self.active_drones)
        if n == 0:
            return []
        cur = [self.positions[d_id].copy() for d_id in self.active_drones]
        available = list(range(n))
        assignment = [None] * n

        distances = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                distances[i, j] = np.linalg.norm(cur[i] - target_positions[j])

        for _ in range(n):
            min_dist = float('inf')
            best_i, best_j = 0, 0
            for i in range(n):
                if assignment[i] is not None:
                    continue
                for j in available:
                    if distances[i, j] < min_dist:
                        min_dist = distances[i, j]
                        best_i, best_j = i, j
            assignment[best_i] = target_positions[best_j]
            available.remove(best_j)

        return assignment

    def set_position(self, drone_id: int, pos: np.ndarray):
        """Drone'un mevcut pozisyonunu bildir (entity'den geri besleme)."""
        self.positions[drone_id] = pos.copy()

    def get_centroid_actual(self) -> np.ndarray:
        """Aktif drone'ların gerçek ortalaması."""
        if not self.active_drones:
            return self.centroid.copy()
        positions = [self.positions[d] for d in self.active_drones]
        return np.mean(positions, axis=0)
