"""
Formasyon Geometrileri — TEKNOFEST Sürü İHA
Desteklenen formasyonlar: Okbaşı (Arrow), Çizgi (Line), V, Delta, Elmas (Diamond)
"""
import numpy as np
from enum import Enum


class FormationType(Enum):
    ARROW = "arrow"
    LINE = "line"
    V = "v"
    DELTA = "delta"
    DIAMOND = "diamond"


FORMATION_NAME_MAP = {
    "ok basi": FormationType.ARROW,
    "okbasi": FormationType.ARROW,
    "arrow": FormationType.ARROW,
    "cizgi": FormationType.LINE,
    "line": FormationType.LINE,
    "v": FormationType.V,
    "delta": FormationType.DELTA,
    "elmas": FormationType.DIAMOND,
    "diamond": FormationType.DIAMOND,
}


def get_offsets_2d(formation: FormationType, n: int, spacing: float) -> list:
    """
    n adet drone için formasyon çerçevesinde (x_fwd, y_right) ofsetleri döndür.
    Merkez (0,0) sürü centroid'idir.
    x = ileri (heading yönü), y = sağ
    """
    if n <= 0:
        return []

    offsets = []

    if formation == FormationType.LINE:
        for i in range(n):
            y = (i - (n - 1) / 2.0) * spacing
            offsets.append((0.0, y))

    elif formation == FormationType.ARROW:
        if n == 1:
            offsets = [(0.0, 0.0)]
        elif n == 2:
            offsets = [(spacing * 0.5, 0.0), (-spacing * 0.5, 0.0)]
        else:
            offsets.append((spacing, 0.0))  # Lider önde
            for i in range(1, n):
                row = (i + 1) // 2
                side = 1 if i % 2 == 1 else -1
                x = spacing - row * spacing * 0.7
                y = side * row * spacing
                offsets.append((x, y))

    elif formation == FormationType.V:
        offsets.append((spacing * 0.5, 0.0))  # Lider
        for i in range(1, n):
            row = (i + 1) // 2
            side = 1 if i % 2 == 1 else -1
            offsets.append((-row * spacing * 0.8, side * row * spacing))

    elif formation == FormationType.DELTA:
        if n <= 3:
            if n == 1:
                offsets = [(0.0, 0.0)]
            elif n == 2:
                offsets = [(spacing * 0.5, 0.0), (-spacing * 0.5, 0.0)]
            else:
                offsets = [
                    (spacing, 0.0),
                    (-spacing * 0.5, spacing * 0.8),
                    (-spacing * 0.5, -spacing * 0.8),
                ]
        else:
            rows = n  # enough rows to fit all drones
            k = 0
            for r in range(rows):
                count = min(n - k, r + 1)
                for c in range(count):
                    x = -r * spacing * 0.8
                    y = (c - (count - 1) / 2.0) * spacing
                    offsets.append((x, y))
                    k += 1
                    if k >= n:
                        break
                if k >= n:
                    break

    elif formation == FormationType.DIAMOND:
        if n == 1:
            offsets = [(0.0, 0.0)]
        elif n == 2:
            offsets = [(spacing * 0.5, 0.0), (-spacing * 0.5, 0.0)]
        elif n == 3:
            offsets = [
                (spacing, 0.0),
                (0.0, spacing),
                (0.0, -spacing),
            ]
        else:
            offsets = [
                (spacing, 0.0),       # Ön
                (0.0, spacing),       # Sağ
                (-spacing, 0.0),      # Arka
                (0.0, -spacing),      # Sol
            ]
            # Ekstra drone'lar aralara
            for i in range(4, n):
                angle = (i - 4) * (2 * np.pi / max(n - 4, 1))
                r = spacing * 0.5
                offsets.append((r * np.cos(angle), r * np.sin(angle)))

    # Centroid'i sıfıra taşı
    if offsets:
        cx = sum(o[0] for o in offsets) / len(offsets)
        cy = sum(o[1] for o in offsets) / len(offsets)
        offsets = [(o[0] - cx, o[1] - cy) for o in offsets]

    return offsets


def compute_world_positions(
    offsets_2d: list,
    centroid: np.ndarray,
    heading: float,
    pitch_angle: float = 0.0,
    roll_angle: float = 0.0,
) -> list:
    """
    2D formasyon ofsetlerini 3D dünya konumuna dönüştür.

    centroid    : [x, y, z] sürü merkezi (dünya)
    heading     : rad, +X ekseninden saat yönü tersine (Z ekseni etrafında)
    pitch_angle : rad, pozitif = ön yukarı
    roll_angle  : rad, pozitif = sağ taraf aşağı
    """
    centroid = np.asarray(centroid, dtype=float)

    # Pitch: ileri ekseni (X_body) etrafında eğim → Y ekseni etrafında döndürme
    cp, sp = np.cos(pitch_angle), np.sin(pitch_angle)
    R_pitch = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])

    # Roll: sağ ekseni (Y_body) etrafında eğim → X ekseni etrafında döndürme
    cr, sr = np.cos(roll_angle), np.sin(roll_angle)
    R_roll = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])

    # Yaw: Z ekseni etrafında (heading)
    ch, sh = np.cos(heading), np.sin(heading)
    R_yaw = np.array([[ch, -sh, 0], [sh, ch, 0], [0, 0, 1]])

    positions = []
    for ox, oy in offsets_2d:
        v = np.array([ox, oy, 0.0])
        v = R_pitch @ v
        v = R_roll @ v
        v = R_yaw @ v
        positions.append(centroid + v)

    return positions


def heading_to_target(current: np.ndarray, target: np.ndarray) -> float:
    """İki nokta arasındaki heading açısını hesapla (rad)."""
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    return float(np.arctan2(dy, dx))
