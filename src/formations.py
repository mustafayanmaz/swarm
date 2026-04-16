"""
TEKNOFEST 2026 Sürü İHA - Formasyon Geometri Hesaplamaları

Desteklenen formasyonlar:
  - arrow (ok başı): Lider önde, kanatlar arkada
  - line (çizgi): Yan yana dizilim
  - v: V formasyonu

Koordinat sistemi (body frame):
  forward = formasyonun baktığı yön
  right   = sağ taraf
"""
import math
from typing import Dict, List, Tuple


def get_formation_offsets(
    formation_type: str,
    num_agents: int,
    distance: float,
    agent_ids: List[int] = None,
) -> Dict[int, Tuple[float, float]]:
    """
    Her ajan için body frame'de (forward, right) offset hesapla.

    Args:
        formation_type: "arrow", "line", "v"
        num_agents: Aktif ajan sayısı
        distance: Ajanlar arası mesafe (metre)
        agent_ids: Ajan ID listesi (None ise 0..num_agents-1)

    Returns:
        {agent_id: (forward_offset, right_offset)}
    """
    if agent_ids is None:
        agent_ids = list(range(num_agents))

    n = len(agent_ids)
    offsets = {}

    if formation_type == "arrow":
        # Ok Başı: Sivri ok ucu şekli
        #     D0  (lider, en önde)
        #    / \
        #  D1   D2  (kanatlar, arkada dar açıyla)
        if n >= 1:
            offsets[agent_ids[0]] = (distance * 0.7, 0.0)
        if n >= 2:
            offsets[agent_ids[1]] = (-distance * 0.35, -distance * 0.4)
        if n >= 3:
            offsets[agent_ids[2]] = (-distance * 0.35, distance * 0.4)
        for i in range(3, n):
            row = (i - 1) // 2 + 1
            side = 1 if i % 2 == 0 else -1
            offsets[agent_ids[i]] = (
                -distance * 0.35 * (row + 1),
                side * distance * 0.4 * ((i + 1) // 2),
            )

    elif formation_type == "line":
        # D0 -- D1 -- D2  (heading'e dik, yan yana)
        center = (n - 1) / 2.0
        for i, aid in enumerate(agent_ids):
            offsets[aid] = (0.0, (i - center) * distance)

    elif formation_type == "v":
        # V Formasyonu: Lider arkada, kanatlar ileride açılır
        # D1    D2  (kanatlar, ileride geniş açıyla)
        #   \  /
        #    D0   (lider, arkada)
        if n >= 1:
            offsets[agent_ids[0]] = (-distance * 0.4, 0.0)
        for i in range(1, n):
            side = -1 if i % 2 == 1 else 1
            row = (i + 1) // 2
            offsets[agent_ids[i]] = (
                row * distance * 0.35,
                side * row * distance * 0.7,
            )

    else:
        return get_formation_offsets("line", num_agents, distance, agent_ids)

    return offsets


def rotate_offsets(
    offsets: Dict[int, Tuple[float, float]],
    heading_rad: float,
) -> Dict[int, Tuple[float, float]]:
    """
    Body frame offset'lerini heading'e göre NED frame'e döndür.

    Args:
        offsets: {agent_id: (forward, right)}
        heading_rad: Formasyon heading'i (rad, 0=Kuzey, pi/2=Doğu)

    Returns:
        {agent_id: (north_offset, east_offset)}
    """
    cos_h = math.cos(heading_rad)
    sin_h = math.sin(heading_rad)
    ned = {}
    for aid, (fwd, right) in offsets.items():
        north = fwd * cos_h - right * sin_h
        east = fwd * sin_h + right * cos_h
        ned[aid] = (north, east)
    return ned


def compute_heading(
    from_ne: Tuple[float, float],
    to_ne: Tuple[float, float],
) -> float:
    """İki NE noktası arasındaki heading açısını hesapla (radyan)."""
    dn = to_ne[0] - from_ne[0]
    de = to_ne[1] - from_ne[1]
    return math.atan2(de, dn)


def apply_pitch_offsets(
    body_offsets: Dict[int, Tuple[float, float]],
    pitch_deg: float,
) -> Dict[int, float]:
    """
    Pitch manevrası: Lateral (enlemesine) eksende eğim.
    Şekil 4'e göre pitch ekseni = right (yan) ekseni.
    Sürü merkezi sabit kalır.

    Returns:
        {agent_id: alt_offset} (pozitif = aşağı)
    """
    pitch_rad = math.radians(pitch_deg)
    return {
        aid: right * math.sin(pitch_rad)
        for aid, (_, right) in body_offsets.items()
    }


def apply_roll_offsets(
    body_offsets: Dict[int, Tuple[float, float]],
    roll_deg: float,
) -> Dict[int, float]:
    """
    Roll manevrası: Longitudinal (uzunlamasına) eksende eğim.
    Şekil 4'e göre roll ekseni = forward (ileri) ekseni.
    Sürü merkezi sabit kalır.

    Returns:
        {agent_id: alt_offset} (pozitif = aşağı)
    """
    roll_rad = math.radians(roll_deg)
    return {
        aid: fwd * math.sin(roll_rad)
        for aid, (fwd, _) in body_offsets.items()
    }
