"""
QR Kod Sistemi — Gerçek QR code üretimi + tarama
Şartname Şekil 2 formatına uygun JSON mesajlar.
"""
import json
import numpy as np

import qrcode
from PIL import Image


# ── Resmi QR JSON Formatı (Şekil 2) ───────────────────────

def build_official_json(qr_id: int, qr_data: dict) -> dict:
    """
    YAML config → Şartname Şekil 2 formatında JSON payload.
    {
      "qr_id": 1,
      "gorev": {
        "formasyon": {"aktif": true, "tip": "OKBASI"},
        "manevra_pitch_roll": {"aktif": false, "pitch_deg": "0", "roll_deg": "0"},
        "irtifa_degisim": {"aktif": true, "deger": 20},
        "bekleme_suresi_s": 3
      },
      "suruden_ayrilma": {
        "aktif": false,
        "ayrilacak_drone_id": null,
        "hedef_renk": null,
        "bekleme_suresi_s": null
      },
      "sonraki_qr": {"team_1": 4, "team_2": 3, "team_3": 5}
    }
    """
    gorev = qr_data.get('gorev', {})
    suruden = qr_data.get('suruden_ayrilma', {})
    sonraki = qr_data.get('sonraki_qr', {})

    payload = {
        "qr_id": qr_id,
        "gorev": {
            "formasyon": {
                "aktif": gorev.get('formasyon', {}).get('aktif', False),
                "tip": gorev.get('formasyon', {}).get('tip', None),
            },
            "manevra_pitch_roll": {
                "aktif": gorev.get('manevra_pitch_roll', {}).get('aktif', False),
                "pitch_deg": str(gorev.get('manevra_pitch_roll', {}).get('pitch_deg', '0')),
                "roll_deg": str(gorev.get('manevra_pitch_roll', {}).get('roll_deg', '0')),
            },
            "irtifa_degisim": {
                "aktif": gorev.get('irtifa_degisim', {}).get('aktif', False),
                "deger": gorev.get('irtifa_degisim', {}).get('deger', None),
            },
            "bekleme_suresi_s": gorev.get('bekleme_suresi_s', 0),
        },
        "suruden_ayrilma": {
            "aktif": suruden.get('aktif', False),
            "ayrilacak_drone_id": suruden.get('ayrilacak_drone_id', None),
            "hedef_renk": suruden.get('hedef_renk', None),
            "bekleme_suresi_s": suruden.get('bekleme_suresi_s', None),
        },
        "sonraki_qr": {
            "team_1": sonraki.get('team_1', 0),
            "team_2": sonraki.get('team_2', 0),
            "team_3": sonraki.get('team_3', 0),
        },
    }
    return payload


def generate_qr_image(data_str: str, box_size: int = 8, border: int = 2) -> Image.Image:
    """data_str → PIL Image (gerçek QR code)."""
    qr = qrcode.QRCode(
        version=None,
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=box_size,
        border=border,
    )
    qr.add_data(data_str)
    qr.make(fit=True)
    return qr.make_image(fill_color='black', back_color='white').convert('RGB')


def qr_image_to_pygame(pil_img) -> 'pygame.Surface':
    """PIL Image → pygame.Surface."""
    import pygame
    raw = pil_img.tobytes()
    size = pil_img.size
    return pygame.image.fromstring(raw, size, 'RGB')


# Formasyon tip haritası: resmi Türkçe → kod enum string
FORMATION_TIP_MAP = {
    'OKBASI': 'arrow',
    'CIZGI': 'line',
    'V': 'v',
    'DELTA': 'delta',
    'ELMAS': 'diamond',
}


class QRCodeData:
    """Tek bir QR kodu — pozisyon, resmi JSON veri, görsel."""

    def __init__(self, qr_id: int, position: list, qr_raw: dict):
        self.qr_id = qr_id
        self.position = np.array(position, dtype=float)
        self.raw_data = qr_raw

        # Resmi format JSON
        self.payload = build_official_json(qr_id, qr_raw)
        self.json_str = json.dumps(self.payload, ensure_ascii=False, separators=(',', ':'))

        # Gerçek QR code görseli
        self.pil_image = generate_qr_image(self.json_str)

        # Pygame surface (lazy)
        self._pygame_surface = None

    @property
    def pygame_surface(self):
        if self._pygame_surface is None:
            self._pygame_surface = qr_image_to_pygame(self.pil_image)
        return self._pygame_surface

    def scan(self, team_id: int) -> dict:
        """
        QR'ı tara — gerçek JSON decode.
        Returns: parsed payload dict (resmi format).
        """
        decoded = json.loads(self.json_str)
        return decoded

    def get_next_qr(self, team_id: int) -> int:
        """Sonraki QR id (takıma göre)."""
        return self.payload.get('sonraki_qr', {}).get(f'team_{team_id}', 0)


class QRManager:
    """Sahadaki tüm QR kodlarını yönetir — resmi formatta üretir, tarar."""

    def __init__(self, qr_list: list, team_id: int = 1):
        self.team_id = team_id
        self.qr_codes: dict[int, QRCodeData] = {}

        for qr_data in qr_list:
            qr = QRCodeData(
                qr_id=qr_data['id'],
                position=qr_data['position'],
                qr_raw=qr_data,
            )
            self.qr_codes[qr.qr_id] = qr

    def get_qr(self, qr_id: int) -> QRCodeData | None:
        return self.qr_codes.get(qr_id)

    def scan_qr(self, qr_id: int) -> dict:
        """QR'ı tara — resmi JSON decode döndür."""
        qr = self.qr_codes.get(qr_id)
        if qr is None:
            return {}
        return qr.scan(self.team_id)

    def find_nearest_qr(self, position_xy: np.ndarray, max_range: float = 3.0) -> QRCodeData | None:
        """Pozisyona en yakın QR."""
        best = None
        best_dist = max_range
        for qr in self.qr_codes.values():
            dist = np.linalg.norm(qr.position[:2] - position_xy[:2])
            if dist < best_dist:
                best_dist = dist
                best = qr
        return best
