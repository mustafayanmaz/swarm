"""
Kamera tabanlı algılama modülü.
- QR kod okuma (pyzbar)
- Renkli alan tespiti (kırmızı/mavi HSV)
"""
import json
import logging
import cv2
import numpy as np
from pyzbar.pyzbar import decode as pyzbar_decode

log = logging.getLogger("swarm.detection")


def detect_qr(frame) -> dict | None:
    """
    Frame'den QR kod oku ve JSON olarak parse et.
    Returns: QR içeriği dict veya None
    """
    if frame is None:
        return None

    # Gri tonlama
    if len(frame.shape) == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame

    # Kontrast artır (SITL kameradan gelen görüntü zayıf olabiliyor)
    gray = cv2.equalizeHist(gray)

    results = pyzbar_decode(gray)
    for r in results:
        try:
            data = r.data.decode("utf-8")
            content = json.loads(data)
            log.info(f"QR tespit edildi: qr_id={content.get('qr_id')}")
            return content
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            log.warning(f"QR parse hatası: {e}")
    return None


def detect_color_zone(frame) -> str | None:
    """
    Frame'de kırmızı veya mavi alan tespit et.
    Returns: 'kirmizi', 'mavi' veya None
    """
    if frame is None:
        return None

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = hsv.shape[:2]

    # Sadece frame'in alt yarısına bak (aşağı bakan kamera — merkez bölge)
    roi = hsv[h // 4 : 3 * h // 4, w // 4 : 3 * w // 4]

    # Kırmızı range (HSV): iki aralık gerekiyor (H=0-10 ve 160-180)
    red_lower1 = np.array([0, 80, 80])
    red_upper1 = np.array([15, 255, 255])
    red_lower2 = np.array([160, 80, 80])
    red_upper2 = np.array([180, 255, 255])
    red_mask = cv2.inRange(roi, red_lower1, red_upper1) | cv2.inRange(roi, red_lower2, red_upper2)

    # Mavi range (HSV): H=100-130
    blue_lower = np.array([100, 80, 80])
    blue_upper = np.array([135, 255, 255])
    blue_mask = cv2.inRange(roi, blue_lower, blue_upper)

    red_px = cv2.countNonZero(red_mask)
    blue_px = cv2.countNonZero(blue_mask)
    total_px = roi.shape[0] * roi.shape[1]

    # En az %5 piksel renkli olmalı
    threshold = total_px * 0.05

    if red_px > threshold and red_px > blue_px:
        log.info(f"Kırmızı alan tespit edildi ({red_px}/{total_px} px)")
        return "kirmizi"
    elif blue_px > threshold and blue_px > red_px:
        log.info(f"Mavi alan tespit edildi ({blue_px}/{total_px} px)")
        return "mavi"

    return None
