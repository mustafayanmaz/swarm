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


_debug_counter = 0


def detect_qr(frame, save_debug: bool = False) -> dict | None:
    """
    Frame'den QR kod oku ve JSON olarak parse et.
    Returns: QR içeriği dict veya None
    """
    global _debug_counter
    if frame is None:
        return None

    # Gri tonlama
    if len(frame.shape) == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame

    # Upscale 3x — QR modülleri çok küçük olabiliyor (yüksek irtifa)
    h, w = gray.shape[:2]
    upscaled = cv2.resize(gray, (w * 3, h * 3), interpolation=cv2.INTER_CUBIC)

    # Debug: her 20 denemede frame kaydet
    if save_debug and _debug_counter % 20 == 0:
        cv2.imwrite(f"/tmp/qr_debug_raw_{_debug_counter}.png", gray)
        cv2.imwrite(f"/tmp/qr_debug_upscaled_{_debug_counter}.png", upscaled)
        log.info(f"  🔍 Debug frame kaydedildi: /tmp/qr_debug_*_{_debug_counter}.png")
    _debug_counter += 1

    # Birden fazla yöntem dene
    methods = []

    # Yöntem 1: Sadece upscale (Gazebo render zaten siyah/beyaz)
    methods.append(upscaled)

    # Yöntem 2: Upscale + OTSU threshold
    _, otsu = cv2.threshold(upscaled, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    methods.append(otsu)

    # Yöntem 3: Upscale + sharpen
    sharp_kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
    sharpened = cv2.filter2D(upscaled, -1, sharp_kernel)
    methods.append(sharpened)

    # Yöntem 4: Orijinal (upscale yok)
    methods.append(gray)

    for i, img in enumerate(methods):
        # pyzbar
        results = pyzbar_decode(img)
        for r in results:
            try:
                data = r.data.decode("utf-8")
                content = json.loads(data)
                log.info(f"QR tespit edildi: qr_id={content.get('qr_id')} (pyzbar, yöntem {i})")
                return content
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                log.warning(f"QR parse hatası: {e}")

        # OpenCV QRCodeDetector (pyzbar bulamazsa)
        detector = cv2.QRCodeDetector()
        val, pts, straight = detector.detectAndDecode(img)
        if val:
            try:
                content = json.loads(val)
                log.info(f"QR tespit edildi: qr_id={content.get('qr_id')} (opencv, yöntem {i})")
                return content
            except (json.JSONDecodeError, ValueError) as e:
                log.warning(f"QR parse hatası (opencv): {e}")

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
