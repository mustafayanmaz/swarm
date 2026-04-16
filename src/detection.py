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


def detect_qr(frame, save_debug: bool = False, return_meta: bool = False) -> dict | tuple[dict, float, int] | None:
    """
    Frame'den QR kod oku ve JSON olarak parse et.
        Returns:
            - return_meta=False: QR içeriği dict veya None
            - return_meta=True: (QR içeriği dict, güven skoru, yöntem_idx) veya None
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
        confidence = max(40.0, 100.0 - (i * 15.0))

        # pyzbar
        results = pyzbar_decode(img)
        for r in results:
            try:
                data = r.data.decode("utf-8")
                content = json.loads(data)
                log.info(f"QR tespit edildi: qr_id={content.get('qr_id')} (pyzbar, yöntem {i})")
                if return_meta:
                    return (content, confidence, i)
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
                if return_meta:
                    return (content, confidence, i)
                return content
            except (json.JSONDecodeError, ValueError) as e:
                log.warning(f"QR parse hatası (opencv): {e}")

    return None


def detect_color_zone(frame, return_confidence: bool = False) -> str | tuple[str, float] | None:
    """
    Frame'de kırmızı veya mavi alan tespit et.
        Returns:
            - return_confidence=False: 'kirmizi', 'mavi' veya None
            - return_confidence=True: ('kirmizi'|'mavi', confidence_pct) veya None
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

    # En az %2 piksel renkli olmalı (8m irtifada 1.2m alan küçük görünür)
    threshold = total_px * 0.02

    if red_px > threshold and red_px > blue_px:
        pct = red_px / total_px * 100
        log.info(f"🔴 KIRMIZI ALAN TESPİT EDİLDİ! ({pct:.1f}% piksel, {red_px}/{total_px})")
        if return_confidence:
            return ("kirmizi", pct)
        return "kirmizi"
    elif blue_px > threshold and blue_px > red_px:
        pct = blue_px / total_px * 100
        log.info(f"🔵 MAVİ ALAN TESPİT EDİLDİ! ({pct:.1f}% piksel, {blue_px}/{total_px})")
        if return_confidence:
            return ("mavi", pct)
        return "mavi"

    return None


def detect_motion_ratio(prev_frame, curr_frame) -> float:
    """Estimate motion ratio between two frames (0..1) for landing area stability checks."""
    if prev_frame is None or curr_frame is None:
        return 0.0

    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(prev_gray, curr_gray)
    _, mask = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
    changed = cv2.countNonZero(mask)
    total = mask.shape[0] * mask.shape[1]
    if total <= 0:
        return 0.0
    return float(changed) / float(total)


def detect_color_offset(frame, target_color: str, save_debug: bool = False, debug_tag: str = "") -> tuple[float, float] | None:
    """
    Frame'de hedef renkli alanın piksel merkezini bul.
    Returns: (raw_x, raw_y) normalize -1..+1, veya None
             raw_x: (cx - center) / half_width  — pozitif = sağ
             raw_y: (cy - center) / half_height — pozitif = aşağı

    NOT: Bu ham piksel offsetleridir. NED eksenlerine dönüşüm
    swarm_controller'daki Jacobian kalibrasyonu ile yapılır.
    """
    if frame is None:
        return None

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = hsv.shape[:2]

    if target_color == "kirmizi":
        mask = (
            cv2.inRange(hsv, np.array([0, 80, 80]), np.array([15, 255, 255]))
            | cv2.inRange(hsv, np.array([160, 80, 80]), np.array([180, 255, 255]))
        )
    elif target_color == "mavi":
        mask = cv2.inRange(hsv, np.array([100, 80, 80]), np.array([135, 255, 255]))
    else:
        return None

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    colored_px = cv2.countNonZero(mask)
    if colored_px < (h * w * 0.002):
        return None

    M = cv2.moments(mask)
    if M["m00"] == 0:
        return None

    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]

    # Ham piksel offsetleri (NED dönüşümü yapılmıyor)
    raw_x = (cx - w / 2) / (w / 2)   # pozitif = sağ
    raw_y = (cy - h / 2) / (h / 2)   # pozitif = aşağı

    if save_debug:
        try:
            debug_frame = frame.copy()
            colored = np.zeros_like(debug_frame)
            colored[mask > 0] = (0, 255, 0)
            debug_frame = cv2.addWeighted(debug_frame, 0.7, colored, 0.3, 0)
            cv2.circle(debug_frame, (int(cx), int(cy)), 10, (0, 0, 255), 3)
            cv2.circle(debug_frame, (w // 2, h // 2), 8, (255, 255, 255), 2)
            txt = f"rx={raw_x:.3f} ry={raw_y:.3f} px=({int(cx)},{int(cy)}) cnt={colored_px}"
            cv2.putText(debug_frame, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            path = f"/tmp/align_debug_{debug_tag}.png"
            cv2.imwrite(path, debug_frame)
            log.info(f"  🔍 DEBUG {debug_tag}: {txt} → {path}")
        except Exception as e:
            log.warning(f"  Debug frame kayıt hatası: {e}")

    return (raw_x, raw_y)
