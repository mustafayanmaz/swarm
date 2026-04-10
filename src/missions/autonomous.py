"""
TEKNOFEST 2026 Sürü İHA - Görev 5.1: Dinamik Sürü Kabiliyeti (Otonom Görev)

Akış:
  1. Hakemler formasyonu belirler, kalkış
  2. İlk QR noktasına (FIRST_QR) git
  3. QR'ı KAMERA ile oku → görev içeriğini parse et
  4. Görevleri sırayla icra et: formasyon → manevra → irtifa → birey çıkarma
  5. Rota üzerinde renkli alanları KAMERA ile tespit et
  6. sonraki_qr[team_id] → bir sonraki QR'a git
  7. sonraki_qr == 0 → eve dön, iniş
"""
import asyncio
import logging
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

from src.config import (
    DEFAULT_ALTITUDE,
    DEFAULT_FORMATION,
    DEFAULT_AGENT_DISTANCE,
    FIRST_QR,
    FORMATION_MAP,
    LANDING_ZONES,
    QR_CONTENTS,
    QR_POSITIONS,
    TEAM_ID,
    CRUISE_SPEED,
    NUM_DRONES,
)
from src.swarm_controller import SwarmController
from src.camera import SwarmCameras
from src.detection import detect_qr, detect_color_zone

log = logging.getLogger("swarm")


def read_qr_from_camera(cameras: SwarmCameras, timeout: float = 10.0) -> dict | None:
    """
    Kameradan QR kod oku. Lider drone (0) kamerasını kullanır.
    timeout saniye boyunca dener, bulamazsa None döner.
    """
    start = time.time()
    while time.time() - start < timeout:
        frame = cameras.get_frame(0)  # Lider drone kamerası
        if frame is not None:
            content = detect_qr(frame)
            if content is not None:
                return content
        time.sleep(0.1)
    return None


def read_qr(qr_id: int, cameras: SwarmCameras = None) -> dict:
    """
    QR kod içeriğini oku.
    1. Kameradan oku (cameras varsa)
    2. Kameradan okunamazsa config fallback
    """
    log.info(f"  📷 QR{qr_id} okunuyor...")

    # Kameradan dene
    if cameras is not None:
        content = read_qr_from_camera(cameras, timeout=8.0)
        if content is not None:
            log.info(f"  ✅ QR{qr_id} KAMERA ile okundu!")
            return content
        log.warning(f"  ⚠ QR{qr_id} kameradan okunamadı, config fallback...")

    # Config fallback
    content = QR_CONTENTS.get(qr_id)
    if content is None:
        log.error(f"  ❌ QR{qr_id} içeriği bulunamadı!")
        return None
    log.info(f"  ✅ QR{qr_id} config'den okundu")
    return content


def parse_next_qr(content: dict) -> int:
    """sonraki_qr alanından takım ID'sine göre sonraki QR numarasını al."""
    team_key = f"team_{TEAM_ID}"
    next_qr = content["sonraki_qr"].get(team_key, 0)
    return next_qr


async def execute_qr_mission(
    ctrl: SwarmController, content: dict,
    cameras: SwarmCameras = None, detected_zones: dict = None
):
    """
    QR içeriğindeki görevleri şartname sırasına göre icra et:
      1. Formasyon değişikliği
      2. Pitch/Roll manevrası
      3. İrtifa değişimi
      4. Sürüden birey ekleme/çıkarma
      5. Bekleme
    """
    gorev = content["gorev"]
    qr_id = content["qr_id"]

    # ── 1) Formasyon ──
    form = gorev["formasyon"]
    if form["aktif"]:
        tip = form["tip"]
        mesafe = form.get("mesafe", DEFAULT_AGENT_DISTANCE)
        code_type = FORMATION_MAP.get(tip, tip.lower())
        log.info(f"  ▶ Formasyon: {tip} (mesafe: {mesafe}m)")
        await ctrl.change_formation(code_type, mesafe)

    # ── 2) Manevra ──
    manevra = gorev["manevra_pitch_roll"]
    if manevra["aktif"]:
        pitch = float(manevra["pitch_deg"])
        roll = float(manevra["roll_deg"])
        if pitch != 0:
            log.info(f"  ▶ Pitch manevrası: {pitch}°")
            await ctrl.pitch_maneuver(pitch, hold=4.0)
        if roll != 0:
            log.info(f"  ▶ Roll manevrası: {roll}°")
            await ctrl.roll_maneuver(roll, hold=4.0)

    # ── 3) İrtifa ──
    irtifa = gorev["irtifa_degisim"]
    if irtifa["aktif"]:
        deger = float(irtifa["deger"])
        log.info(f"  ▶ İrtifa değişimi: {deger}m")
        await ctrl.change_altitude(deger)

    # ── 4) Sürüden birey ekleme/çıkarma ──
    ayrilma = content["suruden_ayrilma"]
    if ayrilma["aktif"]:
        drone_id = ayrilma["ayrilacak_drone_id"]
        renk = ayrilma["hedef_renk"]
        bekle = ayrilma.get("bekleme_suresi_s", 5)

        # Önce kamera ile tespit edilen konumu kullan, yoksa config fallback
        landing_zone = None
        if detected_zones and renk in detected_zones:
            landing_zone = detected_zones[renk]
            log.info(f"  ▶ {renk} bölge KAMERA ile tespit edilmişti: {landing_zone}")
        else:
            landing_zone = LANDING_ZONES.get(renk)
            if landing_zone:
                log.info(f"  ▶ {renk} bölge config'den: {landing_zone}")

        if landing_zone is None:
            log.error(f"  ❌ '{renk}' iniş bölgesi bulunamadı!")
        else:
            log.info(f"  ▶ Drone {drone_id} sürüden ayrılıyor → {renk} bölge")
            await ctrl.remove_agent(drone_id, landing_zone)

            log.info(f"  ▶ Drone {drone_id} yerde bekliyor ({bekle}s)...")
            await asyncio.sleep(bekle)

            log.info(f"  ▶ Drone {drone_id} sürüye geri katılıyor...")
            await ctrl.add_agent(drone_id)

    # ── 5) Bekleme ──
    bekle_s = gorev.get("bekleme_suresi_s", 0)
    if bekle_s > 0:
        log.info(f"  ▶ QR{qr_id} bekleme: {bekle_s}s")
        await ctrl.hold(bekle_s)


async def run_autonomous_mission(ctrl: SwarmController, cameras: SwarmCameras = None):
    """
    Görev 5.1 — Dinamik Sürü Kabiliyeti.

    Drone'lar QR→QR zıplayarak görev icra eder.
    Rota önceden bilinmez, her QR sonraki QR'ı söyler.
    Kamera ile QR okuma + renkli alan tespiti.
    """
    log.info("═" * 55)
    log.info("  GÖREV 5.1 — DİNAMİK SÜRÜ KABİLİYETİ (OTONOM)")
    log.info(f"  Takım ID: {TEAM_ID}")
    log.info(f"  İlk QR: QR{FIRST_QR}")
    log.info("═" * 55)

    # 1) Kalkış
    ctrl.formation_type = DEFAULT_FORMATION
    ctrl.formation_distance = DEFAULT_AGENT_DISTANCE
    await ctrl.takeoff(altitude=DEFAULT_ALTITUDE)

    # 2) Dinamik QR rotası
    current_qr = FIRST_QR
    visited = []
    step = 0
    detected_zones = {}  # {"kirmizi": (N, E), "mavi": (N, E)}

    while current_qr != 0:
        step += 1
        qr_pos = QR_POSITIONS.get(current_qr)
        if qr_pos is None:
            log.error(f"QR{current_qr} pozisyonu bilinmiyor!")
            break

        log.info("")
        log.info(f"{'═' * 50}")
        log.info(f"  ADIM {step}: QR{current_qr}")
        log.info(f"  Konum: N={qr_pos[0]:.1f}m, E={qr_pos[1]:.1f}m")
        log.info(f"  Ziyaret edilen: {visited}")
        log.info(f"{'═' * 50}")

        # QR noktasına git (formasyon rotasyonu ile)
        # Hareket sırasında kamera ile renkli alan taraması yap
        await ctrl.move_to(qr_pos, speed=CRUISE_SPEED)

        # Hareket sonrası renkli alan kontrolü
        if cameras is not None:
            frame = cameras.get_frame(0)
            color = detect_color_zone(frame)
            if color and color not in detected_zones:
                detected_zones[color] = ctrl.swarm_center
                log.info(f"  🎨 {color} alan tespit edildi: {ctrl.swarm_center}")

        # QR'ı oku (kamera + fallback)
        content = read_qr(current_qr, cameras)
        if content is None:
            log.error(f"QR{current_qr} okunamadı, görev durduruluyor!")
            break

        # Görevleri icra et
        await execute_qr_mission(ctrl, content, cameras, detected_zones)

        # Bir sonraki QR
        visited.append(current_qr)
        next_qr = parse_next_qr(content)
        log.info(f"  → Sonraki QR: {'EVE DÖNÜŞ' if next_qr == 0 else f'QR{next_qr}'}")

        if next_qr in visited:
            log.warning(f"  ⚠ QR{next_qr} zaten ziyaret edildi! Döngü olabilir.")

        current_qr = next_qr

    # 3) Eve dönüş
    log.info("")
    log.info("═" * 50)
    log.info("  EVE DÖNÜŞ")
    log.info(f"  Ziyaret edilen QR'lar: {visited}")
    log.info("═" * 50)
    await ctrl.return_home(speed=CRUISE_SPEED)

    log.info("")
    log.info("═" * 55)
    log.info("  GÖREV 5.1 TAMAMLANDI!")
    log.info("═" * 55)


if __name__ == "__main__":
    import logging
    from src.config import DRONE_PORTS, GRPC_BASE_PORT

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    async def _run():
        ctrl = SwarmController()
        await ctrl.connect(DRONE_PORTS, GRPC_BASE_PORT)

        # Kameraları başlat
        cameras = None
        try:
            cameras = SwarmCameras(num_drones=NUM_DRONES)
            log.info("Kameralar başlatıldı, 2s bekleniyor...")
            await asyncio.sleep(2)  # İlk frame'lerin gelmesini bekle
        except Exception as e:
            log.warning(f"Kamera başlatılamadı: {e} — config fallback aktif")

        try:
            await run_autonomous_mission(ctrl, cameras)
        except KeyboardInterrupt:
            await ctrl.land_all()

    asyncio.run(_run())
