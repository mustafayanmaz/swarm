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
# PROTOBUF UYUMLULUĞU: gz-msgs + MAVSDK birlikte çalışması için
# TÜM import'lardan ÖNCE set edilmeli
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
import logging
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

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


def read_qr_from_camera(cameras, timeout: float = 10.0) -> dict | None:
    """
    TÜM drone kameralarından QR kod oku.
    Sürü merkezinde olan drone QR'ın üstünde olmayabilir (formasyon offset),
    bu yüzden tüm kameraları tarayarak en yakın olanı bulsun.
    """
    num = cameras.num_drones
    start = time.time()
    while time.time() - start < timeout:
        for cam_id in range(num):
            frame = cameras.get_frame(cam_id)
            if frame is not None:
                content = detect_qr(frame, save_debug=(cam_id == 0))
                if content is not None:
                    log.info(f"    (Drone {cam_id} kamerasından okundu)")
                    return content
        time.sleep(0.1)
    return None


QR_READ_ALTITUDE = 6.0   # İlk okuma denemesi irtifası
QR_READ_ALTITUDE_2 = 4.0  # İlk başarısız olursa daha da alçal


async def read_qr(qr_id: int, ctrl: SwarmController, cameras=None) -> dict:
    """
    QR kod içeriğini oku.
    1. Mevcut irtifadan dene (3s)
    2. 8m'ye in, dene (5s)
    3. 5m'ye in, dene (5s)
    4. Görev irtifasına geri çık
    5. Okunamazsa config fallback
    """
    log.info(f"  📷 QR{qr_id} okunuyor...")

    mission_alt = ctrl.altitude

    if cameras is not None:
        # Önce mevcut irtifadan dene (hızlı)
        log.info(f"  📷 [1/3] Mevcut irtifadan deneniyor ({mission_alt:.0f}m, 3s)...")
        content = read_qr_from_camera(cameras, timeout=3.0)
        if content is not None:
            log.info(f"  ✅ QR{qr_id} KAMERA ile okundu! (irtifa: {mission_alt:.0f}m)")
            return content
        log.info(f"  ❌ {mission_alt:.0f}m'den okunamadı")

        # Kademeli alçalma
        step = 2
        for read_alt in [QR_READ_ALTITUDE, QR_READ_ALTITUDE_2]:
            if ctrl.altitude <= read_alt + 0.5:
                log.info(f"  📷 [{step}/3] Zaten {ctrl.altitude:.0f}m'de, {read_alt}m atlanıyor")
                step += 1
                continue
            log.info(f"  📷 [{step}/3] Alçalıyor: {ctrl.altitude:.0f}m → {read_alt}m")
            await ctrl.change_altitude(read_alt)
            await asyncio.sleep(1.0)

            log.info(f"  📷 [{step}/3] {read_alt}m'den deneniyor (5s)...")
            content = read_qr_from_camera(cameras, timeout=5.0)
            if content is not None:
                log.info(f"  ✅ QR{qr_id} KAMERA ile okundu! (irtifa: {read_alt}m)")
                log.info(f"  📷 Görev irtifasına dönüş: {read_alt}m → {mission_alt:.0f}m")
                await ctrl.change_altitude(mission_alt)
                return content
            log.info(f"  ❌ {read_alt}m'den okunamadı")
            step += 1

        # Hiçbirinde okunamadı, irtifaya dön
        log.warning(f"  ⚠ QR{qr_id} kameradan okunamadı (3 irtifa denendi), config fallback...")
        await ctrl.change_altitude(mission_alt)

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
    cameras=None, detected_zones: dict = None
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
        MAX_ALT = 6.0
        if deger > MAX_ALT:
            log.warning(f"  ⚠ İrtifa {deger}m > {MAX_ALT}m üst sınır, {MAX_ALT}m'ye kısıtlandı")
            deger = MAX_ALT
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
            await ctrl.remove_agent(drone_id, landing_zone, cameras=cameras, target_color=renk)

            log.info(f"  ▶ Drone {drone_id} yerde bekliyor ({bekle}s)...")
            await asyncio.sleep(bekle)

            log.info(f"  ▶ Drone {drone_id} sürüye geri katılıyor...")
            await ctrl.add_agent(drone_id)

    # ── 5) Bekleme ──
    bekle_s = gorev.get("bekleme_suresi_s", 0)
    if bekle_s > 0:
        log.info(f"  ▶ QR{qr_id} bekleme: {bekle_s}s")
        await ctrl.hold(bekle_s)


async def move_to_with_color_scan(ctrl, target_ne, cameras, detected_zones):
    """
    Sürüyü hedefe taşırken kamera ile renkli alan taraması yap.
    Hareket sırasında her 0.5 saniyede bir frame kontrol eder.
    Renkli alan tespit ederse o anki NED koordinatını kaydeder.
    """
    import math

    start = ctrl.swarm_center
    dist = math.hypot(target_ne[0] - start[0], target_ne[1] - start[1])

    if dist < 0.5:
        return

    # Hareket başlat (arka planda ctrl.move_to çalışırken tarama yapmak için)
    move_task = asyncio.ensure_future(ctrl.move_to(target_ne, speed=CRUISE_SPEED))

    # Hareket süresince renkli alan taraması (tüm kameralar)
    if cameras is not None:
        scan_interval = 0.1  # saniye (3m/s hızda 0.3m arayla tarama)
        while not move_task.done():
            for cam_id in range(cameras.num_drones):
                frame = cameras.get_frame(cam_id)
                if frame is not None:
                    color = detect_color_zone(frame)
                    if color and color not in detected_zones:
                        # Algılayan drone'un gerçek NED pozisyonunu hesapla
                        # (swarm center değil — formasyon offset'i dahil)
                        targets = ctrl._get_global_targets()
                        if cam_id in targets:
                            drone_n, drone_e = targets[cam_id][0], targets[cam_id][1]
                            detected_zones[color] = (drone_n, drone_e)
                        else:
                            detected_zones[color] = tuple(ctrl.swarm_center)
                        pos = detected_zones[color]
                        log.info(
                            f"  🎨 KAMERA (Drone {cam_id}): {color} alan tespit edildi! "
                            f"NED=({pos[0]:.1f}, {pos[1]:.1f})"
                        )
            await asyncio.sleep(scan_interval)

    await move_task


async def run_autonomous_mission(ctrl: SwarmController, cameras=None):
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

        # QR noktasına git — hareket sırasında kamera ile renk taraması
        await move_to_with_color_scan(ctrl, qr_pos, cameras, detected_zones)

        # QR'ı oku (alçal → kamera → yüksel, fallback config)
        content = await read_qr(current_qr, ctrl, cameras)
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

    async def _camera_viewer(cameras):
        """Arka planda kamera görüntülerini göster (5 FPS)."""
        import cv2
        while True:
            if cameras is not None:
                cameras.show_frames()
            await asyncio.sleep(0.2)

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

        # Kamera görüntüleyici arka planda çalışsın
        viewer_task = asyncio.ensure_future(_camera_viewer(cameras))

        try:
            await run_autonomous_mission(ctrl, cameras)
        except KeyboardInterrupt:
            await ctrl.land_all()
        finally:
            viewer_task.cancel()
            import cv2
            cv2.destroyAllWindows()

    asyncio.run(_run())
