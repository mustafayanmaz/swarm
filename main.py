#!/usr/bin/env python3
"""
TEKNOFEST 2026 Sürü İHA — Ana Giriş Noktası

Kullanım:
  python main.py mission     Görev 5.1 — Tam otonom görev
  python main.py semi        Görev 5.2 — Yarı otonom kontrol
  python main.py formation   Demo: Formasyon değişimleri
  python main.py maneuver    Demo: Pitch/Roll manevraları
"""
import asyncio
import logging
import sys
from datetime import datetime, timezone

from src.config import (
    DRONE_PORTS,
    GRPC_BASE_PORT,
    INCIDENT_LOG_ENABLED,
    INCIDENT_LOG_PATH,
)
from src.swarm_controller import SwarmController

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("swarm")


async def demo_formations(ctrl: SwarmController):
    """Formasyon değişimi demosu."""
    log.info("═══ FORMASYON DEMO ═══")
    await ctrl.takeoff(altitude=15.0)

    for fmt in ["line", "arrow", "v", "triangle", "line"]:
        await ctrl.change_formation(fmt, distance=6.0)
        await ctrl.hold(3)

    await ctrl.return_home()


async def demo_maneuvers(ctrl: SwarmController):
    """Pitch/Roll manevra demosu."""
    log.info("═══ MANEVRA DEMO ═══")
    ctrl.formation_type = "arrow"
    ctrl.formation_distance = 6.0
    await ctrl.takeoff(altitude=15.0)

    log.info("--- Pitch manevrası ---")
    await ctrl.pitch_maneuver(-15.0, hold=4.0)
    await ctrl.hold(2)
    await ctrl.pitch_maneuver(15.0, hold=4.0)
    await ctrl.hold(2)

    log.info("--- Roll manevrası ---")
    await ctrl.roll_maneuver(15.0, hold=4.0)
    await ctrl.hold(2)
    await ctrl.roll_maneuver(-15.0, hold=4.0)
    await ctrl.hold(2)

    await ctrl.return_home()


async def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "mission"

    if mode == "--help" or mode == "-h":
        print(__doc__)
        return

    log.info("═" * 55)
    log.info("  TEKNOFEST 2026 — SÜRÜ İHA SİMÜLASYON")
    log.info(f"  Mod: {mode}")
    log.info(f"  Drone sayısı: {len(DRONE_PORTS)}")
    log.info("═" * 55)

    ctrl = SwarmController()
    await ctrl.connect(DRONE_PORTS, GRPC_BASE_PORT)

    async def _camera_viewer(cams):
        """Arka planda kamera görüntülerini göster (5 FPS)."""
        import cv2
        while True:
            cams.show_frames()
            await asyncio.sleep(0.2)

    try:
        if mode == "mission":
            from src.missions.autonomous import run_autonomous_mission
            from src.camera import SwarmCameras
            cameras = SwarmCameras(num_drones=len(DRONE_PORTS))
            await asyncio.sleep(2)  # İlk frame'lerin gelmesini bekle
            viewer_task = asyncio.ensure_future(_camera_viewer(cameras))
            await run_autonomous_mission(ctrl, cameras=cameras)
            viewer_task.cancel()

        elif mode == "semi":
            from src.missions.semi_auto import run_semi_auto
            await run_semi_auto(ctrl)

        elif mode == "formation":
            await demo_formations(ctrl)

        elif mode == "maneuver":
            await demo_maneuvers(ctrl)

        else:
            log.error(f"Bilinmeyen mod: {mode}")
            print(__doc__)

    except KeyboardInterrupt:
        log.info("Ctrl+C — Acil iniş...")
        await ctrl.land_all()

    except Exception as e:
        log.error(f"Hata: {e}", exc_info=True)
        log.info("Acil iniş deneniyor...")
        try:
            await ctrl.land_all()
        except Exception:
            pass
    finally:
        if INCIDENT_LOG_ENABLED:
            timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            output_path = ctrl.incident_log.save_json(INCIDENT_LOG_PATH.format(timestamp=timestamp))
            summary = ctrl.incident_log.summary()
            log.info("Incident log kaydedildi: %s", output_path)
            log.info(
                "Incident özeti: toplam=%s kodlar=%s",
                summary["total_incidents"],
                summary["by_code"],
            )


if __name__ == "__main__":
    asyncio.run(main())
