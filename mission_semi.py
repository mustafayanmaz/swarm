"""
TEKNOFEST 2026 Sürü İHA - Görev 5.2: Yarı Otonom Sürü Kontrolü

Tek klavye/joystick ile sürüyü yönlendirme.
"""
import asyncio
import logging
import math
import sys
import termios
import tty

from config import DEFAULT_ALTITUDE, DEFAULT_AGENT_DISTANCE
from swarm_controller import SwarmController

log = logging.getLogger("swarm")

# Kontrol parametreleri
MOVE_STEP = 3.0       # metre (her tuş basımında)
ALT_STEP = 2.0        # metre
YAW_STEP = 15.0       # derece
MANEUVER_ANGLE = 15.0  # derece


def _get_key() -> str:
    """Terminelden tek tuş oku (blocking)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def _print_controls():
    """Kontrol kılavuzunu yazdır."""
    print()
    print("╔══════════════════════════════════════════════╗")
    print("║    YARI OTONOM SÜRÜ KONTROL PANELİ          ║")
    print("╠══════════════════════════════════════════════╣")
    print("║  T : Kalkış          L : İniş + Çıkış       ║")
    print("╠══════════════════════════════════════════════╣")
    print("║  SÜRÜ HAREKET MODU:                         ║")
    print("║  W : İleri           S : Geri                ║")
    print("║  A : Sol             D : Sağ                 ║")
    print("║  Q : Yaw Sol (CCW)   E : Yaw Sağ (CW)       ║")
    print("║  R : İrtifa Artır    F : İrtifa Azalt        ║")
    print("╠══════════════════════════════════════════════╣")
    print("║  FORMASYON DEĞİŞİMİ:                        ║")
    print("║  1 : Çizgi           2 : Ok Başı             ║")
    print("║  3 : V Formasyonu    4 : Üçgen               ║")
    print("╠══════════════════════════════════════════════╣")
    print("║  MANEVRA MODU:                               ║")
    print("║  P : Pitch (+15°)    O : Roll (+15°)         ║")
    print("║  I : Pitch (-15°)    U : Roll (-15°)         ║")
    print("╠══════════════════════════════════════════════╣")
    print("║  X : Çıkış (İniş yapmadan)                  ║")
    print("╚══════════════════════════════════════════════╝")
    print()


async def run_semi_auto(ctrl: SwarmController):
    """
    Görev 5.2 — Yarı Otonom Sürü Kontrolü.

    Klavyeden gelen komutlarla sürüyü formasyon halinde kontrol et.
    Gerçek yarışmada joystick/RC kumanda kullanılacak.
    """
    log.info("═" * 55)
    log.info("  GÖREV 5.2 — YARI OTONOM SÜRÜ KONTROLÜ")
    log.info("═" * 55)

    ctrl.formation_type = "line"
    ctrl.formation_distance = DEFAULT_AGENT_DISTANCE

    _print_controls()

    loop = asyncio.get_event_loop()
    running = True

    while running:
        # Durum bilgisi
        h_deg = math.degrees(ctrl.formation_heading)
        cx, cy = ctrl.swarm_center
        status = (
            f"[Merkez: N={cx:.1f} E={cy:.1f} | "
            f"İrtifa: {ctrl.altitude:.0f}m | "
            f"Heading: {h_deg:.0f}° | "
            f"Formasyon: {ctrl.formation_type}] > "
        )
        sys.stdout.write(f"\r{status}")
        sys.stdout.flush()

        key = await loop.run_in_executor(None, _get_key)
        key = key.lower()

        if key == "t":
            print("\n[KALKIŞ]")
            await ctrl.takeoff(altitude=DEFAULT_ALTITUDE)

        elif key == "l":
            print("\n[İNİŞ]")
            await ctrl.return_home()
            running = False

        elif key == "w":
            dn = MOVE_STEP * math.cos(ctrl.formation_heading)
            de = MOVE_STEP * math.sin(ctrl.formation_heading)
            ctrl.swarm_center = (cx + dn, cy + de)
            await ctrl.send_positions()

        elif key == "s":
            dn = -MOVE_STEP * math.cos(ctrl.formation_heading)
            de = -MOVE_STEP * math.sin(ctrl.formation_heading)
            ctrl.swarm_center = (cx + dn, cy + de)
            await ctrl.send_positions()

        elif key == "a":
            dn = MOVE_STEP * math.cos(ctrl.formation_heading - math.pi / 2)
            de = MOVE_STEP * math.sin(ctrl.formation_heading - math.pi / 2)
            ctrl.swarm_center = (cx + dn, cy + de)
            await ctrl.send_positions()

        elif key == "d":
            dn = MOVE_STEP * math.cos(ctrl.formation_heading + math.pi / 2)
            de = MOVE_STEP * math.sin(ctrl.formation_heading + math.pi / 2)
            ctrl.swarm_center = (cx + dn, cy + de)
            await ctrl.send_positions()

        elif key == "r":
            ctrl.altitude += ALT_STEP
            await ctrl.send_positions()

        elif key == "f":
            ctrl.altitude = max(3.0, ctrl.altitude - ALT_STEP)
            await ctrl.send_positions()

        elif key == "q":
            ctrl.formation_heading -= math.radians(YAW_STEP)
            await ctrl.send_positions()

        elif key == "e":
            ctrl.formation_heading += math.radians(YAW_STEP)
            await ctrl.send_positions()

        elif key == "1":
            print("\n[FORMASYON: Çizgi]")
            await ctrl.change_formation("line")

        elif key == "2":
            print("\n[FORMASYON: Ok Başı]")
            await ctrl.change_formation("arrow")

        elif key == "3":
            print("\n[FORMASYON: V]")
            await ctrl.change_formation("v")

        elif key == "4":
            print("\n[FORMASYON: Üçgen]")
            await ctrl.change_formation("triangle")

        elif key == "p":
            print("\n[PITCH +15°]")
            await ctrl.pitch_maneuver(MANEUVER_ANGLE, hold=3.0)

        elif key == "i":
            print("\n[PITCH -15°]")
            await ctrl.pitch_maneuver(-MANEUVER_ANGLE, hold=3.0)

        elif key == "o":
            print("\n[ROLL +15°]")
            await ctrl.roll_maneuver(MANEUVER_ANGLE, hold=3.0)

        elif key == "u":
            print("\n[ROLL -15°]")
            await ctrl.roll_maneuver(-MANEUVER_ANGLE, hold=3.0)

        elif key == "x":
            print("\n[ÇIKIŞ]")
            running = False

        elif key == "\x03":  # Ctrl+C
            print("\n[CTRL+C]")
            running = False

    log.info("Yarı otonom kontrol tamamlandı.")


if __name__ == "__main__":
    import logging
    from config import DRONE_PORTS, GRPC_BASE_PORT

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    async def _run():
        ctrl = SwarmController()
        await ctrl.connect(DRONE_PORTS, GRPC_BASE_PORT)
        try:
            await run_semi_auto(ctrl)
        except KeyboardInterrupt:
            await ctrl.land_all()

    asyncio.run(_run())
