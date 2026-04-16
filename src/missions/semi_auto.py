"""
TEKNOFEST 2026 Sürü İHA - Görev 5.2: Yarı Otonom Sürü Kontrolü

Şartname 5.2:
  - Tek joystick/kumanda ile sürüyü yönlendirme
  - İki mod: Sürü Hareket Modu + Manevra Modu
  - Sürü Hareket Modu: pitch=ileri/geri, roll=sağ/sol, yaw=formasyon rotasyonu, throttle=irtifa
  - Manevra Modu: pitch=eğim, roll=eğim, yaw=formasyon rotasyonu, throttle=irtifa
  - Formasyon değişimi kumandadan
  - Kalkış/iniş kumandadan, sürü halinde

Çalıştırma:
  python main.py semi
"""
import asyncio
import logging
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from src.config import DEFAULT_ALTITUDE, DEFAULT_AGENT_DISTANCE
from src.swarm_controller import SwarmController
from src.input_manager import InputManager, InputState

log = logging.getLogger("swarm")

# Formasyon döngüsü (kumandadan gezilecek sıra)
FORMATIONS = ["line", "arrow", "v"]
FORMATION_NAMES = {"line": "Çizgi", "arrow": "Ok Başı", "v": "V"}

# Kontrol döngüsü frekansı
LOOP_HZ = 20  # 20 Hz = 50ms per frame


class SemiAutoController:
    """Görev 5.2 — Yarı otonom sürü kontrolü."""

    def __init__(self, ctrl: SwarmController):
        self.ctrl = ctrl
        self.input = InputManager()
        self.mode = "hareket"  # "hareket" veya "manevra"
        self.formation_idx = 0  # FORMATIONS listesindeki index
        self.flying = False
        self.running = True
        self._takeoff_in_progress = False
        self._land_in_progress = False
        self._formation_in_progress = False
        self._maneuver_in_progress = False
        self._maneuver_key_was_pressed = False  # Tuş tekrar tetikleme engeli

    async def run(self):
        """Ana döngü — 20Hz input oku + pozisyon güncelle."""
        log.info("═" * 55)
        log.info("  GÖREV 5.2 — YARI OTONOM SÜRÜ KONTROLÜ")
        log.info(f"  Giriş cihazı: {self.input.device_name}")
        log.info(f"  Mod: {self.mode.upper()}")
        log.info("═" * 55)

        self._print_status_header()

        dt = 1.0 / LOOP_HZ
        last_status = time.time()

        try:
            while self.running:
                frame_start = time.time()

                # 1) Input oku
                state = self.input.read()

                # 2) Buton aksiyonları (one-shot)
                await self._handle_buttons(state)

                if not self.running:
                    break

                # 3) Sürekli kontrol (stick'ler)
                if self.flying and not self._takeoff_in_progress and not self._land_in_progress:
                    if self.mode == "hareket":
                        self.ctrl.apply_velocity(
                            state.pitch, state.roll,
                            state.yaw, state.throttle, dt
                        )
                    elif self.mode == "manevra":
                        self.ctrl.apply_tilt(
                            state.pitch, state.roll,
                            state.yaw, state.throttle, dt
                        )

                    # Pozisyonları güncelle (streaming zaten 10Hz çalışıyor)
                    await self.ctrl.send_positions()

                # 4) Durum bilgisi (her 0.5s)
                if time.time() - last_status > 0.5:
                    self._print_status(state)
                    last_status = time.time()

                # 5) Frame rate limiter
                elapsed = time.time() - frame_start
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)
                else:
                    await asyncio.sleep(0.001)  # yield

        except KeyboardInterrupt:
            log.info("Ctrl+C — Acil iniş...")

        finally:
            if self.flying:
                await self._do_land()
            self.input.close()

    async def _handle_buttons(self, state: InputState):
        """One-shot buton aksiyonları."""
        if state.quit:
            log.info("ÇIKIŞ komutu alındı")
            self.running = False
            return

        if state.takeoff and not self.flying and not self._takeoff_in_progress:
            await self._do_takeoff()

        if state.land and self.flying and not self._land_in_progress:
            await self._do_land()

        if state.toggle_mode and self.flying:
            old = self.mode
            self.mode = "manevra" if self.mode == "hareket" else "hareket"
            # Mod değişirken eğimleri sıfırla
            self.ctrl.pitch_angle = 0.0
            self.ctrl.roll_angle = 0.0
            log.info(f"MOD DEĞİŞİMİ: {old.upper()} → {self.mode.upper()}")

        if state.next_formation and self.flying and not self._formation_in_progress:
            self._formation_in_progress = True
            self.formation_idx = (self.formation_idx + 1) % len(FORMATIONS)
            new_form = FORMATIONS[self.formation_idx]
            log.info(f"FORMASYON → {FORMATION_NAMES[new_form]}")
            await self.ctrl.change_formation(new_form, self.ctrl.formation_distance)
            self._formation_in_progress = False

        if state.prev_formation and self.flying and not self._formation_in_progress:
            self._formation_in_progress = True
            self.formation_idx = (self.formation_idx - 1) % len(FORMATIONS)
            new_form = FORMATIONS[self.formation_idx]
            log.info(f"FORMASYON → {FORMATION_NAMES[new_form]}")
            await self.ctrl.change_formation(new_form, self.ctrl.formation_distance)
            self._formation_in_progress = False

        # Doğrudan formasyon seçimi (klavye 1/2/3)
        if state.set_formation >= 0 and self.flying and not self._formation_in_progress:
            idx = state.set_formation
            if idx < len(FORMATIONS):
                self._formation_in_progress = True
                self.formation_idx = idx
                new_form = FORMATIONS[idx]
                log.info(f"FORMASYON → {FORMATION_NAMES[new_form]}")
                await self.ctrl.change_formation(new_form, self.ctrl.formation_distance)
                self._formation_in_progress = False

        # Sabit açılı manevralar (P/I: pitch, O/U: roll) — KEYDOWN one-shot
        any_m = state.pitch_pos or state.pitch_neg or state.roll_pos or state.roll_neg
        if any_m:
            log.info(f"MANEVRA TUS: pp={state.pitch_pos} pn={state.pitch_neg} rp={state.roll_pos} rn={state.roll_neg} fly={self.flying} busy={self._maneuver_in_progress}")
        if self.flying and not self._maneuver_in_progress:
            if state.pitch_pos:
                asyncio.ensure_future(self._do_maneuver("pitch", 15.0))
            elif state.pitch_neg:
                asyncio.ensure_future(self._do_maneuver("pitch", -15.0))
            elif state.roll_pos:
                asyncio.ensure_future(self._do_maneuver("roll", 15.0))
            elif state.roll_neg:
                asyncio.ensure_future(self._do_maneuver("roll", -15.0))

    async def _do_maneuver(self, axis: str, angle: float):
        """Sabit açılı pitch veya roll manevrası icra et."""
        self._maneuver_in_progress = True
        if axis == "pitch":
            log.info(f"PITCH MANEVRASI: {angle:+.0f}°")
            await self.ctrl.pitch_maneuver(angle, hold=4.0)
        elif axis == "roll":
            log.info(f"ROLL MANEVRASI: {angle:+.0f}°")
            await self.ctrl.roll_maneuver(angle, hold=4.0)
        self._maneuver_in_progress = False

    async def _do_takeoff(self):
        """Sürü halinde kalkış."""
        self._takeoff_in_progress = True
        log.info("═══ KALKIŞ ═══")
        self.ctrl.formation_type = FORMATIONS[self.formation_idx]
        self.ctrl.formation_distance = DEFAULT_AGENT_DISTANCE
        await self.ctrl.takeoff(altitude=DEFAULT_ALTITUDE)
        self.flying = True
        self._takeoff_in_progress = False
        log.info("Sürü havada — kontrol aktif!")

    async def _do_land(self):
        """Sürü halinde iniş."""
        self._land_in_progress = True
        log.info("═══ İNİŞ ═══")
        # Eğimleri sıfırla
        self.ctrl.pitch_angle = 0.0
        self.ctrl.roll_angle = 0.0
        await self.ctrl.send_positions()
        await asyncio.sleep(2)

        # Home'a dön ve iniş
        await self.ctrl.return_home()
        self.flying = False
        self._land_in_progress = False
        self.running = False

    def _print_status_header(self):
        """Başlangıç bilgisi."""
        if self.input.is_joystick:
            print("\n🎮 JOYSTICK MODU")
            print("  A=Kalkış  B=İniş  X=Mod  LB/RB=Formasyon  Start=Çıkış")
        else:
            print("\n⌨️  KLAVYE MODU")
            print("  T=Kalkış  L=İniş  M=Mod Değiştir")
            print("  WASD=Hareket  QE=Yaw  RF=İrtifa")
            print("  1=Çizgi  2=Ok Başı  3=V Formasyonu")
            print("  P/I=Pitch +/-15°  O/U=Roll +/-15°")
            print("  ESC=Çıkış")
        print()

    def _print_status(self, state: InputState):
        """Durum satırı yazdır."""
        h_deg = math.degrees(self.ctrl.formation_heading) % 360
        cx, cy = self.ctrl.swarm_center
        form_name = FORMATION_NAMES.get(self.ctrl.formation_type, self.ctrl.formation_type)
        mode_str = "HAREKET" if self.mode == "hareket" else "MANEVRA"

        status_parts = [
            f"[{mode_str}]",
            f"Form: {form_name}",
            f"N={cx:+.1f} E={cy:+.1f}",
            f"Alt={self.ctrl.altitude:.0f}m",
            f"Hdg={h_deg:.0f}°",
        ]

        if self.mode == "manevra":
            if abs(self.ctrl.pitch_angle) > 0.5 or abs(self.ctrl.roll_angle) > 0.5:
                status_parts.append(
                    f"P={self.ctrl.pitch_angle:+.1f}° R={self.ctrl.roll_angle:+.1f}°"
                )

        if self._maneuver_in_progress:
            status_parts.append("MANEVRA...")

        if not self.flying:
            status_parts.insert(0, "YERDE")

        status = " | ".join(status_parts)
        sys.stdout.write(f"\r{status}          ")
        sys.stdout.flush()


async def run_semi_auto(ctrl: SwarmController):
    """
    Görev 5.2 — Yarı Otonom Sürü Kontrolü.
    main.py'den çağrılır.
    """
    semi = SemiAutoController(ctrl)
    await semi.run()


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
        try:
            await run_semi_auto(ctrl)
        except KeyboardInterrupt:
            await ctrl.land_all()

    asyncio.run(_run())
