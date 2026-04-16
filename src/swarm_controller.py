"""
TEKNOFEST 2026 Sürü İHA - Ana Sürü Kontrolcüsü

Tüm droneların bağlantı, formasyon, navigasyon, manevra
ve ajan ekleme/çıkarma işlemlerini yönetir.
"""
import asyncio
import logging
import math
import os
import sys
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
from mavsdk.telemetry import LandedState

from src.formations import (
    apply_pitch_offsets,
    apply_roll_offsets,
    compute_heading,
    get_formation_offsets,
    rotate_offsets,
)
from src.config import HOME_POSITION
from src.incidents import IncidentLog

try:
    from src.config import DRONE_SPAWNS_NED
except ImportError:
    DRONE_SPAWNS_NED = None

log = logging.getLogger("swarm")


class SwarmController:
    """3+ drone'u formasyon halinde kontrol eden ana sınıf."""

    def __init__(self):
        self.drones: Dict[int, System] = {}
        self.home_offsets: Dict[int, Tuple[float, float]] = {}
        self.ref_home_gps: Optional[Tuple[float, float]] = None
        self.active_agents: List[int] = []
        self.removed_agents: List[int] = []
        self._landing_positions: Dict[int, Tuple[float, float]] = {}

        # Formasyon durumu
        self.formation_type: str = "line"
        self.formation_distance: float = 5.0
        self.formation_heading: float = 0.0  # radyan, 0=Kuzey

        # Sürü durumu
        self.swarm_center: Tuple[float, float] = (0.0, 0.0)  # NED (N, E)
        self.altitude: float = 15.0

        # Manevra durumu
        self.pitch_angle: float = 0.0
        self.roll_angle: float = 0.0

        # Arka plan setpoint stream
        self._streaming: bool = False
        self._stream_task: Optional[asyncio.Task] = None

        # Failsafe olay kayıtları
        self.incident_log = IncidentLog()

    def _snapshot_state(self) -> Dict[str, object]:
        """Current swarm state snapshot for incident records."""
        return {
            "swarm_center": [round(self.swarm_center[0], 3), round(self.swarm_center[1], 3)],
            "altitude": round(self.altitude, 3),
            "active_agents": list(self.active_agents),
            "removed_agents": list(self.removed_agents),
            "formation_type": self.formation_type,
            "formation_distance": round(self.formation_distance, 3),
            "formation_heading_deg": round(math.degrees(self.formation_heading), 3),
        }

    def record_incident(
        self,
        code: str,
        action_taken: str,
        drone_id: Optional[int] = None,
        details: str = "",
    ) -> None:
        """Add a standardized incident entry and mirror it to logger output."""
        incident = self.incident_log.record(
            code=code,
            action_taken=action_taken,
            drone_id=drone_id,
            details=details,
            swarm_state=self._snapshot_state(),
        )
        log.warning(
            "[INCIDENT %s] action=%s drone=%s details=%s",
            incident.code,
            incident.action_taken,
            incident.drone_id,
            incident.details,
        )

    # ─── BAĞLANTI ─────────────────────────────────────────────

    async def connect(self, ports: List[str], grpc_base_port: int = 50040):
        """Tüm drone'lara bağlan."""
        for i, port in enumerate(ports):
            log.info(f"[Drone {i}] Bağlanıyor ({port})...")
            drone = System(port=grpc_base_port + i)
            await drone.connect(system_address=port)

            async for state in drone.core.connection_state():
                if state.is_connected:
                    log.info(f"[Drone {i}] Bağlandı!")
                    break

            self.drones[i] = drone
            self.active_agents.append(i)

        await self._read_home_positions()
        log.info(f"{len(self.drones)} drone bağlandı.")

    async def _read_home_positions(self):
        """Home offset'lerini hesapla. Önce config spawn, yoksa GPS."""
        if DRONE_SPAWNS_NED:
            for did in self.drones:
                n, e = DRONE_SPAWNS_NED.get(did, (0.0, 0.0))
                self.home_offsets[did] = (n, e)
                log.info(f"[Drone {did}] Home NED offset (config): N={n:.2f}m E={e:.2f}m")
            return

        # GPS fallback
        homes = {}
        for did, drone in self.drones.items():
            async for home in drone.telemetry.home():
                homes[did] = (home.latitude_deg, home.longitude_deg)
                log.info(
                    f"[Drone {did}] Home GPS: "
                    f"{home.latitude_deg:.7f}, {home.longitude_deg:.7f}"
                )
                break

        # Drone 0 referans noktası
        self.ref_home_gps = homes[0]
        ref_lat, ref_lon = self.ref_home_gps

        for did, (lat, lon) in homes.items():
            north = (lat - ref_lat) * 111320.0
            east = (lon - ref_lon) * 111320.0 * math.cos(math.radians(ref_lat))
            self.home_offsets[did] = (north, east)
            log.info(f"[Drone {did}] Home NED offset: N={north:.2f}m E={east:.2f}m")

    async def _update_home_offset(self, drone_id: int):
        """Tek bir drone'un home offset'ini yeniden hesapla (iniş sonrası)."""
        ref_lat, ref_lon = self.ref_home_gps
        async for home in self.drones[drone_id].telemetry.home():
            north = (home.latitude_deg - ref_lat) * 111320.0
            east = (
                (home.longitude_deg - ref_lon)
                * 111320.0
                * math.cos(math.radians(ref_lat))
            )
            self.home_offsets[drone_id] = (north, east)
            log.info(
                f"[Drone {drone_id}] Home offset güncellendi: "
                f"N={north:.2f}m E={east:.2f}m"
            )
            break

    # ─── POZİSYON HESAPLAMA ──────────────────────────────────

    def _get_global_targets(self) -> Dict[int, Tuple[float, float, float, float]]:
        """
        Aktif ajanlar için global NED hedef pozisyonları hesapla.
        Returns: {agent_id: (north, east, down, yaw_deg)}
        """
        body_offsets = get_formation_offsets(
            self.formation_type,
            len(self.active_agents),
            self.formation_distance,
            self.active_agents,
        )
        ned_offsets = rotate_offsets(body_offsets, self.formation_heading)

        pitch_alt = apply_pitch_offsets(body_offsets, self.pitch_angle)
        roll_alt = apply_roll_offsets(body_offsets, self.roll_angle)

        yaw_deg = math.degrees(self.formation_heading)
        targets = {}

        for aid in self.active_agents:
            n_off, e_off = ned_offsets[aid]
            alt_off = pitch_alt.get(aid, 0.0) + roll_alt.get(aid, 0.0)

            targets[aid] = (
                self.swarm_center[0] + n_off,
                self.swarm_center[1] + e_off,
                -(self.altitude - alt_off),  # NED down: negatif = yukarı
                yaw_deg,
            )

        return targets

    def _to_local_ned(
        self, drone_id: int, gn: float, ge: float, gd: float, yaw: float
    ) -> PositionNedYaw:
        """Global NED → drone'un lokal NED frame'ine çevir."""
        hn, he = self.home_offsets[drone_id]
        return PositionNedYaw(gn - hn, ge - he, gd, yaw)

    async def send_positions(self):
        """Aktif tüm drone'lara güncel formasyon pozisyonlarını gönder."""
        targets = self._get_global_targets()
        for aid, (n, e, d, yaw) in targets.items():
            try:
                local = self._to_local_ned(aid, n, e, d, yaw)
                await self.drones[aid].offboard.set_position_ned(local)
            except Exception as ex:
                log.debug(f"[Drone {aid}] setpoint hatası: {ex}")

    async def _stream_loop(self):
        """Arka planda 10Hz setpoint gönder (PX4 offboard timeout'u önler)."""
        while self._streaming:
            try:
                await self.send_positions()
            except Exception:
                pass
            await asyncio.sleep(0.1)

    def start_streaming(self):
        """Arka plan setpoint stream'i başlat."""
        if not self._streaming:
            self._streaming = True
            self._stream_task = asyncio.ensure_future(self._stream_loop())
            log.info("Setpoint streaming başlatıldı (10Hz)")

    async def stop_streaming(self):
        """Arka plan setpoint stream'i durdur."""
        self._streaming = False
        if self._stream_task:
            try:
                await self._stream_task
            except Exception:
                pass
            self._stream_task = None
            log.info("Setpoint streaming durduruldu")

    # ─── KALKIŞ / İNİŞ ───────────────────────────────────────

    async def takeoff(self, altitude: float = None):
        """Tüm drone'ları arm edip formasyon halinde kalkır."""
        if altitude:
            self.altitude = altitude

        # Sürü merkezi: home pozisyonlarının ortalaması
        if self.home_offsets:
            vals = list(self.home_offsets.values())
            self.swarm_center = (
                sum(v[0] for v in vals) / len(vals),
                sum(v[1] for v in vals) / len(vals),
            )

        log.info(
            f"KALKIŞ → İrtifa: {self.altitude}m, "
            f"Formasyon: {self.formation_type}, "
            f"Merkez: N={self.swarm_center[0]:.1f} E={self.swarm_center[1]:.1f}"
        )

        # Drone'ları sırayla başlat (paralel başlatma setpoint kaybına yol açıyor)
        for did in self.active_agents:
            drone = self.drones[did]
            initial = PositionNedYaw(0.0, 0.0, -self.altitude, 0.0)
            # Birden fazla setpoint gönder
            await drone.offboard.set_position_ned(initial)
            await asyncio.sleep(0.1)
            await drone.offboard.set_position_ned(initial)
            log.info(f"[Drone {did}] Arm...")
            await drone.action.arm()
            await drone.offboard.set_position_ned(initial)
            await asyncio.sleep(0.1)
            await drone.offboard.set_position_ned(initial)
            log.info(f"[Drone {did}] Offboard başlatılıyor...")
            await drone.offboard.start()
            log.info(f"[Drone {did}] Kalkış!")

        log.info("Tüm dronelar kalkışta! Yükselme bekleniyor...")

        # Hemen streaming başlat — PX4 sürekli setpoint bekliyor
        self.start_streaming()
        await asyncio.sleep(self.altitude / 2 + 3)

        # Formasyon pozisyonlarına geç
        await self.send_positions()
        log.info("Formasyon pozisyonları gönderildi.")
        await asyncio.sleep(8)
        log.info("Kalkış tamamlandı, formasyon hazır.")

    async def land_all(self):
        """Tüm aktif drone'ları indir ve disarm et."""
        log.info("Tüm dronelar iniyor...")

        # Streaming durdur
        await self.stop_streaming()

        for aid in list(self.active_agents):
            try:
                await self.drones[aid].offboard.stop()
            except (OffboardError, Exception):
                pass
            await self.drones[aid].action.land()
            log.info(f"[Drone {aid}] İniş komutu gönderildi.")

        await asyncio.sleep(12)

        for aid in list(self.active_agents):
            try:
                await self.drones[aid].action.disarm()
                log.info(f"[Drone {aid}] Disarm edildi.")
            except Exception:
                pass

        log.info("Tüm dronelar indi.")

    # ─── NAVİGASYON ──────────────────────────────────────────

    async def move_to(
        self, target_ne: Tuple[float, float], speed: float = 3.0
    ):
        """
        Sürü merkezini hedefe taşı.
        Formasyon rotasyonu + interpolasyonlu hareket.
        """
        start = self.swarm_center
        dist = math.hypot(target_ne[0] - start[0], target_ne[1] - start[1])

        if dist < 0.5:
            return

        # Formasyon rotasyonu: hedefe doğru heading
        new_heading = compute_heading(start, target_ne)
        await self._rotate_heading(new_heading)

        # İnterpolasyonlu hareket
        duration = dist / speed
        steps = max(int(duration * 4), 10)
        dt = duration / steps

        log.info(
            f"Hareket → ({target_ne[0]:.1f}, {target_ne[1]:.1f}), "
            f"mesafe: {dist:.1f}m, süre: {duration:.1f}s"
        )

        for i in range(1, steps + 1):
            t = i / steps
            self.swarm_center = (
                start[0] + t * (target_ne[0] - start[0]),
                start[1] + t * (target_ne[1] - start[1]),
            )
            await self.send_positions()
            await asyncio.sleep(dt)

        await asyncio.sleep(2)
        log.info(f"Hedefe varıldı: ({target_ne[0]:.1f}, {target_ne[1]:.1f})")

    async def _rotate_heading(self, target_heading: float, duration: float = 3.0):
        """Formasyon heading'ini yumuşak şekilde döndür."""
        start_heading = self.formation_heading
        diff = target_heading - start_heading

        # En kısa yoldan dön
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi

        if abs(diff) < 0.05:
            return

        steps = 20
        dt = duration / steps
        log.info(
            f"Formasyon rotasyonu: "
            f"{math.degrees(start_heading):.0f}° → "
            f"{math.degrees(target_heading):.0f}°"
        )

        for i in range(1, steps + 1):
            t = i / steps
            self.formation_heading = start_heading + diff * t
            await self.send_positions()
            await asyncio.sleep(dt)

        await asyncio.sleep(1)

    async def return_home(self, speed: float = 3.0):
        """Home konumuna dön ve iniş yap."""
        log.info("═══ EVE DÖNÜŞ ═══")
        await self.move_to(HOME_POSITION, speed)
        await self.land_all()

    # ─── FORMASYON ────────────────────────────────────────────

    async def change_formation(
        self, formation_type: str, distance: float = None
    ):
        """Formasyon tipini değiştir."""
        self.formation_type = formation_type
        if distance is not None:
            self.formation_distance = distance

        log.info(
            f"Formasyon değişikliği → {formation_type}, "
            f"mesafe: {self.formation_distance}m"
        )
        await self.send_positions()
        await asyncio.sleep(6)
        log.info("Formasyon değişikliği tamamlandı.")

    # ─── İRTİFA ───────────────────────────────────────────────

    async def change_altitude(self, new_altitude: float):
        """Sürü irtifasını değiştir (yumuşak geçiş)."""
        log.info(f"İrtifa değişikliği: {self.altitude:.0f}m → {new_altitude:.0f}m")


        start_alt = self.altitude
        steps = max(int(abs(new_altitude - start_alt) * 2), 10)

        for i in range(1, steps + 1):
            t = i / steps
            self.altitude = start_alt + t * (new_altitude - start_alt)
            await self.send_positions()
            await asyncio.sleep(0.25)

        await asyncio.sleep(3)
        log.info(f"İrtifa: {self.altitude:.0f}m")

    # ─── MANEVRALAR ───────────────────────────────────────────

    async def pitch_maneuver(self, angle_deg: float, hold: float = 5.0):
        """
        Pitch manevrası: sürü merkezi sabit, forward ekseninde eğim.
        Pozitif açı = öne eğilme (liderin irtifası düşer).
        """
        log.info(f"Pitch manevrası: {angle_deg}°")

        # Yavaşça açıya ulaş
        steps = 20
        for i in range(1, steps + 1):
            self.pitch_angle = angle_deg * (i / steps)
            await self.send_positions()
            await asyncio.sleep(0.15)

        await asyncio.sleep(hold)

        # Düze dön
        for i in range(steps, -1, -1):
            self.pitch_angle = angle_deg * (i / steps)
            await self.send_positions()
            await asyncio.sleep(0.15)

        self.pitch_angle = 0.0
        log.info("Pitch manevrası tamamlandı.")

    async def roll_maneuver(self, angle_deg: float, hold: float = 5.0):
        """
        Roll manevrası: sürü merkezi sabit, sağ/sol ekseninde eğim.
        Pozitif açı = sağa yatış (sağ kanat alçalır).
        """
        log.info(f"Roll manevrası: {angle_deg}°")

        steps = 20
        for i in range(1, steps + 1):
            self.roll_angle = angle_deg * (i / steps)
            await self.send_positions()
            await asyncio.sleep(0.15)

        await asyncio.sleep(hold)

        for i in range(steps, -1, -1):
            self.roll_angle = angle_deg * (i / steps)
            await self.send_positions()
            await asyncio.sleep(0.15)

        self.roll_angle = 0.0
        log.info("Roll manevrası tamamlandı.")

    # ─── AJAN EKLEME / ÇIKARMA ────────────────────────────────

    async def remove_agent(
        self, agent_id: int, landing_ne: Tuple[float, float],
        cameras=None, target_color: str = None
    ):
        """
        Ajanı sürüden çıkar ve belirtilen renkli bölgeye indir.
        cameras + target_color verilirse iniş öncesi kamera ile
        renkli alanın merkezine hizalanır.
        """
        if agent_id not in self.active_agents:
            log.warning(f"[Drone {agent_id}] Aktif değil, çıkarılamaz!")
            return

        log.info(
            f"[Drone {agent_id}] Sürüden çıkarılıyor → "
            f"İniş: ({landing_ne[0]:.1f}, {landing_ne[1]:.1f})"
        )

        # Aktif listeden çıkar
        self.active_agents.remove(agent_id)
        self.removed_agents.append(agent_id)

        # İniş NED koordinatını kaydet (add_agent'ta kullanılacak)
        self._landing_positions[agent_id] = landing_ne

        # Çıkan drone'u iniş bölgesinin üzerine yönlendir
        drone = self.drones[agent_id]
        hn, he = self.home_offsets[agent_id]

        cur_n = landing_ne[0] - hn
        cur_e = landing_ne[1] - he

        landing_cmd = PositionNedYaw(
            cur_n, cur_e, -self.altitude, 0.0
        )
        log.info(
            f"[Drone {agent_id}] Home offset: N={hn:.2f} E={he:.2f}, "
            f"İniş NED: ({landing_ne[0]:.1f}, {landing_ne[1]:.1f}), "
            f"Local cmd: N={cur_n:.1f} E={cur_e:.1f} D={-self.altitude:.0f}"
        )

        # 1) İniş bölgesine git — SÜREKLİ setpoint gönder (10Hz, 15s)
        log.info(f"[Drone {agent_id}] İniş bölgesine gidiyor...")
        for _ in range(150):
            await drone.offboard.set_position_ned(landing_cmd)
            await asyncio.sleep(0.1)

        # 2) Kamera ile renkli alana hizalan (ampirik kalibrasyon)
        if cameras is not None and target_color is not None:
            from src.detection import detect_color_offset
            import math

            log.info(f"[Drone {agent_id}] 📷 Renkli alan hizalama: {target_color}")

            # ── Kalibrasyon: drone'u küçük adımlarla hareket ettirip ──
            # ── piksel değişimini ölçerek eksen eşlemesini belirle   ──
            PROBE = 0.5  # metre (daha büyük adım = daha güvenilir Jacobian)
            SETTLE = 25  # 2.5 saniye (drone'un pozisyona oturması için)

            async def _get_fresh_offset():
                """Taze frame al ve offset ölç."""
                # Eski buffer'ı temizle
                for _ in range(5):
                    cameras.get_frame(agent_id)
                    await asyncio.sleep(0.05)
                f = cameras.get_frame(agent_id)
                if f is None:
                    return None
                return detect_color_offset(f, target_color)

            async def _go_and_measure(n, e):
                """Pozisyona git, bekle, ölç."""
                cmd = PositionNedYaw(n, e, -self.altitude, 0.0)
                for _ in range(SETTLE):
                    await drone.offboard.set_position_ned(cmd)
                    await asyncio.sleep(0.1)
                return await _get_fresh_offset()

            log.info(f"[Drone {agent_id}] 📐 Kalibrasyon başlıyor...")

            off0 = await _go_and_measure(cur_n, cur_e)
            off_n = await _go_and_measure(cur_n + PROBE, cur_e) if off0 else None
            off_e = await _go_and_measure(cur_n, cur_e + PROBE) if off0 else None

            # Başlangıca dön
            back_cmd = PositionNedYaw(cur_n, cur_e, -self.altitude, 0.0)
            for _ in range(SETTLE):
                await drone.offboard.set_position_ned(back_cmd)
                await asyncio.sleep(0.1)

            calib_ok = False
            inv00 = inv01 = inv10 = inv11 = 0.0

            if off0 and off_n and off_e:
                # Jacobian: J * [dn, de] = [d_raw0, d_raw1]
                j00 = (off_n[0] - off0[0]) / PROBE
                j10 = (off_n[1] - off0[1]) / PROBE
                j01 = (off_e[0] - off0[0]) / PROBE
                j11 = (off_e[1] - off0[1]) / PROBE

                det = j00 * j11 - j01 * j10
                log.info(
                    f"[Drone {agent_id}] 📐 J=[{j00:.3f} {j01:.3f}; "
                    f"{j10:.3f} {j11:.3f}], det={det:.4f}"
                )

                if abs(det) > 0.005:
                    inv00 = j11 / det
                    inv01 = -j01 / det
                    inv10 = -j10 / det
                    inv11 = j00 / det
                    calib_ok = True
                    log.info(
                        f"[Drone {agent_id}] 📐 J_inv=[{inv00:.2f} {inv01:.2f}; "
                        f"{inv10:.2f} {inv11:.2f}] — Kalibrasyon OK"
                    )
                else:
                    log.warning(f"[Drone {agent_id}] ⚠ Jacobian dejenere (det={det:.4f})")
            else:
                log.warning(f"[Drone {agent_id}] ⚠ Kalibrasyon başarısız (renk algılanamadı)")

            # ── Hizalama döngüsü ──
            if calib_ok:
                MAX_ITERS = 30
                MAX_DRIFT = 3.0
                gain = 0.5
                initial_n, initial_e = cur_n, cur_e

                for attempt in range(MAX_ITERS):
                    # Pozisyon komutu gönder ve drone'un ulaşmasını bekle (1.5s)
                    for _ in range(15):
                        await drone.offboard.set_position_ned(landing_cmd)
                        await asyncio.sleep(0.1)

                    # Taze frame al (birkaç frame atla — eski buffer'ı temizle)
                    for _ in range(5):
                        cameras.get_frame(agent_id)
                        await asyncio.sleep(0.05)
                    frame = cameras.get_frame(agent_id)

                    if frame is None:
                        continue

                    offset = detect_color_offset(
                        frame, target_color,
                        save_debug=True,
                        debug_tag=f"d{agent_id}_i{attempt}"
                    )
                    if offset is None:
                        log.info(f"[Drone {agent_id}] 📷 #{attempt}: Renk görünmüyor")
                        continue

                    r0, r1 = offset

                    err = math.hypot(r0, r1)
                    if err < 0.03:
                        log.info(
                            f"[Drone {agent_id}] ✅ Hizalandı! "
                            f"raw=({r0:.3f},{r1:.3f}) err={err:.3f} — iter #{attempt}"
                        )
                        break

                    # Jacobian ile düzeltme
                    dn = -(inv00 * r0 + inv01 * r1) * gain
                    de = -(inv10 * r0 + inv11 * r1) * gain

                    new_n = cur_n + dn
                    new_e = cur_e + de
                    drift = math.hypot(new_n - initial_n, new_e - initial_e)

                    if drift > MAX_DRIFT:
                        log.warning(f"[Drone {agent_id}] ⚠ Drift sınırı ({drift:.1f}m)")
                        break

                    cur_n = new_n
                    cur_e = new_e
                    landing_cmd = PositionNedYaw(cur_n, cur_e, -self.altitude, 0.0)

                    log.info(
                        f"[Drone {agent_id}] 📷 #{attempt}: "
                        f"raw=({r0:.3f},{r1:.3f}) err={err:.3f} Δ=({dn:.2f},{de:.2f}) "
                        f"pos=({cur_n:.1f},{cur_e:.1f}) drift={drift:.2f}m"
                    )
            else:
                log.info(f"[Drone {agent_id}] Config pozisyonunda iniş (kalibrasyon başarısız)")

            # Stabilizasyon: 3s
            log.info(f"[Drone {agent_id}] Stabilizasyon (3s)...")
            for _ in range(30):
                await drone.offboard.set_position_ned(landing_cmd)
                await asyncio.sleep(0.1)

            self._landing_positions[agent_id] = (cur_n + hn, cur_e + he)

        # 3) Offboard durdur ve iniş komutu ver
        log.info(f"[Drone {agent_id}] İniş başlıyor...")
        try:
            await drone.offboard.stop()
        except (OffboardError, Exception):
            pass

        await drone.action.land()
        log.info(f"[Drone {agent_id}] İniş komutu gönderildi, yere inmesi bekleniyor...")

        # 4) Telemetriden yere indiğini doğrula
        async for state in drone.telemetry.landed_state():
            if state == LandedState.ON_GROUND:
                break
        log.info(f"[Drone {agent_id}] Yere indi (telemetri onaylandı).")

        # 5) Disarm
        try:
            await drone.action.disarm()
        except Exception:
            pass
        log.info(f"[Drone {agent_id}] Disarm oldu.")
    async def add_agent(self, agent_id: int):
        """
        Yerdeki ajanı tekrar sürüye ekle.
        PX4 EKF local frame disarm/re-arm sonrası SIFIRLANMIYOR.
        home_offsets DEĞİŞMEZ — orijinal değerler korunur.
        Kalkış komutu iniş bölgesinin local NED karşılığıyla gönderilir.
        """
        if agent_id not in self.removed_agents:
            log.warning(f"[Drone {agent_id}] Çıkarılmış listesinde değil!")
            return

        log.info(f"[Drone {agent_id}] Sürüye geri ekleniyor...")

        drone = self.drones[agent_id]

        # home_offsets DEĞİŞMİYOR — PX4 EKF origin aynı kalıyor
        # İniş bölgesi NED → drone'un local NED frame'inde hesapla
        landing_ne = self._landing_positions[agent_id]
        hn, he = self.home_offsets[agent_id]  # ORİJİNAL offset
        local_land_n = landing_ne[0] - hn
        local_land_e = landing_ne[1] - he
        log.info(
            f"[Drone {agent_id}] Orijinal home offset: N={hn:.2f} E={he:.2f}, "
            f"İniş bölgesi NED: ({landing_ne[0]:.1f}, {landing_ne[1]:.1f}), "
            f"Local: ({local_land_n:.1f}, {local_land_e:.1f})"
        )

        # Arm
        log.info(f"[Drone {agent_id}] Arm...")
        await drone.action.arm()
        await asyncio.sleep(2)

        # Offboard başlat — iniş bölgesi ÜSTÜNDE yüksel (orijinal local NED frame)
        climb_cmd = PositionNedYaw(local_land_n, local_land_e, -self.altitude, 0.0)
        await drone.offboard.set_position_ned(climb_cmd)
        await asyncio.sleep(0.1)
        await drone.offboard.set_position_ned(climb_cmd)
        log.info(f"[Drone {agent_id}] Offboard başlatılıyor...")
        await drone.offboard.start()

        # Yükselme — SÜREKLİ setpoint gönder
        log.info(f"[Drone {agent_id}] Kalkış... ({self.altitude}m)")
        target_alt = self.altitude * 0.85
        rel_alt = 0.0
        for _ in range(200):  # max ~60s
            await drone.offboard.set_position_ned(climb_cmd)
            async for pos in drone.telemetry.position():
                rel_alt = pos.relative_altitude_m
                break
            if rel_alt >= target_alt:
                break
            await asyncio.sleep(0.3)
        log.info(f"[Drone {agent_id}] İrtifaya ulaştı ({rel_alt:.1f}m).")

        # Aktif listeye ekle
        self.removed_agents.remove(agent_id)
        self.active_agents.append(agent_id)
        self.active_agents.sort()
        log.info(f"[Drone {agent_id}] Aktif listeye eklendi, formasyon pozisyonuna gidiyor...")

        # Formasyon pozisyonuna açıkça yönlendir
        for i in range(150):  # 15 saniye, 10Hz
            targets = self._get_global_targets()
            if agent_id in targets:
                n, e, d, yaw = targets[agent_id]
                local = self._to_local_ned(agent_id, n, e, d, yaw)
                await drone.offboard.set_position_ned(local)
                if i == 0:
                    log.info(
                        f"[Drone {agent_id}] Hedef: "
                        f"N={n:.1f} E={e:.1f} D={d:.1f} → "
                        f"local=({local.north_m:.1f}, {local.east_m:.1f})"
                    )
            await asyncio.sleep(0.1)

        log.info(f"[Drone {agent_id}] Sürüye geri katıldı.")

    # ─── SÜREKLI HAREKET (Görev 5.2) ─────────────────────────

    def apply_velocity(self, pitch: float, roll: float, yaw: float, throttle: float, dt: float):
        """
        Input stick değerlerini sürü merkezine uygula (Sürü Hareket Modu).
        Formasyon heading'e göre ileri/geri/sağ/sol hareket.

        Args:
            pitch: -1..+1 (ileri/geri — heading yönünde)
            roll: -1..+1 (sağ/sol — heading'e dik)
            yaw: -1..+1 (sürü merkezi sabit, formasyon rotasyonu)
            throttle: -1..+1 (irtifa)
            dt: frame süresi (saniye)
        """
        MOVE_SPEED = 5.0    # m/s max
        ALT_SPEED = 3.0     # m/s max
        YAW_SPEED = 45.0    # derece/s max

        # İleri/geri (heading yönünde)
        if abs(pitch) > 0.01:
            speed = pitch * MOVE_SPEED * dt
            self.swarm_center = (
                self.swarm_center[0] + speed * math.cos(self.formation_heading),
                self.swarm_center[1] + speed * math.sin(self.formation_heading),
            )

        # Sağ/sol (heading'e dik)
        if abs(roll) > 0.01:
            speed = roll * MOVE_SPEED * dt
            self.swarm_center = (
                self.swarm_center[0] + speed * math.cos(self.formation_heading + math.pi / 2),
                self.swarm_center[1] + speed * math.sin(self.formation_heading + math.pi / 2),
            )

        # Yaw → formasyon rotasyonu (sürü merkezi sabit)
        if abs(yaw) > 0.01:
            self.formation_heading += math.radians(yaw * YAW_SPEED * dt)

        # Throttle → irtifa
        if abs(throttle) > 0.01:
            self.altitude += throttle * ALT_SPEED * dt
            self.altitude = max(3.0, min(50.0, self.altitude))

    def apply_tilt(self, pitch: float, roll: float, yaw: float, throttle: float, dt: float):
        """
        Input stick değerlerini formasyon eğimine uygula (Manevra Modu).
        Sürü merkezi sabit kalır, formasyona pitch/roll/yaw eğimi verilir.

        Args:
            pitch: -1..+1 (formasyon pitch eğimi)
            roll: -1..+1 (formasyon roll eğimi)
            yaw: -1..+1 (formasyon rotasyonu)
            throttle: -1..+1 (irtifa)
            dt: frame süresi (saniye)
        """
        MAX_TILT = 25.0     # derece max eğim
        TILT_SPEED = 30.0   # derece/s
        YAW_SPEED = 45.0    # derece/s
        ALT_SPEED = 3.0     # m/s

        # Pitch eğim
        if abs(pitch) > 0.01:
            self.pitch_angle += pitch * TILT_SPEED * dt
            self.pitch_angle = max(-MAX_TILT, min(MAX_TILT, self.pitch_angle))
        else:
            # Stick bırakıldığında yavaşça düze dön
            self.pitch_angle *= max(0.0, 1.0 - 3.0 * dt)
            if abs(self.pitch_angle) < 0.5:
                self.pitch_angle = 0.0

        # Roll eğim
        if abs(roll) > 0.01:
            self.roll_angle += roll * TILT_SPEED * dt
            self.roll_angle = max(-MAX_TILT, min(MAX_TILT, self.roll_angle))
        else:
            self.roll_angle *= max(0.0, 1.0 - 3.0 * dt)
            if abs(self.roll_angle) < 0.5:
                self.roll_angle = 0.0

        # Yaw → formasyon rotasyonu
        if abs(yaw) > 0.01:
            self.formation_heading += math.radians(yaw * YAW_SPEED * dt)

        # Throttle → irtifa
        if abs(throttle) > 0.01:
            self.altitude += throttle * ALT_SPEED * dt
            self.altitude = max(3.0, min(50.0, self.altitude))

    # ─── YARDIMCI ─────────────────────────────────────────────

    async def hold(self, seconds: float):
        """Mevcut pozisyonda bekle."""
        log.info(f"Pozisyon tutma: {seconds:.0f}s")
        await asyncio.sleep(seconds)
