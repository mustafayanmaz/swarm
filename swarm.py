import asyncio
import logging
import sys
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

# Log ayarları
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("swarm")

DRONE_PORTS = [
    "udp://:14540",  # Drone 1 (instance 0)
    "udp://:14541",  # Drone 2 (instance 1)
    "udp://:14542",  # Drone 3 (instance 2)
]

# Offboard hedef pozisyonlar (NED koordinat)
# Drone 1: orijinde kalır, 5m yükselir
# Drone 2: 3m sağa, 5m yükselir
# Drone 3: 6m sağa, 5m yükselir
TAKEOFF_POSITIONS = [
    PositionNedYaw(0.0, 0.0, -5.0, 0.0),
    PositionNedYaw(3.0, 0.0, -5.0, 0.0),
    PositionNedYaw(6.0, 0.0, -5.0, 0.0),
]


async def connect_drone(port, drone_id):
    """Drone'a baglan ve durumunu kontrol et."""
    log.info(f"[Drone {drone_id}] Baglaniyor ({port})...")

    drone = System(port=50040 + drone_id)
    await drone.connect(system_address=port)

    # Baglantiyi bekle (15 saniye timeout)
    try:
        async for state in drone.core.connection_state():
            if state.is_connected:
                log.info(f"[Drone {drone_id}] Baglandi!")
                break
    except asyncio.TimeoutError:
        log.error(f"[Drone {drone_id}] Baglanti timeout!")
        return None

    # Drone durumunu oku
    try:
        async for health in drone.telemetry.health():
            log.info(f"[Drone {drone_id}] Gyro calisma: {health.is_gyrometer_calibration_ok}")
            log.info(f"[Drone {drone_id}] Accel calisma: {health.is_accelerometer_calibration_ok}")
            log.info(f"[Drone {drone_id}] Magnetometer: {health.is_magnetometer_calibration_ok}")
            log.info(f"[Drone {drone_id}] Global pozisyon: {health.is_global_position_ok}")
            log.info(f"[Drone {drone_id}] Home pozisyon: {health.is_home_position_ok}")
            log.info(f"[Drone {drone_id}] Arm olabiliyor mu: {health.is_armable}")
            break
    except Exception as e:
        log.warning(f"[Drone {drone_id}] Telemetri okunamadi: {e}")

    return drone


async def fly_drone(drone, drone_id, target_pos):
    """Tek bir drone'u offboard modunda uçur."""
    log.info(f"[Drone {drone_id}] Arm ediliyor...")

    try:
        await drone.action.arm()
        log.info(f"[Drone {drone_id}] Arm basarili!")
    except Exception as e:
        log.error(f"[Drone {drone_id}] Arm basarisiz: {e}")
        return

    # Offboard icin ilk pozisyonu set et (baslatmadan once zorunlu)
    log.info(f"[Drone {drone_id}] Offboard hedef: N={target_pos.north_m}, E={target_pos.east_m}, D={target_pos.down_m}")
    await drone.offboard.set_position_ned(target_pos)

    # Offboard'u baslat
    log.info(f"[Drone {drone_id}] Offboard baslatiliyor...")
    try:
        await drone.offboard.start()
        log.info(f"[Drone {drone_id}] Offboard aktif! Hedefe gidiliyor...")
    except OffboardError as e:
        log.error(f"[Drone {drone_id}] Offboard baslatma hatasi: {e}")
        log.info(f"[Drone {drone_id}] Disarm ediliyor...")
        await drone.action.disarm()
        return

    # Hedefte bekle
    await asyncio.sleep(8)

    # Pozisyon takibi
    try:
        async for position in drone.telemetry.position():
            log.info(
                f"[Drone {drone_id}] Pozisyon: lat={position.latitude_deg:.6f}, "
                f"lon={position.longitude_deg:.6f}, rel_alt={position.relative_altitude_m:.1f}m"
            )
            break
    except Exception:
        pass

    # Offboard durdur
    log.info(f"[Drone {drone_id}] Offboard durduruluyor...")
    try:
        await drone.offboard.stop()
        log.info(f"[Drone {drone_id}] Offboard durduruldu.")
    except OffboardError as e:
        log.error(f"[Drone {drone_id}] Offboard durdurma hatasi: {e}")

    # İniş
    log.info(f"[Drone {drone_id}] Inis baslatiliyor...")
    await drone.action.land()
    log.info(f"[Drone {drone_id}] Inis komutu gonderildi.")


async def run():
    log.info("=" * 50)
    log.info("SURU IHA - Simulasyon Baslatiliyor")
    log.info(f"Drone sayisi: {len(DRONE_PORTS)}")
    log.info("=" * 50)

    # 1) Tum dronelara baglan
    drones = []
    for i, port in enumerate(DRONE_PORTS):
        drone = await connect_drone(port, i)
        if drone is None:
            log.error(f"[Drone {i}] Baglanamadi, atlaniliyor!")
            continue
        drones.append((drone, i))

    if len(drones) == 0:
        log.error("Hicbir drone'a baglanilamadi! Simulasyon calisiyor mu?")
        log.error("Kontrol: ./launch_sim.sh calistirildi mi?")
        sys.exit(1)

    log.info(f"{len(drones)} drone baglandi. 3 saniye bekleniyor...")
    await asyncio.sleep(3)

    # 2) Hepsini ayni anda ucur
    log.info("Tum droneler kalkisa geciriliyor!")
    await asyncio.gather(
        *[fly_drone(drone, drone_id, TAKEOFF_POSITIONS[drone_id]) for drone, drone_id in drones]
    )

    log.info("Gorev tamamlandi!")


if __name__ == "__main__":
    asyncio.run(run())
