"""
Görev 1: Dinamik Sürü Kabiliyeti — Panda3D 3D Simülasyon
Gerçek 3D sahne üzerinde sürü uçuşu.

Kontroller:
  Orta tık sürükle = Kamerayı döndür
  Sağ tık sürükle   = Kamerayı kaydır (pan)
  Scroll            = Yakınlaştır / Uzaklaştır
  ESC               = Çıkış
"""
import os
import sys
import yaml
import numpy as np

from direct.task import Task

from src.formation import FormationType, FORMATION_NAME_MAP
from src.sim_controller import SwarmSim
from src.mission_runner import MissionRunner
from src.qr_system import QRManager
from src.renderer_3d import Renderer3D

# ── Config ──────────────────────────────────────────────────
config_path = os.path.join(os.path.dirname(__file__), 'config', 'mission.yaml')
with open(config_path) as f:
    config = yaml.safe_load(f)

NUM_DRONES = config.get('num_drones', 4)
SPACING = config.get('initial_spacing', 3.0)

# ── Sürü + Görev ────────────────────────────────────────────
swarm = SwarmSim(NUM_DRONES)
swarm.set_formation(
    FORMATION_NAME_MAP.get(config.get('initial_formation', 'line'), FormationType.LINE),
    SPACING,
)

# Başlangıç pozisyonları — Home yakınında LINE
home_pos = np.array(config.get('home_position', [0, 0, 0]), dtype=float)
for i in range(NUM_DRONES):
    y_off = i * SPACING - (NUM_DRONES - 1) * SPACING / 2
    swarm.positions[i] = np.array([home_pos[0], home_pos[1] + y_off, 0.0])

# 3 drone — yedek yok, hepsi aktif

# Centroid
active_positions = [swarm.positions[d] for d in swarm.active_drones]
swarm.centroid = np.mean(active_positions, axis=0).copy()

# QR Manager
qr_manager = QRManager(config.get('qr_codes', []), team_id=config.get('team_id', 1))
print(f'  {len(qr_manager.qr_codes)} adet gerçek QR code üretildi')

mission = MissionRunner(
    swarm,
    config.get('qr_codes', []),
    config.get('color_zones', []),
    config,
    qr_manager=qr_manager,
)

# ── Sahne verileri ──────────────────────────────────────────
qr_codes = config.get('qr_codes', [])
color_zones = config.get('color_zones', [])
all_qr_positions = [qr['position'][:2] for qr in qr_codes]

# Rota sırası
route_order = []
current_qr_id = config.get('first_qr', 1)
visited = set()
qr_lookup = {qr['id']: qr for qr in qr_codes}
team_key = f'team_{config.get("team_id", 1)}'
while current_qr_id and current_qr_id not in visited:
    visited.add(current_qr_id)
    qr = qr_lookup.get(current_qr_id)
    if qr is None:
        break
    route_order.append(qr['position'][:2])
    sonraki = qr.get('sonraki_qr', {})
    current_qr_id = sonraki.get(team_key, 0)
full_route = [home_pos[:2].tolist()] + route_order + [home_pos[:2].tolist()]

# ── 3D Renderer ─────────────────────────────────────────────
app = Renderer3D(1280, 720)

# Sahne oluştur (bir kez)
app.create_drones(NUM_DRONES)
app.create_qr_markers(qr_codes, qr_manager=qr_manager)
app.create_home(home_pos[:2])
app.create_color_zones(color_zones)
app.draw_graph_lines(all_qr_positions)
app.draw_route_lines(full_route)

# Drone kamera sistemi — her drone'da ayrı kamera
app.setup_drone_cameras(NUM_DRONES, buf_size=192)

# QR pozisyonları ve renk alanları — kamera tespit mesafesi
QR_DETECT_RANGE = 3.0   # QR tespit mesafesi (120cm QR, 90° FOV)
ZONE_DETECT_RANGE = 5.0  # Renk alanı tespit mesafesi
qr_positions_np = {qr['id']: np.array(qr['position'][:2], dtype=float) for qr in qr_codes}
zone_positions = []
for z in color_zones:
    zone_positions.append({
        'color': z['color'],
        'pos': np.array(z['position'][:2], dtype=float),
        'radius': z.get('radius', 1.5),
    })

# İlk tespit eden drone takip sistemi
_first_detect_qr = {}  # qr_id -> drone_id (ilk tespit eden)
_detected_zones = {}   # color -> {'pos': np.array, 'detected_by': drone_id, 'time': sim_time}

# Kamerayı sahne merkezine ayarla
if all_qr_positions:
    cx = np.mean([p[0] for p in all_qr_positions])
    cy = np.mean([p[1] for p in all_qr_positions])
    from panda3d.core import Vec3
    app._cam_target = Vec3(cy, cx, 0)
    app._update_camera()

print('=' * 60)
print('  TEKNOFEST Sürü İHA — Görev 1: Dinamik Sürü (Panda3D 3D)')
print(f'  Drone: {NUM_DRONES} | Formasyon: {config.get("initial_formation", "line")}')
print('  Orta tık = döndür | Sağ tık = kaydır | Scroll = zoom | ESC = çıkış')
print('  G = GCS bağlantı kes/bağla (failsafe test)')
print('=' * 60)

sim_time = 0.0
# Failsafe — GCS bağlantı durumu
gcs_connected = True

def toggle_gcs():
    global gcs_connected
    gcs_connected = not gcs_connected
    status = 'BAĞLI' if gcs_connected else 'KESİLDİ'
    print(f'[FAILSAFE] Yer İstasyonu bağlantısı: {status}')
    if not gcs_connected:
        print('[FAILSAFE] Sürü otonom devam ediyor — müdahale mümkün değil')
    app.set_gcs_status(gcs_connected)

app.accept('g', toggle_gcs)
# Statik çizgiler zaten oluşturuldu, bunları ayır
_static_line_nodes = list(app._line_nodes)
app._line_nodes = []

# ── Start butonu — görev başlatma ────────────────────────────
mission_started = False

def start_mission(altitude=None):
    global mission_started
    if mission_started:
        return
    if altitude is not None:
        config['initial_altitude'] = altitude
    mission_started = True
    print(f'[GÖREV] Başlatıldı! İrtifa: {config.get("initial_altitude", 1.0)}m')

def restart_mission():
    """Görevi tamamen sıfırla — başlangıç ekranı tekrar gelir."""
    global sim_time, mission_started, gcs_connected
    global _first_detect_qr, _detected_zones, _last_logged_qr

    print('\n' + '=' * 60)
    print('  [GÖREV] SIFIRLANIYOR...')
    print('=' * 60)

    # Görev henüz başlamadı durumuna al
    mission_started = False
    sim_time = 0.0

    # Swarm pozisyonlarını başlangıca döndür
    for i in range(NUM_DRONES):
        y_off = i * SPACING - (NUM_DRONES - 1) * SPACING / 2
        swarm.positions[i] = np.array([home_pos[0], home_pos[1] + y_off, 0.0])
    swarm.active_drones = list(range(NUM_DRONES))
    swarm.target_altitude = config.get('initial_altitude', 1.0)
    swarm.heading = 0.0
    swarm.pitch_angle = 0.0
    swarm.roll_angle = 0.0
    swarm._taking_off = False
    swarm._landing = False
    swarm._nav_target = None
    swarm.centroid = np.mean([swarm.positions[d] for d in swarm.active_drones], axis=0).copy()
    swarm.set_formation(
        FORMATION_NAME_MAP.get(config.get('initial_formation', 'line'), FormationType.LINE),
        SPACING,
    )

    # Mission runner sıfırla
    mission.reset()

    # Tespit hafızasını sıfırla
    _first_detect_qr = {}
    _detected_zones = {}
    _last_logged_qr = set()

    # GCS bağlantısını resetle
    gcs_connected = True
    app.set_gcs_status(True)

    # Per-drone kamera tespitlerini sıfırla
    for i in range(NUM_DRONES):
        app.set_per_drone_detection(i, None)

    print('[GÖREV] Sıfırlandı — başlatma bekleniyor')

app.show_start_screen(start_mission, initial_altitude=config.get('initial_altitude', 1.0))
app.set_restart_callback(restart_mission)

# QR sonuçlarını takip (panele eklemek için)
_last_logged_qr = set()


def sim_update_task(task):
    global sim_time

    if app.should_quit():
        sys.exit(0)

    dt = globalClock.getDt()
    dt = min(dt, 0.05)  # Cap

    # Görev başlatılmadıysa sadece 3D render (drone'lar yerde)
    if not mission_started:
        for i in range(NUM_DRONES):
            dm = app.drone_models[i]
            p = swarm.positions[i]
            dm.set_position(p[0], p[1], p[2])
            dm.set_heading(swarm.heading)
            dm.set_armed(False)
            dm.update_props(dt)
        app.update_hud(
            state='BAŞLATMA BEKLENİYOR',
            formation_name=swarm.formation_type.value,
            spacing=swarm.spacing,
            num_active=len(swarm.active_drones),
            num_total=NUM_DRONES,
            heading_deg=np.degrees(swarm.heading),
            altitude=0.0,
            sim_time=0.0,
            arm_states=[False] * NUM_DRONES,
        )
        return Task.cont

    # Duraklatma kontrolü
    if app.is_mission_paused():
        return Task.cont

    # Görev güncelle
    state = mission.update(dt)
    targets = swarm.update(dt)

    # Drone pozisyonlarını hedefe doğru hareket ettir (basit lerp)
    # Ayrılan drone hariç — o kendi state machine ile hareket eder
    sep_id = mission.separated_drone_id
    for i in range(NUM_DRONES):
        if i == sep_id:
            continue  # Ayrılan drone'u genel lerp'ten hariç tut
        target = targets[i]
        cur = swarm.positions[i]
        diff = target - cur
        dist = np.linalg.norm(diff)
        speed = 6.0
        if dist > 0.01:
            step = min(speed * dt, dist)
            swarm.positions[i] = cur + (diff / dist) * step

    # Ayrılan drone'u hedefine hareket ettir (paralel — sürü durmaz)
    if mission._removed_drone and mission._sep_state in ('FLY_TO_ZONE', 'FLY_BACK'):
        drone_id = mission._removed_drone['id']
        if mission._sep_state == 'FLY_TO_ZONE' and mission._removed_drone_target is not None:
            target_xy = mission._removed_drone_target
        elif mission._sep_state == 'FLY_BACK':
            target_xy = swarm.centroid[:2]
        else:
            target_xy = None

        if target_xy is not None:
            cur = swarm.positions[drone_id][:2]
            diff = target_xy - cur
            dist = np.linalg.norm(diff)
            sep_speed = 5.0  # Yetişmek için biraz daha hızlı
            if dist > 0.01:
                step = min(sep_speed * dt, dist)
                swarm.positions[drone_id][0] += (diff[0] / dist) * step
                swarm.positions[drone_id][1] += (diff[1] / dist) * step

    sim_time += dt

    # ── 3D Güncelle ──────────────────────────────────────────

    # Dinamik çizgileri temizle (formasyon çizgileri)
    for node in app._line_nodes:
        node.removeNode()
    app._line_nodes = []

    # Drone'ları güncelle
    active_pos_3d = []
    for i in range(NUM_DRONES):
        dm = app.drone_models[i]
        p = swarm.positions[i]
        dm.set_position(p[0], p[1], p[2])
        dm.set_heading(swarm.heading)

        # Arm/disarm visual
        dm.set_armed(mission.armed[i])

        active = i in swarm.active_drones
        is_separated = (i == mission.separated_drone_id)

        if is_separated:
            dm.set_active(True)
            dm.set_separated(True, mission.separated_drone_color)
        elif active:
            dm.set_active(True)
            dm.set_separated(False)
            active_pos_3d.append(p.copy())
        else:
            dm.set_active(False)

        dm.update_props(dt)

    # Formasyon çizgileri
    app.draw_formation_lines(active_pos_3d)

    # Kamera sürüyü takip
    app.follow_swarm(swarm.centroid)

    # ── Drone Kameraları Güncelle (per-drone) ─────────────────────
    # Her drone'un kamerasını pozisyonuna yerleştir
    for i in range(NUM_DRONES):
        p = swarm.positions[i]
        app.update_drone_camera(i, p, swarm.heading, swarm.target_altitude)

    # Per-drone QR/renk tespit
    active_cam_detection = None
    active_cam_label = ''
    active_cam_id = app._active_cam_id if hasattr(app, '_active_cam_id') else 0

    for i in range(NUM_DRONES):
        drone_pos_xy = swarm.positions[i][:2]
        det = None

        # QR tarama aktifken (mission state)
        if mission.scanning_qr_id is not None:
            # Centroid'e en yakın drone taramayı yapar
            centroid_xy = swarm.centroid[:2]
            if np.linalg.norm(drone_pos_xy - centroid_xy) < 2.0:
                det = 'scanning'

        if det is None:
            # Her drone kendi konumundan QR tespit
            for qid, qpos in qr_positions_np.items():
                dist = np.linalg.norm(drone_pos_xy - qpos)
                if dist < QR_DETECT_RANGE:
                    det = 'qr'
                    # İlk tespit eden haber veriyor
                    if qid not in _first_detect_qr:
                        _first_detect_qr[qid] = i
                        app.add_notify_log(
                            sim_time, i,
                            f'QR{qid} tespit ettim! Suruye bildiriyorum.'
                        )
                        print(f'[KAMERA] D{i} ilk QR{qid} tespit etti > diger dronelara haber verildi')
                    break

        if det is None:
            # Renk alanı tespiti
            for zone in zone_positions:
                dist = np.linalg.norm(drone_pos_xy - zone['pos'])
                if dist < ZONE_DETECT_RANGE:
                    det = 'color_red' if zone['color'] == 'red' else 'color_blue'
                    # İlk tespit → koordinatları hafızaya kaydet
                    if zone['color'] not in _detected_zones:
                        _detected_zones[zone['color']] = {
                            'pos': zone['pos'].copy(),
                            'detected_by': i,
                            'time': sim_time,
                        }
                        clr_tr = 'KIRMIZI' if zone['color'] == 'red' else 'MAVİ'
                        app.add_notify_log(
                            sim_time, i,
                            f'{clr_tr} alan tespit! Konum: ({zone["pos"][0]:.1f}, {zone["pos"][1]:.1f}) kaydedildi'
                        )
                        print(f'[KAMERA] D{i} {zone["color"].upper()} alan tespit → '
                              f'konum ({zone["pos"][0]:.1f}, {zone["pos"][1]:.1f}) hafızaya kaydedildi')
                    break

        app.set_per_drone_detection(i, det)

        # Aktif kamera için büyük PIP bilgisi
        if i == active_cam_id:
            if mission.scanning_qr_id is not None and det == 'scanning':
                active_cam_detection = 'scanning'
                pct = int(mission.scan_progress * 100)
                active_cam_label = f'QR{mission.scanning_qr_id} TARAMA... %{pct}'
            elif det == 'qr':
                active_cam_detection = 'qr'
                # En yakın QR bul
                best_qid = None
                best_dist = QR_DETECT_RANGE
                for qid, qpos in qr_positions_np.items():
                    d = np.linalg.norm(swarm.positions[i][:2] - qpos)
                    if d < best_dist:
                        best_dist = d
                        best_qid = qid
                if best_qid is not None:
                    active_cam_label = f'QR{best_qid} TESPIT [{best_dist:.1f}m]'
            elif det in ('color_red', 'color_blue'):
                active_cam_detection = det
                clr = 'KIRMIZI' if det == 'color_red' else 'MAVI'
                active_cam_label = f'{clr} ALAN TESPIT'
            else:
                active_cam_detection = None

    app.set_drone_cam_detection(active_cam_detection, active_cam_label)

    # Scan durum metni
    scan_text = ''
    if mission.scanning_qr_id is not None:
        pct = int(mission.scan_progress * 100)
        scan_text = f'SCANNING QR{mission.scanning_qr_id}... {pct}%'
    elif mission.last_scan_result and mission._scan_result_fade > 0:
        qid = mission.last_scan_qr_id
        task_data = mission.last_scan_result
        gorev = task_data.get('gorev', {})
        form = gorev.get('formasyon', {})
        parts = [f'QR{qid}:']
        if form.get('aktif'):
            parts.append(f'Form={form.get("tip")}')
        manevra = gorev.get('manevra_pitch_roll', {})
        if manevra.get('aktif'):
            parts.append(f'P={manevra.get("pitch_deg")}° R={manevra.get("roll_deg")}°')
        irtifa = gorev.get('irtifa_degisim', {})
        if irtifa.get('aktif'):
            parts.append(f'Alt={irtifa.get("deger")}m')
        suruden = task_data.get('suruden_ayrilma', {})
        if suruden.get('aktif'):
            parts.append(f'D{suruden.get("ayrilacak_drone_id")}>{suruden.get("hedef_renk")}')
        scan_text = ' | '.join(parts)

    # QR sonucunu açılır panele ekle
    if mission.last_scan_qr_id and mission.last_scan_qr_id not in _last_logged_qr:
        _last_logged_qr.add(mission.last_scan_qr_id)
        qid = mission.last_scan_qr_id
        task_data = mission.last_scan_result or {}
        gorev = task_data.get('gorev', {})
        parts_log = []
        form = gorev.get('formasyon', {})
        if form.get('aktif'):
            parts_log.append(f'Form: {form.get("tip")}')
        manevra = gorev.get('manevra_pitch_roll', {})
        if manevra.get('aktif'):
            parts_log.append(f'Pitch:{manevra.get("pitch_deg")}° Roll:{manevra.get("roll_deg")}°')
        irtifa = gorev.get('irtifa_degisim', {})
        if irtifa.get('aktif'):
            parts_log.append(f'İrtifa: {irtifa.get("deger")}m')
        suruden = task_data.get('suruden_ayrilma', {})
        if suruden.get('aktif'):
            parts_log.append(f'Ayrıl: D{suruden.get("ayrilacak_drone_id")}→{suruden.get("hedef_renk")}')
        sonraki = task_data.get('sonraki_qr', {})
        next_qr = sonraki.get(f'team_{config.get("team_id", 1)}', 0)
        parts_log.append(f'Sonraki: QR{next_qr}' if next_qr else 'SON')
        app.add_qr_log(qid, ' | '.join(parts_log))

    # HUD
    done = state == 'DONE'
    if done and not app._mission_done:
        app.set_mission_done()
    app.update_hud(
        state='GOREV TAMAMLANDI' if done else state,
        formation_name=swarm.formation_type.value,
        spacing=swarm.spacing,
        num_active=len(swarm.active_drones),
        num_total=NUM_DRONES,
        heading_deg=np.degrees(swarm.heading),
        altitude=swarm.target_altitude,
        sim_time=sim_time,
        done=done,
        scan_text=scan_text,
        arm_states=mission.armed,
    )

    return Task.cont


app.taskMgr.add(sim_update_task, 'sim_update', sort=50)
app.run()
