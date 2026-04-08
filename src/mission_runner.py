"""
Görev Yöneticisi — Şartname Şekil 2 formatında QR tarama
QR'dan okunan resmi JSON'a göre görevleri uygular.
"""
import json
import numpy as np
from src.formation import FormationType, FORMATION_NAME_MAP
from src.qr_system import FORMATION_TIP_MAP


class MissionRunner:
    """Görev 1 durum makinesi — resmi QR formatı ile çalışır."""

    STATES = [
        'INIT', 'TAKEOFF', 'FLY_TO_QR', 'SCAN_QR',
        'EXEC_FORMATION', 'EXEC_MANEUVER', 'EXEC_ALTITUDE',
        'EXEC_REMOVE', 'WAIT',
        'NAVIGATE_NEXT', 'RTH', 'LAND', 'DONE',
    ]

    # Ayrılan drone bağımsız durum makinesi
    SEP_STATES = [
        'IDLE',         # Ayrılma yok
        'FLY_TO_ZONE',  # Renk alanına uçuyor
        'LANDING',      # Renk alanında iniyor
        'WAIT_AT_ZONE', # Yerde bekliyor
        'TAKEOFF',      # Kalkıyor
        'FLY_BACK',     # Sürüye geri dönüyor
    ]

    def __init__(self, swarm, qr_codes: list, color_zones: list, config: dict,
                 qr_manager=None):
        self.swarm = swarm
        self.config = config
        self.qr_manager = qr_manager

        # QR pozisyonları (navigasyon için)
        self.qr_positions = {}
        for qr in qr_codes:
            self.qr_positions[qr['id']] = np.array(qr['position'], dtype=float)

        # Renk alanları — pozisyon lookup
        self.color_zones = color_zones
        self.zone_by_color = {}
        for z in color_zones:
            self.zone_by_color[z['color']] = np.array(z['position'][:2], dtype=float)

        self.state = 'INIT'
        self.team_id = config.get('team_id', 1)
        self.current_qr_id = config.get('first_qr', 1)
        self.current_task = None   # decoded JSON from QR
        self._wait_timer = 0
        self._stable_timer = 0
        self._stable_threshold = 1.5
        self.home = np.array(config.get('home_position', [0, 0, 0]), dtype=float)

        # Scan durumu (renderer için)
        self.scanning_qr_id = None
        self.scan_progress = 0.0
        self._scan_duration = 1.2
        self._scan_timer = 0.0
        self.last_scan_result = None
        self.last_scan_qr_id = None
        self._scan_result_fade = 0.0

        # ── Ayrılan drone — bağımsız paralel durum makinesi ──
        self._removed_drone = None
        self._removed_drone_target = None  # renk alanı pozisyonu
        self._removed_drone_return_pos = None
        self._sep_state = 'IDLE'
        self._sep_wait_timer = 0.0
        self._sep_land_alt = 0.0  # iniş/kalkış irtifası

        # Ayrılan drone bilgisi (renderer için)
        self.separated_drone_id = None
        self.separated_drone_color = None

        # Arm/disarm durumları (her drone için)
        self.armed = [True] * swarm.num_drones
        # Görev bitti mi
        self.all_disarmed = False

    def reset(self):
        """Tüm durumu sıfırla — görevi baştan başlatmak için."""
        self.state = 'INIT'
        self.current_qr_id = self.config.get('first_qr', 1)
        self.current_task = None
        self._wait_timer = 0
        self._stable_timer = 0
        self.scanning_qr_id = None
        self.scan_progress = 0.0
        self._scan_timer = 0.0
        self.last_scan_result = None
        self.last_scan_qr_id = None
        self._scan_result_fade = 0.0
        self._removed_drone = None
        self._removed_drone_target = None
        self._removed_drone_return_pos = None
        self._sep_state = 'IDLE'
        self._sep_wait_timer = 0.0
        self._sep_land_alt = 0.0
        self.separated_drone_id = None
        self.separated_drone_color = None
        self.armed = [True] * self.swarm.num_drones
        self.all_disarmed = False

    def update(self, dt: float) -> str:
        """Her frame çağır. Mevcut durumu döndür."""

        if self._scan_result_fade > 0:
            self._scan_result_fade -= dt * 0.3

        # ── Ayrılan drone paralel güncelleme (ana akıştan bağımsız) ──
        self._update_separated(dt)

        # ── Ana sürü durum makinesi ──
        if self.state == 'INIT':
            self.state = 'TAKEOFF'
            self.swarm.takeoff(self.config.get('initial_altitude', 1.0))

        elif self.state == 'TAKEOFF':
            avg_z = np.mean([self.swarm.positions[d][2] for d in self.swarm.active_drones])
            if abs(avg_z - self.swarm.target_altitude) < 0.3:
                self._stable_timer += dt
                if self._stable_timer > self._stable_threshold:
                    self._stable_timer = 0
                    self.state = 'FLY_TO_QR'
                    self._go_to_qr(self.current_qr_id)

        elif self.state == 'FLY_TO_QR':
            if self.swarm.is_at_target(threshold=1.0):
                self._stable_timer += dt
                if self._stable_timer > 0.5:
                    self._stable_timer = 0
                    self.state = 'SCAN_QR'
                    self.scanning_qr_id = self.current_qr_id
                    self.scan_progress = 0.0
                    self._scan_timer = 0.0

        elif self.state == 'SCAN_QR':
            self._scan_timer += dt
            self.scan_progress = min(self._scan_timer / self._scan_duration, 1.0)

            if self.scan_progress >= 1.0:
                # Gerçek QR tarama — JSON decode (resmi format)
                if self.qr_manager is not None:
                    decoded = self.qr_manager.scan_qr(self.current_qr_id)
                    self.current_task = decoded
                    self.last_scan_result = decoded
                    self.last_scan_qr_id = self.current_qr_id
                    self._scan_result_fade = 1.0
                    print(f'[QR SCAN] QR{self.current_qr_id} → {json.dumps(decoded, ensure_ascii=False)}')
                else:
                    self.current_task = {}

                self.scanning_qr_id = None
                self.scan_progress = 0.0
                self.state = 'EXEC_FORMATION'

        elif self.state == 'EXEC_FORMATION':
            gorev = (self.current_task or {}).get('gorev', {})
            formasyon = gorev.get('formasyon', {})
            if formasyon.get('aktif') and formasyon.get('tip'):
                tip = formasyon['tip']
                code_type = FORMATION_TIP_MAP.get(tip, tip.lower())
                f_type = FORMATION_NAME_MAP.get(code_type, FormationType.LINE)
                self.swarm.set_formation(f_type)
            self.state = 'EXEC_MANEUVER'

        elif self.state == 'EXEC_MANEUVER':
            gorev = (self.current_task or {}).get('gorev', {})
            manevra = gorev.get('manevra_pitch_roll', {})
            if manevra.get('aktif'):
                pitch = float(manevra.get('pitch_deg', 0))
                roll = float(manevra.get('roll_deg', 0))
                self.swarm.set_pitch_maneuver(pitch)
                self.swarm.set_roll_maneuver(roll)
            self.state = 'EXEC_ALTITUDE'

        elif self.state == 'EXEC_ALTITUDE':
            gorev = (self.current_task or {}).get('gorev', {})
            irtifa = gorev.get('irtifa_degisim', {})
            if irtifa.get('aktif') and irtifa.get('deger') is not None:
                self.swarm.set_altitude(float(irtifa['deger']))
            self.state = 'EXEC_REMOVE'

        elif self.state == 'EXEC_REMOVE':
            suruden = (self.current_task or {}).get('suruden_ayrilma', {})
            if suruden.get('aktif') and suruden.get('ayrilacak_drone_id') is not None:
                drone_id = suruden['ayrilacak_drone_id']
                hedef_renk = suruden.get('hedef_renk', 'red')
                bekleme = suruden.get('bekleme_suresi_s', 3) or 3

                # Drone'u sürüden çıkar
                self.swarm.remove_drone(drone_id)
                self._removed_drone = {
                    'id': drone_id,
                    'hedef_renk': hedef_renk,
                    'bekleme': bekleme,
                }
                self.separated_drone_id = drone_id
                self.separated_drone_color = hedef_renk

                # Hedef renk alanı pozisyonu
                zone_pos = self.zone_by_color.get(hedef_renk)
                if zone_pos is not None:
                    self._removed_drone_target = zone_pos.copy()
                    self._sep_land_alt = self.swarm.positions[drone_id][2]
                    print(f'[SÜRÜDEN AYRILMA] D{drone_id} → {hedef_renk} alanına uçuyor')
                    self._sep_state = 'FLY_TO_ZONE'  # Paralel başlat
                else:
                    print(f'[SÜRÜDEN AYRILMA] Renk alanı bulunamadı: {hedef_renk}')

            # Ana sürü BEKLEMEDEN devam ediyor
            self.state = 'WAIT'
            self._wait_timer = 0

        elif self.state == 'WAIT':
            gorev = (self.current_task or {}).get('gorev', {})
            wait_sec = gorev.get('bekleme_suresi_s', 2)
            self._wait_timer += dt
            if self._wait_timer >= wait_sec:
                self._wait_timer = 0
                self.swarm.clear_maneuvers()
                self.state = 'NAVIGATE_NEXT'

        elif self.state == 'NAVIGATE_NEXT':
            decoded = self.current_task or {}
            sonraki = decoded.get('sonraki_qr', {})
            next_qr = sonraki.get(f'team_{self.team_id}', 0)
            if next_qr and next_qr in self.qr_positions:
                self.current_qr_id = next_qr
                self.state = 'FLY_TO_QR'
                self._go_to_qr(next_qr)
            else:
                # RTH öncesi: ayrılan drone hâlâ dönemmediyse bekle
                if self._sep_state != 'IDLE':
                    pass  # Bir sonraki frame tekrar buraya gelecek
                else:
                    self.state = 'RTH'
                    self.swarm.navigate_to(self.home[:2])

        elif self.state == 'RTH':
            if self.swarm.is_at_target(threshold=1.0):
                self._stable_timer += dt
                if self._stable_timer > 1.0:
                    self._stable_timer = 0
                    self.state = 'LAND'
                    self.swarm.land()

        elif self.state == 'LAND':
            avg_z = np.mean([self.swarm.positions[d][2] for d in self.swarm.active_drones])
            if avg_z < 0.15:
                # Tüm İHA'lar disarm
                for d in range(self.swarm.num_drones):
                    self.armed[d] = False
                self.all_disarmed = True
                self.state = 'DONE'

        return self.state

    # ── Ayrılan drone bağımsız durum makinesi ─────────────────

    def _update_separated(self, dt):
        """Ayrılan drone'un paralel güncellemesi. Ana sürü durmaz."""
        if self._sep_state == 'IDLE' or self._removed_drone is None:
            return

        drone_id = self._removed_drone['id']

        if self._sep_state == 'FLY_TO_ZONE':
            target = self._removed_drone_target
            if target is None:
                self._sep_finish()
                return
            cur = self.swarm.positions[drone_id][:2]
            diff = target - cur
            dist = np.linalg.norm(diff)
            if dist < 0.5:
                print(f'[SÜRÜDEN AYRILMA] D{drone_id} renk alanına ulaştı, iniyor')
                self._sep_state = 'LANDING'

        elif self._sep_state == 'LANDING':
            # İrtifa azalt → yere in
            cur_z = self.swarm.positions[drone_id][2]
            if cur_z > 0.15:
                self.swarm.positions[drone_id][2] -= dt * 2.0  # İniş hızı
                self.swarm.positions[drone_id][2] = max(0.0, self.swarm.positions[drone_id][2])
            else:
                self.swarm.positions[drone_id][2] = 0.0
                self.armed[drone_id] = False  # Disarm
                print(f'[SÜRÜDEN AYRILMA] D{drone_id} indi, DISARM. Bekliyor ({self._removed_drone["bekleme"]}sn)')
                self._sep_state = 'WAIT_AT_ZONE'
                self._sep_wait_timer = 0.0

        elif self._sep_state == 'WAIT_AT_ZONE':
            bekleme = self._removed_drone.get('bekleme', 3)
            self._sep_wait_timer += dt
            if self._sep_wait_timer >= bekleme:
                self.armed[drone_id] = True  # ARM
                print(f'[SÜRÜDEN AYRILMA] D{drone_id} ARM, kalkıyor')
                self._sep_state = 'TAKEOFF'

        elif self._sep_state == 'TAKEOFF':
            # Sürü irtifasına çık
            target_z = self.swarm.target_altitude
            cur_z = self.swarm.positions[drone_id][2]
            if cur_z < target_z - 0.2:
                self.swarm.positions[drone_id][2] += dt * 2.5  # Kalkış hızı
            else:
                self.swarm.positions[drone_id][2] = target_z
                print(f'[SÜRÜDEN AYRILMA] D{drone_id} kalktı, sürüye geri dönüyor')
                self._sep_state = 'FLY_BACK'

        elif self._sep_state == 'FLY_BACK':
            # Sürü centroid'ine doğru uç
            target = self.swarm.centroid[:2]
            cur = self.swarm.positions[drone_id][:2]
            diff = target - cur
            dist = np.linalg.norm(diff)
            if dist < 1.5:
                self._sep_finish()
                print(f'[SÜRÜDEN AYRILMA] D{drone_id} sürüye geri katıldı')

    def _sep_finish(self):
        """Ayrılan drone görevini tamamladı, sürüye katıl."""
        if self._removed_drone:
            drone_id = self._removed_drone['id']
            self.swarm.add_drone(drone_id)
        self._removed_drone = None
        self._removed_drone_target = None
        self._removed_drone_return_pos = None
        self._sep_state = 'IDLE'
        self._sep_wait_timer = 0.0
        self.separated_drone_id = None
        self.separated_drone_color = None

    def _go_to_qr(self, qr_id):
        pos = self.qr_positions.get(qr_id)
        if pos is not None:
            self.swarm.navigate_to(pos[:2])
