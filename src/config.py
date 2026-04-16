"""
TEKNOFEST 2026 Sürü İHA - Yapılandırma Dosyası
Tüm sabitler, QR pozisyonları, formasyon tanımları burada.
"""

# --- Drone Sayısı ve Bağlantı ---
NUM_DRONES = 3
TEAM_ID = 1  # Hakemler tarafından söylenen takım ID'si

DRONE_PORTS = [f"udp://:1454{i}" for i in range(NUM_DRONES)]
GRPC_BASE_PORT = 50040

# --- Drone Spawn Pozisyonları (Gazebo ENU: x,y) ---
# launch_sim.sh ile aynı: x=East, y=North
# NED'e çevrim: North=y, East=x
DRONE_SPAWNS_NED = {
    0: (0.0, 0.0),
    1: (3.0, 0.0),
    2: (6.0, 0.0),
}

# --- QR Kod Pozisyonları (yarışma öncesi hakemler tarafından paylaşılacak) ---
# NED: (North, East) metre cinsinden home'a göre
# NOT: Gazebo ENU kullanır → NED North = Gazebo Y, NED East = Gazebo X
QR_POSITIONS = {
    1: (-12.5, 8.35),     # Gazebo (x=8.35, y=-12.5)
    2: (12.5, 8.35),      # Gazebo (x=8.35, y=12.5)
    3: (25.0, 30.0),      # Gazebo (x=30.0, y=25.0)
    4: (12.5, 51.65),     # Gazebo (x=51.65, y=12.5)
    5: (-12.5, 51.65),    # Gazebo (x=51.65, y=-12.5)
    6: (-25.0, 30.0),     # Gazebo (x=30.0, y=-25.0)
}

# --- Renkli İniş Bölgeleri (rota üzerinde, kamera ile tespit edilecek) ---
# NED koordinatları; Gazebo ENU → NED: North=GazeboY, East=GazeboX
# Mavi: rota 1→4 orta noktası, Kırmızı: rota 4→2 orta noktası
LANDING_ZONES = {
    "kirmizi": (12.5, 30.0),   # Gazebo (x=30.0, y=12.5)
    "mavi": (0.0, 30.0),       # Gazebo (x=30.0, y=0.0)
}

# --- Başlangıç / Home Konumu ---
HOME_POSITION = (0.0, 0.0)
FIRST_QR = 1  # İlk gidilecek QR

# --- Uçuş Parametreleri ---
DEFAULT_ALTITUDE = 6.0        # metre (QR okunabilmesi için max 6m)
DEFAULT_AGENT_DISTANCE = 5.0  # metre (ajanlar arası, hakemler belirler)
CRUISE_SPEED = 3.0            # m/s

# --- Formasyon Tip Eşleştirme (QR'daki Türkçe → kod) ---
# Şartname Şekil 3: Ok Başı, V, Çizgi
FORMATION_MAP = {
    "OKBASI": "arrow",
    "V": "v",
    "CIZGI": "line",
}
DEFAULT_FORMATION = "line"

# --- QR İçerikleri (Şekil 2 — şartname formatı, her QR'da gömülü) ---
# Gerçek yarışmada kamera ile QR okunup aynı format parse edilecek.
# Format: şartname Şekil 2 ile birebir aynı
QR_CONTENTS = {
    1: {
        "qr_id": 1,
        "gorev": {
            "formasyon": {
                "aktif": True,
                "tip": "OKBASI",
            },
            "manevra_pitch_roll": {
                "aktif": False,
                "pitch_deg": "0",
                "roll_deg": "0",
            },
            "irtifa_degisim": {
                "aktif": True,
                "deger": 6,
            },
            "bekleme_suresi_s": 3,
        },
        "suruden_ayrilma": {
            "aktif": False,
            "ayrilacak_drone_id": None,
            "hedef_renk": None,
            "bekleme_suresi_s": None,
        },
        "sonraki_qr": {
            "team_1": 4,
            "team_2": 3,
            "team_3": 5,
        },
    },
    4: {
        "qr_id": 4,
        "gorev": {
            "formasyon": {
                "aktif": False,
                "tip": None,
            },
            "manevra_pitch_roll": {
                "aktif": True,
                "pitch_deg": "-10",
                "roll_deg": "0",
            },
            "irtifa_degisim": {
                "aktif": False,
                "deger": None,
            },
            "bekleme_suresi_s": 3,
        },
        "suruden_ayrilma": {
            "aktif": False,
            "ayrilacak_drone_id": None,
            "hedef_renk": None,
            "bekleme_suresi_s": None,
        },
        "sonraki_qr": {
            "team_1": 2,
            "team_2": 5,
            "team_3": 1,
        },
    },
    2: {
        "qr_id": 2,
        "gorev": {
            "formasyon": {
                "aktif": True,
                "tip": "CIZGI",
            },
            "manevra_pitch_roll": {
                "aktif": False,
                "pitch_deg": "0",
                "roll_deg": "0",
            },
            "irtifa_degisim": {
                "aktif": True,
                "deger": 6,
            },
            "bekleme_suresi_s": 5,
        },
        "suruden_ayrilma": {
            "aktif": True,
            "ayrilacak_drone_id": 2,
            "hedef_renk": "mavi",
            "bekleme_suresi_s": 5,
        },
        "sonraki_qr": {
            "team_1": 3,
            "team_2": 6,
            "team_3": 4,
        },
    },
    3: {
        "qr_id": 3,
        "gorev": {
            "formasyon": {
                "aktif": True,
                "tip": "V",
            },
            "manevra_pitch_roll": {
                "aktif": True,
                "pitch_deg": "0",
                "roll_deg": "10",
            },
            "irtifa_degisim": {
                "aktif": False,
                "deger": None,
            },
            "bekleme_suresi_s": 3,
        },
        "suruden_ayrilma": {
            "aktif": False,
            "ayrilacak_drone_id": None,
            "hedef_renk": None,
            "bekleme_suresi_s": None,
        },
        "sonraki_qr": {
            "team_1": 5,
            "team_2": 1,
            "team_3": 6,
        },
    },
    5: {
        "qr_id": 5,
        "gorev": {
            "formasyon": {
                "aktif": False,
                "tip": None,
            },
            "manevra_pitch_roll": {
                "aktif": False,
                "pitch_deg": "0",
                "roll_deg": "0",
            },
            "irtifa_degisim": {
                "aktif": True,
                "deger": 5,
            },
            "bekleme_suresi_s": 2,
        },
        "suruden_ayrilma": {
            "aktif": False,
            "ayrilacak_drone_id": None,
            "hedef_renk": None,
            "bekleme_suresi_s": None,
        },
        "sonraki_qr": {
            "team_1": 6,
            "team_2": 4,
            "team_3": 2,
        },
    },
    6: {
        "qr_id": 6,
        "gorev": {
            "formasyon": {
                "aktif": False,
                "tip": None,
            },
            "manevra_pitch_roll": {
                "aktif": False,
                "pitch_deg": "0",
                "roll_deg": "0",
            },
            "irtifa_degisim": {
                "aktif": False,
                "deger": None,
            },
            "bekleme_suresi_s": 0,
        },
        "suruden_ayrilma": {
            "aktif": False,
            "ayrilacak_drone_id": None,
            "hedef_renk": None,
            "bekleme_suresi_s": None,
        },
        "sonraki_qr": {
            "team_1": 0,
            "team_2": 0,
            "team_3": 0,
        },
    },
}


# --- Failsafe / Safety Parametreleri ---

# Çarpışma minimum mesafe (drone-drone)
COLLISION_CHECK_ENABLED = True
COLLISION_MIN_DISTANCE_M = 2.0
COLLISION_VIOLATION_ACTION = "hold"  # hold | abort

# Formasyon güvenlik zarfı (arena sınırları)
FORMATION_ENVELOPE_ENABLED = True
ARENA_NORTH_MIN_M = -35.0
ARENA_NORTH_MAX_M = 35.0
ARENA_EAST_MIN_M = -5.0
ARENA_EAST_MAX_M = 65.0
FORMATION_SAFETY_MARGIN_M = 2.0
ENVELOPE_VIOLATION_ACTION = "hold"  # hold | abort

# Kamera sağlık ve algı güvenilirliği
CAMERA_HEALTH_ENABLED = True
CAMERA_FRAME_TIMEOUT_S = 2.0
CAMERA_MIN_CONFIDENCE_PCT = 2.0
QR_READ_MAX_ATTEMPTS = 10

# Drift / suruklenme
DRIFT_FAILSAFE_ENABLED = True
DRIFT_MAX_DISTANCE_M = 3.0
DRIFT_HOLD_SECONDS = 3.0
DRIFT_ACTION = "hold"  # hold | rtl

# İniş alanı uygunluk
LANDING_ZONE_VALIDATION_ENABLED = True
LANDING_ZONE_MIN_DETECTIONS = 3
LANDING_ZONE_MIN_CONFIDENCE_PCT = 2.0
LANDING_ZONE_MAX_MOTION_RATIO = 0.15
LANDING_ZONE_ACTION = "abort"  # hold | abort

# Incident loglama
INCIDENT_LOG_ENABLED = True
INCIDENT_LOG_PATH = "logs/incidents_{timestamp}.json"
