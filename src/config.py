"""
TEKNOFEST 2026 Sürü İHA - Yapılandırma Dosyası
Tüm sabitler, QR pozisyonları, formasyon tanımları burada.
"""

# --- Drone Sayısı ve Bağlantı ---
NUM_DRONES = 3
TEAM_ID = 1  # Hakemler tarafından söylenen takım ID'si

DRONE_PORTS = [f"udp://:1454{i}" for i in range(NUM_DRONES)]
GRPC_BASE_PORT = 50040

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

# --- Renkli İniş Bölgeleri (rota üzerinde tespit edilecek, şimdilik bilinen konumlar) ---
LANDING_ZONES = {
    "kirmizi": (-12.0, 35.0),  # Gazebo (x=35.0, y=-12.0)
    "mavi": (12.0, 35.0),      # Gazebo (x=35.0, y=12.0)
}

# --- Başlangıç / Home Konumu ---
HOME_POSITION = (0.0, 0.0)
FIRST_QR = 1  # İlk gidilecek QR

# --- Uçuş Parametreleri ---
DEFAULT_ALTITUDE = 15.0       # metre (hakemler belirleyecek)
DEFAULT_AGENT_DISTANCE = 5.0  # metre (ajanlar arası)
CRUISE_SPEED = 3.0            # m/s

# --- Formasyon Tip Eşleştirme (QR'daki Türkçe → kod) ---
FORMATION_MAP = {
    "OKBASI": "arrow",
    "CIZGI": "line",
    "V": "v",
    "UCGEN": "triangle",
}
DEFAULT_FORMATION = "line"

# --- QR İçerikleri (Şekil 2 — şartname formatı, her QR'da gömülü) ---
# Simülasyonda QR okuma yerine bu sözlükten çekilecek.
# Gerçek yarışmada kamera ile QR okunup aynı format parse edilecek.
QR_CONTENTS = {
    1: {
        "qr_id": 1,
        "gorev": {
            "formasyon": {
                "aktif": True,
                "tip": "OKBASI",
                "mesafe": 6.0,
            },
            "manevra_pitch_roll": {
                "aktif": False,
                "pitch_deg": 0,
                "roll_deg": 0,
            },
            "irtifa_degisim": {
                "aktif": True,
                "deger": 20,
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
                "mesafe": None,
            },
            "manevra_pitch_roll": {
                "aktif": True,
                "pitch_deg": -15,
                "roll_deg": 0,
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
                "mesafe": 5.0,
            },
            "manevra_pitch_roll": {
                "aktif": False,
                "pitch_deg": 0,
                "roll_deg": 0,
            },
            "irtifa_degisim": {
                "aktif": True,
                "deger": 15,
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
                "mesafe": 6.0,
            },
            "manevra_pitch_roll": {
                "aktif": True,
                "pitch_deg": 0,
                "roll_deg": 15,
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
                "mesafe": None,
            },
            "manevra_pitch_roll": {
                "aktif": False,
                "pitch_deg": 0,
                "roll_deg": 0,
            },
            "irtifa_degisim": {
                "aktif": True,
                "deger": 10,
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
                "mesafe": None,
            },
            "manevra_pitch_roll": {
                "aktif": False,
                "pitch_deg": 0,
                "roll_deg": 0,
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
