# TEKNOFEST 2026 — Sürü İHA Simülasyon

TEKNOFEST 2026 Sürü İHA Yarışması için PX4 + Gazebo tabanlı 3 drone sürü simülasyonu.  
Otonom görev (5.1) ve yarı otonom kontrol (5.2) destekler.

## Proje Yapısı

```
swarm/
├── PX4-Autopilot/             # PX4 SITL (manuel kurulum, .gitignore'da)
├── src/
│   ├── config.py              # QR pozisyonları, görev tanımları, parametreler
│   ├── formations.py          # Formasyon geometrisi (arrow, line, v)
│   ├── swarm_controller.py    # Ana sürü kontrolcüsü (MAVSDK)
│   ├── camera.py              # Gazebo kamera → OpenCV frame (gz-transport)
│   ├── detection.py           # QR okuma (pyzbar+OpenCV) + renkli alan tespiti
│   ├── missions/
│   │   ├── autonomous.py      # Görev 5.1 — Otonom dinamik sürü
│   │   └── semi_auto.py       # Görev 5.2 — Yarı otonom klavye kontrol
│   └── arena/
│       └── generate.py        # Gazebo world + QR PNG oluşturucu
├── scripts/
│   └── launch_sim.sh          # 3 drone + Gazebo başlatıcı
├── gazebo/                    # generate.py tarafından üretilir
│   ├── textures/
│   └── worlds/
├── requirements.txt           # Python bağımlılıkları
└── README.md
```

## Gereksinimler

- **Ubuntu 22.04** (veya 20.04)
- **Python 3.10+**
- **Git**

---

## Kurulum (Sıfırdan)

### 1. Repoyu klonla

```bash
cd ~/Desktop
git clone https://github.com/mustafayanmaz/swarm.git
cd swarm
```

### 2. PX4-Autopilot kurulumu

```bash
cd ~/Desktop/swarm
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

> `ubuntu.sh` tüm sistem bağımlılıklarını (Gazebo, cmake, protobuf vb.) kurar.
> Kurulum sonrası **terminali kapatıp yeniden aç** (ortam değişkenleri yüklensin).

### 3. PX4'ü derle (ilk build)

```bash
cd ~/Desktop/swarm/PX4-Autopilot
make px4_sitl gz_x500
```

> İlk build uzun sürer (~5-10 dk). Tamamlandığında Gazebo açılır, drone görünür.
> **Ctrl+C** ile kapat. Build artık hazır.

### 4. Python sanal ortam (venv)

```bash
cd ~/Desktop/swarm
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

> Her yeni terminal açtığında `source venv/bin/activate` çalıştır.

### 5. gz-transport Python bağlantısı (kamera için)

```bash
# Sistem paketlerini venv'e linkle
ln -sf /usr/lib/python3/dist-packages/gz ~/Desktop/swarm/venv/lib/python3.10/site-packages/gz
```

### 6. Gazebo arena oluştur
cd ~/Desktop/swarm/PX4-Autopilot
make px4_sitl gz_x500
```

> İlk build uzun sürer (~5-10 dk). Tamamlandığında Gazebo açılır, drone görünür.  
> **Ctrl+C** ile kapat. Build artık hazır.

### 5. Gazebo arena oluştur

```bash
cd ~/Desktop/swarm
source venv/bin/activate
python src/arena/generate.py
```

Bu komut:
- 6 gerçek QR kod PNG oluşturur (şartname JSON formatında)
- Mavi/kırmızı iniş bölgesi PNG'leri oluşturur
- Gazebo world dosyasını (altıgen alan, çizgiler, bölgeler) üretir
- PX4'ün `default.sdf` dosyasını arena ile değiştirir (orijinal yedeklenir)

---

## Simülasyonu Çalıştırma

### Yöntem A: launch_sim.sh ile (önerilen)

```bash
cd ~/Desktop/swarm
chmod +x scripts/launch_sim.sh
./scripts/launch_sim.sh
```

Bu script:
1. Önceki PX4/Gazebo süreçlerini temizler
2. Gazebo'yu arena world ile başlatır
3. 3 PX4 SITL instance'ı sırayla başlatır (instance 0, 1, 2)

### Yöntem B: Manuel başlatma

**Terminal 1 — İlk drone (Gazebo'yu da başlatır):**
```bash
cd ~/Desktop/swarm/PX4-Autopilot
make px4_sitl gz_x500
```

**Terminal 2 — İkinci drone:**
```bash
cd ~/Desktop/swarm/PX4-Autopilot
source Tools/simulation/gz/setup_gz.bash
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,3,0,0,0,0" PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 1
```

**Terminal 3 — Üçüncü drone:**
```bash
cd ~/Desktop/swarm/PX4-Autopilot
source Tools/simulation/gz/setup_gz.bash
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,6,0,0,0,0" PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 2
```

### MAVLink Port Tablosu

| Drone | Instance | Port |
|-------|----------|------|
| Drone 0 | `-i 0` | `udp://:14540` |
| Drone 1 | `-i 1` | `udp://:14541` |
| Drone 2 | `-i 2` | `udp://:14542` |

---

## Görevleri Çalıştırma

Simülasyon çalışırken **ayrı bir terminalde**:

```bash
cd ~/Desktop/swarm
source venv/bin/activate
```

### Görev 5.1 — Otonom Dinamik Sürü

```bash
python main.py mission
# veya
python -m src.missions.autonomous
```

Drone'lar:
1. Formasyon halinde kalkış
2. QR1'e git → QR oku → görev icra et
3. `sonraki_qr[team_id]` ile sonraki QR'a git
4. `sonraki_qr == 0` → eve dön → iniş

### Görev 5.2 — Yarı Otonom Kontrol

```bash
python main.py semi
# veya
python -m src.missions.semi_auto
```

Klavye kontrolleri:

| Tuş | İşlev |
|-----|-------|
| `T` | Kalkış |
| `W/A/S/D` | İleri/Sol/Geri/Sağ |
| `Q/E` | Yaw sol/sağ |
| `R/F` | İrtifa artır/azalt |
| `1/2/3/4` | Çizgi/Ok başı/V/Üçgen formasyon |
| `P/I` | Pitch +15°/-15° |
| `O/U` | Roll +15°/-15° |
| `L` | Eve dön + iniş |

### Demo Modları

```bash
python main.py formation   # Formasyon değişim demosu
python main.py maneuver    # Pitch/Roll manevra demosu
```

---

## QR Kod Formatı (Şartname Şekil 2)

Her QR kodunda JSON formatında:

```json
{
  "qr_id": 1,
  "gorev": {
    "formasyon": {"aktif": true, "tip": "OKBASI"},
    "manevra_pitch_roll": {"aktif": false, "pitch_deg": "0", "roll_deg": "0"},
    "irtifa_degisim": {"aktif": true, "deger": 8},
    "bekleme_suresi_s": 3
  },
  "suruden_ayrilma": {
    "aktif": false,
    "ayrilacak_drone_id": null,
    "hedef_renk": null,
    "bekleme_suresi_s": null
  },
  "sonraki_qr": {"team_1": 4, "team_2": 3, "team_3": 5}
}
```

Formasyon tipleri: `OKBASI`, `CIZGI`, `V`

---

## Yapılandırma

Tüm parametreler `src/config.py` dosyasında:

| Parametre | Açıklama | Varsayılan |
|-----------|----------|------------|
| `TEAM_ID` | Takım numarası (QR rotasını belirler) | `1` |
| `NUM_DRONES` | Drone sayısı | `3` |
| `FIRST_QR` | İlk gidilecek QR noktası | `1` |
| `DEFAULT_ALTITUDE` | Kalkış irtifası (m) | `8.0` |
| `DEFAULT_AGENT_DISTANCE` | Ajanlar arası mesafe (m) | `5.0` |
| `CRUISE_SPEED` | Seyir hızı (m/s) | `3.0` |

QR pozisyonları ve görev içerikleri de `src/config.py`'de tanımlıdır.

---

## Arena Yeniden Oluşturma

QR pozisyonları veya görev içeriklerini değiştirdikten sonra:

```bash
python src/arena/generate.py
```

Bu, Gazebo world dosyasını ve QR PNG'lerini yeniden üretir.

---

## Sorun Giderme

| Sorun | Çözüm |
|-------|-------|
| `ModuleNotFoundError: mavsdk` | `source venv/bin/activate && pip install mavsdk` |
| Drone bağlanmıyor | Simülasyon çalışıyor mu kontrol et (`scripts/launch_sim.sh`) |
| `lockstep_scheduler` hatası | `source Tools/simulation/gz/setup_gz.bash` komutu çalıştır |
| Gazebo'da QR görünmüyor | `python src/arena/generate.py` ile world'ü yeniden üret |
| QGroundControl bağlanmıyor | Aynı portları kullanma, QGC otomatik bağlanır |
| Arena yüklenmiyor | `make px4_sitl gz_x500` ile PX4'ü yeniden başlat |

---

## Lisans

Bu proje TEKNOFEST 2026 Sürü İHA Yarışması için geliştirilmiştir.