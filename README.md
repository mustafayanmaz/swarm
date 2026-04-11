# TEKNOFEST 2026 — Sürü İHA Simülasyon

TEKNOFEST 2026 Sürü İHA Yarışması için PX4 SITL + Gazebo Harmonic tabanlı 3 drone sürü simülasyonu.  
Kamera ile QR okuma, renkli alan tespiti, dinamik formasyon değişimi destekler.

- **Görev 5.1** — Otonom dinamik sürü (QR rota takibi, formasyon, manevra, iniş)
- **Görev 5.2** — Yarı otonom klavye kontrol

---

## Proje Yapısı

```
swarm/
├── PX4-Autopilot/             # PX4 SITL (manuel kurulum, .gitignore'da)
├── src/
│   ├── config.py              # QR pozisyonları, görev tanımları, tüm parametreler
│   ├── formations.py          # Formasyon geometrisi (Ok Başı, Çizgi, V)
│   ├── swarm_controller.py    # Ana sürü kontrolcüsü (MAVSDK)
│   ├── camera.py              # Gazebo kamera → OpenCV frame (gz-transport13)
│   ├── detection.py           # QR okuma (pyzbar + OpenCV) + renkli alan tespiti (HSV)
│   ├── missions/
│   │   ├── autonomous.py      # Görev 5.1 — Otonom dinamik sürü
│   │   └── semi_auto.py       # Görev 5.2 — Yarı otonom klavye kontrol
│   └── arena/
│       └── generate.py        # Gazebo world + QR/zone PNG oluşturucu
├── scripts/
│   └── launch_sim.sh          # 3 drone + Gazebo başlatıcı (NVIDIA PRIME destekli)
├── gazebo/                    # generate.py tarafından otomatik üretilir
│   ├── textures/              # QR PNG'leri (2048x2048) + zone PNG'leri
│   └── worlds/                # swarm_arena.sdf
├── requirements.txt
└── README.md
```

---

## Sistem Gereksinimleri

- **Ubuntu 22.04** (veya 20.04)
- **Python 3.10+**
- **Git**
- **NVIDIA GPU** önerilir (Gazebo rendering için)
- **~20 GB** disk alanı (PX4 + Gazebo + build)

---

## Kurulum (Sıfırdan)

### 1. Repoyu klonla

```bash
cd ~/Desktop
git clone https://github.com/mustafayanmaz/swarm.git
cd swarm
```

### 2. PX4-Autopilot kurulumu

PX4 `.gitignore`'da olduğu için repoyla gelmez, manuel klonlanır:

```bash
cd ~/Desktop/swarm
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

> **ÖNEMLİ:** `ubuntu.sh` tüm sistem bağımlılıklarını kurar (Gazebo Harmonic, cmake, protobuf, gz-transport13 vb.).  
> Kurulum sonrası **terminali kapatıp yeniden aç** (ortam değişkenleri yüklensin).

### 3. PX4'ü derle (ilk build)

```bash
cd ~/Desktop/swarm/PX4-Autopilot
make px4_sitl gz_x500
```

> İlk build uzun sürer (~5-10 dk). Tamamlandığında Gazebo açılır, drone görünür.  
> **Ctrl+C** ile kapat. Build artık hazır.

### 4. Kamera FOV Ayarı (ÖNEMLİ!)

PX4'ün varsayılan `mono_cam` kamerası 1.74 rad (~100°) FOV ile gelir.  
QR kodların 6m irtifadan okunabilmesi için **90° (1.5708 rad)** olmalıdır.

Dosya: `PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf`

```xml
<!-- ÖNCE (varsayılan): -->
<horizontal_fov>1.74</horizontal_fov>

<!-- SONRA (düzeltilmiş): -->
<horizontal_fov>1.5708</horizontal_fov>
```

Düzeltmek için:

```bash
sed -i 's/<horizontal_fov>1.74</<horizontal_fov>1.5708</' \
  ~/Desktop/swarm/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf
```

> **Neden?** Daha geniş FOV = QR kodlar pikselde daha küçük görünür.  
> 1.2m QR kod, 6m irtifada 90° FOV ile ~256 piksel kaplar → okunabilir.  
> 100° FOV'da ~200 piksel → ERROR_CORRECT_L ile sınırda kalır, 7-8m'de okunamaz.

**Kamera özellikleri (düzeltme sonrası):**

| Özellik | Değer |
|---------|-------|
| Çözünürlük | 1280 × 960 |
| FPS | 30 |
| FOV | 90° (1.5708 rad) |
| Yön | Aşağı bakan (downward) |
| Model | `x500_mono_cam_down` |
| Airframe | 4014 |

### 5. Python sanal ortam (venv)

```bash
cd ~/Desktop/swarm
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

> Her yeni terminal açtığında `source venv/bin/activate` çalıştır.

**Python bağımlılıkları:**
- `mavsdk` — Drone kontrolü (gRPC)
- `opencv-python` — Görüntü işleme
- `pyzbar` — QR kod okuma (libzbar gerektirir)
- `qrcode`, `Pillow` — QR PNG üretimi (arena)
- `numpy` — Matris işlemleri

### 6. gz-transport Python bağlantısı (kamera için)

Gazebo kameralarından frame almak için `gz-transport13` Python binding'leri gerekir.  
Bu paketler sistem Python'una kurulur, venv'e symlink ile bağlanır:

```bash
# Sistem paketini venv'e linkle
ln -sf /usr/lib/python3/dist-packages/gz \
  ~/Desktop/swarm/venv/lib/python3.10/site-packages/gz
```

> Python 3.10 değilse (ör. 3.11), path'i kendi sürümüne göre düzelt.

### 7. pyzbar sistem bağımlılığı

```bash
sudo apt install libzbar0
```

### 8. Arena oluştur

```bash
cd ~/Desktop/swarm
source venv/bin/activate
python src/arena/generate.py
```

Bu komut:
- **6 QR kod PNG** oluşturur (2048×2048 piksel, ERROR_CORRECT_L, şartname JSON formatında)
- **Kırmızı/Mavi iniş bölgesi PNG'leri** oluşturur
- **Gazebo world** (altıgen alan, QR paneller, rota çizgileri, iniş bölgeleri) üretir
- PX4'ün `default.sdf` dosyasını arena ile değiştirir (orijinal `default.sdf.bak` olarak yedeklenir)

---

## Simülasyonu Çalıştırma

### Yöntem A: launch_sim.sh ile (önerilen)

```bash
cd ~/Desktop/swarm
chmod +x scripts/launch_sim.sh
./scripts/launch_sim.sh
```

Bu script:
1. Önceki PX4/Gazebo süreçlerini temizler (`pkill`)
2. NVIDIA PRIME offload ortam değişkenlerini ayarlar
3. Gazebo'yu `swarm_arena.sdf` world ile başlatır
4. 3 PX4 SITL instance'ı sırayla başlatır (`x500_mono_cam_down`, airframe 4014)

**GUI'siz çalıştırma (headless):**
```bash
./scripts/launch_sim.sh headless
```

### Yöntem B: Manuel başlatma

**Terminal 1 — İlk drone (Gazebo'yu da başlatır):**
```bash
cd ~/Desktop/swarm/PX4-Autopilot
PX4_SYS_AUTOSTART=4014 PX4_GZ_MODEL=x500_mono_cam_down make px4_sitl gz_x500_mono_cam_down
```

**Terminal 2 — İkinci drone:**
```bash
cd ~/Desktop/swarm/PX4-Autopilot
source Tools/simulation/gz/setup_gz.bash
PX4_SYS_AUTOSTART=4014 PX4_GZ_MODEL_POSE="0,3,0,0,0,0" \
  PX4_GZ_MODEL=x500_mono_cam_down \
  ./build/px4_sitl_default/bin/px4 -i 1
```

**Terminal 3 — Üçüncü drone:**
```bash
cd ~/Desktop/swarm/PX4-Autopilot
source Tools/simulation/gz/setup_gz.bash
PX4_SYS_AUTOSTART=4014 PX4_GZ_MODEL_POSE="0,6,0,0,0,0" \
  PX4_GZ_MODEL=x500_mono_cam_down \
  ./build/px4_sitl_default/bin/px4 -i 2
```

### Port Tablosu

| Drone | Instance | MAVLink (UDP) | gRPC (MAVSDK) |
|-------|----------|---------------|---------------|
| Drone 0 | `-i 0` | `udp://:14540` | `localhost:50040` |
| Drone 1 | `-i 1` | `udp://:14541` | `localhost:50041` |
| Drone 2 | `-i 2` | `udp://:14542` | `localhost:50042` |

---

## Görevleri Çalıştırma

Simülasyon çalışırken **ayrı bir terminalde**:

```bash
cd ~/Desktop/swarm
source venv/bin/activate
```

### Görev 5.1 — Otonom Dinamik Sürü

```bash
cd src/missions
python autonomous.py
```

**Görev akışı:**
1. 3 drone çizgi formasyonunda kalkış (6m)
2. QR1'e git → kamera ile QR oku (kademeli alçalma: mevcut → 6m → 4m)
3. QR'daki görevi icra et (formasyon değişimi, manevra, irtifa, bekleme)
4. Sürüden ayrılma varsa: ilgili drone renkli alana gidip iner
5. `sonraki_qr[team_id]` ile sıradaki QR'a git
6. `sonraki_qr == 0` → eve dön → iniş
7. Hareket sırasında kamera ile renkli alan taraması (her 0.1s, 3 kamera)

**Kamera ile QR okuma detayı:**
- 3x upscale (INTER_CUBIC) + 4 farklı threshold yöntemi
- pyzbar + OpenCV QRCodeDetector paralel deneme
- 3 drone kamerası da taranır
- Okunamazsa config.py'deki fallback verisi kullanılır

**Renkli alan tespiti:**
- HSV renk uzayında kırmızı/mavi alan arama
- Piksel oranı > %2 → alan tespit edildi
- Tespit anındaki sürü merkezi NED koordinatı kaydedilir
- İniş için bu koordinatlar kullanılır

### Görev 5.2 — Yarı Otonom Kontrol

```bash
cd src/missions
python semi_auto.py
```

| Tuş | İşlev |
|-----|-------|
| `T` | Kalkış |
| `W/A/S/D` | İleri / Sol / Geri / Sağ |
| `Q/E` | Yaw sol / sağ |
| `R/F` | İrtifa artır / azalt |
| `1/2/3` | Çizgi / Ok Başı / V formasyonu |
| `P/I` | Pitch +15° / -15° |
| `O/U` | Roll +15° / -15° |
| `L` | Eve dön + iniş |

---

## QR Kod Formatı (Şartname Şekil 2)

Her QR kodunda JSON:

```json
{
  "qr_id": 1,
  "gorev": {
    "formasyon": {"aktif": true, "tip": "OKBASI"},
    "manevra_pitch_roll": {"aktif": false, "pitch_deg": "0", "roll_deg": "0"},
    "irtifa_degisim": {"aktif": true, "deger": 6},
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

**Formasyon tipleri:** `OKBASI` (Ok Başı), `CIZGI` (Çizgi), `V`

**QR teknik detay:**
- 2048×2048 piksel PNG, QR ERROR_CORRECT_L (65 modül)
- PBR material: metalness=0, roughness=1 (düz/mat → pırıltı yok)
- 1.2m × 1.2m fiziksel boyut (şartname: 120cm × 120cm)

---

## Yapılandırma

Tüm parametreler `src/config.py` dosyasında:

| Parametre | Açıklama | Varsayılan |
|-----------|----------|------------|
| `TEAM_ID` | Takım numarası (QR rotasını belirler) | `1` |
| `NUM_DRONES` | Drone sayısı | `3` |
| `FIRST_QR` | İlk gidilecek QR noktası | `1` |
| `DEFAULT_ALTITUDE` | Uçuş irtifası (m) | `6.0` |
| `DEFAULT_AGENT_DISTANCE` | Ajanlar arası mesafe (m) | `5.0` |
| `CRUISE_SPEED` | Seyir hızı (m/s) | `3.0` |
| `DEFAULT_FORMATION` | Başlangıç formasyonu | `line` |

**İrtifa sınırları:**
- Maksimum uçuş irtifası: **6m** (QR okuma güvenilirliği)
- QR okuma kademeli alçalma: mevcut irtifa → 6m → 4m
- QR'daki `irtifa_degisim.deger` değeri 6m ile sınırlıdır

**QR pozisyonları** ve **iniş bölgeleri** de `src/config.py`'de NED koordinatları olarak tanımlıdır.

---

## Arena

### Altıgen yarışma alanı

- 6 QR kod altıgenin köşelerinde (yarıçap ~30m)
- Sarı rota çizgileri QR'lar arasında (QR1→QR4→QR2→QR3→QR5→QR6)
- Kırmızı ve mavi iniş bölgeleri rota üzerinde

### Yeniden oluşturma

QR pozisyonları veya görev içeriklerini değiştirdikten sonra:

```bash
cd ~/Desktop/swarm
source venv/bin/activate
python src/arena/generate.py
```

Bu komut world dosyasını, QR PNG'lerini ve zone PNG'lerini yeniden üretir.

---

## Protobuf Uyumluluğu

MAVSDK'nın kullandığı protobuf sürümü ile sistemdeki C++ protobuf arasında uyumsuzluk olabilir.  
`autonomous.py` dosyasının en üstünde bu otomatik olarak ayarlanır:

```python
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
```

> Bu satır tüm import'lardan **önce** olmalıdır, yoksa segfault alırsınız.

---

## NVIDIA GPU Notları

Eğer sisteminizde hem Intel iGPU hem NVIDIA GPU varsa (PRIME setup), Gazebo'nun dGPU kullanması için:

```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
```

`launch_sim.sh` bunu otomatik yapar. Manuel başlatıyorsanız bu değişkenleri ayarlayın.

---

## Sorun Giderme

| Sorun | Çözüm |
|-------|-------|
| `ModuleNotFoundError: mavsdk` | `source venv/bin/activate && pip install mavsdk` |
| `ModuleNotFoundError: gz` | gz-transport symlink'ini kontrol et (Adım 6) |
| `ImportError: libzbar.so` | `sudo apt install libzbar0` |
| Drone bağlanmıyor | Simülasyon çalışıyor mu kontrol et (`./scripts/launch_sim.sh`) |
| Segfault (protobuf) | `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python` ayarlı mı kontrol et |
| QR okunamıyor (yüksek irtifa) | Kamera FOV'u 1.5708 mi kontrol et (Adım 4) |
| QR okunamıyor (bulanık) | `python src/arena/generate.py` ile QR PNG'leri yeniden oluştur |
| Gazebo'da QR/arena yok | `python src/arena/generate.py` çalıştır, simülasyonu yeniden başlat |
| `lockstep_scheduler` hatası | `source Tools/simulation/gz/setup_gz.bash` |
| Kırmızı alan yanlış tespit | Rota çizgileri sarı olmalı, `generate.py` ile yeniden oluştur |
| Arena yüklenmiyor | `make px4_sitl gz_x500` ile PX4'ü yeniden build et |

---

## Hızlı Başlangıç (TL;DR)

```bash
# 1. Repo + PX4
cd ~/Desktop
git clone https://github.com/mustafayanmaz/swarm.git && cd swarm
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot && bash ./Tools/setup/ubuntu.sh && cd ..
# (terminali kapat, yeniden aç)

# 2. PX4 build
cd PX4-Autopilot && make px4_sitl gz_x500   # Ctrl+C ile kapat
cd ..

# 3. Kamera FOV düzelt (ÖNEMLİ!)
sed -i 's/<horizontal_fov>1.74</<horizontal_fov>1.5708</' \
  PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf

# 4. Python ortamı
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
sudo apt install libzbar0
ln -sf /usr/lib/python3/dist-packages/gz venv/lib/python3.10/site-packages/gz

# 5. Arena oluştur
python src/arena/generate.py

# 6. Simülasyon başlat (Terminal 1)
./scripts/launch_sim.sh

# 7. Görev çalıştır (Terminal 2)
source venv/bin/activate && cd src/missions && python autonomous.py
```

---

## Lisans

Bu proje TEKNOFEST 2026 Sürü İHA Yarışması için geliştirilmiştir.