"""
Gazebo world — inline modeller (dış dosya gerektirmez).
PX4 default.sdf'i bu arena ile değiştirir.
QR PNG'leri config.py'deki QR_CONTENTS'tan üretilir.
"""
import json
import math
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import qrcode
from PIL import Image

BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TEXTURE_DIR = os.path.join(BASE_DIR, "gazebo", "textures")
WORLD_DIR = os.path.join(BASE_DIR, "gazebo", "worlds")
os.makedirs(TEXTURE_DIR, exist_ok=True)
os.makedirs(WORLD_DIR, exist_ok=True)

# config.py'den QR içeriklerini al
from src.config import QR_CONTENTS

# ─── Altıgen QR Pozisyonları ────────────────────────────────
HEXAGON_RADIUS = 25.0
CENTER = (30.0, 0.0)

hex_order = [3, 4, 5, 6, 1, 2]
QR_POS = {}
for i, qr_id in enumerate(hex_order):
    angle = math.radians(90 - i * 60)
    x = CENTER[0] + HEXAGON_RADIUS * math.cos(angle)
    y = CENTER[1] + HEXAGON_RADIUS * math.sin(angle)
    QR_POS[qr_id] = (round(x, 2), round(y, 2))

# Renkli alanlar rota üzerinde (dronelar üstünden geçecek)
# Rota: 1→4→2→3→5→6
# Mavi: segment 1→4 orta noktası,  Kırmızı: segment 4→2 orta noktası
_seg_1_4_mid = ((QR_POS[1][0] + QR_POS[4][0]) / 2, (QR_POS[1][1] + QR_POS[4][1]) / 2)
_seg_4_2_mid = ((QR_POS[4][0] + QR_POS[2][0]) / 2, (QR_POS[4][1] + QR_POS[2][1]) / 2)
LANDING_ZONES = {
    "blue": _seg_1_4_mid,
    "red": _seg_4_2_mid,
}

QR_SIZE = 1.2


def qr_model(qr_id, x, y):
    s = QR_SIZE
    tex = os.path.join(TEXTURE_DIR, f"qr{qr_id}.png")
    return f"""
    <model name="qr{qr_id}">
      <static>true</static>
      <pose>{x} {y} 0.01 0 0 0</pose>
      <link name="base">
        <visual name="qr_face">
          <geometry>
            <box><size>{s} {s} 0.02</size></box>
          </geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <pbr>
              <metal>
                <albedo_map>{tex}</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
        <collision name="col">
          <geometry><box><size>{s} {s} 0.02</size></box></geometry>
        </collision>
      </link>
    </model>
"""


def zone_model(name, x, y, r, g, b):
    sz = QR_SIZE
    color_name = "blue" if b > 0.5 else "red"
    tex = os.path.join(TEXTURE_DIR, f"zone_{color_name}.png")
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0.005 0 0 0</pose>
      <link name="link">
        <visual name="surface">
          <geometry><box><size>{sz} {sz} 0.01</size></box></geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <pbr>
              <metal>
                <albedo_map>{tex}</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
        <collision name="col">
          <geometry><box><size>{sz} {sz} 0.01</size></box></geometry>
        </collision>
      </link>
    </model>
"""


def make_lines():
    result = ""
    # Graf (gri)
    pairs = [(a, b) for a in range(1, 7) for b in range(a + 1, 7)]
    for a, b in pairs:
        x1, y1 = QR_POS[a]
        x2, y2 = QR_POS[b]
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        l = math.hypot(x2 - x1, y2 - y1)
        ang = math.atan2(y2 - y1, x2 - x1)
        result += f"""
    <model name="g_{a}_{b}"><static>true</static>
      <pose>{mx} {my} 0.002 0 0 {ang}</pose>
      <link name="l"><visual name="v">
        <geometry><box><size>{l} 0.04 0.002</size></box></geometry>
        <material><ambient>0.5 0.5 0.5 0.5</ambient><diffuse>0.5 0.5 0.5 0.5</diffuse></material>
      </visual></link>
    </model>
"""
    # Altıgen kenarlar (beyaz)
    ordered = [QR_POS[i] for i in [3, 4, 5, 6, 1, 2]]
    for i in range(6):
        x1, y1 = ordered[i]
        x2, y2 = ordered[(i + 1) % 6]
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        l = math.hypot(x2 - x1, y2 - y1)
        ang = math.atan2(y2 - y1, x2 - x1)
        result += f"""
    <model name="hex_{i}"><static>true</static>
      <pose>{mx} {my} 0.005 0 0 {ang}</pose>
      <link name="l"><visual name="v">
        <geometry><box><size>{l} 0.15 0.005</size></box></geometry>
        <material><ambient>1 1 1 0.8</ambient><diffuse>1 1 1 0.8</diffuse></material>
      </visual></link>
    </model>
"""
    # Rota (kırmızı)
    route = [1, 4, 2, 3, 5, 6]
    for i in range(len(route) - 1):
        a, b = route[i], route[i + 1]
        x1, y1 = QR_POS[a]
        x2, y2 = QR_POS[b]
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        l = math.hypot(x2 - x1, y2 - y1)
        ang = math.atan2(y2 - y1, x2 - x1)
        result += f"""
    <model name="route_{a}_{b}"><static>true</static>
      <pose>{mx} {my} 0.006 0 0 {ang}</pose>
      <link name="l"><visual name="v">
        <geometry><box><size>{l} 0.12 0.006</size></box></geometry>
        <material><ambient>0.9 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material>
      </visual></link>
    </model>
"""
    return result


# ─── QR Kod PNG Üretimi (config.py'deki QR_CONTENTS'tan) ───
def generate_qr_pngs():
    """Her QR için şartname formatında JSON içerikli PNG üret."""
    QR_PX = 512
    for qr_id, content in QR_CONTENTS.items():
        data = json.dumps(content, ensure_ascii=False, separators=(",", ":"))
        qr = qrcode.QRCode(
            error_correction=qrcode.constants.ERROR_CORRECT_H,
            box_size=10,
            border=4,
        )
        qr.add_data(data)
        qr.make(fit=True)
        img = qr.make_image(fill_color="black", back_color="white").convert("RGB")
        img = img.resize((QR_PX, QR_PX), Image.NEAREST)
        path = os.path.join(TEXTURE_DIR, f"qr{qr_id}.png")
        img.save(path)
        print(f"  ✓ QR{qr_id} PNG → {path}")

def generate_zone_pngs():
    """Kırmızı ve mavi renkli alan PNG'leri üret."""
    ZN_PX = 512
    for color_name, rgb in [("blue", (30, 80, 240)), ("red", (230, 40, 40))]:
        img = Image.new("RGB", (ZN_PX, ZN_PX), rgb)
        path = os.path.join(TEXTURE_DIR, f"zone_{color_name}.png")
        img.save(path)
        print(f"  ✓ {color_name} zone PNG → {path}")

print("=== QR Kod PNG'leri (şartname formatı) ===")
generate_qr_pngs()
print("=== Renkli Alan PNG'leri ===")
generate_zone_pngs()


# ─── World SDF ──────────────────────────────────────────────
qr_all = "".join(qr_model(q, *QR_POS[q]) for q in sorted(QR_POS))
zones = zone_model("zone_blue", *LANDING_ZONES["blue"], 0.1, 0.3, 0.95)
zones += zone_model("zone_red", *LANDING_ZONES["red"], 0.9, 0.15, 0.15)
lines = make_lines()

world_sdf = f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="default">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.6 0.8 1.0 1</background>
      <shadows>true</shadows>
    </scene>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
          <surface><friction><ode/></friction><bounce/><contact/></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
          <material>
            <ambient>0.35 0.48 0.25 1</ambient>
            <diffuse>0.35 0.48 0.25 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 0 0</pose>
        <inertial>
          <pose>0 0 0 0 0 0</pose>
          <mass>1</mass>
          <inertia><ixx>1</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1</iyy><iyz>0</iyz><izz>1</izz></inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 0 0</pose>
      <self_collide>false</self_collide>
    </model>

    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic>
      </attenuation>
      <spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot>
    </light>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>

    <!-- HOME -->
    <model name="home_marker">
      <static>true</static>
      <pose>0 0 0.005 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <geometry><cylinder><radius>2.0</radius><length>0.01</length></cylinder></geometry>
          <material><ambient>0.2 0.8 0.2 0.9</ambient><diffuse>0.2 0.8 0.2 0.9</diffuse></material>
        </visual>
      </link>
    </model>

{lines}
{qr_all}
{zones}
  </world>
</sdf>
"""

# Kaydet
arena_path = os.path.join(WORLD_DIR, "swarm_arena.sdf")
with open(arena_path, "w") as f:
    f.write(world_sdf)
print(f"✓ Arena world: {arena_path}")

# PX4 default.sdf'i değiştir (yedekle)
px4_world = os.path.join(
    BASE_DIR, "PX4-Autopilot", "Tools", "simulation", "gz", "worlds", "default.sdf"
)
backup = px4_world + ".backup"
if os.path.exists(px4_world):
    if not os.path.exists(backup):
        shutil.copy2(px4_world, backup)
        print(f"✓ Orijinal yedeklendi: {backup}")
    with open(px4_world, "w") as f:
        f.write(world_sdf)
    print(f"✓ PX4 default.sdf güncellendi!")

print(f"\nQR Pozisyonları:")
for q in sorted(QR_POS):
    print(f"  QR{q}: x={QR_POS[q][0]}, y={QR_POS[q][1]}")
print(f"\nİniş bölgeleri:")
for c, (x, y) in LANDING_ZONES.items():
    print(f"  {c}: x={x}, y={y}")
print(f"\n✅ Simülasyonu yeniden başlat: make px4_sitl gz_x500")
