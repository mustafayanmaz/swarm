"""
Gazebo world — Görev 5.2: Yarı Otonom Sürü Kontrolü

QR/renkli alan yok. Açık uçuş alanı + yer referansları:
  - Home marker (yeşil daire)
  - Grid çizgileri (hareketi takip etmek için)
  - Köşe markerları (sürü hareketini videoda göstermek için)
  - Pusula yönleri (N/S/E/W markerları)

PX4 default.sdf'i bu world ile değiştirir.
"""
import math
import os
import shutil
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WORLD_DIR = os.path.join(BASE_DIR, "gazebo", "worlds")
os.makedirs(WORLD_DIR, exist_ok=True)

# ─── Alan Boyutları ────────────────────────────────────────
AREA_SIZE = 80  # metre (80x80 uçuş alanı)
GRID_SPACING = 10  # metre arayla grid çizgileri


def grid_lines():
    """Yer grid çizgileri — hareketin videoda görünmesi için."""
    result = ""
    half = AREA_SIZE // 2

    # X ekseni boyunca çizgiler (East yönü)
    for y in range(-half, half + 1, GRID_SPACING):
        length = AREA_SIZE
        # Merkez çizgileri (eksenler) daha kalın ve farklı renk
        if y == 0:
            w, r, g, b = 0.08, 0.8, 0.2, 0.2  # Kırmızı (X ekseni = East)
        else:
            w, r, g, b = 0.03, 0.6, 0.6, 0.6  # Gri
        result += f"""
    <model name="grid_x_{y + half}"><static>true</static>
      <pose>0 {y} 0.002 0 0 0</pose>
      <link name="l"><visual name="v">
        <geometry><box><size>{length} {w} 0.002</size></box></geometry>
        <material><ambient>{r} {g} {b} 0.7</ambient><diffuse>{r} {g} {b} 0.7</diffuse></material>
      </visual></link>
    </model>
"""

    # Y ekseni boyunca çizgiler (North yönü)
    for x in range(-half, half + 1, GRID_SPACING):
        length = AREA_SIZE
        if x == 0:
            w, r, g, b = 0.08, 0.2, 0.2, 0.8  # Mavi (Y ekseni = North)
        else:
            w, r, g, b = 0.03, 0.6, 0.6, 0.6
        result += f"""
    <model name="grid_y_{x + half}"><static>true</static>
      <pose>{x} 0 0.002 0 0 1.5708</pose>
      <link name="l"><visual name="v">
        <geometry><box><size>{length} {w} 0.002</size></box></geometry>
        <material><ambient>{r} {g} {b} 0.7</ambient><diffuse>{r} {g} {b} 0.7</diffuse></material>
      </visual></link>
    </model>
"""
    return result


def direction_markers():
    """Pusula yönlerini gösteren markerlar (N/S/E/W) — videoda yön referansı."""
    result = ""
    half = AREA_SIZE // 2
    mark_dist = half - 2  # kenardan 2m içeride

    markers = [
        # (isim, x, y, renk r,g,b, boyut)
        ("north", 0, mark_dist, 0.1, 0.1, 0.9, 3.0),   # Mavi — Kuzey (Gazebo +Y)
        ("south", 0, -mark_dist, 0.9, 0.9, 0.1, 3.0),   # Sarı — Güney
        ("east", mark_dist, 0, 0.9, 0.1, 0.1, 3.0),      # Kırmızı — Doğu (Gazebo +X)
        ("west", -mark_dist, 0, 0.1, 0.9, 0.1, 3.0),     # Yeşil — Batı
    ]

    for name, x, y, r, g, b, sz in markers:
        # Üçgen marker (yön göstergesi)
        result += f"""
    <model name="dir_{name}"><static>true</static>
      <pose>{x} {y} 0.01 0 0 0</pose>
      <link name="l"><visual name="v">
        <geometry><box><size>{sz} {sz} 0.02</size></box></geometry>
        <material><ambient>{r} {g} {b} 0.9</ambient><diffuse>{r} {g} {b} 0.9</diffuse></material>
      </visual></link>
    </model>
"""
    return result


def distance_rings():
    """Home merkezinden 10m, 20m, 30m mesafe halkaları — mesafe referansı."""
    result = ""
    for radius in [10, 20, 30]:
        # Halkayı çok ince silindirle yaklaşık göster
        segments = 36
        for i in range(segments):
            angle = 2 * math.pi * i / segments
            angle2 = 2 * math.pi * (i + 1) / segments
            x1 = radius * math.cos(angle)
            y1 = radius * math.sin(angle)
            x2 = radius * math.cos(angle2)
            y2 = radius * math.sin(angle2)
            mx = (x1 + x2) / 2
            my = (y1 + y2) / 2
            seg_len = math.hypot(x2 - x1, y2 - y1)
            seg_ang = math.atan2(y2 - y1, x2 - x1)
            result += f"""
    <model name="ring_{radius}_{i}"><static>true</static>
      <pose>{mx} {my} 0.003 0 0 {seg_ang}</pose>
      <link name="l"><visual name="v">
        <geometry><box><size>{seg_len} 0.05 0.003</size></box></geometry>
        <material><ambient>0.9 0.9 0.9 0.5</ambient><diffuse>0.9 0.9 0.9 0.5</diffuse></material>
      </visual></link>
    </model>
"""
    return result


# ─── World SDF ──────────────────────────────────────────────
grid = grid_lines()
dirs = direction_markers()
rings = distance_rings()

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

    <!-- HOME MARKER (yeşil daire) -->
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

    <!-- HOME İÇ DAIRE (beyaz) -->
    <model name="home_inner">
      <static>true</static>
      <pose>0 0 0.006 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <geometry><cylinder><radius>0.5</radius><length>0.01</length></cylinder></geometry>
          <material><ambient>1 1 1 0.9</ambient><diffuse>1 1 1 0.9</diffuse></material>
        </visual>
      </link>
    </model>

{grid}
{dirs}
{rings}
  </world>
</sdf>
"""

# Kaydet
arena_path = os.path.join(WORLD_DIR, "semi_arena.sdf")
with open(arena_path, "w") as f:
    f.write(world_sdf)
print(f"✓ Semi-auto arena: {arena_path}")

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
    print(f"✓ PX4 default.sdf güncellendi (semi_arena)!")

print(f"\n✅ Görev 5.2 arena hazır!")
print(f"  Grid: {GRID_SPACING}m arayla, {AREA_SIZE}x{AREA_SIZE}m alan")
print(f"  Mesafe halkaları: 10m, 20m, 30m")
print(f"  Yön markerları: N(mavi), S(sarı), E(kırmızı), W(yeşil)")
