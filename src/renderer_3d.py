"""
Panda3D 3D Renderer — TEKNOFEST Sürü İHA
Mevcut sim_controller + mission_runner ile çalışır.
"""
import math
import numpy as np

from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenText import OnscreenText
from direct.gui.DirectGui import DirectFrame, DirectButton, DirectEntry
from direct.task import Task
from panda3d.core import (
    Vec3, Vec4, Point3, LColor,
    AmbientLight, DirectionalLight,
    CardMaker, TextNode, TextProperties,
    NodePath, LineSegs,
    GeomVertexFormat, GeomVertexData, GeomVertexWriter,
    Geom, GeomNode, GeomTriangles,
    AntialiasAttrib, TransparencyAttrib,
    WindowProperties, FrameBufferProperties, GraphicsPipe,
    Texture, PNMImage,
    CollisionTraverser, CollisionNode,
    BitMask32,
    GraphicsOutput,
    Lens, OrthographicLens, PerspectiveLens,
    Camera,
)

# ── Renkler ──────────────────────────────────────────────────
COL_GROUND   = Vec4(0.28, 0.51, 0.22, 1)
COL_GRID     = Vec4(0.24, 0.46, 0.19, 1)
COL_QR_WHITE = Vec4(0.96, 0.96, 0.96, 1)
COL_GRAPH    = Vec4(0.78, 0.24, 0.24, 1)
COL_ROUTE    = Vec4(0.63, 0.63, 0.63, 0.7)
COL_HOME     = Vec4(0.86, 0.78, 0.20, 1)
COL_FORM_CYAN = Vec4(0, 0.86, 0.86, 0.6)
COL_RED_ZONE = Vec4(0.86, 0.12, 0.12, 0.35)
COL_BLUE_ZONE = Vec4(0.12, 0.24, 0.86, 0.35)


def _make_box(sx=1, sy=1, sz=1, color=(0.3, 0.3, 0.3, 1)):
    """Basit renkli kutu (box) — pozisyon+renk vertex ile."""
    fmt = GeomVertexFormat.get_v3c4()
    vdata = GeomVertexData('box', fmt, Geom.UH_static)
    vertex = GeomVertexWriter(vdata, 'vertex')
    col_w = GeomVertexWriter(vdata, 'color')

    # 8 köşe
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    corners = [
        (-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
        (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz),
    ]
    for c in corners:
        vertex.addData3(*c)
        col_w.addData4(*color)

    tris = GeomTriangles(Geom.UH_static)
    faces = [
        (0,1,2), (0,2,3),  # alt
        (4,6,5), (4,7,6),  # üst
        (0,4,5), (0,5,1),  # ön
        (2,6,7), (2,7,3),  # arka
        (0,3,7), (0,7,4),  # sol
        (1,5,6), (1,6,2),  # sağ
    ]
    for f in faces:
        tris.addVertices(*f)

    geom = Geom(vdata)
    geom.addPrimitive(tris)
    node = GeomNode('box')
    node.addGeom(geom)
    return node


def _make_cylinder(radius=0.5, height=1.0, segments=16, color=(0.5, 0.5, 0.5, 1)):
    """Basit renkli silindir."""
    fmt = GeomVertexFormat.get_v3c4()
    vdata = GeomVertexData('cyl', fmt, Geom.UH_static)
    vertex = GeomVertexWriter(vdata, 'vertex')
    col_w = GeomVertexWriter(vdata, 'color')
    tris = GeomTriangles(Geom.UH_static)

    # Alt merkez (0), üst merkez (1)
    vertex.addData3(0, 0, 0)
    col_w.addData4(*color)
    vertex.addData3(0, 0, height)
    col_w.addData4(*color)

    idx = 2
    for i in range(segments):
        angle = 2 * math.pi * i / segments
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        # Alt çember
        vertex.addData3(x, y, 0)
        col_w.addData4(*color)
        # Üst çember
        vertex.addData3(x, y, height)
        col_w.addData4(*color)
        idx += 2

    for i in range(segments):
        b0 = 2 + i * 2
        b1 = 2 + ((i + 1) % segments) * 2
        t0 = b0 + 1
        t1 = b1 + 1
        # Alt yüz
        tris.addVertices(0, b1, b0)
        # Üst yüz
        tris.addVertices(1, t0, t1)
        # Yan yüz
        tris.addVertices(b0, b1, t1)
        tris.addVertices(b0, t1, t0)

    geom = Geom(vdata)
    geom.addPrimitive(tris)
    node = GeomNode('cyl')
    node.addGeom(geom)
    return node


def _make_disk(radius=1.0, segments=32, color=(0.5, 0.5, 0.5, 0.3)):
    """Düz disk (renk alanları için)."""
    fmt = GeomVertexFormat.get_v3c4()
    vdata = GeomVertexData('disk', fmt, Geom.UH_static)
    vertex = GeomVertexWriter(vdata, 'vertex')
    col_w = GeomVertexWriter(vdata, 'color')
    tris = GeomTriangles(Geom.UH_static)

    # Merkez
    vertex.addData3(0, 0, 0.02)
    col_w.addData4(*color)

    for i in range(segments):
        angle = 2 * math.pi * i / segments
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        vertex.addData3(x, y, 0.02)
        col_w.addData4(*color)

    for i in range(segments):
        tris.addVertices(0, 1 + i, 1 + (i + 1) % segments)

    geom = Geom(vdata)
    geom.addPrimitive(tris)
    node = GeomNode('disk')
    node.addGeom(geom)
    return node


class DroneModel:
    """Tek bir quadrotor 3D modeli."""

    def __init__(self, parent: NodePath, drone_id: int):
        self.root = parent.attachNewNode(f'drone_{drone_id}')
        self.drone_id = drone_id
        self._prop_angle = 0.0
        self._armed = True  # Arm/disarm durumu

        # Gövde (küçük kutu)
        body_node = _make_box(0.25, 0.25, 0.08, color=(0.16, 0.16, 0.18, 1))
        self.body = self.root.attachNewNode(body_node)

        # 4 kol + motor + pervane
        self.arms = []
        self.motors = []
        self.props = []
        arm_len = 0.35
        arm_angles = [math.pi / 4 + k * math.pi / 2 for k in range(4)]
        motor_colors = [
            (0.78, 0.24, 0.16, 1),  # Kırmızı — ön sağ
            (0.78, 0.24, 0.16, 1),  # Kırmızı — ön sol
            (0.3, 0.3, 0.32, 1),    # Gri — arka sağ
            (0.3, 0.3, 0.32, 1),    # Gri — arka sol
        ]

        for k, angle in enumerate(arm_angles):
            # Kol
            arm_node = _make_box(arm_len * 2, 0.04, 0.03, color=(0.32, 0.32, 0.34, 1))
            arm = self.root.attachNewNode(arm_node)
            mid_x = arm_len * math.cos(angle) / 2
            mid_y = arm_len * math.sin(angle) / 2
            arm.setPos(mid_x, mid_y, 0)
            arm.setH(-math.degrees(angle))
            self.arms.append(arm)

            # Motor
            mx = arm_len * math.cos(angle)
            my = arm_len * math.sin(angle)
            motor_node = _make_cylinder(0.06, 0.06, 12, color=motor_colors[k])
            motor = self.root.attachNewNode(motor_node)
            motor.setPos(mx, my, 0.04)
            self.motors.append(motor)

            # Pervane (ince kutu)
            prop_node = _make_box(0.28, 0.03, 0.005, color=(0.7, 0.7, 0.75, 0.6))
            prop = motor.attachNewNode(prop_node)
            prop.setPos(0, 0, 0.06)
            self.props.append(prop)

        # Burun göstergesi (küçük kırmızı kutu)
        nose_node = _make_box(0.08, 0.04, 0.04, color=(1, 0.3, 0.15, 1))
        nose = self.root.attachNewNode(nose_node)
        nose.setPos(0.2, 0, 0)

        # Arm/disarm LED göstergesi (küçük küre)
        led_node = _make_cylinder(0.04, 0.04, 8, color=(0.1, 0.9, 0.2, 1))
        self.led = self.root.attachNewNode(led_node)
        self.led.setPos(-0.15, 0, 0.08)

        # İD etiketi
        self.label = TextNode(f'drone_label_{drone_id}')
        self.label.setText(f'D{drone_id}')
        self.label.setTextColor(1, 1, 0.3, 1)
        self.label.setAlign(TextNode.ACenter)
        self.label_np = self.root.attachNewNode(self.label)
        self.label_np.setPos(0, 0, 0.5)
        self.label_np.setScale(0.3)
        self.label_np.setBillboardPointEye()

    def set_position(self, x, y, z):
        # Panda3D: X=sağ, Y=ileri, Z=yukarı
        # Sim: X=ileri, Y=sağ, Z=yukarı
        self.root.setPos(y, x, z)  # Sim X → Panda Y, Sim Y → Panda X

    def set_heading(self, heading_rad):
        # Sim heading: rad, +X'ten CCW
        # Panda3D H: degrees, +Y'den CW
        deg = -math.degrees(heading_rad) + 90
        self.root.setH(deg)

    def set_pitch_roll(self, pitch_rad, roll_rad):
        self.root.setP(math.degrees(pitch_rad))
        self.root.setR(-math.degrees(roll_rad))

    def set_active(self, active):
        if active:
            self.root.show()
        else:
            self.root.hide()

    def set_separated(self, separated, color_name=None):
        if separated and color_name:
            c = (0.86, 0.12, 0.12, 1) if color_name == 'red' else (0.12, 0.24, 0.86, 1)
            for m in self.motors:
                m.node().modifyGeom(0).modifyVertexData()  # Skip — too complex
            # Basitçe label güncelle
            self.label.setText(f'D{self.drone_id}>{color_name.upper()}')
            self.label.setTextColor(*c)
        else:
            self.label.setText(f'D{self.drone_id}')
            self.label.setTextColor(1, 1, 0.3, 1)

    def set_armed(self, armed: bool):
        """Arm/disarm durumunu ayarla — pervane + LED."""
        self._armed = armed
        if armed:
            # Yeşil LED, pervane dönecek
            led_node = _make_cylinder(0.04, 0.04, 8, color=(0.1, 0.9, 0.2, 1))
            new_led = self.root.attachNewNode(led_node)
            new_led.setPos(-0.15, 0, 0.08)
            self.led.removeNode()
            self.led = new_led
            for prop in self.props:
                prop.show()
        else:
            # Kırmızı LED, pervane durur
            led_node = _make_cylinder(0.04, 0.04, 8, color=(0.9, 0.1, 0.1, 1))
            new_led = self.root.attachNewNode(led_node)
            new_led.setPos(-0.15, 0, 0.08)
            self.led.removeNode()
            self.led = new_led
            for prop in self.props:
                prop.hide()

    def update_props(self, dt):
        if not self._armed:
            return  # Disarm → pervane dönmez
        self._prop_angle += dt * 800
        for prop in self.props:
            prop.setH(self._prop_angle)


class Renderer3D(ShowBase):
    """Panda3D 3D renderer — tam sahne ile."""

    def __init__(self, width=1280, height=720):
        ShowBase.__init__(self)

        # Pencere boyutu
        props = WindowProperties()
        props.setSize(width, height)
        props.setTitle('TEKNOFEST Sürü İHA — 3D Simülasyon')
        self.win.requestProperties(props)

        self.setBackgroundColor(0.55, 0.75, 0.95, 1)  # Gökyüzü mavi
        self.render.setAntialias(AntialiasAttrib.MAuto)

        # Kamera
        self.disableMouse()
        self.camera.setPos(0, -40, 30)
        self.camera.lookAt(15, 0, 0)
        self._cam_target = Vec3(15, 0, 0)
        self._cam_distance = 50
        self._cam_heading = -45
        self._cam_pitch = -35
        self._cam_dragging = False
        self._cam_last_mouse = None

        # Işıklar
        self._setup_lights()

        # Zemin
        self._create_ground()

        # Node'lar
        self.scene_root = self.render.attachNewNode('scene')
        self._line_nodes = []  # Temizlenebilir çizgi node'ları

        # Drone modelleri
        self.drone_models: dict[int, DroneModel] = {}

        # HUD
        self._setup_hud()

        # Mouse kontrolü
        self.accept('mouse2', self._on_mid_down)
        self.accept('mouse2-up', self._on_mid_up)
        self.accept('mouse3', self._on_right_down)
        self.accept('mouse3-up', self._on_right_up)
        self.accept('wheel_up', self._on_zoom_in)
        self.accept('wheel_down', self._on_zoom_out)
        self.accept('escape', self._quit_app)

        self._right_dragging = False
        self._should_quit = False
        self._auto_follow = True  # Kamera sürüyü takip etsin mi?

        self.accept('f', self._toggle_follow)
        self.accept('mouse1', self._on_left_down)
        self.accept('mouse1-up', self._on_left_up)
        self._left_dragging = False

        self.taskMgr.add(self._mouse_task, 'mouse_task')

    def _quit_app(self):
        self._should_quit = True

    def _toggle_follow(self):
        self._auto_follow = not self._auto_follow
        print(f'Kamera takip: {"ON" if self._auto_follow else "OFF"} (F tusu ile degistir)')

    def _on_left_down(self):
        self._left_dragging = True
        self._auto_follow = False  # Kullanici surukleyince takibi kapat
        if self.mouseWatcherNode.hasMouse():
            self._cam_last_mouse = (
                self.mouseWatcherNode.getMouseX(),
                self.mouseWatcherNode.getMouseY(),
            )

    def _on_left_up(self):
        self._left_dragging = False

    def should_quit(self):
        return self._should_quit

    def _setup_lights(self):
        # Ambient
        alight = AmbientLight('ambient')
        alight.setColor(Vec4(0.35, 0.35, 0.4, 1))
        self.render.setLight(self.render.attachNewNode(alight))

        # Directional (güneş)
        dlight = DirectionalLight('sun')
        dlight.setColor(Vec4(0.9, 0.85, 0.75, 1))
        dlnp = self.render.attachNewNode(dlight)
        dlnp.setHpr(-45, -50, 0)
        self.render.setLight(dlnp)

        # İkincil dolgu ışığı
        dlight2 = DirectionalLight('fill')
        dlight2.setColor(Vec4(0.3, 0.35, 0.4, 1))
        dlnp2 = self.render.attachNewNode(dlight2)
        dlnp2.setHpr(135, -30, 0)
        self.render.setLight(dlnp2)

    def _create_ground(self):
        """Yeşil zemin düzlemi + grid çizgileri."""
        # Büyük yeşil kare
        cm = CardMaker('ground')
        cm.setFrame(-100, 100, -100, 100)
        cm.setColor(COL_GROUND)
        ground = self.render.attachNewNode(cm.generate())
        ground.setP(-90)  # Yatay yap
        ground.setPos(0, 0, 0)

        # Grid çizgileri
        ls = LineSegs('grid')
        ls.setColor(COL_GRID)
        ls.setThickness(1)
        for i in range(-80, 81, 10):
            # Sim X → Panda Y, Sim Y → Panda X
            ls.moveTo(i, -80, 0.01)
            ls.drawTo(i, 80, 0.01)
            ls.moveTo(-80, i, 0.01)
            ls.drawTo(80, i, 0.01)
        self.render.attachNewNode(ls.create())

    def _setup_hud(self):
        """Ekran üstü bilgi metinleri — kenar yapışık düzen."""
        # aspect2d: 1280x720 (16:9) → X: -1.78..+1.78, Z: -1..+1
        L = -1.75   # Sol kenar
        R = 1.75    # Sağ kenar
        T = 0.95    # Üst kenar
        B = -0.97   # Alt kenar

        # ── Sol üst köşe: Durum bilgileri ──
        self.hud_state = OnscreenText(
            text='', pos=(L, T), scale=0.05,
            fg=(1, 1, 0.2, 1), shadow=(0, 0, 0, 0.7),
            align=TextNode.ALeft, mayChange=True,
        )
        self.hud_info = OnscreenText(
            text='', pos=(L, T - 0.065), scale=0.035,
            fg=(0.9, 0.9, 0.9, 1), shadow=(0, 0, 0, 0.7),
            align=TextNode.ALeft, mayChange=True,
        )
        self.hud_info2 = OnscreenText(
            text='', pos=(L, T - 0.115), scale=0.035,
            fg=(0.9, 0.9, 0.9, 1), shadow=(0, 0, 0, 0.7),
            align=TextNode.ALeft, mayChange=True,
        )

        # ── Sağ üst köşe: Kronometre ──
        self.hud_chrono = OnscreenText(
            text='00:00', pos=(R, T), scale=0.065,
            fg=(1, 1, 1, 1), shadow=(0, 0, 0, 0.8),
            align=TextNode.ARight, mayChange=True,
        )
        # ── Sağ üst: Scan bilgisi ──
        self.hud_scan = OnscreenText(
            text='', pos=(R, T - 0.075), scale=0.03,
            fg=(0.2, 1, 0.3, 1), shadow=(0, 0, 0, 0.7),
            align=TextNode.ARight, mayChange=True,
        )

        # ── Sol alt köşe: GCS + Arm ──
        self.hud_gcs = OnscreenText(
            text='GCS: BAGLI', pos=(L, B + 0.04), scale=0.032,
            fg=(0.2, 1, 0.3, 1), shadow=(0, 0, 0, 0.8),
            align=TextNode.ALeft, mayChange=True,
        )
        self.hud_arm = OnscreenText(
            text='', pos=(L, B), scale=0.026,
            fg=(0.9, 0.9, 0.9, 1), shadow=(0, 0, 0, 0.8),
            align=TextNode.ALeft, mayChange=True,
        )

        # ── QR Görev Log — sol kenar, durum bilgilerinin altı ──
        self._qr_log_visible = True
        self._qr_log_entries = []
        # Panel sol kenara yapışık
        qr_l = L       # Sol kenar
        qr_top = T - 0.18  # Durum yazılarının hemen altı
        pw = 0.65       # Panel genişliği
        # Arka plan
        self._qr_log_frame = DirectFrame(
            frameSize=(0, pw, -0.14, 0.04),
            frameColor=(0.06, 0.06, 0.08, 0.88),
            pos=(qr_l, 0, qr_top),
            parent=self.aspect2d,
        )
        self._qr_log_frame.setTransparency(TransparencyAttrib.MAlpha)
        # Üst çizgi (accent)
        accent = DirectFrame(
            frameSize=(0, pw, 0, 0.003),
            frameColor=(1, 0.8, 0.1, 0.9),
            pos=(0, 0, 0.04),
            parent=self._qr_log_frame,
        )
        # Başlık
        self._qr_log_title = OnscreenText(
            text='QR GOREV LOG  [Q]',
            pos=(qr_l + 0.02, qr_top + 0.015), scale=0.024,
            fg=(1, 0.85, 0.15, 1), shadow=(0, 0, 0, 0.8),
            align=TextNode.ALeft, mayChange=True, parent=self.aspect2d,
        )
        # İçerik
        self._qr_log_text = OnscreenText(
            text='  Henuz QR tarlanmadi',
            pos=(qr_l + 0.02, qr_top - 0.02), scale=0.022,
            fg=(0.75, 0.85, 0.75, 1), shadow=(0, 0, 0, 0.7),
            align=TextNode.ALeft, mayChange=True, parent=self.aspect2d,
            wordwrap=40,
        )
        self._qr_log_left = qr_l
        self._qr_log_pw = pw
        self.accept('q', self._toggle_qr_log)

    # ── Start Screen ──────────────────────────────────────────

    def show_start_screen(self, callback, initial_altitude=1.0):
        """Görev başlatma ekranı — büyük buton + başlık + kalıcı kontrol."""
        self._mission_callback = callback
        self._mission_running = False
        self._initial_altitude = initial_altitude
        self._show_start_overlay()

    def _show_start_overlay(self):
        """Başlangıç overlay'ini oluştur/göster."""
        # Varsa eski overlay'i temizle
        self.hide_start_screen()
        self._start_overlay = DirectFrame(
            frameSize=(-2, 2, -2, 2),
            frameColor=(0, 0, 0, 0.6),
            parent=self.aspect2d,
        )
        # Başlık
        self._start_title = OnscreenText(
            text='TEKNOFEST 2026\nSÜRÜ İHA SİMÜLASYONU',
            pos=(0, 0.35), scale=0.09,
            fg=(1, 1, 1, 1), shadow=(0, 0, 0, 0.9),
            align=TextNode.ACenter, parent=self._start_overlay,
        )
        self._start_subtitle = OnscreenText(
            text='Görev 1: Dinamik Sürü Kabiliyeti',
            pos=(0, 0.18), scale=0.055,
            fg=(0.8, 0.8, 0.8, 1), shadow=(0, 0, 0, 0.8),
            align=TextNode.ACenter, parent=self._start_overlay,
        )
        # Başlat butonu (büyük overlay)
        self._start_btn = DirectButton(
            text='GÖREVE BAŞLA',
            scale=0.1,
            pos=(0, 0, -0.15),
            frameSize=(-3.5, 3.5, -0.8, 1.2),
            frameColor=(0.12, 0.62, 0.15, 1),
            text_fg=(1, 1, 1, 1),
            text_shadow=(0, 0, 0, 0.8),
            text_scale=0.7,
            relief='flat',
            command=self._on_start_click,
            parent=self._start_overlay,
        )
        # İrtifa giriş alanı
        self._alt_label = OnscreenText(
            text='Kalkis Irtifasi (m):',
            pos=(-0.22, 0.02), scale=0.04,
            fg=(0.9, 0.9, 0.9, 1), shadow=(0, 0, 0, 0.8),
            align=TextNode.ARight, parent=self._start_overlay,
        )
        self._alt_entry = DirectEntry(
            text='',
            scale=0.06,
            pos=(-0.15, 0, 0.0),
            width=4,
            numLines=1,
            initialText=str(self._initial_altitude),
            frameColor=(0.15, 0.15, 0.2, 0.9),
            text_fg=(1, 1, 0.3, 1),
            cursorKeys=True,
            parent=self._start_overlay,
        )
        self._start_info = OnscreenText(
            text='Orta tık=döndür | Sağ tık=kaydır | Scroll=zoom | G=GCS kes | Q=QR log',
            pos=(0, -0.25), scale=0.032,
            fg=(0.6, 0.6, 0.6, 1),
            align=TextNode.ACenter, parent=self._start_overlay,
        )

        # ── Kalıcı küçük kontrol butonu (her zaman ekranın üstünde) ──
        self._ctrl_btn = DirectButton(
            text='>> BASLAT',
            scale=0.04,
            pos=(-0.18, 0, 0.95),
            frameSize=(-2.8, 2.8, -0.7, 1.0),
            frameColor=(0.12, 0.62, 0.15, 1),
            text_fg=(1, 1, 1, 1),
            text_shadow=(0, 0, 0, 0.7),
            text_scale=0.6,
            relief='flat',
            command=self._on_ctrl_click,
            parent=self.aspect2d,
        )
        # ── Yeniden Başlat butonu ──
        self._restart_btn = DirectButton(
            text='(R) YENIDEN BASLAT',
            scale=0.034,
            pos=(0.26, 0, 0.955),
            frameSize=(-3.6, 3.6, -0.7, 1.0),
            frameColor=(0.15, 0.15, 0.6, 1),
            text_fg=(1, 1, 1, 1),
            text_shadow=(0, 0, 0, 0.7),
            text_scale=0.6,
            relief='flat',
            command=self._on_restart_click,
            parent=self.aspect2d,
        )
        self._restart_btn.hide()  # Görev başlayana kadar gizli
        self._restart_callback = None
        self._mission_done = False
        self.accept('r', self._on_restart_click)

    def _on_start_click(self):
        """Overlay BAŞLA butonuna tıklama."""
        # İrtifa değerini oku
        alt = self._get_entered_altitude()
        self.hide_start_screen()
        self._mission_running = True
        self._mission_done = False
        self._ctrl_btn['text'] = '|| DURDUR'
        self._ctrl_btn['frameColor'] = (0.75, 0.15, 0.12, 1)
        self._restart_btn.show()
        if self._mission_callback:
            self._mission_callback(alt)

    def _get_entered_altitude(self):
        """Entry'den irtifa değerini oku, geçersizse default dön."""
        try:
            val = float(self._alt_entry.get().strip())
            if val < 0.5:
                val = 0.5
            elif val > 30.0:
                val = 30.0
            return val
        except (ValueError, AttributeError):
            return self._initial_altitude

    def _on_ctrl_click(self):
        """Kalıcı kontrol butonu — Başlat / Durdur / Devam / Yeniden Başlat."""
        if self._mission_done:
            # Görev bittiyse bu buton yeniden başlatır
            self._on_restart_click()
            return
        if not self._mission_running:
            alt = self._get_entered_altitude()
            self.hide_start_screen()
            self._mission_running = True
            self._ctrl_btn['text'] = '|| DURDUR'
            self._ctrl_btn['frameColor'] = (0.75, 0.15, 0.12, 1)
            self._restart_btn.show()
            if self._mission_callback:
                self._mission_callback(alt)
        else:
            self._mission_running = False
            self._ctrl_btn['text'] = '>> DEVAM ET'
            self._ctrl_btn['frameColor'] = (0.12, 0.62, 0.15, 1)

    def _on_restart_click(self):
        """Görevi sıfırla → başlangıç ekranını göster."""
        # Önce data sıfırla
        if self._restart_callback:
            self._restart_callback()
        # QR log sıfırla
        self.reset_qr_log()
        # Haberleşme log sıfırla
        if hasattr(self, '_notify_lines'):
            self._notify_lines = []
            self._notify_text.setText('')
        # UI'ı başlangıç durumuna getir
        self._mission_running = False
        self._mission_done = False
        self._restart_btn.hide()
        self._ctrl_btn['text'] = '>> BASLAT'
        self._ctrl_btn['frameColor'] = (0.12, 0.62, 0.15, 1)
        # Başlangıç overlay'ini tekrar göster
        self._show_start_overlay()

    def set_restart_callback(self, cb):
        """Yeniden başlatma callback'i ata."""
        self._restart_callback = cb

    def set_mission_done(self):
        """Görev tamamlandığında ctrl butonunu güncelle."""
        self._mission_done = True
        self._mission_running = False
        self._ctrl_btn['text'] = '(R) YENIDEN BASLAT'
        self._ctrl_btn['frameColor'] = (0.15, 0.15, 0.6, 1)

    def is_mission_paused(self):
        """Görev duraklatılmış mı?"""
        return hasattr(self, '_mission_running') and not self._mission_running

    def hide_start_screen(self):
        """Başlatma overlay ekranını kaldır."""
        if hasattr(self, '_start_overlay') and self._start_overlay:
            self._start_overlay.destroy()
            self._start_overlay = None

    # ── QR Görev Log paneli ───────────────────────────────────

    def _toggle_qr_log(self):
        """Q tuşu ile QR görev log panelini aç/kapat."""
        self._qr_log_visible = not self._qr_log_visible
        if self._qr_log_visible:
            self._qr_log_frame.show()
            self._qr_log_title.show()
            self._qr_log_text.show()
        else:
            self._qr_log_frame.hide()
            self._qr_log_title.hide()
            self._qr_log_text.hide()

    def add_qr_log(self, qr_id, summary):
        """QR tarama sonucunu log paneline ekle (renkli bullet)."""
        self._qr_log_entries.append((qr_id, summary))
        lines = []
        for qid, txt in self._qr_log_entries:
            lines.append(f'  > QR{qid}: {txt}')
        self._qr_log_text.setText('\n'.join(lines))
        n = len(lines)
        h = max(0.10, n * 0.032 + 0.05)
        pw = self._qr_log_pw
        self._qr_log_frame['frameSize'] = (0, pw, -h, 0.04)

    def reset_qr_log(self):
        """QR log panelini sıfırla."""
        self._qr_log_entries = []
        self._qr_log_text.setText('  Henuz QR tarlanmadi')
        pw = self._qr_log_pw
        self._qr_log_frame['frameSize'] = (0, pw, -0.10, 0.04)

    # ── Kamera kontrolü ──────────────────────────────────────

    def _on_mid_down(self):
        self._cam_dragging = True
        if self.mouseWatcherNode.hasMouse():
            self._cam_last_mouse = (
                self.mouseWatcherNode.getMouseX(),
                self.mouseWatcherNode.getMouseY(),
            )

    def _on_mid_up(self):
        self._cam_dragging = False

    def _on_right_down(self):
        self._right_dragging = True
        if self.mouseWatcherNode.hasMouse():
            self._cam_last_mouse = (
                self.mouseWatcherNode.getMouseX(),
                self.mouseWatcherNode.getMouseY(),
            )

    def _on_right_up(self):
        self._right_dragging = False

    def _on_zoom_in(self):
        self._cam_distance = max(8, self._cam_distance * 0.88)
        self._update_camera()

    def _on_zoom_out(self):
        self._cam_distance = min(150, self._cam_distance / 0.88)
        self._update_camera()

    def _mouse_task(self, task):
        if not self.mouseWatcherNode.hasMouse():
            return Task.cont
        mx = self.mouseWatcherNode.getMouseX()
        my = self.mouseWatcherNode.getMouseY()

        if (self._cam_dragging or self._left_dragging) and self._cam_last_mouse:
            dx = mx - self._cam_last_mouse[0]
            dy = my - self._cam_last_mouse[1]
            self._cam_heading -= dx * 120
            self._cam_pitch = max(-85, min(-5, self._cam_pitch - dy * 80))
            self._update_camera()

        if self._right_dragging and self._cam_last_mouse:
            dx = mx - self._cam_last_mouse[0]
            dy = my - self._cam_last_mouse[1]
            # Pan
            h_rad = math.radians(self._cam_heading)
            self._cam_target[0] -= (dx * math.cos(h_rad) + dy * math.sin(h_rad)) * self._cam_distance * 0.3
            self._cam_target[1] -= (-dx * math.sin(h_rad) + dy * math.cos(h_rad)) * self._cam_distance * 0.3
            self._update_camera()

        self._cam_last_mouse = (mx, my)
        return Task.cont

    def _update_camera(self):
        h_rad = math.radians(self._cam_heading)
        p_rad = math.radians(self._cam_pitch)
        d = self._cam_distance
        cam_x = self._cam_target[0] + d * math.cos(p_rad) * math.sin(h_rad)
        cam_y = self._cam_target[1] - d * math.cos(p_rad) * math.cos(h_rad)
        cam_z = max(2, self._cam_target[2] - d * math.sin(p_rad))
        self.camera.setPos(cam_x, cam_y, cam_z)
        self.camera.lookAt(self._cam_target[0], self._cam_target[1], self._cam_target[2])

    # ── Sahne nesneleri ───────────────────────────────────────

    def create_drones(self, num_drones):
        for i in range(num_drones):
            dm = DroneModel(self.scene_root, i)
            self.drone_models[i] = dm

    def create_qr_markers(self, qr_list, qr_manager=None):
        """QR pozisyonlarına 3D QR pano yerleştir — gerçek QR texture ile."""
        for qr in qr_list:
            pos = qr['position']
            qid = qr['id']
            # Sim X → Panda Y, Sim Y → Panda X
            px, py, pz = pos[1], pos[0], 0.01

            qr_size = 1.2  # 120cm x 120cm — şartname ölçüsü

            # Siyah çerçeve (biraz büyük)
            cm2 = CardMaker(f'qr_border_{qid}')
            b = qr_size / 2 + 0.08
            cm2.setFrame(-b, b, -b, b)
            cm2.setColor(Vec4(0.08, 0.08, 0.08, 1))
            border_card = self.scene_root.attachNewNode(cm2.generate())
            border_card.setP(-90)
            border_card.setPos(px, py, 0.02)

            # QR panosu — gerçek QR texture
            cm = CardMaker(f'qr_bg_{qid}')
            cm.setFrame(-qr_size/2, qr_size/2, -qr_size/2, qr_size/2)
            qr_card = self.scene_root.attachNewNode(cm.generate())
            qr_card.setP(-90)
            qr_card.setPos(px, py, 0.03)

            # PIL image -> Panda3D texture
            tex = self._pil_to_texture(qr_manager, qid)
            if tex is not None:
                qr_card.setTexture(tex)
            else:
                qr_card.setColor(COL_QR_WHITE)

            # QR Id etiketi (havada)
            label = TextNode(f'qr_label_{qid}')
            label.setText(f'QR{qid}')
            label.setTextColor(0.96, 0.96, 0.96, 1)
            label.setAlign(TextNode.ACenter)
            label.setCardColor(0.2, 0.2, 0.2, 0.8)
            label.setCardAsMargin(0.15, 0.15, 0.08, 0.08)
            label_np = self.scene_root.attachNewNode(label)
            label_np.setPos(px, py, 2.5)
            label_np.setScale(0.6)
            label_np.setBillboardPointEye()

    def _pil_to_texture(self, qr_manager, qid):
        """QRManager'dan PIL imajini al, Panda3D Texture'a donustur."""
        if qr_manager is None:
            return None
        qr_data = qr_manager.get_qr(qid)
        if qr_data is None:
            return None
        pil_img = qr_data.pil_image.convert('RGBA')
        w, h = pil_img.size
        tex = Texture(f'qr_tex_{qid}')
        tex.setup2dTexture(w, h, Texture.T_unsigned_byte, Texture.F_rgba8)
        # PIL -> PNMImage
        pnm = PNMImage(w, h, 4)
        pixels = pil_img.load()
        for y in range(h):
            for x in range(w):
                r, g, b, a = pixels[x, y]
                pnm.setXelA(x, y, r / 255.0, g / 255.0, b / 255.0, a / 255.0)
        tex.load(pnm)
        tex.setMagfilter(Texture.FT_nearest)  # QR kodlarda keskin piksel
        tex.setMinfilter(Texture.FT_nearest)
        return tex

    def create_home(self, position):
        """Home platform."""
        px, py = position[1], position[0]  # Sim → Panda
        disk_node = _make_disk(1.5, 32, (0.86, 0.78, 0.2, 0.9))
        disk = self.scene_root.attachNewNode(disk_node)
        disk.setPos(px, py, 0.02)
        disk.setTransparency(TransparencyAttrib.MAlpha)

        # Kenar halkası
        disk2_node = _make_disk(1.8, 32, (0.24, 0.24, 0.24, 0.9))
        disk2 = self.scene_root.attachNewNode(disk2_node)
        disk2.setPos(px, py, 0.01)
        disk2.setTransparency(TransparencyAttrib.MAlpha)

        label = TextNode('home_label')
        label.setText('HOME')
        label.setTextColor(0.24, 0.24, 0.24, 1)
        label.setAlign(TextNode.ACenter)
        lnp = self.scene_root.attachNewNode(label)
        lnp.setPos(px, py, 0.05)
        lnp.setScale(0.5)
        lnp.setP(-90)
        lnp.setH(90)

    def create_color_zones(self, zones):
        """Renk alanları diskleri."""
        for z in zones:
            pos = z['position']
            px, py = pos[1], pos[0]
            radius = z.get('radius', 1.5)
            if z['color'] == 'red':
                c = (0.86, 0.12, 0.12, 0.4)
            else:
                c = (0.12, 0.24, 0.86, 0.4)

            disk_node = _make_disk(radius, 32, c)
            disk = self.scene_root.attachNewNode(disk_node)
            disk.setPos(px, py, 0.02)
            disk.setTransparency(TransparencyAttrib.MAlpha)

            # Kenar
            edge_c = (0.86, 0.12, 0.12, 0.9) if z['color'] == 'red' else (0.12, 0.24, 0.86, 0.9)
            edge_node = _make_disk(radius, 32, edge_c)
            edge = self.scene_root.attachNewNode(edge_node)
            edge.setPos(px, py, 0.025)
            edge.setScale(1, 1, 1)
            edge.setTransparency(TransparencyAttrib.MAlpha)

            # Etiket
            label = TextNode(f'zone_label_{z["color"]}')
            label.setText(z['color'].upper())
            label.setTextColor(*edge_c)
            label.setAlign(TextNode.ACenter)
            lnp = self.scene_root.attachNewNode(label)
            lnp.setPos(px, py, 1.5)
            lnp.setScale(0.4)
            lnp.setBillboardPointEye()

    def draw_graph_lines(self, qr_positions):
        """QR'lar arası kırmızı graf çizgileri."""
        if len(qr_positions) < 2:
            return
        ls = LineSegs('graph')
        ls.setColor(COL_GRAPH)
        ls.setThickness(2)
        n = len(qr_positions)
        for i in range(n):
            for j in range(i + 1, n):
                # Sim → Panda
                ls.moveTo(qr_positions[i][1], qr_positions[i][0], 0.05)
                ls.drawTo(qr_positions[j][1], qr_positions[j][0], 0.05)
        node = self.scene_root.attachNewNode(ls.create())
        self._line_nodes.append(node)

    def draw_route_lines(self, waypoints):
        """Rota çizgileri (gri)."""
        if len(waypoints) < 2:
            return
        ls = LineSegs('route')
        ls.setColor(COL_ROUTE)
        ls.setThickness(3)
        for i in range(len(waypoints) - 1):
            ls.moveTo(waypoints[i][1], waypoints[i][0], 0.06)
            ls.drawTo(waypoints[i + 1][1], waypoints[i + 1][0], 0.06)
        node = self.scene_root.attachNewNode(ls.create())
        node.setTransparency(TransparencyAttrib.MAlpha)
        self._line_nodes.append(node)

    def clear_dynamic_lines(self):
        """Her frame temizlenecek çizgiler."""
        for node in self._line_nodes:
            node.removeNode()
        self._line_nodes = []

    def draw_formation_lines(self, drone_positions):
        """Aktif drone'lar arası formasyon çizgileri."""
        if len(drone_positions) < 2:
            return
        ls = LineSegs('formation')
        ls.setColor(COL_FORM_CYAN)
        ls.setThickness(2)
        n = len(drone_positions)
        for i in range(n):
            j = (i + 1) % n
            # Sim → Panda
            p1 = drone_positions[i]
            p2 = drone_positions[j]
            ls.moveTo(p1[1], p1[0], p1[2] + 0.05)
            ls.drawTo(p2[1], p2[0], p2[2] + 0.05)
        node = self.scene_root.attachNewNode(ls.create())
        node.setTransparency(TransparencyAttrib.MAlpha)
        self._line_nodes.append(node)

    # ── HUD güncelle ──────────────────────────────────────────

    def update_hud(self, state, formation_name, spacing, num_active, num_total,
                   heading_deg, altitude, sim_time, done=False,
                   scan_text='', arm_states=None):
        state_text = 'GOREV TAMAMLANDI' if done else state
        self.hud_state.setText(f'[{state_text}]')
        self.hud_info.setText(
            f'Formasyon: {formation_name.upper()}  |  Spacing: {spacing:.1f}m  |  '
            f'Heading: {heading_deg:.0f}'
        )
        self.hud_info2.setText(
            f'Drone: {num_active}/{num_total}  |  Irtifa: {altitude:.1f}m'
        )

        mins = int(sim_time) // 60
        secs = int(sim_time) % 60
        self.hud_chrono.setText(f'{mins:02d}:{secs:02d}')

        self.hud_scan.setText(scan_text)

        # Arm/disarm durumu
        if arm_states is not None:
            parts = []
            for i, armed in enumerate(arm_states):
                s = 'ARM' if armed else 'DISARM'
                parts.append(f'D{i}:{s}')
            self.hud_arm.setText('  '.join(parts))

    def set_gcs_status(self, connected: bool):
        """GCS bağlantı durumunu HUD'da göster."""
        if connected:
            self.hud_gcs.setText('GCS: BAĞLI')
            self.hud_gcs.setFg((0.2, 1, 0.3, 1))
        else:
            self.hud_gcs.setText('GCS: KESİLDİ — OTONOM')
            self.hud_gcs.setFg((1, 0.3, 0.15, 1))

    # ── Kamera takibi ─────────────────────────────────────────

    def follow_swarm(self, centroid):
        """Kamerayı sürü centroid'ine yumuşak takip ettir (auto_follow acikken)."""
        if not self._auto_follow:
            return
        # Sim → Panda
        target = Vec3(centroid[1], centroid[0], centroid[2])
        # Yumuşak geçiş
        self._cam_target = self._cam_target * 0.95 + target * 0.05
        self._update_camera()

    # ── Drone Kamera Sistemi (per-drone) ────────────────────────

    def setup_drone_cameras(self, num_drones, buf_size=192):
        """
        Her drone icin ayri off-screen kamera + HUD'da yan yana eşit boyutlu PIP'ler.
        FOV = 90 derece (max goruntu acisi).
        Tüm kameralar aynı anda ekranda görünür.
        """
        self._num_cam_drones = num_drones
        self._cam_buf_size = buf_size
        self._drone_cams = {}       # drone_id -> dict of cam objects
        self._active_cam_id = 0     # Hangi drone cam büyük gösterilecek (TAB)
        self._cam_notify_log = []   # [(sim_time, drone_id, text), ...]

        # Tab ile aktif kamera değiştir (tespit overlay için)
        self.accept('tab', self._cycle_active_cam)

        # ── PIP yerleşim — tüm kameralar eşit boyutlu yan yana ──
        cam_w, cam_h = 0.30, 0.25
        gap = 0.03
        total_w = num_drones * cam_w + (num_drones - 1) * gap
        start_x = 1.30 - total_w  # Sağ tarafa yasla
        base_y = -0.92 + cam_h / 2

        for i in range(num_drones):
            cam_data = {}

            # Off-screen buffer
            buf = self.win.makeTextureBuffer(f'droneCam_{i}', buf_size, buf_size)
            buf.setSort(-100 - i)
            buf.setClearColorActive(True)
            buf.setClearColor(Vec4(0.08, 0.08, 0.08, 1))

            # Kamera — aşağı bakan perspektif, FOV = 90°
            lens = PerspectiveLens()
            lens.setFov(90)
            lens.setNearFar(0.1, 100)

            cam_node = Camera(f'droneCam_{i}')
            cam_node.setLens(lens)
            cam_np = self.render.attachNewNode(cam_node)
            cam_np.setPos(0, 0, 5)
            cam_np.setHpr(0, -90, 0)  # Aşağı bak

            dr = buf.makeDisplayRegion()
            dr.setCamera(cam_np)

            tex = buf.getTexture()
            tex.setMagfilter(Texture.FT_linear)
            tex.setMinfilter(Texture.FT_linear)

            cam_data['buffer'] = buf
            cam_data['lens'] = lens
            cam_data['cam_np'] = cam_np
            cam_data['texture'] = tex
            cam_data['detection'] = None

            # ── Kamera PIP konumu ──
            cx = start_x + i * (cam_w + gap) + cam_w / 2
            cy = base_y

            # Border (tespit rengi değişir)
            border = DirectFrame(
                frameSize=(-cam_w / 2 - 0.008, cam_w / 2 + 0.008,
                           -cam_h / 2 - 0.008, cam_h / 2 + 0.008),
                frameColor=(0.15, 0.15, 0.15, 0.95),
                pos=(cx, 0, cy),
                parent=self.aspect2d,
            )
            cam_data['border'] = border

            # Kamera card
            cm = CardMaker(f'pip_cam_{i}')
            cm.setFrame(-cam_w / 2, cam_w / 2, -cam_h / 2, cam_h / 2)
            cm.setUvRange((0, 0), (1, 1))
            card_np = self.aspect2d.attachNewNode(cm.generate())
            card_np.setPos(cx, 0, cy)
            card_np.setTexture(tex)
            cam_data['card'] = card_np
            cam_data['pip_pos'] = (cx, cy)
            cam_data['pip_size'] = (cam_w, cam_h)

            # Drone ID label (sol üst)
            lbl = OnscreenText(
                text=f'D{i} CAM', pos=(cx - cam_w / 2 + 0.01, cy + cam_h / 2 - 0.03),
                scale=0.028, fg=(0.1, 1, 0.2, 1), shadow=(0, 0, 0, 0.8),
                align=TextNode.ALeft, mayChange=True, parent=self.aspect2d,
            )
            cam_data['label'] = lbl

            # Tespit durumu etiketi (alt orta)
            det_lbl = OnscreenText(
                text='', pos=(cx, cy - cam_h / 2 + 0.02),
                scale=0.024, fg=(0, 1, 0.3, 1), shadow=(0, 0, 0, 0.9),
                align=TextNode.ACenter, mayChange=True, parent=self.aspect2d,
            )
            cam_data['det_label'] = det_lbl

            # Crosshair
            ls = LineSegs(f'crosshair_{i}')
            ls.setColor(0, 1, 0.3, 0.5)
            ls.setThickness(1)
            ch = 0.018
            ls.moveTo(-ch, 0, 0); ls.drawTo(ch, 0, 0)
            ls.moveTo(0, 0, -ch); ls.drawTo(0, 0, ch)
            xhair = self.aspect2d.attachNewNode(ls.create())
            xhair.setPos(cx, 0, cy)
            cam_data['crosshair'] = xhair

            self._drone_cams[i] = cam_data

        # Haberleşme log metni (sol alt, GCS üstü)
        self._notify_text = OnscreenText(
            text='', pos=(-1.75, -0.76), scale=0.026,
            fg=(0.2, 1, 0.4, 1), shadow=(0, 0, 0, 0.8),
            align=TextNode.ALeft, mayChange=True, parent=self.aspect2d,
        )

        # Uyumluluk değişkenleri (eski API)
        self._pip_qr_box = None
        self._pip_color_box = None
        self._pip_pos = (start_x + cam_w / 2, base_y)
        self._pip_size = (cam_w, cam_h)

    def _cycle_active_cam(self):
        """TAB ile aktif kamerayı değiştir (tespit overlay için)."""
        self._active_cam_id = (self._active_cam_id + 1) % self._num_cam_drones

    def update_drone_camera(self, drone_id, sim_pos, heading_rad, altitude):
        """Belirli bir drone'un kamerasını güncelle (her frame)."""
        if not hasattr(self, '_drone_cams') or drone_id not in self._drone_cams:
            return
        cam_np = self._drone_cams[drone_id]['cam_np']
        # Sim → Panda
        px, py, pz = sim_pos[1], sim_pos[0], sim_pos[2]
        cam_np.setPos(px, py, pz)
        deg = -math.degrees(heading_rad) + 90
        cam_np.setHpr(deg, -90, 0)

    def update_active_cam_display(self):
        """Artık tüm kameralar görünür — özel işlem yok."""
        pass

    def set_drone_cam_detection(self, detection_type, label_text=''):
        """Eski API uyumluluğu — artık per-drone detection kullanıyoruz."""
        pass

    def set_per_drone_detection(self, drone_id, detection_type):
        """Her drone'un PIP'inde tespit durumunu göster (border rengi + label)."""
        if not hasattr(self, '_drone_cams') or drone_id not in self._drone_cams:
            return
        cam_data = self._drone_cams[drone_id]
        lbl = cam_data['label']
        det_lbl = cam_data['det_label']
        border = cam_data['border']

        if detection_type == 'qr':
            border['frameColor'] = (0, 1, 0.3, 0.95)
            lbl.setFg((0, 1, 0.3, 1))
            lbl.setText(f'D{drone_id} QR DETECTED')
            det_lbl.setText('QR TESPIT')
            det_lbl.setFg((0, 1, 0.3, 1))
        elif detection_type == 'scanning':
            border['frameColor'] = (1, 0.9, 0.1, 0.95)
            lbl.setFg((1, 0.9, 0.1, 1))
            lbl.setText(f'D{drone_id} SCANNING...')
            det_lbl.setText('TARAMA...')
            det_lbl.setFg((1, 0.9, 0.1, 1))
        elif detection_type == 'color_red':
            border['frameColor'] = (1, 0.15, 0.15, 0.95)
            lbl.setFg((1, 0.3, 0.3, 1))
            lbl.setText(f'D{drone_id} KIRMIZI')
            det_lbl.setText('KIRMIZI ALAN')
            det_lbl.setFg((1, 0.3, 0.3, 1))
        elif detection_type == 'color_blue':
            border['frameColor'] = (0.2, 0.3, 1, 0.95)
            lbl.setFg((0.3, 0.4, 1, 1))
            lbl.setText(f'D{drone_id} MAVİ')
            det_lbl.setText('MAVİ ALAN')
            det_lbl.setFg((0.3, 0.4, 1, 1))
        else:
            border['frameColor'] = (0.15, 0.15, 0.15, 0.95)
            lbl.setFg((0.1, 1, 0.2, 1))
            lbl.setText(f'D{drone_id} CAM')
            det_lbl.setText('')
        cam_data['detection'] = detection_type

    def add_notify_log(self, sim_time, drone_id, message):
        """Haberleşme log'una mesaj ekle (ilk tespit eden bildiriyor)."""
        if not hasattr(self, '_cam_notify_log'):
            return
        self._cam_notify_log.append((sim_time, drone_id, message))
        # Son 4 log'u göster
        lines = []
        for t, did, msg in self._cam_notify_log[-4:]:
            mins = int(t) // 60
            secs = int(t) % 60
            lines.append(f'[{mins:02d}:{secs:02d}] D{did}: {msg}')
        self._notify_text.setText('\n'.join(lines))

    def _draw_pip_qr_box(self):
        """QR tespit kutusu — kamera görüntüsü ortasında yeşil dikdörtgen."""
        self._remove_pip_boxes()
        pip_x, pip_y = self._pip_pos
        bw, bh = 0.08, 0.08  # kutu yarı boyutu
        ls = LineSegs('qr_detect_box')
        ls.setColor(0, 1, 0.3, 0.9)
        ls.setThickness(2.5)
        # Dikdörtgen
        ls.moveTo(-bw, 0, -bh)
        ls.drawTo(bw, 0, -bh)
        ls.drawTo(bw, 0, bh)
        ls.drawTo(-bw, 0, bh)
        ls.drawTo(-bw, 0, -bh)
        # Köşe vurgu çizgileri (kalın)
        corner_len = 0.03
        for cx, cz in [(-bw, -bh), (bw, -bh), (bw, bh), (-bw, bh)]:
            dx = corner_len if cx < 0 else -corner_len
            dz = corner_len if cz < 0 else -corner_len
            ls.moveTo(cx, 0, cz)
            ls.drawTo(cx + dx, 0, cz)
            ls.moveTo(cx, 0, cz)
            ls.drawTo(cx, 0, cz + dz)
        self._pip_qr_box = self.aspect2d.attachNewNode(ls.create())
        self._pip_qr_box.setPos(pip_x, 0, pip_y)

    def _draw_pip_color_box(self, color_name):
        """Renk tespit — daha büyük dikdörtgen."""
        self._remove_pip_boxes()
        pip_x, pip_y = self._pip_pos
        bw, bh = 0.12, 0.10
        c = (1, 0.2, 0.2, 0.9) if color_name == 'red' else (0.2, 0.3, 1, 0.9)
        ls = LineSegs('color_detect_box')
        ls.setColor(*c)
        ls.setThickness(2.5)
        ls.moveTo(-bw, 0, -bh)
        ls.drawTo(bw, 0, -bh)
        ls.drawTo(bw, 0, bh)
        ls.drawTo(-bw, 0, bh)
        ls.drawTo(-bw, 0, -bh)
        self._pip_color_box = self.aspect2d.attachNewNode(ls.create())
        self._pip_color_box.setPos(pip_x, 0, pip_y)

    def _remove_pip_boxes(self):
        """Overlay kutularını temizle."""
        if self._pip_qr_box is not None:
            self._pip_qr_box.removeNode()
            self._pip_qr_box = None
        if self._pip_color_box is not None:
            self._pip_color_box.removeNode()
            self._pip_color_box = None
