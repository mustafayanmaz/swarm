"""
TEKNOFEST 2026 Sürü İHA - Giriş Cihazı Modülü (Joystick / Klavye)

Şartname Görev 5.2: "tek bir joystick veya RC kumanda üzerinden yönlendirme"

Desteklenen giriş:
  1. Gamepad/Joystick (pygame) — öncelikli
  2. Klavye fallback (pygame) — gamepad yoksa

Çıkış formatı (her frame):
  InputState:
    pitch    : -1.0..+1.0  (ileri/geri — sol stick Y)
    roll     : -1.0..+1.0  (sol/sağ — sol stick X)
    yaw      : -1.0..+1.0  (dönüş — sağ stick X)
    throttle : -1.0..+1.0  (irtifa — sağ stick Y)
    buttons  : dict[str, bool]  (buton durumları)

Buton atamaları:
  Gamepad:
    A / Cross       → takeoff
    B / Circle      → land
    X / Square      → mod değiştir (hareket ↔ manevra)
    Y / Triangle    → (boş)
    LB / L1         → önceki formasyon
    RB / R1         → sonraki formasyon
    Start           → çıkış

  Klavye:
    T → takeoff,  L → land,  M → mod değiştir
    1/2/3 → formasyon (çizgi/ok başı/V)
    WASD → pitch/roll, QE → yaw, RF → throttle
    ESC/X → çıkış
"""
import logging
import os

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame

log = logging.getLogger("swarm.input")

# Joystick dead zone — küçük kaymaları yoksay
DEADZONE = 0.12

# Klavye hareket hızı (tuş basıldığında stick değeri)
KEY_AXIS_VALUE = 0.6


class InputState:
    """Tek frame'lik giriş durumu."""

    def __init__(self):
        self.pitch: float = 0.0      # -1=geri, +1=ileri
        self.roll: float = 0.0       # -1=sol, +1=sağ
        self.yaw: float = 0.0        # -1=sol dönüş, +1=sağ dönüş
        self.throttle: float = 0.0   # -1=alçal, +1=yüksel

        # One-shot butonlar (basıldığı frame'de True)
        self.takeoff: bool = False
        self.land: bool = False
        self.toggle_mode: bool = False
        self.next_formation: bool = False
        self.prev_formation: bool = False
        self.set_formation: int = -1  # -1=yok, 0=line, 1=arrow, 2=v
        self.pitch_pos: bool = False   # Pitch manevrası +15°
        self.pitch_neg: bool = False   # Pitch manevrası -15°
        self.roll_pos: bool = False    # Roll manevrası +15°
        self.roll_neg: bool = False    # Roll manevrası -15°
        self.quit: bool = False

    def __repr__(self):
        return (
            f"Input(P={self.pitch:+.2f} R={self.roll:+.2f} "
            f"Y={self.yaw:+.2f} T={self.throttle:+.2f})"
        )


def _apply_deadzone(value: float) -> float:
    """Dead zone uygula ve normalize et."""
    if abs(value) < DEADZONE:
        return 0.0
    # Dead zone sonrası yeniden 0..1 aralığına normalize
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - DEADZONE) / (1.0 - DEADZONE)


class InputManager:
    """Joystick veya klavye girdisini okur."""

    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        self._joystick = None
        self._use_joystick = False
        self._prev_buttons = {}
        self._prev_keys = {}  # Klavye edge detection için

        # Küçük bir pencere gerekli (pygame event loop için)
        # Ama görünür olmak zorunda değil
        self._screen = pygame.display.set_mode((400, 200))
        pygame.display.set_caption("Sürü Kontrol — Görev 5.2")

        # Joystick ara
        if pygame.joystick.get_count() > 0:
            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()
            self._use_joystick = True
            log.info(
                f"🎮 Joystick bulundu: {self._joystick.get_name()} "
                f"(axes={self._joystick.get_numaxes()}, "
                f"buttons={self._joystick.get_numbuttons()})"
            )
        else:
            log.info("⌨️ Joystick bulunamadı — klavye modu aktif")
            self._draw_keyboard_help()

    def _draw_keyboard_help(self):
        """Pygame penceresine klavye kılavuzu çiz."""
        self._screen.fill((30, 30, 40))
        font = pygame.font.SysFont("monospace", 14)
        lines = [
            "SÜRÜ KONTROL - KLAVYe MODU",
            "",
            "W/S: İleri/Geri    A/D: Sol/Sağ",
            "Q/E: Yaw Sol/Sağ   R/F: Yüksel/Alçal",
            "T: Kalkış   L: İniş   M: Mod Değiştir",
            "1: Çizgi  2: Ok Başı  3: V",
            "P/I: Pitch +/-  O/U: Roll +/-",
            "ESC: Çıkış",
        ]
        for i, line in enumerate(lines):
            color = (100, 200, 255) if i == 0 else (200, 200, 200)
            surf = font.render(line, True, color)
            self._screen.blit(surf, (10, 10 + i * 22))
        pygame.display.flip()

    def _button_pressed(self, button: int) -> bool:
        """Butonun bu frame'de YENİ basilip basilmadığını kontrol et (edge detection)."""
        current = self._joystick.get_button(button) if self._joystick else False
        prev = self._prev_buttons.get(button, False)
        self._prev_buttons[button] = current
        return current and not prev

    def read(self) -> InputState:
        """
        Giriş cihazından bir frame oku.
        Pygame event pump çağrılmalı.
        """
        state = InputState()

        # Tüm eventleri topla
        events = pygame.event.get()

        for event in events:
            if event.type == pygame.QUIT:
                state.quit = True
                return state

        if self._use_joystick and self._joystick:
            state = self._read_joystick(state)
        else:
            state = self._read_keyboard(state, events)

        return state

    def _read_joystick(self, state: InputState) -> InputState:
        """Gamepad/joystick oku — Xbox/PS layout."""
        js = self._joystick

        # Sol stick: pitch (Y) ve roll (X)
        # Pygame Y ekseni ters: yukarı = -1, aşağı = +1
        state.roll = _apply_deadzone(js.get_axis(0))       # Sol stick X
        state.pitch = _apply_deadzone(-js.get_axis(1))      # Sol stick Y (ters)

        # Sağ stick: yaw (X) ve throttle (Y)
        state.yaw = _apply_deadzone(js.get_axis(3))         # Sağ stick X
        state.throttle = _apply_deadzone(-js.get_axis(4))   # Sağ stick Y (ters)

        # Butonlar (Xbox layout: A=0, B=1, X=2, Y=3, LB=4, RB=5, Start=7)
        state.takeoff = self._button_pressed(0)         # A
        state.land = self._button_pressed(1)             # B
        state.toggle_mode = self._button_pressed(2)      # X
        state.prev_formation = self._button_pressed(4)   # LB
        state.next_formation = self._button_pressed(5)   # RB
        state.quit = self._button_pressed(7)             # Start

        # D-pad veya Y butonu ile manevralar
        state.pitch_pos = self._button_pressed(3)        # Y — pitch +
        # D-pad: hat switch üzerinden
        try:
            hat = self._joystick.get_hat(0)
            if self._button_pressed(100):  # dummy — hat edge detection aşağıda
                pass
        except Exception:
            hat = (0, 0)
        # Hat: (x, y) — up=(0,1), down=(0,-1), left=(-1,0), right=(1,0)
        # Hat basınca manevra tetikle
        _hat_key = f"hat_{hat}"
        _hat_prev = self._prev_buttons.get("hat", (0, 0))
        self._prev_buttons["hat"] = hat
        if hat != _hat_prev:
            if hat == (0, 1):    state.pitch_pos = True   # D-pad up
            elif hat == (0, -1): state.pitch_neg = True   # D-pad down
            elif hat == (1, 0):  state.roll_pos = True    # D-pad right
            elif hat == (-1, 0): state.roll_neg = True    # D-pad left

        return state

    def _key_edge(self, key_id, pressed: bool) -> bool:
        """Tuşun bu frame'de İLK DEFA basılıp basılmadığını kontrol et."""
        prev = self._prev_keys.get(key_id, False)
        self._prev_keys[key_id] = pressed
        return pressed and not prev

    def _read_keyboard(self, state: InputState, events: list) -> InputState:
        """Klavye oku — WASD + QE + RF."""
        keys = pygame.key.get_pressed()

        # Eksenler (sürekli — tuş basılı olduğu sürece)
        if keys[pygame.K_w]:
            state.pitch = KEY_AXIS_VALUE
        elif keys[pygame.K_s]:
            state.pitch = -KEY_AXIS_VALUE

        if keys[pygame.K_d]:
            state.roll = KEY_AXIS_VALUE
        elif keys[pygame.K_a]:
            state.roll = -KEY_AXIS_VALUE

        if keys[pygame.K_e]:
            state.yaw = KEY_AXIS_VALUE
        elif keys[pygame.K_q]:
            state.yaw = -KEY_AXIS_VALUE

        if keys[pygame.K_r]:
            state.throttle = KEY_AXIS_VALUE
        elif keys[pygame.K_f]:
            state.throttle = -KEY_AXIS_VALUE

        # One-shot butonlar — get_pressed + edge detection
        if self._key_edge('t', keys[pygame.K_t]):
            state.takeoff = True
        if self._key_edge('l', keys[pygame.K_l]):
            state.land = True
        if self._key_edge('m', keys[pygame.K_m]):
            state.toggle_mode = True
        if self._key_edge('1', keys[pygame.K_1]):
            state.set_formation = 0
        if self._key_edge('2', keys[pygame.K_2]):
            state.set_formation = 1
        if self._key_edge('3', keys[pygame.K_3]):
            state.set_formation = 2
        if self._key_edge('p', keys[pygame.K_p]):
            state.pitch_pos = True
            log.info("[INPUT] P tuşu algılandı → pitch_pos")
        if self._key_edge('i', keys[pygame.K_i]):
            state.pitch_neg = True
            log.info("[INPUT] I tuşu algılandı → pitch_neg")
        if self._key_edge('o', keys[pygame.K_o]):
            state.roll_pos = True
            log.info("[INPUT] O tuşu algılandı → roll_pos")
        if self._key_edge('u', keys[pygame.K_u]):
            state.roll_neg = True
            log.info("[INPUT] U tuşu algılandı → roll_neg")
        if self._key_edge('esc', keys[pygame.K_ESCAPE]) or self._key_edge('x', keys[pygame.K_x]):
            state.quit = True

        return state

    @property
    def is_joystick(self) -> bool:
        return self._use_joystick

    @property
    def device_name(self) -> str:
        if self._use_joystick and self._joystick:
            return self._joystick.get_name()
        return "Klavye"

    def close(self):
        """Temizlik."""
        pygame.joystick.quit()
        pygame.quit()
