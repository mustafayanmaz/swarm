# TEKNOFEST 2026 — Suru IHA Simulasyonu

TEKNOFEST 2026 Suru IHA Yarismasi icin gelistirilen Panda3D tabanli 3D simulasyon.

## Ozellikler

- **3 drone suru ucusu** — Panda3D 3D sahnede gercek zamanli
- **QR tarama** — Resmi Sekil 2 JSON formati, gercek QR uretimi ve decode
- **3 formasyon** — Okbasi (V), Ucgen, Cizgi + formasyon rotasyonu
- **Manevralar** — Pitch/roll matris donusumleri, irtifa degisim
- **Suruden birey ayrilma/katilma** — Paralel state machine, renk alanina inis/kalkis
- **Per-drone kamera** — 90 derece FOV, 3 esit PIP goruntusu
- **QR + renk alani tespiti** — Ilk tespit eden drone suruye bildirir
- **Arm/disarm** — LED renk degisimi + pervane durdurma
- **Failsafe** — GCS baglanti kesme (G tusu), otonom devam
- **HUD** — Durum, kronometre, QR gorev log paneli, haberlesme logu
- **Baslat/durdur/yeniden baslat** — Irtifa girisi ile

## Dosya Yapisi

```
main_panda3d.py          # Ana giris noktasi
config/mission.yaml      # Gorev yapilandirmasi (QR, rota, renkli alanlar)
src/
  renderer_3d.py         # Panda3D 3D sahne + HUD + kamera sistemi
  mission_runner.py      # Gorev 1 durum makinesi
  sim_controller.py      # Suru hareket kontrolcusu (fiziksiz)
  formation.py           # Formasyon hesaplama (okbasi/ucgen/cizgi)
  qr_system.py           # QR uretimi + decode (resmi JSON format)
```

## Kurulum

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Calistirma

```bash
python main_panda3d.py
```

## Kontroller

| Tus / Fare        | Islem                        |
|--------------------|------------------------------|
| Orta tik surukle   | Kamerayi dondur              |
| Sag tik surukle    | Kamerayi kaydir (pan)        |
| Scroll             | Yakinlastir / Uzaklastir     |
| G                  | GCS baglanti kes/bagla       |
| Q                  | QR gorev log ac/kapat        |
| R                  | Gorevi yeniden baslat         |
| TAB                | Aktif kamera degistir        |
| ESC                | Cikis                        |