# --- Smart Barbell Tracker ---

import sys
import json
import os
import time
import gzip
import math
import calendar
from datetime import datetime
import numpy as np
from scipy.signal import butter, filtfilt
import asyncio
from bleak import BleakClient, BleakScanner
from PIL import Image, ImageOps
import qtawesome as qta

from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QFrame, 
                             QStackedWidget, QScrollArea, QCheckBox, 
                             QGridLayout, QButtonGroup, QSizePolicy, QSlider,
                             QGraphicsOpacityEffect, QDoubleSpinBox, QAbstractSpinBox,
                             QFileDialog, QMessageBox)
from PyQt6.QtCore import (Qt, pyqtSignal, QPropertyAnimation, QEasingCurve, 
                          QParallelAnimationGroup, QSequentialAnimationGroup, 
                          QTimer, QThread, QVariantAnimation, QSize, QObject, QEvent, QUrl)
from PyQt6.QtGui import QPixmap, QImage, QPainter, QColor, QPen, QDesktopServices
import pyqtgraph as pg

# --- THEME COLORS ---
BG_MAIN = "#131314"       
BG_SEC = "#1e1f22"        
BG_TER = "#282a2d"        
BG_HOVER = "#333538"      
ACCENT = "#a8c7fa"        
ACCENT_HOVER = "#c2d7fb"  
TEXT_ACCENT = "#062e6f"   
TEXT_MAIN = "#e3e3e3"     
TEXT_MUTED = "#c4c7c5"    
DANGER = "#f28b82"        
DANGER_HOVER = "#f6ada6"

pg.setConfigOptions(antialias=True)
pg.setConfigOption('background', BG_MAIN)
pg.setConfigOption('foreground', TEXT_MUTED)

# --- BLE SETTINGS ---
BLE_DEVICE_NAME = "Smart_Collar_ESP32"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

# --- QTHREAD FOR BLE COMMUNICATION ---
class BLEWorker(QThread):
    # FIX: Signal also emits dt value
    data_prijate = pyqtSignal(float, float, float, float)
    chyba_spojenia = pyqtSignal(str) 
    kalibracia_hotova = pyqtSignal(float, float)
    accel_kalibracia_hotova = pyqtSignal()
    status_zmena = pyqtSignal(str)
    bateria_zmena = pyqtSignal(int, float) 

    def __init__(self):
        super().__init__()
        self.bezi = True
        self.nahravam = False
        self.rezim_kalibracie = False
        self.kalibracne_data_L = []
        self.kalibracne_data_R = []
        self.chce_pripojit = False
        self.client = None
        self.loop = None
        self.poslat_cmd_calib = False
        self.poslat_cmd_accel_calib = False
        self.poslat_cmd_batt = False 
        
        self.posledny_cas = 0.0
        self.zahodit_vzoriek = 0  # Number of packets to burn (clear buffer)

    def notification_handler(self, sender, data):
        try:
            line = data.decode('utf-8').strip()
            
            if "CALIB_DONE" in line:
                self.stop_calibration()
                return
                
            if "ACCEL_CALIB_DONE" in line:
                self.accel_kalibracia_hotova.emit()
                return
                
            if line.startswith("BAT:"):
                bat_data = line.replace("BAT:", "").split(",")
                if len(bat_data) == 2:
                    percento = int(bat_data[0])
                    napatie = float(bat_data[1].replace("V", ""))
                    self.bateria_zmena.emit(percento, napatie)
                return

            hodnoty = line.split(',')
            # FIX: Reading 3 values - L, R, and real_dt
            if len(hodnoty) == 3:
                acc_L, acc_R, real_dt = float(hodnoty[0]), float(hodnoty[1]), float(hodnoty[2])
                
                # --- PYTHON BURN PHASE ---
                # Clears old data that might be stuck in the OS BLE buffer
                if self.zahodit_vzoriek > 0:
                    self.zahodit_vzoriek -= 1
                    return  # Burn this old packet
                
                self.posledny_cas += real_dt
                
                if self.rezim_kalibracie:
                    self.kalibracne_data_L.append(acc_L)
                    self.kalibracne_data_R.append(acc_R)
                elif self.nahravam:
                    self.data_prijate.emit(self.posledny_cas, acc_L, acc_R, real_dt)
        except Exception as e:
            pass

    async def connect_and_run(self):
        while self.bezi:
            if self.chce_pripojit and (self.client is None or not self.client.is_connected):
                self.status_zmena.emit("Hľadám zariadenie...")
                devices = await BleakScanner.discover(timeout=3.0)
                target_device = None
                for d in devices:
                    if d.name and d.name == BLE_DEVICE_NAME:
                        target_device = d
                        break

                if target_device:
                    self.status_zmena.emit("Pripájam sa...")
                    try:
                        self.client = BleakClient(target_device)
                        await self.client.connect()
                        await self.client.start_notify(CHARACTERISTIC_UUID, self.notification_handler)
                        self.status_zmena.emit("Pripojené!")
                    except Exception as e:
                        self.chyba_spojenia.emit(f"Chyba pripojenia: {e}")
                        self.chce_pripojit = False
                else:
                    self.chyba_spojenia.emit("Zariadenie nenájdené.")
                    self.chce_pripojit = False

            if self.client and self.client.is_connected:
                if self.poslat_cmd_calib:
                    try:
                        await self.client.write_gatt_char(CHARACTERISTIC_UUID, b"C", response=False)
                        self.poslat_cmd_calib = False
                    except Exception as e:
                        self.chyba_spojenia.emit(f"Chyba zápisu: {e}")
                
                if self.poslat_cmd_accel_calib:
                    try:
                        await self.client.write_gatt_char(CHARACTERISTIC_UUID, b"A", response=False)
                        self.poslat_cmd_accel_calib = False
                    except Exception as e:
                        self.chyba_spojenia.emit(f"Chyba zápisu accel calib: {e}")
                
                if self.poslat_cmd_batt:
                    try:
                        await self.client.write_gatt_char(CHARACTERISTIC_UUID, b"B", response=False)
                        self.poslat_cmd_batt = False
                    except Exception as e:
                        print(f"Chyba poziadavky baterie: {e}")
                        
                await asyncio.sleep(0.01)
            else:
                await asyncio.sleep(0.1)

        if self.client and self.client.is_connected:
            try:
                await self.client.stop_notify(CHARACTERISTIC_UUID)
            except KeyError:
                pass
            except Exception as e:
                print(f"Chyba pri stop_notify: {e}")
                
            try:
                await self.client.disconnect()
            except Exception:
                pass

    def run(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.connect_and_run())

    def pripoj(self):
        self.chce_pripojit = True

    def odpoj(self):
        self.chce_pripojit = False
        if self.client and self.loop:
            asyncio.run_coroutine_threadsafe(self.client.disconnect(), self.loop)

    def start_recording(self):
        self.posledny_cas = 0.0
        self.zahodit_vzoriek = 15  # Discard the first 15 BLE packets after start (approx 0.15s)
        self.nahravam = True

    def stop_recording(self):
        self.nahravam = False

    def posli_prikaz_kalibracia(self):
        self.poslat_cmd_calib = True
        
    def posli_prikaz_accel_kalibracia(self):
        self.poslat_cmd_accel_calib = True
        
    def posli_prikaz_bateria(self):
        self.poslat_cmd_batt = True

    def start_calibration(self):
        self.posledny_cas = 0.0
        self.kalibracne_data_L = []
        self.kalibracne_data_R = []
        self.rezim_kalibracie = True

    def stop_calibration(self):
        self.rezim_kalibracie = False
        bias_L = np.mean(self.kalibracne_data_L) if self.kalibracne_data_L else 0.0
        bias_R = np.mean(self.kalibracne_data_R) if self.kalibracne_data_R else 0.0
        self.kalibracia_hotova.emit(bias_L, bias_R)

    def zastav(self):
        self.bezi = False
        self.chce_pripojit = False 
        self.wait()

# --- TRANSLATION DICTIONARY ---
TRANSLATIONS = {
    'sk': {
        'greeting': "Ahoj, aký tréning dnes zanalyzujeme?",
        'btn_1': "VBT Analýza\n1 Senzor",
        'btn_2': "Symetria Tyče\n2 Senzory",
        'btn_set': "Nastavenie limitu zlyhania",
        'btn_diary': "Otvoriť tréningový denník",
        'btn_calc': "IPF GL Kalkulačka",
        'lang_switch': "EN", 
        'back': "Späť",
        'menu': "Menu",
        'list': "Zoznam",
        'diary_title': "História aktivít",
        'calendar': "Kalendár",
        'del_sel': "Zmazať vybrané",
        'del_count': "Zmazať ({})",
        'analyze': "Analyzovať",
        'no_rec': "Zatiaľ žiadne záznamy.",
        'all': "Všetky",
        'squat': "Drep",
        'bench': "Tlak",
        'deadlift': "Mŕtvy ťah",
        'close_cal': "Zavrieť",
        'record_live': "Zaznamenávať dáta",
        'record_live_stop': "Zastaviť záznam",
        'calib_btn': "Kalibrácia",
        'calib_title': "Kalibrácia gyroskopu",
        'calib_done': "Hotovo!",
        'single': "Samostatne",
        'custom': "Vlastný",
        'compare': "Porovnať série (Únava)",
        'save_title': "Uložiť do denníka",
        'show_vel': "Zobraziť: Rýchlosť",
        'show_acc': "Zobraziť: Zrýchlenie",
        'left_det': "Ľavý detail",
        'right_det': "Pravý detail",
        'back_sym': "Návrat na Symetriu",
        'tip': "Detailný režim: Ťahaním ľavej myši sa pohybuješ v grafe. Kolieskom zoomuješ.",
        'set': "Séria",
        'part': "Časť",
        'diff': "Rozdiel",
        'left': "Ľ",
        'right': "P",
        'saved': "Uložené: {}",
        'lim_title': "Limit únavy: {} %",
        'save_close': "Uložiť a zavrieť",
        'time': "Čas",
        'filter': "Filter: {}",
        'graph1_1': "Zvislé zrýchlenie (Z) - m/s²",
        'graph2_1': "Zvislá rýchlosť (Z) - m/s",
        'graph3_1': "Zvislá dráha (Z) - cm",
        'graph4_1': "Horizontálna dráha (Y) - cm",
        'graph1_2a': "Zrýchlenie (Ľavá vs Pravá) - m/s²",
        'graph1_2v': "Rýchlosť (Ľavá vs Pravá) - m/s",
        'graph2_2': "Symetria / Náklon tyče - cm",
        'graph3_2': "Zvislá dráha (Ľavá vs Pravá)",
        'left_suffix': " (Ľavý)",
        'right_suffix': " (Pravý)",
        'warn1': "Porovnanie S{} vs S{}: Výkon na {:.1f} %",
        'warn2': "Vyber druhú sériu na porovnanie...",
        'warn3': "Zvoľ referenčnú sériu kliknutím v zozname...",
        'tt_warn': "POZOR! Priemerné zrýchlenie kleslo na {:.1f} %.\nPrekročil si limit únavy ({} %).\nBlížiš sa k maximálnej váhe alebo svalovému zlyhaniu!",
        'rep_drop_warn_title': "Odporúčanie: Zníženie počtu opakovaní",
        'rep_drop_warn_text': "Posledné opakovanie ({:.2f} m/s) nedosiahlo limit {} % rýchlosti z prvého opakovania ({:.2f} m/s).\nOdporúčame znížiť počet opakovaní v ďalšej sérii pre udržanie kvality pohybu (VBT).",
        'rep_drop_ok_title': "Kvalita série: Výborná",
        'rep_drop_ok_text': "Pokles rýchlosti (posledné: {:.2f} m/s, prvé: {:.2f} m/s) je v norme nad limitom {} %.\nKvalita (VBT) bola v tejto sérii udržaná.",
        'btn_check_fatigue': "Skontrolovať únavu",
        '1_sens_txt': "Single Sensor",
        '2_sens_txt': "Dual Sensor",
        'logo_not_found': "[ Logo nenájdené ]",
        'weight_suffix': " kg",
        'calc_title': "IPF GL Kalkulačka",
        'male': "Muž",
        'female': "Žena",
        'classic': "Classic (Raw)",
        'equipped': "Equipped",
        'powerlifting': "Silový trojboj",
        'bench_only': "Len Tlak",
        'bw': "Telesná hmotnosť:",
        'total': "Zdvihnutá váha:",
        'gl_points': "GL Body:",
        'profile_title': "Môj Profil a Rekordy",
        'prof_raw': "Raw",
        'prof_eq': "Equip",
        'import_max': "Importovať z profilu",
        'ipf_rules': "IPF Pravidlá 2026",
        'szst_cal': "SZST Kalendár",
        'battery': "Batéria: {}% ({}V)",
        'calib_accel_btn': "6-bodová Kalibrácia",
        'months': ['Január', 'Február', 'Marec', 'Apríl', 'Máj', 'Jún', 'Júl', 'August', 'September', 'Október', 'November', 'December']
    },
    'en': {
        'greeting': "Hello, what workout are we analyzing today?",
        'btn_1': "VBT Analysis\n1 Sensor",
        'btn_2': "Bar Symmetry\n2 Sensors",
        'btn_set': "Fatigue Limit Settings",
        'btn_diary': "Open Training Diary",
        'btn_calc': "IPF GL Calculator",
        'lang_switch': "SK",
        'back': "Back",
        'menu': "Menu",
        'list': "List",
        'diary_title': "Activity History",
        'calendar': "Calendar",
        'del_sel': "Delete selected",
        'del_count': "Delete ({})",
        'analyze': "Analyze",
        'no_rec': "No records yet.",
        'all': "All",
        'squat': "Squat",
        'bench': "Bench Press",
        'deadlift': "Deadlift",
        'close_cal': "Close",
        'record_live': "Record Data",
        'record_live_stop': "Stop Recording",
        'calib_btn': "Calibrate",
        'calib_title': "Gyro Calibration",
        'calib_done': "Done!",
        'single': "Single",
        'custom': "Custom",
        'compare': "Compare Sets (Fatigue)",
        'save_title': "Save to Diary",
        'show_vel': "Show: Velocity",
        'show_acc': "Show: Acceleration",
        'left_det': "Left Detail",
        'right_det': "Right Detail",
        'back_sym': "Back to Symmetry",
        'tip': "Detail mode: Drag left mouse to pan. Use wheel to zoom.",
        'set': "Set",
        'part': "Part",
        'diff': "Diff",
        'left': "L",
        'right': "R",
        'saved': "Saved: {}",
        'lim_title': "Fatigue Limit: {} %",
        'save_close': "Save and Close",
        'time': "Time",
        'filter': "Filter: {}",
        'graph1_1': "Vertical Acceleration (Z) - m/s²",
        'graph2_1': "Vertical Velocity (Z) - m/s",
        'graph3_1': "Vertical Displacement (Z) - cm",
        'graph4_1': "Horizontal Displacement (Y) - cm",
        'graph1_2a': "Acceleration (Left vs Right) - m/s²",
        'graph1_2v': "Velocity (Left vs Right) - m/s",
        'graph2_2': "Symmetry / Bar Tilt - cm",
        'graph3_2': "Vertical Displacement (Left vs Right)",
        'left_suffix': " (Left)",
        'right_suffix': " (Right)",
        'warn1': "Comparison S{} vs S{}: Performance at {:.1f} %",
        'warn2': "Select second set for comparison...",
        'warn3': "Choose reference set by clicking in the list...",
        'tt_warn': "WARNING! Average acceleration dropped to {:.1f} %.\nYou exceeded the fatigue limit ({} %).\nApproaching max weight or muscle failure!",
        'rep_drop_warn_title': "Recommendation: Reduce Repetitions",
        'rep_drop_warn_text': "The last repetition ({:.2f} m/s) did not reach the {} % velocity limit of the first repetition ({:.2f} m/s).\nWe recommend reducing the number of repetitions in the next set to maintain movement quality (VBT).",
        'rep_drop_ok_title': "Set Quality: Excellent",
        'rep_drop_ok_text': "Velocity drop (last: {:.2f} m/s, first: {:.2f} m/s) is within the safe zone above the {} % limit.\nMovement quality (VBT) was maintained in this set.",
        'btn_check_fatigue': "Check Fatigue",
        '1_sens_txt': "Single Sensor",
        '2_sens_txt': "Dual Sensor",
        'logo_not_found': "[ Logo not found ]",
        'weight_suffix': " kg",
        'calc_title': "IPF GL Calculator",
        'male': "Male",
        'female': "Female",
        'classic': "Classic (Raw)",
        'equipped': "Equipped",
        'powerlifting': "Powerlifting",
        'bench_only': "Bench Only",
        'bw': "Bodyweight:",
        'total': "Lifted Weight:",
        'gl_points': "GL Points:",
        'profile_title': "My Profile & Records",
        'prof_raw': "Raw",
        'prof_eq': "Equip",
        'import_max': "Import from profile",
        'ipf_rules': "IPF Rules 2026",
        'szst_cal': "SZST Calendar",
        'battery': "Battery: {}% ({}V)",
        'calib_accel_btn': "6-point Calibration",
        'months': ['January', 'February', 'March', 'April', 'May', 'June', 'July', 'August', 'September', 'October', 'November', 'December']
    }
}

class UvodneOkno(QWidget):
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        w = self.width()
        h = self.height()
        
        t = 60             
        inner_radius = 50  
        
        painter.fillRect(0, 0, w, h, QColor(19, 19, 20)) 
        
        center_offset = t + inner_radius
        corner_dist = int(math.ceil(math.sqrt(center_offset**2 + center_offset**2)))
        total_layers = corner_dist - inner_radius
        
        painter.setBrush(Qt.BrushStyle.NoBrush)
        
        for step in range(total_layers):
            current_radius = corner_dist - step
            i = center_offset - current_radius
            
            alpha = int(100 * (1 - (i / t)))
            alpha = max(0, min(255, alpha)) 
            
            pen = QPen(QColor(0, 195, 255, alpha))
            pen.setWidth(1)
            painter.setPen(pen)
            
            painter.drawRoundedRect(int(i), int(i), int(w - 2*i), int(h - 2*i), int(current_radius), int(current_radius))
            
        painter.end()

class CleanFocusSpinBox(QDoubleSpinBox):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.povodna_hodnota = 0.0
        self.setFocusPolicy(Qt.FocusPolicy.ClickFocus)

    def focusInEvent(self, event):
        self.povodna_hodnota = self.value()
        super().focusInEvent(event)
        QTimer.singleShot(0, self.lineEdit().clear)

    def focusOutEvent(self, event):
        text = self.lineEdit().text().replace(self.suffix(), '').strip()
        if not text:
            self.blockSignals(True)
            self.setValue(self.povodna_hodnota)
            self.blockSignals(False)
            self.lineEdit().setText(self.textFromValue(self.povodna_hodnota) + self.suffix())
            
        super().focusOutEvent(event)

class FadeStackedWidget(QStackedWidget):
    pre_transition = pyqtSignal()
    post_transition = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.fade_duration = 300 
        self.is_animating = False

    def setCurrentWidget(self, widget):
        if self.currentWidget() == widget or self.is_animating:
            return

        self.pre_transition.emit()
        self.is_animating = True

        self.current_pixmap = self.currentWidget().grab()
        super().setCurrentWidget(widget)
        
        widget.setGeometry(self.rect())
        self.next_pixmap = widget.grab()

        self.overlay = QLabel(self)
        self.overlay.setGeometry(self.rect())
        self.overlay.setPixmap(self.current_pixmap)
        self.overlay.show()
        self.overlay.raise_()

        self.anim = QVariantAnimation(self)
        self.anim.setDuration(self.fade_duration)
        self.anim.setEasingCurve(QEasingCurve.Type.InOutSine)
        self.anim.setStartValue(0.0)
        self.anim.setEndValue(1.0)
        self.anim.valueChanged.connect(self._update_opacity)
        self.anim.finished.connect(self._animation_done)
        self.anim.start()

    def _update_opacity(self, value):
        blended = QPixmap(self.size())
        blended.fill(Qt.GlobalColor.transparent)
        
        painter = QPainter(blended)
        painter.drawPixmap(0, 0, self.current_pixmap)
        painter.setOpacity(value)
        painter.drawPixmap(0, 0, self.next_pixmap)
        painter.end()
        
        self.overlay.setPixmap(blended)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.is_animating and hasattr(self, 'overlay'):
            self.overlay.setGeometry(self.rect())

    def _animation_done(self):
        self.is_animating = False
        self.overlay.hide()
        self.overlay.deleteLater()
        self.post_transition.emit()

class SegmentedButton(QFrame):
    valueChanged = pyqtSignal(str)

    def __init__(self, items, default_key):
        super().__init__()
        self.setStyleSheet(f"QFrame {{ background-color: {BG_SEC}; border-radius: 12px; }}")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(8) 
        
        self.btn_group = QButtonGroup(self)
        self.btn_group.setExclusive(True)
        
        self.buttons = {}
        for key, text in items:
            btn = QPushButton(text)
            btn.setCheckable(True)
            btn.setMinimumHeight(40) 
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: transparent; border: none; border-radius: 8px;
                    color: {TEXT_MUTED}; font-weight: bold; font-family: "Segoe UI", sans-serif; font-size: 14px;
                    padding: 6px 14px; 
                }}
                QPushButton:checked {{ background-color: {BG_TER}; color: {TEXT_MAIN}; }}
                QPushButton:hover:!checked {{ background-color: {BG_HOVER}; }}
            """)
            self.btn_group.addButton(btn)
            layout.addWidget(btn)
            self.buttons[key] = btn
            
            def make_callback(k):
                return lambda checked: self.valueChanged.emit(k) if checked else None
            
            btn.toggled.connect(make_callback(key))
            
        if default_key in self.buttons:
            self.buttons[default_key].setChecked(True)

    def update_texts(self, items):
        for key, text in items:
            if key in self.buttons:
                self.buttons[key].setText(text)
                
    def get_value(self):
        for key, btn in self.buttons.items():
            if btn.isChecked():
                return key
        return None

class TrackerApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.jazyk = 'sk' 
        self.setWindowTitle("Smart Barbell Tracker - Pro (PyQt6) v3.7.2")
        
        self.setStyleSheet(f"""
            QMainWindow {{ background-color: {BG_MAIN}; }}
            QToolTip {{
                background-color: {BG_SEC}; color: {TEXT_MAIN};
                border: 1px solid {ACCENT}; border-radius: 6px;
                padding: 6px; font-size: 14px; font-family: 'Segoe UI', sans-serif;
            }}
        """)
        
        self.setWindowState(Qt.WindowState.WindowMaximized)

        self.threshold_zlyhania = 85 
        self.logo_anim_started = False 
        self.is_live_recording = False
        self.is_calibrating = False 
        self.saved_biases = None 
        
        self.bateria_percent = 0
        self.bateria_napatie = 0.0
        
        self.kalendar_mesiac = datetime.now().month
        self.kalendar_rok = datetime.now().year

        self.ble_worker = BLEWorker()
        self.ble_worker.data_prijate.connect(self.prijmi_live_data)
        self.ble_worker.chyba_spojenia.connect(self.zlyhanie_ble)
        self.ble_worker.kalibracia_hotova.connect(self.dokoncena_kalibracia)
        self.ble_worker.accel_kalibracia_hotova.connect(self.dokoncena_accel_kalibracia)
        self.ble_worker.status_zmena.connect(self.aktualizuj_status_ble)
        self.ble_worker.bateria_zmena.connect(self.aktualizuj_bateriu)
        self.ble_worker.start()

        self.subor_dennika = "barbell_diary.json.gz"
        if not os.path.exists(self.subor_dennika):
            with gzip.open(self.subor_dennika, "wt", encoding="utf-8") as f:
                json.dump([], f)

        self.subor_profilu = "user_profile.json"
        self.nacitaj_profil()

        self.farby_pokusov = ['#a8c7fa', '#fbbc04', '#81c995', '#f28b82', '#c58af9', '#fde293', '#34a853', '#ea4335']
        
        self.esp_live_data = []
        
        self.reset_statu()

        self.stacked_widget = FadeStackedWidget()
        self.stacked_widget.pre_transition.connect(self.pred_prechodom)
        self.stacked_widget.post_transition.connect(self.po_prechode)
        
        self.main_container = QWidget()
        self.main_layout = QVBoxLayout(self.main_container)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        
        self.main_layout.addWidget(self.stacked_widget)
        
        self.setCentralWidget(self.main_container)

        self.init_uvodne_okno()
        self.init_dennik()
        self.init_hlavne_ui()
        
        self.overlay_bg = QFrame(self.main_container)
        self.overlay_bg.setStyleSheet("background-color: rgba(19, 19, 20, 0.85);")
        self.overlay_layout = QVBoxLayout(self.overlay_bg)
        self.overlay_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.overlay_bg.hide()
        
        self.overlay_bg.mousePressEvent = lambda e: e.accept()

        self.stacked_widget.addWidget(self.uvodne_okno)
        self.stacked_widget.addWidget(self.dennik_widget)
        self.stacked_widget.addWidget(self.main_ui_widget)
        
        self.stacked_widget.setCurrentWidget(self.uvodne_okno)
        
        self.overlay_buttons_w = QWidget(self.main_container)
        self.overlay_buttons_w.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, False)
        
        self.btn_ipf_rules = self.create_btn(self.t('ipf_rules'), "transparent", TEXT_MUTED, 15, 32, hover=BG_SEC, font_size=13, border=BG_TER, icon_name='fa5s.book-open')
        self.btn_ipf_rules.setParent(self.overlay_buttons_w)
        self.btn_ipf_rules.clicked.connect(lambda: QDesktopServices.openUrl(QUrl("https://www.powerlifting.sport/fileadmin/ipf/data/rules/technical-rules/english/2026_IPF_Technical_Rulebook__effective_01_March_2026__v3.pdf")))
        
        self.btn_szst_cal = self.create_btn(self.t('szst_cal'), "transparent", TEXT_MUTED, 15, 32, hover=BG_SEC, font_size=13, border=BG_TER, icon_name='fa5s.calendar-alt')
        self.btn_szst_cal.setParent(self.overlay_buttons_w)
        self.btn_szst_cal.clicked.connect(lambda: QDesktopServices.openUrl(QUrl("https://szst.sk/kalendar")))

        QApplication.instance().installEventFilter(self)

    def closeEvent(self, event):
        if hasattr(self, 'ble_worker') and self.ble_worker:
            self.ble_worker.zastav()
        super().closeEvent(event)

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.MouseButtonPress:
            fw = QApplication.focusWidget()
            if isinstance(fw, QWidget) and isinstance(obj, QWidget):
                if obj != fw and not fw.isAncestorOf(obj):
                    fw.clearFocus() 
        return super().eventFilter(obj, event)

    def nacitaj_profil(self):
        if os.path.exists(self.subor_profilu):
            try:
                with open(self.subor_profilu, "r", encoding="utf-8") as f:
                    self.profil = json.load(f)
            except:
                self._predvoleny_profil()
        else:
            self._predvoleny_profil()

    def _predvoleny_profil(self):
        self.profil = {
            "bw": 0.0,
            "raw": {"squat": 0.0, "bench": 0.0, "deadlift": 0.0},
            "equip": {"squat": 0.0, "bench": 0.0, "deadlift": 0.0}
        }

    def uloz_profil(self):
        self.profil["bw"] = self.spin_prof_bw.value()
        self.profil["raw"]["squat"] = self.spin_prof_sq_raw.value()
        self.profil["raw"]["bench"] = self.spin_prof_bn_raw.value()
        self.profil["raw"]["deadlift"] = self.spin_prof_dl_raw.value()
        self.profil["equip"]["squat"] = self.spin_prof_sq_eq.value()
        self.profil["equip"]["bench"] = self.spin_prof_bn_eq.value()
        self.profil["equip"]["deadlift"] = self.spin_prof_dl_eq.value()
        
        try:
            with open(self.subor_profilu, "w", encoding="utf-8") as f:
                json.dump(self.profil, f)
        except Exception as e:
            print(f"Chyba pri ukladaní profilu: {e}")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, 'overlay_bg'):
            self.overlay_bg.setGeometry(self.rect())
            
        if hasattr(self, 'overlay_buttons_w'):
            btn_w1 = self.btn_ipf_rules.sizeHint().width()
            btn_w2 = self.btn_szst_cal.sizeHint().width()
            
            w_total = btn_w1 + btn_w2 + 10
            h_total = 32
            
            pos_x = self.width() - w_total - 100 
            pos_y = self.height() - h_total - 80 
            
            self.overlay_buttons_w.setGeometry(pos_x, pos_y, w_total, h_total)
            
            self.btn_ipf_rules.setGeometry(0, 0, btn_w1, h_total)
            self.btn_szst_cal.setGeometry(btn_w1 + 10, 0, btn_w2, h_total)

    def zatvor_overlay(self):
        self.overlay_bg.hide()
        self.clear_layout(self.overlay_layout)

    def t(self, key):
        return TRANSLATIONS[self.jazyk].get(key, key)

    def prepni_jazyk(self):
        self.jazyk = 'en' if self.jazyk == 'sk' else 'sk'
        
        self.btn_lang_uvod.setText(self.t('lang_switch'))
        self.lbl_title_uvod.setText(self.t('greeting'))
        
        self.btn_1_uvod.text_lbl.setText(self.t('btn_1'))
        self.btn_2_uvod.text_lbl.setText(self.t('btn_2'))
        
        self.btn_nastavenia_uvod.setText(self.t('btn_set'))
        self.btn_dennik_uvod.setText(self.t('btn_diary'))
        self.btn_kalkulacka_uvod.setText(self.t('btn_calc'))
        
        self.lbl_prof_nadpis.setText(self.t('profile_title'))
        self.lbl_prof_raw.setText(self.t('prof_raw'))
        self.lbl_prof_eq.setText(self.t('prof_eq'))
        self.lbl_prof_bw.setText(self.t('bw'))
        self.lbl_prof_sq.setText(self.t('squat'))
        self.lbl_prof_bn.setText(self.t('bench'))
        self.lbl_prof_dl.setText(self.t('deadlift'))
        
        if self.logo_label.text() != "":
            self.logo_label.setText(self.t('logo_not_found'))
            
        self.btn_spat_dennik.setText(self.t('back'))
        self.lbl_nadpis_dennik.setText(self.t('diary_title'))
        self.btn_kal_dennik.setText(self.t('calendar'))
        self.filter_cvik.update_texts([
            ('all', self.t('all')), ('squat', self.t('squat')), 
            ('bench', self.t('bench')), ('deadlift', self.t('deadlift'))
        ])
        if len(self.oznacene_zaznamy) > 0:
            self.btn_zmazat.setText(self.t('del_count').format(len(self.oznacene_zaznamy)))
        else:
            self.btn_zmazat.setText(self.t('del_sel'))
        self.prekresli_dennik()
        
        self.btn_spat_hlavne.setText(self.t('list') if self.je_historia else self.t('menu'))
        self.btn_kalibracia.setText(self.t('calib_btn'))
        
        if hasattr(self, 'btn_kalibracia_accel'):
            self.btn_kalibracia_accel.setText(self.t('calib_accel_btn'))
        
        if not (hasattr(self, 'is_live_recording') and self.is_live_recording):
            self.btn_live_esp.setText(self.t('record_live'))
        else:
            self.btn_live_esp.setText(self.t('record_live_stop'))
            
        self.seg_rezim.update_texts([('single', self.t('single')), ('custom', self.t('custom'))])
        self.btn_analyza_unavy_hlavne.setText(self.t('compare'))
        self.btn_check_fatigue.setText(self.t('btn_check_fatigue'))
        self.lbl_ulozit_nadpis.setText(self.t('save_title'))
        
        for k in ['squat', 'bench', 'deadlift']:
            self.btn_ulozit_cviky[k].setText(self.t(k))
            
        self.lbl_tip_hlavne.setText(self.t('tip'))
        
        self.vykresli_tlacidla_pre_2_senzory()
        self.obnov_ui_zoznam()
        self.aktualizuj_grafy()

        if hasattr(self, 'btn_ipf_rules'):
            self.btn_ipf_rules.setText(self.t('ipf_rules'))
            self.btn_szst_cal.setText(self.t('szst_cal'))
            
        if self.bateria_percent > 0:
            self.lbl_bat_hlavne.setText(self.t('battery').format(self.bateria_percent, self.bateria_napatie))
            
        if hasattr(self, 'overlay_buttons_w'):
            self.resizeEvent(None)
            
        if self.overlay_bg.isVisible() and hasattr(self, 'kalendar_lbl'):
            nazov_mes = self.t('months')[self.kalendar_mesiac - 1]
            self.kalendar_lbl.setText(f"{nazov_mes} {self.kalendar_rok}")

    def _zapni_animaciu_loga(self):
        self.logo_opacity = QGraphicsOpacityEffect(self.logo_label)
        self.logo_label.setGraphicsEffect(self.logo_opacity)
        
        self.anim_fade_in = QPropertyAnimation(self.logo_opacity, b"opacity")
        self.anim_fade_in.setDuration(2000) 
        self.anim_fade_in.setStartValue(0.3)
        self.anim_fade_in.setEndValue(1.0)
        self.anim_fade_in.setEasingCurve(QEasingCurve.Type.InOutSine)
        
        self.anim_fade_out = QPropertyAnimation(self.logo_opacity, b"opacity")
        self.anim_fade_out.setDuration(2000) 
        self.anim_fade_out.setStartValue(1.0)
        self.anim_fade_out.setEndValue(0.3)
        self.anim_fade_out.setEasingCurve(QEasingCurve.Type.InOutSine)
        
        self.logo_anim_group = QSequentialAnimationGroup()
        self.logo_anim_group.addAnimation(self.anim_fade_in)
        self.logo_anim_group.addAnimation(self.anim_fade_out)
        self.logo_anim_group.setLoopCount(-1)
        self.logo_anim_group.start()

    def _vypni_animaciu_loga(self):
        if hasattr(self, 'logo_anim_group'):
            self.logo_anim_group.stop()
        self.logo_label.setGraphicsEffect(None)

    def showEvent(self, event):
        super().showEvent(event)
        if not self.logo_anim_started:
            self.logo_anim_started = True
            if self.stacked_widget.currentWidget() == self.uvodne_okno:
                self._zapni_animaciu_loga()

    def pred_prechodom(self):
        if not self.logo_anim_started: return
        self._vypni_animaciu_loga()
        
        if self.stacked_widget.currentWidget() == self.uvodne_okno:
            self.overlay_buttons_w.hide()

    def po_prechode(self):
        if not self.logo_anim_started: return
        if self.stacked_widget.currentWidget() == self.uvodne_okno:
            self._zapni_animaciu_loga()
            self.overlay_buttons_w.show()

    def reset_statu(self):
        self.pocet_senzorov = 1
        self.databaza = {}  
        self.databaza_weights = {}  
        self.aktualny_zaznam_id = None 
        self.aktualna_seria = 1
        self.vybrana_seria = 1
        self.vybrany_pokus_idx = 0
        self.rezim_porovnania = "single" 
        self.vlastny_vyber = set() 
        self.ref_vyber = None 
        self.comp_vyber = None 
        self.zobrazene_data = [] 
        self.zobrazit_zrychlenie = False 
        self.zobrazenie_2_senzorov = "oba"
        self.je_historia = False
        self.oznacene_zaznamy = set()
        self.aktualny_filter_cviku = "all" 
        self.aktualny_datum_filter = None

    def clear_layout(self, layout):
        if layout is not None:
            while layout.count():
                item = layout.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    widget.deleteLater()
                elif item.layout() is not None:
                    self.clear_layout(item.layout())

    def create_btn(self, text, bg, color, radius, height, hover=None, font_size=15, border=None, icon_name=None):
        btn = QPushButton(text)
        btn.setFixedHeight(height)
        btn.setCursor(Qt.CursorShape.PointingHandCursor)
        hov = hover if hover else BG_HOVER
        border_css = f"border: 2px solid {border};" if border else "border: none;"
        
        if icon_name:
            icon = qta.icon(icon_name, color=color)
            btn.setIcon(icon)
            btn.setIconSize(QSize(int(font_size * 1.2), int(font_size * 1.2)))

        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {bg}; color: {color};
                border-radius: {radius}px; font-weight: bold; 
                font-family: "Segoe UI", sans-serif; font-size: {font_size}px;
                text-align: center;
                padding: 0px 14px;
                {border_css}
            }}
            QPushButton:hover {{ background-color: {hov}; }}
        """)
        return btn

    def create_split_btn(self, text, bg, color, radius, height, icon_name, font_size=18, icon_size=40):
        btn = QPushButton()
        btn.setFixedHeight(height)
        btn.setCursor(Qt.CursorShape.PointingHandCursor)
        
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {bg}; border-radius: {radius}px; border: none;
            }}
            QPushButton:hover {{ background-color: {BG_HOVER}; }}
        """)
        
        layout = QHBoxLayout(btn)
        layout.setContentsMargins(30, 0, 30, 0)
        
        icon_lbl = QLabel()
        icon_lbl.setPixmap(qta.icon(icon_name, color=color).pixmap(QSize(icon_size, icon_size)))
        icon_lbl.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        
        text_lbl = QLabel(text)
        text_lbl.setStyleSheet(f"color: {color}; font-weight: bold; font-family: 'Segoe UI', sans-serif; font-size: {font_size}px; background: transparent; border: none;")
        text_lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        text_lbl.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        
        btn.text_lbl = text_lbl 
        
        layout.addWidget(icon_lbl)
        layout.addStretch() 
        layout.addWidget(text_lbl)
        
        return btn

    def otvor_nastavenia(self):
        self.clear_layout(self.overlay_layout)
        
        card = QFrame()
        card.setFixedSize(400, 260)
        card.setStyleSheet(f"background-color: {BG_SEC}; border: 2px solid {BG_TER}; border-radius: 24px;")
        
        layout = QVBoxLayout(card)
        layout.setContentsMargins(30, 30, 30, 30)

        lbl_title = QLabel(self.t('lim_title').format(self.threshold_zlyhania))
        lbl_title.setStyleSheet(f"color: {TEXT_MAIN}; font-size: 24px; font-weight: bold; border: none; font-family: 'Segoe UI', sans-serif;")
        layout.addWidget(lbl_title, alignment=Qt.AlignmentFlag.AlignCenter)
        
        layout.addSpacing(10)
        
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(50, 100)
        slider.setValue(self.threshold_zlyhania)
        slider.setStyleSheet(f"""
            QSlider::groove:horizontal {{ background: {BG_TER}; height: 10px; border-radius: 5px; }}
            QSlider::handle:horizontal {{ background: {ACCENT}; width: 24px; margin: -7px 0; border-radius: 12px; }}
        """)
        
        def update_lbl(val):
            lbl_title.setText(self.t('lim_title').format(val))
            self.threshold_zlyhania = val
            
        slider.valueChanged.connect(update_lbl)
        layout.addWidget(slider)
        
        layout.addStretch()
        btn_close = self.create_btn(self.t('save_close'), BG_TER, TEXT_MUTED, 20, 44, font_size=15, icon_name='fa5s.times')
        btn_close.setStyleSheet(btn_close.styleSheet() + "border: none;")
        btn_close.clicked.connect(self.zatvor_overlay)
        layout.addWidget(btn_close)

        self.overlay_layout.addWidget(card)
        self.overlay_bg.setGeometry(self.rect())
        self.overlay_bg.raise_()
        self.overlay_bg.show()

    def otvor_kalkulacku(self):
        self.clear_layout(self.overlay_layout)

        card = QFrame()
        card.setFixedSize(480, 750)
        card.setStyleSheet(f"background-color: {BG_SEC}; border: 2px solid {BG_TER}; border-radius: 24px;")

        layout = QVBoxLayout(card)
        layout.setContentsMargins(30, 30, 30, 30)

        lbl_title = QLabel(self.t('calc_title'))
        lbl_title.setStyleSheet(f"color: {TEXT_MAIN}; font-size: 24px; font-weight: bold; border: none; font-family: 'Segoe UI', sans-serif;")
        layout.addWidget(lbl_title, alignment=Qt.AlignmentFlag.AlignCenter)
        layout.addSpacing(15)

        seg_event = SegmentedButton([('powerlifting', self.t('powerlifting')), ('bench_only', self.t('bench_only'))], 'powerlifting')
        layout.addWidget(seg_event)
        
        seg_equip = SegmentedButton([('classic', self.t('classic')), ('equipped', self.t('equipped'))], 'classic')
        layout.addWidget(seg_equip)
        
        seg_gender = SegmentedButton([('male', self.t('male')), ('female', self.t('female'))], 'male')
        layout.addWidget(seg_gender)

        seg_unit = SegmentedButton([('kg', 'kg'), ('lbs', 'lbs')], 'kg')
        layout.addWidget(seg_unit)

        layout.addSpacing(20)

        spin_style = f"""
            QDoubleSpinBox {{
                background-color: {BG_TER}; color: {TEXT_MAIN};
                border: 2px solid transparent; border-radius: 8px;
                font-weight: bold; font-family: "Segoe UI", sans-serif; font-size: 16px;
                padding: 4px;
            }}
            QDoubleSpinBox:focus {{
                border: 2px solid {ACCENT};
                background-color: {BG_MAIN};
            }}
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
                width: 0px; height: 0px; border: none; background: transparent;
            }}
        """

        form_layout = QGridLayout()
        form_layout.setSpacing(15)

        lbl_bw = QLabel(self.t('bw'))
        lbl_bw.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 16px; border: none; font-weight: bold;")
        
        spin_bw = CleanFocusSpinBox()
        spin_bw.setRange(0, 1000)
        spin_bw.setValue(0.0)
        spin_bw.setSuffix(' kg')
        spin_bw.setFixedHeight(40)
        spin_bw.setAlignment(Qt.AlignmentFlag.AlignCenter)
        spin_bw.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        spin_bw.setStyleSheet(spin_style)

        lbl_tot = QLabel(self.t('total'))
        lbl_tot.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 16px; border: none; font-weight: bold;")
        
        spin_tot = CleanFocusSpinBox()
        spin_tot.setRange(0, 4000)
        spin_tot.setValue(0.0)
        spin_tot.setSuffix(' kg')
        spin_tot.setFixedHeight(40)
        spin_tot.setAlignment(Qt.AlignmentFlag.AlignCenter)
        spin_tot.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        spin_tot.setStyleSheet(spin_style)

        form_layout.addWidget(lbl_bw, 0, 0)
        form_layout.addWidget(spin_bw, 0, 1)
        form_layout.addWidget(lbl_tot, 1, 0)
        form_layout.addWidget(spin_tot, 1, 1)
        layout.addLayout(form_layout)

        btn_import = self.create_btn(self.t('import_max'), "transparent", ACCENT, 8, 45, hover=BG_SEC, font_size=14, border=BG_TER, icon_name='fa5s.download')
        btn_import.setStyleSheet(btn_import.styleSheet() + " padding: 0px 45px;")
        btn_import.setMinimumWidth(260)
        
        def importuj_z_profilu():
            bw = self.profil["bw"]
            if bw > 0:
                bw_val = bw * 2.20462 if seg_unit.get_value() == 'lbs' else bw
                spin_bw.setValue(bw_val)
                
            ev = seg_event.get_value()
            eq = seg_equip.get_value()
            kat = "raw" if eq == "classic" else "equip"
            
            if ev == "powerlifting":
                tot = self.profil[kat]["squat"] + self.profil[kat]["bench"] + self.profil[kat]["deadlift"]
            else:
                tot = self.profil[kat]["bench"]
                
            tot_val = tot * 2.20462 if seg_unit.get_value() == 'lbs' else tot
            spin_tot.setValue(tot_val)
            
        btn_import.clicked.connect(importuj_z_profilu)
        layout.addWidget(btn_import, alignment=Qt.AlignmentFlag.AlignCenter)

        layout.addSpacing(15)

        lbl_res_text = QLabel(self.t('gl_points'))
        lbl_res_text.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 14px; border: none; font-weight: bold;")
        layout.addWidget(lbl_res_text, alignment=Qt.AlignmentFlag.AlignCenter)
        
        lbl_res = QLabel("0.00")
        lbl_res.setStyleSheet(f"color: {ACCENT}; font-size: 42px; font-weight: bold; border: none; font-family: 'Segoe UI';")
        layout.addWidget(lbl_res, alignment=Qt.AlignmentFlag.AlignCenter)

        COEFFS = {
            'male': {
                'classic': {
                    'powerlifting': (1199.72839, 1025.18162, 0.00921),
                    'bench_only': (320.98041, 281.40258, 0.01008)
                },
                'equipped': {
                    'powerlifting': (1236.25115, 1449.21864, 0.01644),
                    'bench_only': (381.22073, 733.79378, 0.02398)
                }
            },
            'female': {
                'classic': {
                    'powerlifting': (610.32796, 1045.59282, 0.03048),
                    'bench_only': (142.40398, 442.52671, 0.04724)
                },
                'equipped': {
                    'powerlifting': (758.63878, 949.31382, 0.02435),
                    'bench_only': (221.82209, 357.00377, 0.02937)
                }
            }
        }

        def zmen_jednotky(nova_jednotka):
            aktualna = 'lbs' if spin_bw.suffix() == ' lbs' else 'kg'
            
            if nova_jednotka != aktualna:
                spin_bw.blockSignals(True)
                spin_tot.blockSignals(True)
                
                if nova_jednotka == 'lbs':
                    spin_bw.setValue(spin_bw.value() * 2.20462)
                    spin_tot.setValue(spin_tot.value() * 2.20462)
                    spin_bw.setSuffix(' lbs')
                    spin_tot.setSuffix(' lbs')
                else:
                    spin_bw.setValue(spin_bw.value() / 2.20462)
                    spin_tot.setValue(spin_tot.value() / 2.20462)
                    spin_bw.setSuffix(' kg')
                    spin_tot.setSuffix(' kg')
                    
                spin_bw.blockSignals(False)
                spin_tot.blockSignals(False)
                
            prepocti_body()

        def prepocti_body():
            bw = spin_bw.value()
            tot = spin_tot.value()
            gender = seg_gender.get_value()
            equip = seg_equip.get_value()
            event = seg_event.get_value()
            unit = seg_unit.get_value()

            bw_kg = bw / 2.20462 if unit == 'lbs' else bw
            tot_kg = tot / 2.20462 if unit == 'lbs' else tot

            A, B, C = COEFFS[gender][equip][event]

            denom = A - B * math.exp(-C * bw_kg)
            if denom > 0 and tot_kg > 0 and bw_kg > 0:
                gl = tot_kg * 100.0 / denom
                lbl_res.setText(f"{gl:.2f}")
            else:
                lbl_res.setText("0.00")

        seg_event.valueChanged.connect(lambda _: prepocti_body())
        seg_equip.valueChanged.connect(lambda _: prepocti_body())
        seg_gender.valueChanged.connect(lambda _: prepocti_body())
        seg_unit.valueChanged.connect(zmen_jednotky)
        spin_bw.valueChanged.connect(lambda _: prepocti_body())
        spin_tot.valueChanged.connect(lambda _: prepocti_body())
        
        prepocti_body() 

        layout.addStretch()
        btn_close = self.create_btn(self.t('close_cal'), BG_TER, TEXT_MUTED, 22, 44, font_size=15, icon_name='fa5s.times')
        btn_close.setStyleSheet(btn_close.styleSheet() + "border: none;")
        btn_close.clicked.connect(self.zatvor_overlay)
        layout.addWidget(btn_close)

        self.overlay_layout.addWidget(card)
        self.overlay_bg.setGeometry(self.rect())
        self.overlay_bg.raise_()
        self.overlay_bg.show()

    def _vytvor_profil_spinbox(self, max_val=1000):
        spin = CleanFocusSpinBox()
        spin.setRange(0, max_val)
        spin.setSingleStep(2.5)
        spin.setDecimals(1)
        spin.setSuffix(' kg')
        spin.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        spin.setAlignment(Qt.AlignmentFlag.AlignCenter)
        spin.setFixedHeight(36)
        spin.setStyleSheet(f"""
            QDoubleSpinBox {{
                background-color: {BG_TER}; color: {TEXT_MAIN};
                border: 2px solid transparent; border-radius: 8px;
                font-weight: bold; font-family: "Segoe UI", sans-serif; font-size: 14px;
            }}
            QDoubleSpinBox:focus {{
                border: 2px solid {ACCENT}; background-color: {BG_MAIN};
            }}
        """)
        return spin

    def init_uvodne_okno(self):
        self.uvodne_okno = UvodneOkno()
        self.uvodne_okno.setObjectName("uvodne_okno")
        self.uvodne_okno.setStyleSheet(f"#uvodne_okno {{ background-color: {BG_MAIN}; }}")
        
        layout = QHBoxLayout(self.uvodne_okno)
        
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        self.logo_label = QLabel()
        try:
            povodny_img = Image.open("Gemini_Generated_Image_7q2ic27q2ic27q2i.png").convert("RGB")
            invertovany_img = ImageOps.invert(povodny_img)
            img_array = np.array(invertovany_img.convert("RGBA"))
            img_array[:, :, 3] = np.max(img_array[:, :, :3], axis=2)
            img_array = np.require(img_array, np.uint8, 'C')
            h, w, c = img_array.shape
            qImg = QImage(img_array.data, w, h, 4 * w, QImage.Format.Format_RGBA8888)
            pixmap = QPixmap.fromImage(qImg).scaled(660, 660, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
            self.logo_label.setPixmap(pixmap)
        except Exception:
            self.logo_label.setText(self.t('logo_not_found'))
            self.logo_label.setStyleSheet(f"color: {DANGER}; font-size: 20px;")
        
        left_layout.addWidget(self.logo_label, alignment=Qt.AlignmentFlag.AlignCenter)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        right_layout.addStretch(1) 
        
        self.lbl_title_uvod = QLabel(self.t('greeting'))
        self.lbl_title_uvod.setStyleSheet(f"color: {TEXT_MAIN}; font-size: 32px; font-weight: bold; font-family: 'Segoe UI', sans-serif;")
        right_layout.addWidget(self.lbl_title_uvod, alignment=Qt.AlignmentFlag.AlignCenter)
        right_layout.addSpacing(40)

        top_content_layout = QHBoxLayout()
        top_content_layout.setSpacing(40)
        top_content_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.prof_frame = QFrame()
        self.prof_frame.setFixedWidth(380)
        self.prof_frame.setStyleSheet(f"background-color: {BG_SEC}; border-radius: 16px;")
        prof_layout = QVBoxLayout(self.prof_frame)
        prof_layout.setContentsMargins(20, 20, 20, 20)

        self.lbl_prof_nadpis = QLabel(self.t('profile_title'))
        self.lbl_prof_nadpis.setStyleSheet(f"color: {ACCENT}; font-size: 18px; font-weight: bold; font-family: 'Segoe UI', sans-serif;")
        prof_layout.addWidget(self.lbl_prof_nadpis, alignment=Qt.AlignmentFlag.AlignCenter)
        prof_layout.addSpacing(10)

        grid_prof = QGridLayout()
        grid_prof.setSpacing(10)

        self.lbl_prof_raw = QLabel(self.t('prof_raw'))
        self.lbl_prof_raw.setStyleSheet(f"color: {TEXT_MUTED}; font-weight: bold; font-size: 13px;")
        self.lbl_prof_eq = QLabel(self.t('prof_eq'))
        self.lbl_prof_eq.setStyleSheet(f"color: {TEXT_MUTED}; font-weight: bold; font-size: 13px;")
        
        grid_prof.addWidget(self.lbl_prof_raw, 0, 1, alignment=Qt.AlignmentFlag.AlignCenter)
        grid_prof.addWidget(self.lbl_prof_eq, 0, 2, alignment=Qt.AlignmentFlag.AlignCenter)

        self.lbl_prof_bw = QLabel(self.t('bw'))
        self.lbl_prof_bw.setStyleSheet(f"color: {TEXT_MAIN}; font-weight: bold; font-size: 14px;")
        self.spin_prof_bw = self._vytvor_profil_spinbox(300)
        self.spin_prof_bw.setValue(self.profil.get("bw", 0.0))
        grid_prof.addWidget(self.lbl_prof_bw, 1, 0)
        grid_prof.addWidget(self.spin_prof_bw, 1, 1, 1, 2)

        self.lbl_prof_sq = QLabel(self.t('squat'))
        self.lbl_prof_sq.setStyleSheet(f"color: {TEXT_MAIN}; font-weight: bold; font-size: 14px;")
        self.spin_prof_sq_raw = self._vytvor_profil_spinbox()
        self.spin_prof_sq_raw.setValue(self.profil["raw"]["squat"])
        self.spin_prof_sq_eq = self._vytvor_profil_spinbox()
        self.spin_prof_sq_eq.setValue(self.profil["equip"]["squat"])
        grid_prof.addWidget(self.lbl_prof_sq, 2, 0)
        grid_prof.addWidget(self.spin_prof_sq_raw, 2, 1)
        grid_prof.addWidget(self.spin_prof_sq_eq, 2, 2)

        self.lbl_prof_bn = QLabel(self.t('bench'))
        self.lbl_prof_bn.setStyleSheet(f"color: {TEXT_MAIN}; font-weight: bold; font-size: 14px;")
        self.spin_prof_bn_raw = self._vytvor_profil_spinbox()
        self.spin_prof_bn_raw.setValue(self.profil["raw"]["bench"])
        self.spin_prof_bn_eq = self._vytvor_profil_spinbox()
        self.spin_prof_bn_eq.setValue(self.profil["equip"]["bench"])
        grid_prof.addWidget(self.lbl_prof_bn, 3, 0)
        grid_prof.addWidget(self.spin_prof_bn_raw, 3, 1)
        grid_prof.addWidget(self.spin_prof_bn_eq, 3, 2)

        self.lbl_prof_dl = QLabel(self.t('deadlift'))
        self.lbl_prof_dl.setStyleSheet(f"color: {TEXT_MAIN}; font-weight: bold; font-size: 14px;")
        self.spin_prof_dl_raw = self._vytvor_profil_spinbox()
        self.spin_prof_dl_raw.setValue(self.profil["raw"]["deadlift"])
        self.spin_prof_dl_eq = self._vytvor_profil_spinbox()
        self.spin_prof_dl_eq.setValue(self.profil["equip"]["deadlift"])
        grid_prof.addWidget(self.lbl_prof_dl, 4, 0)
        grid_prof.addWidget(self.spin_prof_dl_raw, 4, 1)
        grid_prof.addWidget(self.spin_prof_dl_eq, 4, 2)

        prof_layout.addLayout(grid_prof)
        top_content_layout.addWidget(self.prof_frame, alignment=Qt.AlignmentFlag.AlignTop)

        mode_btns_layout = QVBoxLayout()
        mode_btns_layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.btn_1_uvod = self.create_split_btn(self.t('btn_1'), BG_SEC, ACCENT, 24, 100, icon_name='fa5s.dumbbell', font_size=20, icon_size=42)
        self.btn_1_uvod.setFixedWidth(320)
        self.btn_1_uvod.clicked.connect(lambda: self.spusti_hlavne_ui(1))
        mode_btns_layout.addWidget(self.btn_1_uvod, alignment=Qt.AlignmentFlag.AlignCenter)
        mode_btns_layout.addSpacing(20)

        self.btn_2_uvod = self.create_split_btn(self.t('btn_2'), BG_SEC, "#81c995", 24, 100, icon_name='fa5s.balance-scale', font_size=20, icon_size=42)
        self.btn_2_uvod.setFixedWidth(320)
        self.btn_2_uvod.clicked.connect(lambda: self.spusti_hlavne_ui(2))
        mode_btns_layout.addWidget(self.btn_2_uvod, alignment=Qt.AlignmentFlag.AlignCenter)

        top_content_layout.addLayout(mode_btns_layout)
        right_layout.addLayout(top_content_layout)
        
        right_layout.addSpacing(40)
        
        bottom_btns_layout = QVBoxLayout()
        bottom_btns_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        self.btn_nastavenia_uvod = self.create_btn(self.t('btn_set'), BG_MAIN, TEXT_MUTED, 20, 40, font_size=15, icon_name='fa5s.cog')
        self.btn_nastavenia_uvod.setFixedWidth(320)
        self.btn_nastavenia_uvod.clicked.connect(self.otvor_nastavenia)
        bottom_btns_layout.addWidget(self.btn_nastavenia_uvod, alignment=Qt.AlignmentFlag.AlignCenter)
        bottom_btns_layout.addSpacing(15)
        
        self.btn_dennik_uvod = self.create_btn(self.t('btn_diary'), BG_SEC, TEXT_MAIN, 25, 50, font_size=16, icon_name='fa5s.book')
        self.btn_dennik_uvod.setFixedWidth(280) 
        self.btn_dennik_uvod.clicked.connect(self.otvor_dennik)
        bottom_btns_layout.addWidget(self.btn_dennik_uvod, alignment=Qt.AlignmentFlag.AlignCenter)
        bottom_btns_layout.addSpacing(15)

        self.btn_kalkulacka_uvod = self.create_btn(self.t('btn_calc'), BG_SEC, "#c58af9", 25, 50, font_size=16, icon_name='fa5s.calculator')
        self.btn_kalkulacka_uvod.setFixedWidth(280) 
        self.btn_kalkulacka_uvod.clicked.connect(self.otvor_kalkulacku)
        bottom_btns_layout.addWidget(self.btn_kalkulacka_uvod, alignment=Qt.AlignmentFlag.AlignCenter)

        right_layout.addLayout(bottom_btns_layout)
        
        right_layout.addSpacing(30)
        
        self.btn_lang_uvod = self.create_btn(self.t('lang_switch'), "transparent", TEXT_MUTED, 18, 36, hover=BG_SEC, font_size=15, border=BG_TER, icon_name='fa5s.globe')
        self.btn_lang_uvod.setFixedWidth(100)
        self.btn_lang_uvod.clicked.connect(self.prepni_jazyk)
        right_layout.addWidget(self.btn_lang_uvod, alignment=Qt.AlignmentFlag.AlignCenter)

        right_layout.addStretch(1) 

        layout.addStretch(1)
        layout.addWidget(left_panel, 3)
        layout.addStretch(1)
        layout.addWidget(right_panel, 6)
        layout.addStretch(1)

        self.spin_prof_bw.valueChanged.connect(self.uloz_profil)
        self.spin_prof_sq_raw.valueChanged.connect(self.uloz_profil)
        self.spin_prof_sq_eq.valueChanged.connect(self.uloz_profil)
        self.spin_prof_bn_raw.valueChanged.connect(self.uloz_profil)
        self.spin_prof_bn_eq.valueChanged.connect(self.uloz_profil)
        self.spin_prof_dl_raw.valueChanged.connect(self.uloz_profil)
        self.spin_prof_dl_eq.valueChanged.connect(self.uloz_profil)

    def spusti_hlavne_ui(self, pocet):
        self.pocet_senzorov = pocet
        self.je_historia = False
        self.vykresli_tlacidla_pre_2_senzory()
        
        self.vypni_analyza_mode()
        self.seg_rezim.buttons["single"].setChecked(True)
        
        self.obnov_ui_zoznam()
        self.aktualizuj_grafy()
        self.lbl_ulozit_nadpis.show()
        self.ulozit_frame.show()
        self.btn_kalibracia.show()
        self.btn_live_esp.show()
        self.btn_spat_hlavne.setText(self.t('menu'))
        
        if not self.sidebar_otvoreny:
            self.animuj_sidebar()
            
        self.stacked_widget.setCurrentWidget(self.main_ui_widget)
        self.overlay_buttons_w.hide()

    def init_dennik(self):
        self.dennik_widget = QWidget()
        self.dennik_widget.setStyleSheet(f"background-color: {BG_MAIN};")
        layout = QVBoxLayout(self.dennik_widget)
        layout.setContentsMargins(60, 40, 60, 40)

        top_bar = QHBoxLayout()
        self.btn_spat_dennik = self.create_btn(self.t('back'), BG_SEC, TEXT_MAIN, 20, 40, font_size=16, icon_name='fa5s.arrow-left')
        self.btn_spat_dennik.setFixedWidth(120) 
        self.btn_spat_dennik.clicked.connect(lambda: self.stacked_widget.setCurrentWidget(self.uvodne_okno))
        top_bar.addWidget(self.btn_spat_dennik)

        self.lbl_nadpis_dennik = QLabel(self.t('diary_title'))
        self.lbl_nadpis_dennik.setStyleSheet(f"color: {TEXT_MAIN}; font-size: 26px; font-weight: bold; font-family: 'Segoe UI', sans-serif;")
        top_bar.addWidget(self.lbl_nadpis_dennik)
        top_bar.addStretch()

        self.datum_filter_widget = QWidget()
        datum_layout = QHBoxLayout(self.datum_filter_widget)
        datum_layout.setContentsMargins(0, 0, 15, 0)
        
        self.lbl_datum_filter = QLabel("")
        self.lbl_datum_filter.setStyleSheet(f"color: {ACCENT}; font-size: 15px; font-weight: bold; font-family: 'Segoe UI', sans-serif;")
        datum_layout.addWidget(self.lbl_datum_filter)
        
        self.btn_zrusit_datum = QPushButton("✖")
        self.btn_zrusit_datum.setFixedSize(24, 24)
        self.btn_zrusit_datum.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_zrusit_datum.setStyleSheet(f"QPushButton {{ background: transparent; color: {DANGER}; border: none; font-weight: bold; font-size: 15px; }} QPushButton:hover {{ color: {DANGER_HOVER}; }}")
        self.btn_zrusit_datum.clicked.connect(self.zrus_filter_datumu)
        datum_layout.addWidget(self.btn_zrusit_datum)
        
        self.datum_filter_widget.hide() 
        top_bar.addWidget(self.datum_filter_widget)

        self.btn_kal_dennik = self.create_btn(self.t('calendar'), BG_SEC, ACCENT, 20, 40, font_size=16, icon_name='fa5s.calendar-alt')
        self.btn_kal_dennik.setFixedWidth(120)
        self.btn_kal_dennik.clicked.connect(self.zobraz_kalendar)
        top_bar.addWidget(self.btn_kal_dennik)

        self.btn_export = QPushButton("Export CSV")
        self.btn_export.setFixedSize(140, 40)
        self.btn_export.setStyleSheet(f"""
            QPushButton {{ border: 1px solid {ACCENT}; border-radius: 20px; color: {ACCENT}; font-weight: bold; font-family: 'Segoe UI', sans-serif; font-size: 15px; background: transparent; }}
            QPushButton:hover {{ background-color: {BG_TER}; }}
            QPushButton:disabled {{ border: 1px solid #555; color: #555; }}
        """)
        self.btn_export.setEnabled(False)
        self.btn_export.clicked.connect(self.exportuj_vybrane_csv)
        top_bar.addWidget(self.btn_export)

        self.btn_zmazat = QPushButton(self.t('del_sel'))
        self.btn_zmazat.setFixedSize(160, 40)
        self.btn_zmazat.setStyleSheet(f"""
            QPushButton {{ border: 1px solid {DANGER}; border-radius: 20px; color: {DANGER}; font-weight: bold; font-family: 'Segoe UI', sans-serif; font-size: 15px; background: transparent; }}
            QPushButton:hover {{ background-color: {BG_TER}; }}
            QPushButton:disabled {{ border: 1px solid #555; color: #555; }}
        """)
        self.btn_zmazat.setEnabled(False)
        self.btn_zmazat.clicked.connect(self.zmazat_vybrane)
        top_bar.addWidget(self.btn_zmazat)
        layout.addLayout(top_bar)

        self.filter_cvik = SegmentedButton([
            ('all', self.t('all')), 
            ('squat', self.t('squat')), 
            ('bench', self.t('bench')), 
            ('deadlift', self.t('deadlift'))
        ], 'all')
        self.filter_cvik.valueChanged.connect(self.zmen_filter_cviku)
        layout.addWidget(self.filter_cvik) 

        self.scroll_dennik = QScrollArea()
        self.scroll_dennik.setWidgetResizable(True)
        self.scroll_dennik.setStyleSheet("QScrollArea { border: none; background-color: transparent; }")
        self.zoznam_dennik_widget = QWidget()
        self.zoznam_dennik_widget.setStyleSheet("background-color: transparent;")
        self.zoznam_dennik_layout = QVBoxLayout(self.zoznam_dennik_widget)
        self.zoznam_dennik_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.scroll_dennik.setWidget(self.zoznam_dennik_widget)
        layout.addWidget(self.scroll_dennik)

    def otvor_dennik(self):
        self.oznacene_zaznamy.clear()
        self.zrus_filter_datumu() 
        self.prekresli_dennik()
        self.stacked_widget.setCurrentWidget(self.dennik_widget)
        self.overlay_buttons_w.hide()

    def zmen_filter_cviku(self, cvik_key):
        self.aktualny_filter_cviku = cvik_key
        self.prekresli_dennik()

    def spocitaj_e1rm(self, cvik, load, mcv):
        if load <= 0 or mcv <= 0: return 0.0
        cvik = cvik.lower()
        if 'squat' in cvik or 'drep' in cvik: v0, v1 = 1.3, 0.30
        elif 'bench' in cvik or 'tlak' in cvik: v0, v1 = 1.2, 0.15
        elif 'deadlift' in cvik or 'tah' in cvik or 'ťah' in cvik: v0, v1 = 1.0, 0.15
        else: v0, v1 = 1.2, 0.20
        if mcv < v1: mcv = v1
        if mcv > v0: mcv = v0
        perc = 100 - ((mcv - v1) / (v0 - v1)) * 100
        if perc <= 0: return 0.0
        return load * (100.0 / perc)

    def prekresli_dennik(self):
        self.clear_layout(self.zoznam_dennik_layout)

        try:
            with gzip.open(self.subor_dennika, "rt", encoding="utf-8") as f:
                dennik = json.load(f)
        except Exception:
            dennik = []

        legacy_map = {"Drep": "squat", "Tlak": "bench", "Mŕtvy ťah": "deadlift", "Všetky": "all"}
        zobrazene = 0
        
        for zaznam in reversed(dennik):
            db_cvik = zaznam.get("cvik", "")
            cvik_key = legacy_map.get(db_cvik, db_cvik)
            
            if self.aktualny_filter_cviku != 'all' and cvik_key != self.aktualny_filter_cviku: continue
            if self.aktualny_datum_filter and not zaznam["timestamp"].startswith(self.aktualny_datum_filter): continue
            
            zobrazene += 1
            karta = QFrame()
            karta.setStyleSheet(f"QFrame {{ background-color: {BG_SEC}; border-radius: 20px; }}")
            karta_layout = QHBoxLayout(karta)
            karta_layout.setContentsMargins(20, 15, 20, 15)

            chk = QCheckBox()
            chk.setStyleSheet(f"""
                QCheckBox::indicator {{ width: 20px; height: 20px; border-radius: 6px; border: 1px solid {BG_HOVER}; background: transparent; }}
                QCheckBox::indicator:checked {{ background: {ACCENT}; }}
            """)
            chk.stateChanged.connect(lambda state, z=zaznam['id']: self.zmen_oznacenie(z, state))
            karta_layout.addWidget(chk)

            info_layout = QVBoxLayout()
            senzor_text = self.t('1_sens_txt') if zaznam["senzory"] == 1 else self.t('2_sens_txt')
            
            max_e1rm = 0.0
            for set_key, set_data in zaznam.get("data", {}).items():
                w = zaznam.get("weights", {}).get(set_key, 0.0)
                if w > 0:
                    for rep_list in set_data:
                        reps = []
                        if zaznam["senzory"] == 1 and len(rep_list) > 10: reps = rep_list[10]
                        elif zaznam["senzory"] == 2 and len(rep_list) > 12: reps = rep_list[12]
                        if reps and len(reps) > 0:
                            best_mcv = max([r.get('mcv', 0) for r in reps])
                            e = self.spocitaj_e1rm(cvik_key, w, best_mcv)
                            if e > max_e1rm: max_e1rm = e

            e1rm_text = f" • e1RM: {max_e1rm:.1f} kg" if max_e1rm > 0 else ""
            lbl_time = QLabel(f"{zaznam['timestamp']} • {senzor_text}{e1rm_text}")
            lbl_time.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 14px; font-family: 'Segoe UI', sans-serif;")
            lbl_cvik = QLabel(self.t(cvik_key))
            lbl_cvik.setStyleSheet(f"color: {TEXT_MAIN}; font-size: 22px; font-weight: bold; font-family: 'Segoe UI', sans-serif;")
            info_layout.addWidget(lbl_time)
            info_layout.addWidget(lbl_cvik)
            karta_layout.addLayout(info_layout)
            karta_layout.addStretch()

            btn_analyza = self.create_btn(self.t('analyze'), BG_TER, ACCENT, 18, 36, font_size=15, icon_name='fa5s.chart-line')
            btn_analyza.setFixedWidth(130)
            btn_analyza.clicked.connect(lambda checked, z=zaznam: self.nacitat_trening(z))
            karta_layout.addWidget(btn_analyza)

            self.zoznam_dennik_layout.addWidget(karta)

        if zobrazene == 0:
            lbl = QLabel(self.t('no_rec'))
            lbl.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 16px; font-family: 'Segoe UI', sans-serif;")
            self.zoznam_dennik_layout.addWidget(lbl, alignment=Qt.AlignmentFlag.AlignCenter)
            
        if len(self.oznacene_zaznamy) > 0:
            self.btn_zmazat.setEnabled(True)
            self.btn_zmazat.setText(self.t('del_count').format(len(self.oznacene_zaznamy)))
            self.btn_zmazat.setStyleSheet(f"QPushButton {{ background-color: {DANGER}; border-radius: 20px; color: {BG_MAIN}; font-weight: bold; font-family: 'Segoe UI', sans-serif; font-size: 15px; }}")
        else:
            self.btn_zmazat.setEnabled(False)
            self.btn_zmazat.setText(self.t('del_sel'))
            self.btn_zmazat.setStyleSheet(f"QPushButton {{ border: 1px solid {DANGER}; border-radius: 20px; color: {DANGER}; font-weight: bold; font-family: 'Segoe UI', sans-serif; font-size: 15px; background: transparent; }}")

    def zmen_oznacenie(self, id_zaznamu, state):
        if state == Qt.CheckState.Checked.value: self.oznacene_zaznamy.add(id_zaznamu)
        else: self.oznacene_zaznamy.discard(id_zaznamu)
        
        if self.oznacene_zaznamy:
            self.btn_zmazat.setEnabled(True)
            self.btn_export.setEnabled(True)
            self.btn_zmazat.setText(self.t('del_count').format(len(self.oznacene_zaznamy)))
            self.btn_zmazat.setStyleSheet(f"QPushButton {{ background-color: {DANGER}; border-radius: 20px; color: {BG_MAIN}; font-weight: bold; font-family: 'Segoe UI', sans-serif; font-size: 15px; }}")
        else:
            self.btn_zmazat.setEnabled(False)
            self.btn_export.setEnabled(False)
            self.btn_zmazat.setText(self.t('del_sel'))
            self.btn_zmazat.setStyleSheet(f"QPushButton {{ border: 1px solid {DANGER}; border-radius: 20px; color: {DANGER}; font-weight: bold; font-family: 'Segoe UI', sans-serif; font-size: 15px; background: transparent; }}")

    def exportuj_vybrane_csv(self):
        if not self.oznacene_zaznamy: return
        try:
            with gzip.open(self.subor_dennika, "rt", encoding="utf-8") as f:
                dennik = json.load(f)
        except Exception:
            return
        
        cesta, _ = QFileDialog.getSaveFileName(self, "Export CSV", "export.csv", "CSV Files (*.csv)")
        if not cesta: return
        
        import csv
        with open(cesta, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['ID', 'Datum', 'Cvik', 'Seria', 'Hmotnost', 'MCV', 'PeakVel', 'ROM', 'StickingPoint'])
            
            for zaznam in reversed(dennik):
                if zaznam['id'] in self.oznacene_zaznamy:
                    for s_key, s_data in zaznam.get('data', {}).items():
                        w = zaznam.get('weights', {}).get(s_key, 0.0)
                        for rep_list in s_data:
                            reps = []
                            if zaznam["senzory"] == 1 and len(rep_list) > 10: reps = rep_list[10]
                            elif zaznam["senzory"] == 2 and len(rep_list) > 12: reps = rep_list[12]
                            
                            for rep in reps:
                                writer.writerow([
                                    zaznam['id'], zaznam['timestamp'], zaznam['cvik'], s_key, w,
                                    rep.get('mcv', 0), rep.get('peak_v', 0), rep.get('rom', 0), rep.get('sp', 0)
                                ])
        
        QMessageBox.information(self, "Export", f"Export úspešný do:\n{cesta}")

    def zmazat_vybrane(self):
        if not self.oznacene_zaznamy: return
        with gzip.open(self.subor_dennika, "rt", encoding="utf-8") as f: 
            dennik = json.load(f)
            
        dennik = [z for z in dennik if z["id"] not in self.oznacene_zaznamy]
        
        with gzip.open(self.subor_dennika, "wt", encoding="utf-8") as f: 
            json.dump(dennik, f)
            
        self.oznacene_zaznamy.clear()
        self.btn_zmazat.setEnabled(False)
        self.btn_zmazat.setText(self.t('del_sel'))
        self.btn_zmazat.setStyleSheet(f"QPushButton {{ border: 1px solid {DANGER}; border-radius: 20px; color: {DANGER}; font-weight: bold; font-family: 'Segoe UI', sans-serif; font-size: 15px; background: transparent; }}")
        self.prekresli_dennik()

    def vykresli_dní_v_kalendari(self):
        self.clear_layout(self.kalendar_grid)
        
        with gzip.open(self.subor_dennika, "rt", encoding="utf-8") as f: 
            dennik = json.load(f)
        treningove_dni = {z["timestamp"].split(" ")[0] for z in dennik}

        _, pocet_dni = calendar.monthrange(self.kalendar_rok, self.kalendar_mesiac)

        for day in range(1, pocet_dni + 1):
            day_str = f"{day:02d}.{self.kalendar_mesiac:02d}.{self.kalendar_rok}"
            is_tr = day_str in treningove_dni
            btn = QPushButton(str(day))
            btn.setFixedSize(40, 40)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            
            fg = ACCENT if is_tr else BG_MAIN
            txt = TEXT_ACCENT if is_tr else TEXT_MAIN
            hov = ACCENT_HOVER if is_tr else BG_TER
            
            btn.setStyleSheet(f"""
                QPushButton {{ background-color: {fg}; color: {txt}; border: none; border-radius: 20px; font-weight: bold; font-size: 16px; font-family: 'Segoe UI', sans-serif; }}
                QPushButton:hover {{ background-color: {hov}; }}
            """)
            btn.clicked.connect(lambda checked, d=day_str: self.filter_podla_datumu(d))
            self.kalendar_grid.addWidget(btn, (day-1)//7, (day-1)%7)

    def zmen_mesiac_kalendara(self, delta):
        self.kalendar_mesiac += delta
        if self.kalendar_mesiac > 12:
            self.kalendar_mesiac = 1
            self.kalendar_rok += 1
        elif self.kalendar_mesiac < 1:
            self.kalendar_mesiac = 12
            self.kalendar_rok -= 1
            
        nazov_mes = self.t('months')[self.kalendar_mesiac - 1]
        self.kalendar_lbl.setText(f"{nazov_mes} {self.kalendar_rok}")
        self.vykresli_dní_v_kalendari()

    def zobraz_kalendar(self):
        self.clear_layout(self.overlay_layout)
        
        card = QFrame()
        card.setFixedSize(400, 480)
        card.setStyleSheet(f"background-color: {BG_SEC}; border: 2px solid {BG_TER}; border-radius: 24px;")

        layout = QVBoxLayout(card)
        layout.setContentsMargins(30, 30, 30, 30)

        nav_layout = QHBoxLayout()
        btn_prev = self.create_btn("<", "transparent", TEXT_MUTED, 15, 30, hover=BG_TER, font_size=18)
        btn_prev.clicked.connect(lambda: self.zmen_mesiac_kalendara(-1))
        
        self.kalendar_lbl = QLabel()
        nazov_mes = self.t('months')[self.kalendar_mesiac - 1]
        self.kalendar_lbl.setText(f"{nazov_mes} {self.kalendar_rok}")
        self.kalendar_lbl.setStyleSheet(f"color: {TEXT_MAIN}; font-size: 24px; font-weight: bold; border: none; font-family: 'Segoe UI', sans-serif;")
        self.kalendar_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        btn_next = self.create_btn(">", "transparent", TEXT_MUTED, 15, 30, hover=BG_TER, font_size=18)
        btn_next.clicked.connect(lambda: self.zmen_mesiac_kalendara(1))

        nav_layout.addWidget(btn_prev)
        nav_layout.addWidget(self.kalendar_lbl, stretch=1)
        nav_layout.addWidget(btn_next)
        
        layout.addLayout(nav_layout)
        layout.addSpacing(15)
        
        self.kalendar_grid = QGridLayout()
        self.kalendar_grid.setSpacing(10)
        
        self.vykresli_dní_v_kalendari()
            
        layout.addLayout(self.kalendar_grid)
        layout.addStretch()
        
        btn_close = self.create_btn(self.t('close_cal'), BG_TER, TEXT_MUTED, 22, 44, font_size=15, icon_name='fa5s.times')
        btn_close.setStyleSheet(btn_close.styleSheet() + "border: none;")
        btn_close.clicked.connect(self.zatvor_overlay)
        layout.addWidget(btn_close)

        self.overlay_layout.addWidget(card)
        self.overlay_bg.setGeometry(self.rect())
        self.overlay_bg.raise_()
        self.overlay_bg.show()

    def filter_podla_datumu(self, d):
        self.aktualny_datum_filter = d
        self.lbl_datum_filter.setText(self.t('filter').format(d))
        self.datum_filter_widget.show() 
        self.zatvor_overlay()
        self.prekresli_dennik()

    def zrus_filter_datumu(self):
        self.aktualny_datum_filter = None
        self.datum_filter_widget.hide() 
        self.prekresli_dennik()

    # --- MAIN UI ---
    def init_hlavne_ui(self):
        self.main_ui_widget = QWidget()
        self.main_ui_widget.setStyleSheet(f"background-color: {BG_MAIN};")
        layout = QHBoxLayout(self.main_ui_widget)
        layout.setContentsMargins(20, 20, 20, 20)

        self.sidebar = QFrame()
        self.sidebar.setMinimumWidth(380)
        self.sidebar.setMaximumWidth(380)
        self.sidebar_otvoreny = True
        self.sidebar.setStyleSheet(f"QFrame {{ background-color: {BG_SEC}; border-radius: 24px; }}")
        
        side_layout = QVBoxLayout(self.sidebar)
        side_layout.setContentsMargins(20, 25, 20, 25)

        self.btn_spat_hlavne = self.create_btn(self.t('menu'), BG_TER, TEXT_MAIN, 18, 36, font_size=15, icon_name='fa5s.arrow-left')
        self.btn_spat_hlavne.setFixedWidth(100)
        self.btn_spat_hlavne.clicked.connect(self.ukonci_trening)
        side_layout.addWidget(self.btn_spat_hlavne, alignment=Qt.AlignmentFlag.AlignLeft)
        side_layout.addSpacing(15)

        h_kalib = QHBoxLayout()
        self.btn_kalibracia = self.create_btn(self.t('calib_btn'), BG_TER, ACCENT, 22, 44, hover=BG_HOVER, font_size=15, icon_name='fa5s.tools')
        self.btn_kalibracia.clicked.connect(self.spusti_kalibraciu)
        h_kalib.addWidget(self.btn_kalibracia)
        
        self.btn_kalibracia_accel = self.create_btn(self.t('calib_accel_btn'), BG_TER, "#c58af9", 22, 44, hover=BG_HOVER, font_size=15, icon_name='fa5s.cube')
        self.btn_kalibracia_accel.clicked.connect(self.spusti_accel_kalibraciu)
        h_kalib.addWidget(self.btn_kalibracia_accel)
        
        side_layout.addLayout(h_kalib)
        
        self.btn_live_esp = self.create_btn(self.t('record_live'), DANGER, BG_MAIN, 22, 44, hover=DANGER_HOVER, font_size=15, icon_name='fa5s.broadcast-tower')
        self.btn_live_esp.clicked.connect(self.toggle_esp32_stream)
        side_layout.addWidget(self.btn_live_esp)

        self.frame_2_senzory = QFrame()
        self.layout_2_senzory = QVBoxLayout(self.frame_2_senzory)
        self.layout_2_senzory.setContentsMargins(0,0,0,0)
        side_layout.addWidget(self.frame_2_senzory)

        self.seg_rezim = SegmentedButton([('single', self.t('single')), ('custom', self.t('custom'))], 'single')
        self.seg_rezim.valueChanged.connect(self.zmen_rezim)
        side_layout.addWidget(self.seg_rezim)

        self.analyza_frame = QFrame()
        self.analyza_frame.setStyleSheet("QFrame { background: transparent; }")
        analyza_layout = QHBoxLayout(self.analyza_frame)
        analyza_layout.setContentsMargins(0, 5, 0, 10)
        analyza_layout.setSpacing(8)

        self.btn_analyza_unavy_hlavne = QPushButton(self.t('compare'))
        self.btn_analyza_unavy_hlavne.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_analyza_unavy_hlavne.setFixedHeight(36)
        self.style_btn_analyza(False)
        self.btn_analyza_unavy_hlavne.clicked.connect(self.toggle_analyza_mode)
        analyza_layout.addWidget(self.btn_analyza_unavy_hlavne)

        self.btn_check_fatigue = QPushButton(self.t('btn_check_fatigue'))
        self.btn_check_fatigue.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_check_fatigue.setFixedHeight(36)
        self.btn_check_fatigue.setStyleSheet(f"""
            QPushButton {{ background-color: transparent; border: 2px solid #ffb74d; color: #ffb74d; border-radius: 12px; font-weight: bold; font-family: "Segoe UI", sans-serif; font-size: 14px; }}
            QPushButton:hover {{ background-color: #ffb74d; color: {BG_MAIN}; }}
        """)
        self.btn_check_fatigue.clicked.connect(self.skontroluj_pokles_rychlosti)
        analyza_layout.addWidget(self.btn_check_fatigue)

        self.btn_analyza_reset = QPushButton("↺")
        self.btn_analyza_reset.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_analyza_reset.setFixedSize(36, 36)
        self.btn_analyza_reset.setStyleSheet(f"""
            QPushButton {{ background-color: transparent; border: 2px solid {DANGER}; color: {DANGER}; border-radius: 18px; font-weight: bold; font-size: 18px; }}
            QPushButton:hover {{ background-color: {DANGER}; color: {BG_MAIN}; }}
        """)
        self.btn_analyza_reset.clicked.connect(self.reset_analyza_vyber)
        self.btn_analyza_reset.hide()
        analyza_layout.addWidget(self.btn_analyza_reset)
        
        side_layout.addWidget(self.analyza_frame)

        self.scroll_pokusy = QScrollArea()
        self.scroll_pokusy.setWidgetResizable(True)
        self.scroll_pokusy.setStyleSheet("QScrollArea { border: none; background-color: transparent; }")
        self.zoznam_pokusov_w = QWidget()
        self.zoznam_pokusov_w.setStyleSheet("background-color: transparent;")
        self.zoznam_pokusov_layout = QVBoxLayout(self.zoznam_pokusov_w)
        self.zoznam_pokusov_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.scroll_pokusy.setWidget(self.zoznam_pokusov_w)
        side_layout.addWidget(self.scroll_pokusy)

        self.lbl_ulozit_nadpis = QLabel(self.t('save_title'))
        self.lbl_ulozit_nadpis.setStyleSheet(f"color: {TEXT_MUTED}; font-weight: bold; font-family: 'Segoe UI', sans-serif; font-size: 14px;")
        self.lbl_ulozit_nadpis.setAlignment(Qt.AlignmentFlag.AlignCenter)
        side_layout.addWidget(self.lbl_ulozit_nadpis)

        self.ulozit_frame = QFrame()
        ulozit_layout = QHBoxLayout(self.ulozit_frame)
        ulozit_layout.setContentsMargins(0,0,0,0)
        ulozit_layout.setSpacing(8)
        
        self.btn_ulozit_cviky = {}
        for k in ['squat', 'bench', 'deadlift']:
            b = self.create_btn(self.t(k), BG_TER, TEXT_MAIN, 16, 32, font_size=14)
            b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
            b.clicked.connect(lambda checked, cvik_key=k: self.ulozit_trening(cvik_key))
            ulozit_layout.addWidget(b)
            self.btn_ulozit_cviky[k] = b
            
        side_layout.addWidget(self.ulozit_frame)

        layout.addWidget(self.sidebar)

        self.graph_container = QWidget()
        graph_layout = QVBoxLayout(self.graph_container)
        graph_layout.setContentsMargins(0, 0, 0, 0)

        top_bar_graphs = QHBoxLayout()
        
        self.btn_toggle_sidebar = self.create_btn("☰", BG_TER, TEXT_MAIN, 18, 36, font_size=18)
        self.btn_toggle_sidebar.setFixedWidth(42)
        self.btn_toggle_sidebar.clicked.connect(self.animuj_sidebar)
        top_bar_graphs.addWidget(self.btn_toggle_sidebar)
        top_bar_graphs.addSpacing(10)
        
        self.lbl_porovnanie = QLabel("")
        self.lbl_porovnanie.setStyleSheet(f"color: {ACCENT}; font-size: 16px; font-weight: bold; font-family: 'Segoe UI';")
        top_bar_graphs.addWidget(self.lbl_porovnanie)
        
        self.lbl_varovanie_ikona = QLabel()
        self.lbl_varovanie_ikona.setPixmap(qta.icon('fa5s.exclamation-triangle', color=DANGER).pixmap(QSize(22, 22)))
        self.lbl_varovanie_ikona.setCursor(Qt.CursorShape.PointingHandCursor)
        self.lbl_varovanie_ikona.hide() 
        top_bar_graphs.addWidget(self.lbl_varovanie_ikona)
        
        top_bar_graphs.addStretch()

        self.lbl_bat_hlavne = QLabel(self.t('battery').format(self.bateria_percent, self.bateria_napatie) if self.bateria_percent > 0 else "")
        self.lbl_bat_hlavne.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 14px; font-weight: bold; font-family: 'Segoe UI'; margin-right: 15px;")
        top_bar_graphs.addWidget(self.lbl_bat_hlavne)

        self.lbl_tip_hlavne = QLabel(self.t('tip'))
        self.lbl_tip_hlavne.setStyleSheet(f"color: {ACCENT}; font-size: 15px; font-weight: bold; font-family: 'Segoe UI';")
        self.lbl_tip_hlavne.hide() 
        top_bar_graphs.addWidget(self.lbl_tip_hlavne)

        # --- 1. BUTTONS AND INDICATORS FOR BOUNDARIES ---
        self.btn_hranice_y = self.create_btn("Hranice Y", BG_TER, ACCENT, 16, 32, font_size=13)
        self.btn_hranice_y.clicked.connect(self.toggle_hranice_y)
        self.btn_hranice_y.hide()
        
        self.lbl_vzdialenost_y = QLabel("")
        self.lbl_vzdialenost_y.setStyleSheet(f"color: {ACCENT}; font-weight: bold; font-family: 'Segoe UI'; font-size: 14px; margin-right: 10px;")
        self.lbl_vzdialenost_y.hide()
        
        self.btn_hranice_z = self.create_btn("Hranice Z", BG_TER, ACCENT, 16, 32, font_size=13)
        self.btn_hranice_z.clicked.connect(self.toggle_hranice_z)
        self.btn_hranice_z.hide()
        
        self.lbl_vzdialenost_z = QLabel("")
        self.lbl_vzdialenost_z.setStyleSheet(f"color: {ACCENT}; font-weight: bold; font-family: 'Segoe UI'; font-size: 14px; margin-right: 10px;")
        self.lbl_vzdialenost_z.hide()
        
        self.btn_reset_hranic = self.create_btn("Zmazať", BG_TER, DANGER, 16, 32, font_size=13)
        self.btn_reset_hranic.clicked.connect(self.reset_hranice)
        self.btn_reset_hranic.hide()
        
        top_bar_graphs.addWidget(self.btn_hranice_y)
        top_bar_graphs.addWidget(self.lbl_vzdialenost_y)
        top_bar_graphs.addWidget(self.btn_hranice_z)
        top_bar_graphs.addWidget(self.lbl_vzdialenost_z)
        top_bar_graphs.addWidget(self.btn_reset_hranic)

        graph_layout.addLayout(top_bar_graphs)

        self.gw = pg.GraphicsLayoutWidget()
        graph_layout.addWidget(self.gw, stretch=1)
        layout.addWidget(self.graph_container, stretch=1)
        
        self.ax1 = self.gw.addPlot(row=0, col=0)
        self.ax2 = self.gw.addPlot(row=1, col=0)
        self.ax3 = self.gw.addPlot(row=0, col=1)
        self.ax4 = self.gw.addPlot(row=1, col=1)

        for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
            ax.showGrid(x=True, y=True, alpha=0.2)
            ax.getAxis('bottom').setPen(BG_HOVER)
            ax.getAxis('left').setPen(BG_HOVER)
            ax.setMouseEnabled(x=False, y=False)
            ax.enableAutoRange() 
            ax.hideButtons()
            
            vLine = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen(color=ACCENT, width=1.5, style=Qt.PenStyle.DashLine))
            vLine.setVisible(False)
            ax.addItem(vLine)
            ax.vLine = vLine

        # --- 2. DASHED LINES (BOUNDARIES) FOR AX3 ---
        pen_hranica = pg.mkPen(color="#fbbc04", width=2.5, style=Qt.PenStyle.DashLine) # Orange color for high contrast
        
        self.hranica_y_min = pg.InfiniteLine(angle=0, movable=True, pen=pen_hranica)
        self.hranica_y_max = pg.InfiniteLine(angle=0, movable=True, pen=pen_hranica)
        self.hranica_z_min = pg.InfiniteLine(angle=0, movable=True, pen=pen_hranica)
        self.hranica_z_max = pg.InfiniteLine(angle=0, movable=True, pen=pen_hranica)
        
        for h in [self.hranica_y_min, self.hranica_y_max]:
            h.setVisible(False)
            h.is_boundary = True 
            if hasattr(self, 'ax4') and self.ax4:
                self.ax4.addItem(h)
            else:
                self.ax3.addItem(h)
                
        for h in [self.hranica_z_min, self.hranica_z_max]:
            h.setVisible(False)
            h.is_boundary = True 
            self.ax3.addItem(h)

        # Connect signals for dynamic distance calculation when dragging the line
        self.hranica_y_min.sigPositionChanged.connect(self.aktualizuj_vzdialenost_y)
        self.hranica_y_max.sigPositionChanged.connect(self.aktualizuj_vzdialenost_y)
        self.hranica_z_min.sigPositionChanged.connect(self.aktualizuj_vzdialenost_z)
        self.hranica_z_max.sigPositionChanged.connect(self.aktualizuj_vzdialenost_z)

        font_avg = pg.QtGui.QFont("Segoe UI", 10, pg.QtGui.QFont.Weight.Bold)
        
        self.avg_text_acc = pg.TextItem("", fill=pg.mkBrush(BG_TER), border=pg.mkPen(ACCENT), color=TEXT_MAIN, anchor=(1, 0))
        self.avg_text_acc.setFont(font_avg)
        self.avg_text_acc.setZValue(10)
        self.avg_text_acc.setVisible(False)
        self.ax1.addItem(self.avg_text_acc, ignoreBounds=True)

        self.avg_text_vel = pg.TextItem("", fill=pg.mkBrush(BG_TER), border=pg.mkPen(ACCENT), color=TEXT_MAIN, anchor=(1, 0))
        self.avg_text_vel.setFont(font_avg)
        self.avg_text_vel.setZValue(10)
        self.avg_text_vel.setVisible(False)
        self.ax2.addItem(self.avg_text_vel, ignoreBounds=True)

        self.ax1.sigRangeChanged.connect(self.update_avg_pos)
        self.ax2.sigRangeChanged.connect(self.update_avg_pos)

        font = pg.QtGui.QFont("Segoe UI", 10)
        self.tooltip1 = pg.TextItem(fill=pg.mkBrush(BG_TER), border=pg.mkPen(ACCENT), color=TEXT_MAIN)
        self.tooltip1.setFont(font); self.ax1.addItem(self.tooltip1, ignoreBounds=True); self.tooltip1.setVisible(False)

        self.tooltip2 = pg.TextItem(fill=pg.mkBrush(BG_TER), border=pg.mkPen(ACCENT), color=TEXT_MAIN)
        self.tooltip2.setFont(font); self.ax2.addItem(self.tooltip2, ignoreBounds=True); self.tooltip2.setVisible(False)

        self.tooltip3 = pg.TextItem(fill=pg.mkBrush(BG_TER), border=pg.mkPen(ACCENT), color=TEXT_MAIN)
        self.tooltip3.setFont(font); self.ax3.addItem(self.tooltip3, ignoreBounds=True); self.tooltip3.setVisible(False)

        self.tooltip4 = pg.TextItem(fill=pg.mkBrush(BG_TER), border=pg.mkPen(ACCENT), color=TEXT_MAIN)
        self.tooltip4.setFont(font); self.ax4.addItem(self.tooltip4, ignoreBounds=True); self.tooltip4.setVisible(False)

        self.proxy = pg.SignalProxy(self.gw.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved)

    def aktualizuj_bateriu(self, percento, napatie):
        self.bateria_percent = percento
        self.bateria_napatie = napatie
        self.lbl_bat_hlavne.setText(self.t('battery').format(percento, napatie))
        
        if percento <= 20:
            self.lbl_bat_hlavne.setStyleSheet(f"color: {DANGER}; font-size: 14px; font-weight: bold; font-family: 'Segoe UI'; margin-right: 15px;")
        elif percento <= 50:
            self.lbl_bat_hlavne.setStyleSheet(f"color: #fbbc04; font-size: 14px; font-weight: bold; font-family: 'Segoe UI'; margin-right: 15px;")
        else:
            self.lbl_bat_hlavne.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 14px; font-weight: bold; font-family: 'Segoe UI'; margin-right: 15px;")

    def update_avg_pos(self, *args):
        if hasattr(self, 'avg_text_acc') and self.avg_text_acc.isVisible():
            x_range, y_range = self.ax1.vb.viewRange()
            x_pos = x_range[1] - (x_range[1] - x_range[0]) * 0.02
            y_pos = y_range[1] - (y_range[1] - y_range[0]) * 0.05
            self.avg_text_acc.setPos(x_pos, y_pos)
            
        if hasattr(self, 'avg_text_vel') and self.avg_text_vel.isVisible():
            x_range, y_range = self.ax2.vb.viewRange()
            x_pos = x_range[1] - (x_range[1] - x_range[0]) * 0.02
            y_pos = y_range[1] - (y_range[1] - y_range[0]) * 0.05
            self.avg_text_vel.setPos(x_pos, y_pos)

    def animuj_sidebar(self):
        self.sidebar_otvoreny = not self.sidebar_otvoreny
        ciel = 380 if self.sidebar_otvoreny else 0
        aktualna_sirka = self.sidebar.width()
        
        if self.sidebar_otvoreny:
            self.btn_toggle_sidebar.setStyleSheet(f"""
                QPushButton {{ background-color: {BG_TER}; color: {TEXT_MAIN}; border-radius: 18px; font-weight: bold; font-family: "Segoe UI"; font-size: 18px; border: none; }}
                QPushButton:hover {{ background-color: {BG_HOVER}; }}
            """)
            self.lbl_tip_hlavne.hide()
            self.btn_hranice_y.hide()
            self.btn_hranice_z.hide()
            self.btn_reset_hranic.hide()
            self.lbl_vzdialenost_y.hide()  # <--- New
            self.lbl_vzdialenost_z.hide()  # <--- New
            
            for ax in [self.ax1, self.ax2, self.ax3]:
                ax.setMouseEnabled(x=False, y=False)
            
            self.aktualizuj_grafy()
            
        else:
            self.btn_toggle_sidebar.setStyleSheet(f"""
                QPushButton {{ background-color: {ACCENT}; color: {BG_MAIN}; border-radius: 18px; font-weight: bold; font-family: "Segoe UI"; font-size: 18px; border: none; }}
                QPushButton:hover {{ background-color: {ACCENT_HOVER}; }}
            """)
            self.lbl_tip_hlavne.show()
            self.btn_hranice_y.show()
            self.btn_hranice_z.show()
            self.btn_reset_hranic.show()
            # Show indicators only if the respective lines are active
            if self.hranica_y_min.isVisible(): self.lbl_vzdialenost_y.show()
            if self.hranica_z_min.isVisible(): self.lbl_vzdialenost_z.show()

            for ax in [self.ax1, self.ax2, self.ax3]:
                ax.setMouseEnabled(x=True, y=True)
                ax.disableAutoRange() 

        self.anim_group = QParallelAnimationGroup()
        
        anim_min = QPropertyAnimation(self.sidebar, b"minimumWidth")
        anim_min.setDuration(350) 
        anim_min.setStartValue(aktualna_sirka)
        anim_min.setEndValue(ciel)
        anim_min.setEasingCurve(QEasingCurve.Type.InOutCubic)
        
        anim_max = QPropertyAnimation(self.sidebar, b"maximumWidth")
        anim_max.setDuration(350)
        anim_max.setStartValue(aktualna_sirka)
        anim_max.setEndValue(ciel)
        anim_max.setEasingCurve(QEasingCurve.Type.InOutCubic)
        
        self.anim_group.addAnimation(anim_min)
        self.anim_group.addAnimation(anim_max)
        self.anim_group.start()

    def aktualizuj_vzdialenost_y(self):
        dist = abs(self.hranica_y_max.value() - self.hranica_y_min.value())
        self.lbl_vzdialenost_y.setText(f"↔ {dist:.1f} cm")

    def aktualizuj_vzdialenost_z(self):
        dist = abs(self.hranica_z_max.value() - self.hranica_z_min.value())
        self.lbl_vzdialenost_z.setText(f"↕ {dist:.1f} cm")

    def toggle_hranice_y(self):
        viditelne = not self.hranica_y_min.isVisible()
        self.hranica_y_min.setVisible(viditelne)
        self.hranica_y_max.setVisible(viditelne)
        self.lbl_vzdialenost_y.setVisible(viditelne)
        
        if viditelne:
            if hasattr(self, 'ax4') and self.ax4:
                _, y_range = self.ax4.viewRange()
            else:
                _, y_range = self.ax3.viewRange()
            stred = (y_range[0] + y_range[1]) / 2.0
            rozpatie = (y_range[1] - y_range[0]) * 0.15
            self.hranica_y_min.setPos(stred - rozpatie)
            self.hranica_y_max.setPos(stred + rozpatie)
            self.aktualizuj_vzdialenost_y()

    def toggle_hranice_z(self):
        viditelne = not self.hranica_z_min.isVisible()
        self.hranica_z_min.setVisible(viditelne)
        self.hranica_z_max.setVisible(viditelne)
        self.lbl_vzdialenost_z.setVisible(viditelne)
        
        if viditelne:
            _, y_range = self.ax3.viewRange()
            stred = (y_range[0] + y_range[1]) / 2.0
            rozpatie = (y_range[1] - y_range[0]) * 0.15
            self.hranica_z_min.setPos(stred - rozpatie)
            self.hranica_z_max.setPos(stred + rozpatie)
            self.aktualizuj_vzdialenost_z()

    def reset_hranice(self):
        self.hranica_y_min.setVisible(False)
        self.hranica_y_max.setVisible(False)
        self.hranica_z_min.setVisible(False)
        self.hranica_z_max.setVisible(False)
        self.lbl_vzdialenost_y.setVisible(False)
        self.lbl_vzdialenost_z.setVisible(False)
        
    def style_btn_analyza(self, active):
        if active:
            self.btn_analyza_unavy_hlavne.setStyleSheet(f"""
                QPushButton {{ background-color: {ACCENT}; color: {BG_MAIN}; border-radius: 12px; font-weight: bold; font-family: "Segoe UI", sans-serif; font-size: 14px; border: none; }}
                QPushButton:hover {{ background-color: {ACCENT_HOVER}; }}
            """)
        else:
            self.btn_analyza_unavy_hlavne.setStyleSheet(f"""
                QPushButton {{ background-color: transparent; border: 2px solid {BG_TER}; color: {ACCENT}; border-radius: 12px; font-weight: bold; font-family: "Segoe UI", sans-serif; font-size: 14px; }}
                QPushButton:hover {{ background-color: {BG_TER}; }}
            """)

    def vypni_analyza_mode(self):
        self.style_btn_analyza(False)
        self.btn_analyza_reset.hide()
        self.ref_vyber = None
        self.comp_vyber = None
        self.reset_hranice()

    def toggle_analyza_mode(self):
        if self.rezim_porovnania == "analyze_mode":
            self.vypni_analyza_mode()
            self.seg_rezim.buttons["single"].setChecked(True)
            self.rezim_porovnania = "single"
        else:
            self.rezim_porovnania = "analyze_mode"
            self.style_btn_analyza(True)
            self.btn_analyza_reset.show()
            self.ref_vyber = None
            self.comp_vyber = None
            
            for btn in self.seg_rezim.btn_group.buttons():
                self.seg_rezim.btn_group.setExclusive(False)
                btn.setChecked(False)
                self.seg_rezim.btn_group.setExclusive(True)

        self.obnov_ui_zoznam()
        self.aktualizuj_grafy()

    def reset_analyza_vyber(self):
        self.ref_vyber = None
        self.comp_vyber = None
        self.obnov_ui_zoznam()
        self.aktualizuj_grafy()

    def mouseMoved(self, evt):
        osi = [self.ax1, self.ax2, self.ax3] + ([self.ax4] if hasattr(self, 'ax4') and self.ax4 else [])
        if self.sidebar_otvoreny:
            for ax in osi: ax.vLine.setVisible(False)
            self.tooltip1.setVisible(False)
            self.tooltip2.setVisible(False)
            self.tooltip3.setVisible(False)
            if hasattr(self, 'tooltip4'): self.tooltip4.setVisible(False)
            return

        if not self.zobrazene_data: return
        pos = evt[0]
        
        in_plot = False
        x_val = None
        active_ax = None
        mousePoint = None
        
        for ax in osi:
            if ax.sceneBoundingRect().contains(pos):
                in_plot = True
                active_ax = ax
                mousePoint = ax.vb.mapSceneToView(pos)
                x_val = mousePoint.x()
                break
                
        if not in_plot or x_val is None:
            for ax in osi: ax.vLine.setVisible(False)
            self.tooltip1.setVisible(False)
            self.tooltip2.setVisible(False)
            self.tooltip3.setVisible(False)
            if hasattr(self, 'tooltip4'): self.tooltip4.setVisible(False)
            return

        cas = self.zobrazene_data[0][0]
        idx = np.searchsorted(cas, x_val)
        if idx >= len(cas): 
            idx = len(cas) - 1
        real_x = cas[idx]

        for ax in osi:
            if hasattr(ax, 'vLine'):
                if self.pocet_senzorov == 1 and (ax == self.ax3 or ax == self.ax4):
                    # Show vLine only if data is present
                    y_pos_z = self.zobrazene_data[0][6][idx] * 100
                    y_pos_y = self.zobrazene_data[0][5][idx] * 100
                    val_to_check = y_pos_z if ax == self.ax3 else y_pos_y
                    
                    if not np.isnan(val_to_check):
                        ax.vLine.setPos(real_x)
                        ax.vLine.setVisible(True)
                    else:
                        ax.vLine.setVisible(False)
                else:
                    ax.vLine.setPos(real_x)
                    ax.vLine.setVisible(True)

        txt1, txt2, txt3, txt4 = f"{self.t('time')}: {real_x:.2f} s\n", f"{self.t('time')}: {real_x:.2f} s\n", f"{self.t('time')}: {real_x:.2f} s\n", f"{self.t('time')}: {real_x:.2f} s\n"
        
        y_vals_ax1, y_vals_ax2, y_vals_ax3, y_vals_ax4 = [], [], [], []

        for data_item in self.zobrazene_data:
            pre = f"{self.t('set')} {data_item[-2]}: " if self.rezim_porovnania != "single" else ""
            
            if self.pocet_senzorov == 1:
                txt1 += f"{pre}{data_item[2][idx]:.2f} m/s²\n"
                txt2 += f"{pre}{data_item[4][idx]:.2f} m/s\n"
                
                # Safe text formatting when no data is present (barbell is racked)
                vy = data_item[5][idx] * 100
                vz = data_item[6][idx] * 100
                if np.isnan(vy) or np.isnan(vz):
                    txt3 += f"{pre}Z: -- cm\n"
                    txt4 += f"{pre}Y: -- cm\n"
                else:
                    txt3 += f"{pre}Z: {vz:.1f} cm\n"
                    txt4 += f"{pre}Y: {vy:.1f} cm\n"
                    y_vals_ax3.append(vz)
                    y_vals_ax4.append(vy)
                
                y_vals_ax1.append(data_item[2][idx])
                y_vals_ax2.append(data_item[4][idx])
                
            else:
                if self.zobrazenie_2_senzorov != "oba":
                    acc_val = data_item[1][idx] if self.zobrazenie_2_senzorov=="lavy_detail" else data_item[2][idx]
                    vel_val = data_item[3][idx] if self.zobrazenie_2_senzorov=="lavy_detail" else data_item[4][idx]
                    pos_val = data_item[6][idx] if self.zobrazenie_2_senzorov=="lavy_detail" else data_item[7][idx]
                    
                    txt1 += f"{pre}{acc_val:.2f} m/s²\n"
                    txt2 += f"{pre}{vel_val:.2f} m/s\n"
                    
                    if np.isnan(pos_val):
                        txt3 += f"{pre}-- cm\n"
                    else:
                        txt3 += f"{pre}{pos_val*100:.1f} cm\n"
                        y_vals_ax3.append(pos_val)
                    
                    y_vals_ax1.append(acc_val)
                    y_vals_ax2.append(vel_val)
                else:
                    if self.zobrazit_zrychlenie:
                        txt1 += f"{pre}{self.t('left')}: {data_item[1][idx]:.2f} | {self.t('right')}: {data_item[2][idx]:.2f}\n"
                        y_vals_ax1.extend([data_item[1][idx], data_item[2][idx]])
                    else:
                        txt1 += f"{pre}{self.t('left')}: {data_item[3][idx]:.2f} | {self.t('right')}: {data_item[4][idx]:.2f}\n"
                        y_vals_ax1.extend([data_item[3][idx], data_item[4][idx]])
                        
                    txt2 += f"{pre}{self.t('diff')}: {data_item[5][idx]:.1f} cm\n"
                    
                    vz_L = data_item[6][idx]*100
                    vz_R = data_item[7][idx]*100
                    if np.isnan(vz_L) or np.isnan(vz_R):
                        txt3 += f"{pre}{self.t('left')}: -- | {self.t('right')}: --\n"
                    else:
                        txt3 += f"{pre}{self.t('left')}: {vz_L:.1f} | {self.t('right')}: {vz_R:.1f}\n"
                        y_vals_ax3.extend([data_item[6][idx], data_item[7][idx]])
                    
                    y_vals_ax2.append(data_item[5][idx])

        self.tooltip1.setText(txt1.strip())
        self.tooltip2.setText(txt2.strip())
        self.tooltip3.setText(txt3.strip())
        if hasattr(self, 'tooltip4'): self.tooltip4.setText(txt4.strip())
        
        axes = osi
        tooltips = [self.tooltip1, self.tooltip2, self.tooltip3] + ([self.tooltip4] if hasattr(self, 'tooltip4') else [])
        y_data_lists = [y_vals_ax1, y_vals_ax2, y_vals_ax3, y_vals_ax4][:len(axes)]
        
        for ax, tt, data_y_vals in zip(axes, tooltips, y_data_lists):
            if not data_y_vals: 
                tt.setVisible(False)
                continue
                
            x_min, x_max = ax.vb.viewRange()[0]
            y_min, y_max = ax.vb.viewRange()[1]
            
            anchor_x = 1 if real_x > (x_min + x_max) / 2 else 0
            
            real_x_draw = real_x
            avg_y = sum(data_y_vals) / len(data_y_vals)
            if avg_y > (y_min + y_max) / 2:
                anchor_y = 1
                bezpecne_y = y_min + (y_max - y_min) * 0.03
            else:
                anchor_y = 0
                bezpecne_y = y_max - (y_max - y_min) * 0.03
                
            tt.setAnchor((anchor_x, anchor_y))
            tt.setPos(real_x_draw, bezpecne_y) 
            tt.setVisible(True)

    def vykresli_tlacidla_pre_2_senzory(self):
        self.clear_layout(self.layout_2_senzory)
        if self.pocet_senzorov == 1: return

        if self.zobrazenie_2_senzorov == "oba":
            txt = self.t('show_vel') if self.zobrazit_zrychlenie else self.t('show_acc')
            b1 = self.create_btn(txt, BG_TER, TEXT_MAIN, 18, 36, font_size=15)
            b1.clicked.connect(self.prepni_zrychlenie)
            self.layout_2_senzory.addWidget(b1)
            
            h = QHBoxLayout()
            bl = self.create_btn(self.t('left_det'), BG_MAIN, TEXT_MUTED, 16, 32, font_size=14)
            bl.clicked.connect(lambda: self.zmen_2s("lavy_detail"))
            br = self.create_btn(self.t('right_det'), BG_MAIN, TEXT_MUTED, 16, 32, font_size=14)
            br.clicked.connect(lambda: self.zmen_2s("pravy_detail"))
            h.addWidget(bl); h.addWidget(br)
            self.layout_2_senzory.addLayout(h)
        else:
            b = self.create_btn(self.t('back_sym'), "#fbbc04", "#131314", 18, 36, font_size=15)
            b.clicked.connect(lambda: self.zmen_2s("oba"))
            self.layout_2_senzory.addWidget(b)

    def zmen_2s(self, rezim):
        self.zobrazenie_2_senzorov = rezim
        self.vykresli_tlacidla_pre_2_senzory()
        self.aktualizuj_grafy()

    def prepni_zrychlenie(self):
        self.zobrazit_zrychlenie = not self.zobrazit_zrychlenie
        self.vykresli_tlacidla_pre_2_senzory()
        self.aktualizuj_grafy()

    # --- ESP32 CALIBRATION ---
    def spusti_accel_kalibraciu(self):
        from PyQt6.QtWidgets import QMessageBox
        reply = QMessageBox.question(self, 'Potvrdenie kalibrácie', 
                                     'Naozaj chcete spustiť 6-bodovú kalibráciu akcelerometra?\n\n'
                                     'Položte senzor na prvú z 6 strán. Akonáhle bude senzor 2 sekundy v pokoji, \n'
                                     'LED blikne ZELENO a potom začne svietiť na MODRO. To znamená, že máte senzor \n'
                                     'otočiť na ďalšiu stranu.\n\n'
                                     'Opakujte pre všetkých 6 strán (X+, X-, Y+, Y-, Z+, Z-).', 
                                     QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No, QMessageBox.StandardButton.No)
        if reply == QMessageBox.StandardButton.Yes:
            self.ble_worker.pripoj()
            self.ble_worker.posli_prikaz_accel_kalibracia()
            
            self.clear_layout(self.overlay_layout)
            self.calib_card = QFrame()
            self.calib_card.setFixedSize(350, 200)
            self.calib_card.setStyleSheet(f"background-color: {BG_SEC}; border: 2px solid #c58af9; border-radius: 24px;")
            
            layout = QVBoxLayout(self.calib_card)
            layout.setContentsMargins(30, 30, 30, 30)
            
            self.calib_lbl = QLabel("Prebieha 6-bodová kalibrácia...")
            self.calib_lbl.setStyleSheet(f"color: #c58af9; font-size: 20px; font-weight: bold; border: none; font-family: 'Segoe UI', sans-serif;")
            self.calib_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.calib_lbl.setWordWrap(True)
            layout.addWidget(self.calib_lbl)
            
            self.overlay_layout.addWidget(self.calib_card)
            self.overlay_bg.setGeometry(self.rect())
            self.overlay_bg.raise_()
            self.overlay_bg.show()

    def dokoncena_accel_kalibracia(self):
        self.calib_lbl.setText("Kalibrácia úspešne dokončená\na uložená v NVS!")
        self.calib_lbl.setStyleSheet(f"color: #00FF00; font-size: 20px; font-weight: bold; border: none; font-family: 'Segoe UI', sans-serif;")
        QTimer.singleShot(2500, self.zatvor_overlay)

    def spusti_kalibraciu(self):
        if hasattr(self, 'is_live_recording') and self.is_live_recording:
            return 
        
        self.ble_worker.pripoj()
        # We removed sending the command here. The countdown must finish first.
        
        self.clear_layout(self.overlay_layout)
        self.calib_card = QFrame()
        self.calib_card.setFixedSize(350, 200)
        self.calib_card.setStyleSheet(f"background-color: {BG_SEC}; border: 2px solid {ACCENT}; border-radius: 24px;")
        
        layout = QVBoxLayout(self.calib_card)
        layout.setContentsMargins(30, 30, 30, 30)
        
        self.calib_lbl = QLabel(self.t('calib_title'))
        self.calib_lbl.setStyleSheet(f"color: {ACCENT}; font-size: 22px; font-weight: bold; border: none; font-family: 'Segoe UI', sans-serif;")
        layout.addWidget(self.calib_lbl, alignment=Qt.AlignmentFlag.AlignCenter)
        
        self.calib_count_lbl = QLabel("3")
        self.calib_count_lbl.setStyleSheet(f"color: {TEXT_MAIN}; font-size: 48px; font-weight: bold; border: none;")
        layout.addWidget(self.calib_count_lbl, alignment=Qt.AlignmentFlag.AlignCenter)
        
        self.overlay_layout.addWidget(self.calib_card)
        self.overlay_bg.setGeometry(self.rect())
        self.overlay_bg.raise_()
        self.overlay_bg.show()
        
        self.calib_seconds = 3
        self.is_calibrating = True
        
        self.calib_timer = QTimer(self)
        self.calib_timer.timeout.connect(self.kalibracia_krok)
        self.calib_timer.start(1000)

    def kalibracia_krok(self):
        self.calib_seconds -= 1
        if self.calib_seconds > 0:
            self.calib_count_lbl.setText(str(self.calib_seconds))
        else:
            self.calib_timer.stop()
            # Only now, when it is 0 (barbell should be resting), send the command!
            self.calib_count_lbl.setText("Meria sa...")
            self.calib_count_lbl.setStyleSheet(f"color: #fbbc04; font-size: 28px; font-weight: bold; border: none;")
            
            self.ble_worker.posli_prikaz_kalibracia()
            self.ble_worker.start_calibration()
            # UI now stays "hanging" and waits for a signal from ESP32

    def dokoncena_kalibracia(self, bias_L, bias_R):
        self.saved_biases = {'1': bias_L, '2': bias_R}
        self.is_calibrating = False
        
        # ESP32 reported CALIB_DONE, we can close the window
        if hasattr(self, 'calib_count_lbl'):
            self.calib_count_lbl.setText(self.t('calib_done'))
            self.calib_count_lbl.setStyleSheet(f"color: {ACCENT}; font-size: 36px; font-weight: bold; border: none;")
            
        QTimer.singleShot(1000, self.zatvor_overlay)

    # --- ESP32 RECORDING ---
    def toggle_esp32_stream(self):
        if self.is_calibrating: return

        if hasattr(self, 'is_live_recording') and self.is_live_recording:
            self.is_live_recording = False
            self.ble_worker.stop_recording()
            self.btn_live_esp.setText(self.t('record_live'))
            self.spracuj_zive_data()
            self.ble_worker.posli_prikaz_bateria() 
        else:
            self.ble_worker.pripoj() 
            self.is_live_recording = True
            self.esp_live_data = []
            self.ble_worker.start_recording()
            self.btn_live_esp.setText(self.t('record_live_stop'))

    def skontroluj_pokles_rychlosti(self):
        seria_na_kontrolu = getattr(self, 'vybrana_seria', None)
        if seria_na_kontrolu is None or not self.databaza or seria_na_kontrolu not in self.databaza:
            return
            
        posledna_zaznamenana_seria = self.databaza[seria_na_kontrolu]
        if not posledna_zaznamenana_seria:
            return
            
        posledny_pokus = posledna_zaznamenana_seria[-1]
        
        if self.pocet_senzorov == 1:
            if len(posledny_pokus) > 10:
                reps_data = posledny_pokus[10]
            else:
                return
        else:
            if len(posledny_pokus) > 12:
                reps_data = posledny_pokus[12]
            else:
                return
                
        if len(reps_data) >= 2:
            first_mcv = reps_data[0].get('mcv', 0)
            last_mcv = reps_data[-1].get('mcv', 0)
            
            limit_pomer = self.threshold_zlyhania / 100.0
            
            if first_mcv > 0:
                if last_mcv < (limit_pomer * first_mcv):
                    QApplication.beep() 
                    QMessageBox.warning(
                        self, 
                        self.t('rep_drop_warn_title'), 
                        self.t('rep_drop_warn_text').format(last_mcv, self.threshold_zlyhania, first_mcv)
                    )
                else:
                    QMessageBox.information(
                        self,
                        self.t('rep_drop_ok_title'),
                        self.t('rep_drop_ok_text').format(last_mcv, first_mcv, self.threshold_zlyhania)
                    )

    def zlyhanie_ble(self, chyba):
        if self.is_calibrating:
            self.is_calibrating = False
            if hasattr(self, 'calib_timer'):
                self.calib_timer.stop()
            self.calib_count_lbl.setText("Chyba!")
            self.calib_count_lbl.setStyleSheet(f"color: {DANGER}; font-size: 32px; font-weight: bold; border: none;")
            QTimer.singleShot(1500, self.zatvor_overlay)

        self.is_live_recording = False
        self.btn_live_esp.setText(self.t('record_live'))
        self.lbl_porovnanie.setText(f"BLE chyba: {chyba}")
        self.lbl_porovnanie.setStyleSheet(f"color: {DANGER}; font-size: 14px; font-weight: bold;")
        self.lbl_varovanie_ikona.show()
        
    def aktualizuj_status_ble(self, status):
        self.lbl_porovnanie.setText(status)
        self.lbl_porovnanie.setStyleSheet(f"color: {ACCENT}; font-size: 14px; font-weight: bold;")
        self.lbl_varovanie_ikona.hide()
        if "Pripojené!" in status and not self.is_live_recording:
             self.ble_worker.posli_prikaz_bateria()

    # FIX: We also receive the exact time interval (dt)
    def prijmi_live_data(self, t, acc_L, acc_R, dt):
        self.esp_live_data.append((t, acc_L, acc_R, dt))

    
    # COMPLETELY REWRITTEN FUNCTION: Asymmetric ZUPT and Retroactive artifact cleaning
    # COMPLETELY REWRITTEN FUNCTION: ZUPT via raw data and rep separation (np.nan)
    def spracuj_zive_data(self):
        if not self.esp_live_data: return
        self.aktualna_seria = len(self.databaza) + 1 if self.databaza else 1
        if self.aktualna_seria not in self.databaza_weights:
            self.databaza_weights[self.aktualna_seria] = self.databaza_weights.get(self.aktualna_seria - 1, 0.0)
            
        self.databaza[self.aktualna_seria] = []
        
        cas = np.array([d[0] for d in self.esp_live_data])
        if len(cas) > 0:
            cas = cas - cas[0]
            
        dts = np.array([d[3] for d in self.esp_live_data]) 
        
        b, a = None, None
        priemerny_dt = np.mean(dts) if len(dts) > 0 else 0.01
        fs = 1.0 / priemerny_dt if priemerny_dt > 0 else 100.0
        nyq = 0.5 * fs
        
        if nyq > 5.0:
            b, a = butter(4, 5.0 / nyq, btype='low')
        
        if self.pocet_senzorov == 1:
            raw_acc_y = np.array([d[1] for d in self.esp_live_data], dtype=float)
            raw_acc_z = np.array([-d[2] for d in self.esp_live_data], dtype=float)
            
            acc_y = np.copy(raw_acc_y)
            acc_z = np.copy(raw_acc_z)
            
            if b is not None and a is not None and len(cas) > 15:
                acc_y = filtfilt(b, a, raw_acc_y)
                acc_z = filtfilt(b, a, raw_acc_z)
                
            vel_y = np.zeros(len(cas), dtype=float)
            vel_z = np.zeros(len(cas), dtype=float)
            pos_y = np.zeros(len(cas), dtype=float)
            pos_z = np.zeros(len(cas), dtype=float)
            
            vizualne_y = np.full(len(cas), np.nan)
            vizualne_z = np.full(len(cas), np.nan)
            
            # --- BASE VARIABLES 1 SENSOR ---
            # --- BASE VARIABLES 1 SENSOR ---
            # --- BASE VARIABLES 1 SENSOR ---
            # --- BASE VARIABLES 1 SENSOR ---
            # --- BASE VARIABLES 1 SENSOR ---
            is_resting = True
            rep_start_idx = 0  
            window = 40 
            is_rest_array = np.zeros(len(cas), dtype=bool) 
            
            # === LOOP 1: PHYSICS AND DRIFT CORRECTION ===
            for i in range(1, len(cas)):
                current_dt = dts[i]
                
                vel_y[i] = vel_y[i-1] + acc_y[i] * current_dt
                vel_z[i] = vel_z[i-1] + acc_z[i] * current_dt
                
                start_idx = max(0, i - window)
                rozptyl_z = np.ptp(raw_acc_z[start_idx:i+1]) 
                rozptyl_y = np.ptp(raw_acc_y[start_idx:i+1])
                
                # LOWERED LIMIT (0.4 m/s²) to better filter out minor shaking
                je_v_pokoji = (rozptyl_z < 0.4 and rozptyl_y < 0.4 and abs(raw_acc_z[i]) < 0.4) or (i < 50)
                
                if je_v_pokoji: 
                    if not is_resting:
                        is_resting = True
                        rep_len = start_idx - rep_start_idx
                        
                        max_p = np.max(np.abs(pos_z[rep_start_idx:start_idx])) if rep_len > 0 else 0.0
                        
                        if rep_len > 20 and max_p > 0.02:
                            # 1. Smooth out VELOCITY (Velocity Detrending)
                            err_vel_y = vel_y[start_idx]
                            err_vel_z = vel_z[start_idx]
                            
                            for k in range(rep_start_idx, start_idx):
                                podiel = (k - rep_start_idx) / float(rep_len)
                                vel_y[k] -= err_vel_y * podiel
                                vel_z[k] -= err_vel_z * podiel
                                
                            # 2. Restored 'POSITION DETRENDING' to close the loop flawlessly
                            pos_y[rep_start_idx] = 0.0
                            pos_z[rep_start_idx] = 0.0
                            for k in range(rep_start_idx + 1, start_idx):
                                pos_y[k] = pos_y[k-1] + vel_y[k] * dts[k]
                                pos_z[k] = pos_z[k-1] + vel_z[k] * dts[k]
                                
                            err_pos_y = pos_y[start_idx - 1]
                            err_pos_z = pos_z[start_idx - 1]
                            for k in range(rep_start_idx, start_idx):
                                podiel = (k - rep_start_idx) / float(rep_len)
                                pos_y[k] -= err_pos_y * podiel
                                pos_z[k] -= err_pos_z * podiel
                        else:
                            for k in range(rep_start_idx, start_idx):
                                vel_y[k], vel_z[k] = 0.0, 0.0
                                pos_y[k], pos_z[k] = 0.0, 0.0
                                is_rest_array[k] = True 
                                
                        for j in range(start_idx, i + 1):
                            vel_y[j], vel_z[j] = 0.0, 0.0
                            pos_y[j], pos_z[j] = 0.0, 0.0
                            is_rest_array[j] = True
                    else:
                        vel_y[i], vel_z[i] = 0.0, 0.0
                        pos_y[i], pos_z[i] = 0.0, 0.0
                        is_rest_array[i] = True
                else:
                    if is_resting:
                        is_resting = False
                        rep_start_idx = i - 1
                        pos_y[i-1], pos_z[i-1] = 0.0, 0.0 
                        
                    pos_y[i] = pos_y[i-1] + vel_y[i] * current_dt
                    pos_z[i] = pos_z[i-1] + vel_z[i] * current_dt

             # --- CLEANING UP THE OPEN END (Removing artifact on stop) ---
            if not is_resting:
                rep_len = len(cas) - rep_start_idx
                max_p = np.max(np.abs(pos_z[rep_start_idx:])) if rep_len > 0 else 0.0
                
                # If the last movement was just a tiny shake before stopping (< 5 cm), remove it
                if rep_len <= 20 or max_p <= 0.05:
                    for k in range(rep_start_idx, len(cas)):
                        vel_y[k], vel_z[k] = 0.0, 0.0
                        pos_y[k], pos_z[k] = 0.0, 0.0
                        is_rest_array[k] = True

            # === LOOP 2: VISUALIZATION ===
            max_z_achieved = 0.0
            in_rep = False
            
            for i in range(len(cas)):
                if is_rest_array[i]:
                    vizualne_y[i] = pos_y[i]
                    vizualne_z[i] = pos_z[i]
                    
                    if in_rep and max_z_achieved > 0.10:
                        pass # Waterfall offset removed
                        
                    in_rep = False
                    max_z_achieved = 0.0
                else:
                    current_z = pos_z[i] 
                    abs_z = abs(current_z)
                    
                    if not in_rep:
                        if abs_z > 0.02:
                            in_rep = True
                            max_z_achieved = abs_z
                    else:
                        if abs_z > max_z_achieved:
                            max_z_achieved = abs_z
                            
                    # First, draw the point so the trajectory goes perfectly down to ZERO
                    vizualne_y[i] = pos_y[i]
                    vizualne_z[i] = current_z 
                    
                    # Only then check for touch-and-go (if barbell is on the ground after a large movement)
                    if in_rep and abs_z < 0.02 and max_z_achieved > 0.10:
                        in_rep = False
                        max_z_achieved = 0.0
                        
                        # This trick creates a hole in the line to prevent a connected horizontal line
                        if i + 1 < len(cas):
                            is_rest_array[i+1] = True 
                            
            # === REPETITION DETECTION AND VBT METRICS ===
            reps_data = []
            rep_start = None
            for i in range(len(cas)):
                if not is_rest_array[i]:
                    if rep_start is None:
                        rep_start = i
                else:
                    if rep_start is not None:
                        rep_end = i
                        vz = vel_z[rep_start:rep_end]
                        pz = pos_z[rep_start:rep_end]
                        az = acc_z[rep_start:rep_end]
                        conc_indices = np.where(vz > 0.01)[0]
                        if len(conc_indices) > 5:
                            mcv = np.mean(vz[conc_indices])
                            peak_v = np.max(vz[conc_indices])
                            rom = (np.max(pz) - np.min(pz)) * 100 # in cm
                            sp_idx = np.argmin(az[conc_indices])
                            sticking_point = pz[conc_indices][sp_idx] * 100
                            reps_data.append({'mcv': mcv, 'peak_v': peak_v, 'rom': rom, 'sp': sticking_point})
                        rep_start = None
            
            if rep_start is not None:
                rep_end = len(cas)
                vz = vel_z[rep_start:rep_end]
                pz = pos_z[rep_start:rep_end]
                az = acc_z[rep_start:rep_end]
                conc_indices = np.where(vz > 0.01)[0]
                if len(conc_indices) > 5:
                    mcv = np.mean(vz[conc_indices])
                    peak_v = np.max(vz[conc_indices])
                    rom = (np.max(pz) - np.min(pz)) * 100
                    sp_idx = np.argmin(az[conc_indices])
                    sticking_point = pz[conc_indices][sp_idx] * 100
                    reps_data.append({'mcv': mcv, 'peak_v': peak_v, 'rom': rom, 'sp': sticking_point})
                            
            self.databaza[self.aktualna_seria].append((cas, acc_y, acc_z, vel_y, vel_z, vizualne_y, vizualne_z, np.max(vel_z), self.aktualna_seria, 0, reps_data))
        else:
            raw_acc_L = np.array([-d[1] for d in self.esp_live_data], dtype=float) 
            raw_acc_R = np.array([-d[2] for d in self.esp_live_data], dtype=float)
            
            acc_L = np.copy(raw_acc_L)
            acc_R = np.copy(raw_acc_R)
            
            if b is not None and a is not None and len(cas) > 15:
                acc_L = filtfilt(b, a, raw_acc_L)
                acc_R = filtfilt(b, a, raw_acc_R)
                
            vel_L = np.zeros(len(cas), dtype=float)
            vel_R = np.zeros(len(cas), dtype=float)
            pos_L = np.zeros(len(cas), dtype=float)
            pos_R = np.zeros(len(cas), dtype=float)
            
            vizualne_L = np.full(len(cas), np.nan)
            vizualne_R = np.full(len(cas), np.nan)
            vizualne_zL = np.full(len(cas), np.nan)
            vizualne_zR = np.full(len(cas), np.nan)
            
            # --- BASE VARIABLES 2 SENSORS ---
            # --- BASE VARIABLES 2 SENSORS ---
            # --- BASE VARIABLES 2 SENSORS ---
            # --- BASE VARIABLES 2 SENSORS ---
            # --- BASE VARIABLES 2 SENSORS ---
            # --- BASE VARIABLES 2 SENSORS ---
            is_resting = True
            rep_start_idx = 0
            window = 40
            is_rest_array = np.zeros(len(cas), dtype=bool)
            
            # === LOOP 1: PHYSICS AND DRIFT CORRECTION ===
            for i in range(1, len(cas)):
                current_dt = dts[i]
                
                vel_L[i] = vel_L[i-1] + acc_L[i] * current_dt
                vel_R[i] = vel_R[i-1] + acc_R[i] * current_dt
                
                start_idx = max(0, i - window)
                rozptyl_L = np.ptp(raw_acc_L[start_idx:i+1])
                rozptyl_R = np.ptp(raw_acc_R[start_idx:i+1])
                
                je_v_pokoji = (rozptyl_L < 0.4 and rozptyl_R < 0.4 and abs(raw_acc_L[i]) < 0.4 and abs(raw_acc_R[i]) < 0.4) or (i < 50)
                
                if je_v_pokoji: 
                    if not is_resting:
                        is_resting = True
                        rep_len = start_idx - rep_start_idx
                        
                        max_p_L = np.max(np.abs(pos_L[rep_start_idx:start_idx])) if rep_len > 0 else 0.0
                        max_p_R = np.max(np.abs(pos_R[rep_start_idx:start_idx])) if rep_len > 0 else 0.0
                        
                        if rep_len > 20 and (max_p_L > 0.02 or max_p_R > 0.02):
                            err_vel_L = vel_L[start_idx]
                            err_vel_R = vel_R[start_idx]
                            
                            for k in range(rep_start_idx, start_idx):
                                podiel = (k - rep_start_idx) / float(rep_len)
                                vel_L[k] -= err_vel_L * podiel
                                vel_R[k] -= err_vel_R * podiel
                                
                            pos_L[rep_start_idx] = 0.0
                            pos_R[rep_start_idx] = 0.0
                            for k in range(rep_start_idx + 1, start_idx):
                                pos_L[k] = pos_L[k-1] + vel_L[k] * dts[k]
                                pos_R[k] = pos_R[k-1] + vel_R[k] * dts[k]
                                
                            err_pos_L = pos_L[start_idx - 1]
                            err_pos_R = pos_R[start_idx - 1]
                            for k in range(rep_start_idx, start_idx):
                                podiel = (k - rep_start_idx) / float(rep_len)
                                pos_L[k] -= err_pos_L * podiel
                                pos_R[k] -= err_pos_R * podiel
                        else:
                            for k in range(rep_start_idx, start_idx):
                                vel_L[k], vel_R[k] = 0.0, 0.0
                                pos_L[k], pos_R[k] = 0.0, 0.0
                                is_rest_array[k] = True
                                
                        for j in range(start_idx, i + 1):
                            vel_L[j], vel_R[j] = 0.0, 0.0
                            pos_L[j], pos_R[j] = 0.0, 0.0
                            is_rest_array[j] = True
                    else:
                        vel_L[i], vel_R[i] = 0.0, 0.0
                        pos_L[i], pos_R[i] = 0.0, 0.0
                        is_rest_array[i] = True
                else:
                    if is_resting:
                        is_resting = False
                        rep_start_idx = i - 1
                        pos_L[i-1], pos_R[i-1] = 0.0, 0.0
                        
                    pos_L[i] = pos_L[i-1] + vel_L[i] * current_dt
                    pos_R[i] = pos_R[i-1] + vel_R[i] * current_dt

            # --- CLEANING UP THE OPEN END (Removing artifact on stop) ---
            if not is_resting:
                rep_len = len(cas) - rep_start_idx
                max_p_L = np.max(np.abs(pos_L[rep_start_idx:])) if rep_len > 0 else 0.0
                max_p_R = np.max(np.abs(pos_R[rep_start_idx:])) if rep_len > 0 else 0.0
                
                if rep_len <= 20 or (max_p_L <= 0.05 and max_p_R <= 0.05):
                    for k in range(rep_start_idx, len(cas)):
                        vel_L[k], vel_R[k] = 0.0, 0.0
                        pos_L[k], pos_R[k] = 0.0, 0.0
                        is_rest_array[k] = True

            # === LOOP 2: VISUALIZATION ===
            max_z_achieved = 0.0
            in_rep = False
            
            for i in range(len(cas)):
                if is_rest_array[i]:
                    vizualne_L[i] = pos_L[i]
                    vizualne_R[i] = pos_R[i]
                    vizualne_zL[i] = pos_L[i]
                    vizualne_zR[i] = pos_R[i]
                    
                    if in_rep and max_z_achieved > 0.10:
                        pass
                        
                    in_rep = False
                    max_z_achieved = 0.0
                else:
                    current_zL = pos_L[i]
                    current_zR = pos_R[i]
                    abs_mean_z = abs((current_zL + current_zR) / 2.0)
                    
                    if not in_rep:
                        if abs_mean_z > 0.02:
                            in_rep = True
                            max_z_achieved = abs_mean_z
                    else:
                        if abs_mean_z > max_z_achieved:
                            max_z_achieved = abs_mean_z
                            
                    vizualne_L[i] = pos_L[i]
                    vizualne_R[i] = pos_R[i]
                    vizualne_zL[i] = current_zL
                    vizualne_zR[i] = current_zR
                    
                    if in_rep and abs_mean_z < 0.02 and max_z_achieved > 0.10:
                        in_rep = False
                        max_z_achieved = 0.0
                        if i + 1 < len(cas):
                            is_rest_array[i+1] = True
                            
            # === REPETITION DETECTION AND VBT METRICS ===
            reps_data = []
            rep_start = None
            for i in range(len(cas)):
                if not is_rest_array[i]:
                    if rep_start is None:
                        rep_start = i
                else:
                    if rep_start is not None:
                        rep_end = i
                        vz = (vel_L[rep_start:rep_end] + vel_R[rep_start:rep_end]) / 2.0
                        pz = (pos_L[rep_start:rep_end] + pos_R[rep_start:rep_end]) / 2.0
                        az = (acc_L[rep_start:rep_end] + acc_R[rep_start:rep_end]) / 2.0
                        conc_indices = np.where(vz > 0.01)[0]
                        if len(conc_indices) > 5:
                            mcv = np.mean(vz[conc_indices])
                            peak_v = np.max(vz[conc_indices])
                            rom = (np.max(pz) - np.min(pz)) * 100
                            sp_idx = np.argmin(az[conc_indices])
                            sticking_point = pz[conc_indices][sp_idx] * 100
                            reps_data.append({'mcv': mcv, 'peak_v': peak_v, 'rom': rom, 'sp': sticking_point})
                        rep_start = None
            
            if rep_start is not None:
                rep_end = len(cas)
                vz = (vel_L[rep_start:rep_end] + vel_R[rep_start:rep_end]) / 2.0
                pz = (pos_L[rep_start:rep_end] + pos_R[rep_start:rep_end]) / 2.0
                az = (acc_L[rep_start:rep_end] + acc_R[rep_start:rep_end]) / 2.0
                conc_indices = np.where(vz > 0.01)[0]
                if len(conc_indices) > 5:
                    mcv = np.mean(vz[conc_indices])
                    peak_v = np.max(vz[conc_indices])
                    rom = (np.max(pz) - np.min(pz)) * 100
                    sp_idx = np.argmin(az[conc_indices])
                    sticking_point = pz[conc_indices][sp_idx] * 100
                    reps_data.append({'mcv': mcv, 'peak_v': peak_v, 'rom': rom, 'sp': sticking_point})
                            
            self.databaza[self.aktualna_seria].append((cas, acc_L, acc_R, vel_L, vel_R, (pos_L - pos_R)*100, vizualne_zL, vizualne_zR, np.max(vel_L), np.max(vel_R), self.aktualna_seria, 0, reps_data))
        # Audio feedback has been moved to check_velocity_drop
        self.aktualna_seria += 1

        if self.rezim_porovnania != "analyze_mode":
            self.vybrana_seria, self.vybrany_pokus_idx = self.aktualna_seria - 1, 0
            self.seg_rezim.buttons["single"].setChecked(True)
            self.rezim_porovnania = "single"
            
        self.obnov_ui_zoznam()
        self.aktualizuj_grafy()

    def zmen_rezim(self, val):
        if self.rezim_porovnania == "analyze_mode":
            self.vypni_analyza_mode()
            
        self.rezim_porovnania = val
        if val == "custom" and not self.vlastny_vyber: self.vlastny_vyber.add((self.vybrana_seria, self.vybrany_pokus_idx))
        self.obnov_ui_zoznam()
        self.aktualizuj_grafy()

    def vyber_pokus(self, s, i):
        if self.rezim_porovnania == "analyze_mode":
            if self.ref_vyber is None:
                self.ref_vyber = (s, i)
            elif self.ref_vyber == (s, i):
                pass 
            else:
                self.comp_vyber = (s, i) 
        elif self.rezim_porovnania == "custom":
            if (s, i) in self.vlastny_vyber: self.vlastny_vyber.remove((s, i))
            else: self.vlastny_vyber.add((s, i))
        else:
            self.vybrana_seria, self.vybrany_pokus_idx = s, i
            self.seg_rezim.buttons["single"].setChecked(True)
            self.rezim_porovnania = "single"
            
        self.obnov_ui_zoznam()
        self.aktualizuj_grafy()

    def zmen_hmotnost(self, s, hodnota):
        self.databaza_weights[s] = hodnota
        if self.je_historia and self.aktualny_zaznam_id:
            self.aktualizuj_vahu_v_denniku(self.aktualny_zaznam_id, s, hodnota)

    def aktualizuj_vahu_v_denniku(self, id_zaznamu, s, hodnota):
        try:
            with gzip.open(self.subor_dennika, "rt", encoding="utf-8") as f:
                dennik = json.load(f)
            
            for zaznam in dennik:
                if zaznam["id"] == id_zaznamu:
                    if "weights" not in zaznam:
                        zaznam["weights"] = {}
                    zaznam["weights"][str(s)] = hodnota
                    break
                    
            with gzip.open(self.subor_dennika, "wt", encoding="utf-8") as f:
                json.dump(dennik, f)
        except Exception as e:
            print(f"Chyba pri zápise hmotnosti do histórie: {e}")

    def vytvor_spinbox(self, s_id):
        spin = CleanFocusSpinBox()
        spin.setRange(0, 500)
        spin.setSingleStep(2.5)
        spin.setDecimals(1)
        spin.setSuffix(self.t('weight_suffix'))
        spin.setValue(self.databaza_weights.get(s_id, 0.0))
        spin.setFixedSize(90, 44)
        spin.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        spin.setAlignment(Qt.AlignmentFlag.AlignCenter)
        spin.setStyleSheet(f"""
            QDoubleSpinBox {{
                background-color: {BG_TER}; color: {TEXT_MAIN};
                border: 2px solid transparent; border-radius: 12px;
                font-weight: bold; font-family: "Segoe UI", sans-serif; font-size: 14px;
            }}
            QDoubleSpinBox:focus {{
                border: 2px solid {ACCENT};
                background-color: {BG_MAIN};
            }}
        """)
        spin.valueChanged.connect(lambda val, idx=s_id: self.zmen_hmotnost(idx, val))
        return spin

    def obnov_ui_zoznam(self):
        self.clear_layout(self.zoznam_pokusov_layout)

        for s, pokusy in self.databaza.items():
            if not pokusy: continue
            
            if len(pokusy) == 1:
                i = 0
                data = pokusy[0]
                
                is_sel, is_ref, is_comp = False, False, False
                if self.rezim_porovnania == "single" and s == self.vybrana_seria: is_sel = True
                elif self.rezim_porovnania == "custom" and (s, i) in self.vlastny_vyber: is_sel = True
                elif self.rezim_porovnania == "analyze_mode":
                    if self.ref_vyber == (s, i): is_ref = True
                    elif self.comp_vyber == (s, i): is_comp = True

                icon_name = None
                if is_ref: 
                    bg_c, txt_c, hov_c = ACCENT, BG_MAIN, ACCENT_HOVER
                    border_c = "#729ee6" 
                    icon_name = 'fa5s.thumbtack'
                elif is_comp:
                    bg_c, txt_c, hov_c = DANGER, BG_MAIN, DANGER_HOVER
                    border_c = "#d9655b" 
                    icon_name = 'fa5s.balance-scale-right'
                elif is_sel:
                    bg_c, txt_c, hov_c = ACCENT, BG_MAIN, ACCENT_HOVER
                    border_c = "#729ee6"
                else:
                    bg_c, txt_c, hov_c = BG_SEC, TEXT_MAIN, BG_HOVER
                    border_c = "#111214" 

                # Metric extraction from reps_data
                mcv_str = ""
                reps = data[10] if self.pocet_senzorov == 1 and len(data) > 10 else (data[12] if self.pocet_senzorov == 2 and len(data) > 12 else [])
                if reps and len(reps) > 0:
                    best_rep = max(reps, key=lambda x: x['mcv'])
                    mcv_str = f" | MCV: {best_rep['mcv']:.2f} m/s | ROM: {best_rep['rom']:.1f} cm"

                vmax_val = data[7] if self.pocet_senzorov == 1 else data[8]
                vmax_lbl = "Vmax" if self.pocet_senzorov == 1 else f"Vmax {self.t('left')}"
                txt = f"{self.t('set')} {s} | {vmax_lbl}: {vmax_val:.2f} m/s{mcv_str}"
                
                row_w = QWidget()
                row_layout = QHBoxLayout(row_w)
                row_layout.setContentsMargins(0, 0, 0, 5)
                row_layout.setSpacing(8)

                b = self.create_btn(txt, bg_c, txt_c, 12, 44, hover=hov_c, font_size=15, border=border_c, icon_name=icon_name)
                b.clicked.connect(lambda checked, s_id=s, idx=i: self.vyber_pokus(s_id, idx))
                
                spin = self.vytvor_spinbox(s)

                row_layout.addWidget(b, stretch=1)
                row_layout.addWidget(spin)
                self.zoznam_pokusov_layout.addWidget(row_w)
            else:
                seria_frame = QFrame()
                seria_frame.setStyleSheet(f"QFrame {{ background-color: {BG_SEC}; border: 2px solid #111214; border-radius: 12px; margin-bottom: 5px; }}")
                seria_layout = QVBoxLayout(seria_frame)
                seria_layout.setContentsMargins(6, 6, 6, 6)
                
                header_layout = QHBoxLayout()
                header_layout.setContentsMargins(0,0,0,4)
                
                lbl = QLabel(f"{self.t('set')} {s}")
                lbl.setStyleSheet(f"color: {TEXT_MUTED}; font-weight: bold; border: none;")
                
                spin = self.vytvor_spinbox(s)
                spin.setFixedHeight(30)
                spin.setStyleSheet(spin.styleSheet().replace("border-radius: 12px;", "border-radius: 8px;"))

                header_layout.addWidget(lbl)
                header_layout.addStretch()
                header_layout.addWidget(spin)
                seria_layout.addLayout(header_layout)
                
                for i, data in enumerate(pokusy):
                    is_sel, is_ref, is_comp = False, False, False
                    if self.rezim_porovnania == "single" and s == self.vybrana_seria and i == self.vybrany_pokus_idx: is_sel = True
                    elif self.rezim_porovnania == "custom" and (s, i) in self.vlastny_vyber: is_sel = True
                    elif self.rezim_porovnania == "analyze_mode":
                        if self.ref_vyber == (s, i): is_ref = True
                        elif self.comp_vyber == (s, i): is_comp = True

                    icon_name = None
                    if is_ref: 
                        bg_c, txt_c, hov_c = ACCENT, BG_MAIN, ACCENT_HOVER
                        border_c = "#729ee6"
                        icon_name = 'fa5s.thumbtack'
                    elif is_comp:
                        bg_c, txt_c, hov_c = DANGER, BG_MAIN, DANGER_HOVER
                        border_c = "#d9655b"
                        icon_name = 'fa5s.balance-scale-right'
                    elif is_sel:
                        bg_c, txt_c, hov_c = ACCENT, BG_MAIN, ACCENT_HOVER
                        border_c = "#729ee6"
                    else:
                        bg_c, txt_c, hov_c = "transparent", TEXT_MUTED, BG_HOVER
                        border_c = "transparent"

                    mcv_str_p = ""
                    reps_p = data[10] if self.pocet_senzorov == 1 and len(data) > 10 else (data[12] if self.pocet_senzorov == 2 and len(data) > 12 else [])
                    if reps_p and len(reps_p) > 0:
                        best_rep = max(reps_p, key=lambda x: x['mcv'])
                        mcv_str_p = f" | MCV: {best_rep['mcv']:.2f} m/s | ROM: {best_rep['rom']:.1f} cm"
                        
                    vmax_val_p = data[7] if self.pocet_senzorov == 1 else data[8]
                    vmax_lbl_p = "Vmax" if self.pocet_senzorov == 1 else f"Vmax {self.t('left')}"
                    txt_pokus = f"{self.t('part')} {i+1} | {vmax_lbl_p}: {vmax_val_p:.2f} m/s{mcv_str_p}"

                    b_pokus = self.create_btn(txt_pokus, bg_c, txt_c, 6, 28, hover=hov_c, font_size=14, border=border_c, icon_name=icon_name)
                    b_pokus.clicked.connect(lambda checked, s_id=s, idx=i: self.vyber_pokus(s_id, idx))
                    seria_layout.addWidget(b_pokus)
                self.zoznam_pokusov_layout.addWidget(seria_frame)
                
        self.zoznam_pokusov_layout.addStretch()

    def aktualizuj_grafy(self):
        self.zobrazene_data = []
        
        if self.rezim_porovnania == "analyze_mode":
            if self.ref_vyber and self.ref_vyber[0] in self.databaza:
                self.zobrazene_data.append(self.databaza[self.ref_vyber[0]][self.ref_vyber[1]])
            if self.comp_vyber and self.comp_vyber[0] in self.databaza:
                self.zobrazene_data.append(self.databaza[self.comp_vyber[0]][self.comp_vyber[1]])
                
            self.kresli_na_osi(self.ax1, self.ax2, self.ax3, self.ax4)
            
            if len(self.zobrazene_data) == 2:
                s_first = self.zobrazene_data[0][-2]
                s_last = self.zobrazene_data[1][-2]
                
                def ziskaj_priemerne_zrychlenie(data):
                    if self.pocet_senzorov == 1: a = data[2] 
                    else: a = (data[1] + data[2]) / 2.0
                    kladne_hodnoty = a[a > 0]
                    return np.mean(kladne_hodnoty) if len(kladne_hodnoty) > 0 else 0.1
                    
                avg_acc1 = ziskaj_priemerne_zrychlenie(self.zobrazene_data[0])
                avg_acc2 = ziskaj_priemerne_zrychlenie(self.zobrazene_data[1])
                perc = (avg_acc2 / avg_acc1) * 100
                
                self.lbl_porovnanie.setText(self.t('warn1').format(s_first, s_last, perc))
                
                if perc < self.threshold_zlyhania:
                    self.lbl_porovnanie.setStyleSheet(f"color: {DANGER}; font-size: 16px; font-weight: bold; font-family: 'Segoe UI';")
                    self.lbl_varovanie_ikona.show()
                    self.lbl_varovanie_ikona.setToolTip(self.t('tt_warn').format(perc, self.threshold_zlyhania))
                else:
                    self.lbl_porovnanie.setStyleSheet(f"color: {ACCENT}; font-size: 16px; font-weight: bold; font-family: 'Segoe UI';")
                    self.lbl_varovanie_ikona.hide()
                    
            elif len(self.zobrazene_data) == 1:
                self.lbl_porovnanie.setText(self.t('warn2'))
                self.lbl_porovnanie.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 14px; font-family: 'Segoe UI';")
                self.lbl_varovanie_ikona.hide()
            else:
                self.lbl_porovnanie.setText(self.t('warn3'))
                self.lbl_porovnanie.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 14px; font-family: 'Segoe UI';")
                self.lbl_varovanie_ikona.hide()

        else:
            if self.vybrana_seria not in self.databaza or not self.databaza[self.vybrana_seria]: 
                self.kresli_na_osi(self.ax1, self.ax2, self.ax3, self.ax4)
                self.lbl_porovnanie.setText("")
                self.lbl_varovanie_ikona.hide()
                return

            if self.rezim_porovnania == "custom": 
                self.zobrazene_data = [self.databaza[s][idx] for s, idx in sorted(list(self.vlastny_vyber)) if s in self.databaza and idx < len(self.databaza[s])]
            else: 
                self.zobrazene_data = [self.databaza[self.vybrana_seria][self.vybrany_pokus_idx]]

            self.kresli_na_osi(self.ax1, self.ax2, self.ax3, self.ax4)
            self.lbl_porovnanie.setText("")
            self.lbl_varovanie_ikona.hide()

    def kresli_na_osi(self, ax1, ax2, ax3, ax4=None):
        def odstran_kratke_useky(mask, min_len=5):
            m = mask.copy()
            z = np.where(np.diff(np.concatenate(([0], m, [0]))))[0]
            if len(z) == 0: return m
            for s, e in zip(z[::2], z[1::2]):
                if e - s < min_len:
                    m[s:e] = False
            return m

        for ax in [ax1, ax2, ax3, ax4]:
            if ax is None: continue
            items = [i for i in ax.items if isinstance(i, (pg.PlotDataItem, pg.InfiniteLine))]
            for i in items: 
                if hasattr(i, 'is_boundary'): 
                    if getattr(self, 'rezim_porovnania', 'single') != "analyze_mode":
                        i.setVisible(False)
                    continue 
                if not getattr(i, 'movable', True): continue
                ax.removeItem(i)
                
        if getattr(self, 'rezim_porovnania', 'single') != "analyze_mode":
            if hasattr(self, 'lbl_vzdialenost_y'): self.lbl_vzdialenost_y.setVisible(False)
            if hasattr(self, 'lbl_vzdialenost_z'): self.lbl_vzdialenost_z.setVisible(False)
            
        strana = self.t('left_suffix') if self.pocet_senzorov == 2 and self.zobrazenie_2_senzorov == "lavy_detail" else (self.t('right_suffix') if self.pocet_senzorov == 2 and self.zobrazenie_2_senzorov == "pravy_detail" else "")
        
        if self.pocet_senzorov == 1:
            ax3.setXLink(ax1)
            if ax4: ax4.setXLink(ax1) 
            ax1.setTitle(f"<span style='color: {TEXT_MAIN}; font-size: 12pt; font-family: Segoe UI;'>{self.t('graph1_1')}</span>")
            ax2.setTitle(f"<span style='color: {TEXT_MAIN}; font-size: 12pt; font-family: Segoe UI;'>{self.t('graph2_1')}</span>")
            ax3.setTitle(f"<span style='color: {TEXT_MAIN}; font-size: 12pt; font-family: Segoe UI;'>{self.t('graph3_1')}</span>")
            if ax4: ax4.setTitle(f"<span style='color: {TEXT_MAIN}; font-size: 12pt; font-family: Segoe UI;'>{self.t('graph4_1')}</span>")
            
            # v_center was deleted, not needed for time axis
        else:
            if self.pocet_senzorov == 2 and self.zobrazenie_2_senzorov != "oba":
                ax1.setTitle(f"<span style='color: {TEXT_MAIN}; font-size: 12pt; font-family: Segoe UI;'>Zrýchlenie{strana} - m/s²</span>")
                ax2.setTitle(f"<span style='color: {TEXT_MAIN}; font-size: 12pt; font-family: Segoe UI;'>Rýchlosť{strana} - m/s</span>")
                ax3.setTitle(f"<span style='color: {TEXT_MAIN}; font-size: 12pt; font-family: Segoe UI;'>Zvislá dráha{strana} - metre</span>")
            else:
                if self.zobrazit_zrychlenie: ax1.setTitle(f"<span style='color: {TEXT_MAIN}; font-size: 12pt; font-family: Segoe UI;'>{self.t('graph1_2a')}</span>")
                else: ax1.setTitle(f"<span style='color: {TEXT_MAIN}; font-size: 12pt; font-family: Segoe UI;'>{self.t('graph1_2v')}</span>")
                ax2.setTitle(f"<span style='color: {TEXT_MAIN}; font-size: 12pt; font-family: Segoe UI;'>{self.t('graph2_2')}</span>")
                ax3.setTitle(f"<span style='color: {TEXT_MAIN}; font-size: 12pt; font-family: Segoe UI;'>{self.t('graph3_2')}</span>")
            ax3.setXLink(ax1)
            if ax4: 
                ax4.hideAxis('bottom')
                ax4.hideAxis('left')
                ax4.clear()


        zvolene_farby = [ACCENT, DANGER] if self.rezim_porovnania == "analyze_mode" else self.farby_pokusov
        
        acc_texts = []
        vel_texts = []

        for i, d in enumerate(self.zobrazene_data):
            c = zvolene_farby[i % len(zvolene_farby)]
            pen = pg.mkPen(color=c, width=2.5)
            pen_dash = pg.mkPen(color=c, width=2.5, style=Qt.PenStyle.DashLine)

            prefix = f"S{d[-2]} " if len(self.zobrazene_data) > 1 else ""
            if self.pocet_senzorov == 1:
                a_val = d[2]
                v_val = d[4]
            else:
                if self.zobrazenie_2_senzorov != "oba":
                    a_val = d[1] if self.zobrazenie_2_senzorov=="lavy_detail" else d[2]
                    v_val = d[3] if self.zobrazenie_2_senzorov=="lavy_detail" else d[4]
                else:
                    a_val = (d[1] + d[2]) / 2.0
                    v_val = (d[3] + d[4]) / 2.0

            a_pos = a_val[a_val > 0.05]
            avg_a = np.mean(a_pos) if len(a_pos) > 0 else 0.0
            
            v_pos = v_val[v_val > 0.05]
            avg_v = np.mean(v_pos) if len(v_pos) > 0 else 0.0
            
            acc_texts.append(f"<span style='color:{c};'>{prefix}Ø {avg_a:.2f} m/s²</span>")
            vel_texts.append(f"<span style='color:{c};'>{prefix}Ø {avg_v:.2f} m/s</span>")

            pen_conc = pg.mkPen(color="#00E676", width=2.5) # Green for concentric
            pen_exc = pg.mkPen(color=DANGER, width=2.5)     # Red for eccentric
            pen_sp = pg.mkPen(color="#FFD600", width=0) # Dot for Sticking Point (SP)
            brush_sp = pg.mkBrush(color="#FFD600")

            if self.pocet_senzorov == 1:
                # Splitting into phases and removing micro-bounces
                v_arr = d[4]
                conc_mask = odstran_kratke_useky(v_arr > 0.01)
                exc_mask = odstran_kratke_useky(v_arr < -0.01)
                
                # Base (original color - dead zones)
                ax1.plot(d[0], d[2], pen=pen) 
                ax2.plot(d[0], d[4], pen=pen) 
                ax3.plot(d[0], d[6]*100, pen=pen) 
                if ax4: ax4.plot(d[0], d[5]*100, pen=pen)
                
                # Concentric (Green over-plot)
                d2_conc, d4_conc = np.copy(d[2]), np.copy(d[4])
                d2_conc[~conc_mask] = np.nan
                d4_conc[~conc_mask] = np.nan
                d5_conc, d6_conc = np.copy(d[5]), np.copy(d[6])
                d5_conc[~conc_mask] = np.nan
                d6_conc[~conc_mask] = np.nan
                
                ax1.plot(d[0], d2_conc, pen=pen_conc)
                ax2.plot(d[0], d4_conc, pen=pen_conc)
                ax3.plot(d[0], d6_conc*100, pen=pen_conc)
                if ax4: ax4.plot(d[0], d5_conc*100, pen=pen_conc)
                
                # Eccentric (Red over-plot)
                d2_exc, d4_exc = np.copy(d[2]), np.copy(d[4])
                d2_exc[~exc_mask] = np.nan
                d4_exc[~exc_mask] = np.nan
                d5_exc, d6_exc = np.copy(d[5]), np.copy(d[6])
                d5_exc[~exc_mask] = np.nan
                d6_exc[~exc_mask] = np.nan
                
                ax1.plot(d[0], d2_exc, pen=pen_exc)
                ax2.plot(d[0], d4_exc, pen=pen_exc)
                ax3.plot(d[0], d6_exc*100, pen=pen_exc)
                if ax4: ax4.plot(d[0], d5_exc*100, pen=pen_exc)
                
                # Sticking Point visualization
                reps = d[10] if len(d) > 10 else []
                for rep in reps:
                    if 'sp' in rep:
                        # Draw a small circle on the trajectory at the Sticking Point (SP)
                        sp_val = rep['sp']
                        # Find the nearest point on the Z trajectory
                        idx = np.nanargmin(np.abs(d[6]*100 - sp_val))
                        ax3.plot([d[0][idx]], [d[6][idx]*100], pen=pen_sp, symbol='o', symbolBrush=brush_sp, symbolSize=8)
                        if ax4: ax4.plot([d[0][idx]], [d[5][idx]*100], pen=pen_sp, symbol='o', symbolBrush=brush_sp, symbolSize=8)
                        
            else:
                if self.zobrazenie_2_senzorov != "oba":
                    acc_val = d[1] if self.zobrazenie_2_senzorov=="lavy_detail" else d[2]
                    vel_val = d[3] if self.zobrazenie_2_senzorov=="lavy_detail" else d[4]
                    pos_val = d[6] if self.zobrazenie_2_senzorov=="lavy_detail" else d[7]
                    
                    conc_mask = odstran_kratke_useky(vel_val > 0.01)
                    exc_mask = odstran_kratke_useky(vel_val < -0.01)
                    
                    ax1.plot(d[0], acc_val, pen=pen)
                    ax2.plot(d[0], vel_val, pen=pen)
                    ax3.plot(d[0], pos_val, pen=pen)
                    
                    acc_conc = np.copy(acc_val)
                    vel_conc = np.copy(vel_val)
                    pos_conc = np.copy(pos_val)
                    acc_conc[~conc_mask] = np.nan
                    vel_conc[~conc_mask] = np.nan
                    pos_conc[~conc_mask] = np.nan
                    
                    ax1.plot(d[0], acc_conc, pen=pen_conc)
                    ax2.plot(d[0], vel_conc, pen=pen_conc)
                    ax3.plot(d[0], pos_conc, pen=pen_conc)

                    acc_exc = np.copy(acc_val)
                    vel_exc = np.copy(vel_val)
                    pos_exc = np.copy(pos_val)
                    acc_exc[~exc_mask] = np.nan
                    vel_exc[~exc_mask] = np.nan
                    pos_exc[~exc_mask] = np.nan
                    
                    ax1.plot(d[0], acc_exc, pen=pen_exc)
                    ax2.plot(d[0], vel_exc, pen=pen_exc)
                    ax3.plot(d[0], pos_exc, pen=pen_exc)
                else:
                    if self.zobrazit_zrychlenie: ax1.plot(d[0], d[1], pen=pen); ax1.plot(d[0], d[2], pen=pen_dash) 
                    else: ax1.plot(d[0], d[3], pen=pen); ax1.plot(d[0], d[4], pen=pen_dash) 
                        
                    ax2.plot(d[0], d[5], pen=pen)          
                    ax3.plot(d[0], d[6], pen=pen)
                    ax3.plot(d[0], d[7], pen=pen_dash)

        if acc_texts:
            self.avg_text_acc.setHtml("<br>".join(acc_texts))
            self.avg_text_acc.setVisible(True)
        else:
            self.avg_text_acc.setVisible(False)
            
        if vel_texts:
            self.avg_text_vel.setHtml("<br>".join(vel_texts))
            self.avg_text_vel.setVisible(True)
        else:
            self.avg_text_vel.setVisible(False)

        if getattr(self, 'sidebar_otvoreny', True):
            for ax in [ax1, ax2, ax3]:
                ax.enableAutoRange()
                try:
                    ax.autoRange()
                except ValueError:
                    pass # Ignore library crash if the array contains only NaN values

        self.update_avg_pos()

    def ulozit_trening(self, cvik_key):
        if not any(len(p) > 0 for p in self.databaza.values()): return
            
        def zmensit_cislo(val):
            return round(float(val), 4)

        serializovana_db = {
            str(s): [
                [
                    [zmensit_cislo(x) for x in polozka] if isinstance(polozka, np.ndarray) 
                    else (zmensit_cislo(polozka) if isinstance(polozka, np.floating) else polozka) 
                    for polozka in pokus
                ] for pokus in ps
            ] for s, ps in self.databaza.items()
        }

        novy_zaznam = {
            "id": str(time.time()), 
            "timestamp": datetime.now().strftime("%d.%m.%Y %H:%M"), 
            "cvik": cvik_key, 
            "senzory": self.pocet_senzorov, 
            "weights": self.databaza_weights,
            "data": serializovana_db
        }
        
        try:
            with gzip.open(self.subor_dennika, "rt", encoding="utf-8") as f: 
                dennik = json.load(f)
        except Exception:
            dennik = []
            
        dennik.append(novy_zaznam)
        
        with gzip.open(self.subor_dennika, "wt", encoding="utf-8") as f: 
            json.dump(dennik, f)
            
        self.lbl_ulozit_nadpis.setText(self.t('saved').format(self.t(cvik_key)))
        self.lbl_ulozit_nadpis.setStyleSheet(f"color: #81c995; font-weight: bold; font-family: 'Segoe UI', sans-serif; font-size: 14px;")
        
        self.reset_statu()
        
        self.vypni_analyza_mode()
        self.seg_rezim.buttons["single"].setChecked(True)
        
        self.obnov_ui_zoznam()
        self.aktualizuj_grafy()

    def nacitat_trening(self, zaznam):
        self.reset_statu()
        self.je_historia = True
        self.pocet_senzorov = zaznam["senzory"]
        self.aktualny_zaznam_id = zaznam["id"]
        
        weights_z_dennika = zaznam.get("weights", {})
        self.databaza_weights = {int(k): float(v) for k, v in weights_z_dennika.items()}
        
        for s_str, pokusy in zaznam["data"].items():
            s_int = int(s_str)
            upravene_pokusy = []
            
            for p in pokusy:
                p_tup = tuple(np.array(item) if isinstance(item, list) and (not item or not isinstance(item[0], dict)) else item for item in p)
                if self.pocet_senzorov == 1 and len(p_tup) == 7:
                    cas = p_tup[0]
                    zeros = np.zeros_like(cas)
                    p_tup = (cas, zeros, p_tup[1], zeros, p_tup[2], zeros, p_tup[3], p_tup[4], p_tup[5], p_tup[6])
                upravene_pokusy.append(p_tup)
                
            self.databaza[s_int] = upravene_pokusy
            
            if s_int not in self.databaza_weights:
                self.databaza_weights[s_int] = 0.0
                
        if self.databaza: 
            self.vybrana_seria = list(self.databaza.keys())[0]

        self.vykresli_tlacidla_pre_2_senzory()
        
        self.vypni_analyza_mode()
        self.seg_rezim.buttons["single"].setChecked(True)
        
        self.obnov_ui_zoznam()
        self.aktualizuj_grafy()
        
        self.lbl_ulozit_nadpis.hide()
        self.ulozit_frame.hide()
        self.btn_kalibracia.hide()
        self.btn_live_esp.hide()
        self.btn_spat_hlavne.setText(self.t('list'))
        self.stacked_widget.setCurrentWidget(self.main_ui_widget)

    def ukonci_trening(self):
        if hasattr(self, 'is_live_recording') and self.is_live_recording:
            self.toggle_esp32_stream()
            
        bol_v_historii = self.je_historia
        
        self.reset_statu()
        self.lbl_ulozit_nadpis.setText(self.t('save_title'))
        self.lbl_ulozit_nadpis.setStyleSheet(f"color: {TEXT_MUTED}; font-weight: bold; font-family: 'Segoe UI', sans-serif; font-size: 14px;")
        
        if bol_v_historii: 
            self.stacked_widget.setCurrentWidget(self.dennik_widget)
            self.prekresli_dennik()
        else: 
            self.stacked_widget.setCurrentWidget(self.uvodne_okno)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TrackerApp()
    window.show()
    sys.exit(app.exec())