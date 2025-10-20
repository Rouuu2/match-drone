# ROS2 SLAM-Workspace-Setup & Simulationsstart-Anleitung 🛠️🤖

Diese Anleitung hilft dir, ein ROS2 SLAM-Workspace einzurichten, häufige Installationsfehler zu lösen und einen vollständigen Simulations-Stack mit QGroundControl, SLAM-Nodes und Gazebo zu starten. Sie enthält auch Tipps zur Fehlerbehebung und eine Vorschau deines TF-Baums.

---

## Inhaltsverzeichnis 📚

1. [Anforderungen](#Anforderungen)
2. [Installation & Setup](#installation--setup)
3. [Häufige Fehler & Lösungen](#häufige-fehler--lösungen)
4. [Simulation starten](#simulation-starten)
5. [TF-Baum Vorschau](#tf-baum-vorschau)
6. [Demnächst verfügbar](#demnächst-verfügbar)

---

## Anforderungen 📝

- **Betriebssystem:** Ubuntu 24.04 LTS Noble 🐧
- **ROS2:** Jazzy 
- **Simulation:** Gazebo Harmonic  + RViz 

## Wichtige Repositories für das Setup Workspace!

Hier sind die wichtigsten Repositories für die Installation (Verbesserungen folgen!):

- [match-drone](https://github.com/rouuu2/match-drone)
- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)
- [mavros](https://github.com/mavlink/mavros)
- [Qgroundcontrol](https://github.com/match-MobRob2/match-drone/blob/main/docs/QGroundControl.md)
- [ros_gz (jazzy branch)](https://github.com/gazebosim/ros_gz/tree/jazzy)
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [FAST-LIO2 (ROS2 branch)](https://github.com/hku-mars/FAST_LIO)

---

## Installation & Setup ⚙️

### 1. Umgebung vorbereiten 🌱

- Installiere ROS2 Jazzy und Gazebo Harmonic (siehe offizielle ROS2- und Gazebo-Dokumentation).
- Klone dein Workspace und initialisiere:
    ```bash
    mkdir -p ~/slam_ws/src
    cd ~/slam_ws/src
    # Klone deine Pakete hier, z.B.
    # git clone https://github.com/.... (Repo)
    ```

### 2. Python Virtual Environment erstellen und aktivieren 

```bash
python3 -m venv ~/.../venv
source ~/.../venv/bin/activate
```

### 3. Python-Abhängigkeiten installieren 📦

- Bearbeite `requirements.txt`, um nur echte Python-Pakete aufzulisten (siehe [Häufige Fehler & Lösungen](#häufige-fehler--lösungen) unten).
- Abhängigkeiten installieren:
    ```bash
    pip install -r ~/.../src/match-drone/setup/requirements.txt
    ```

### 4. Systemtools installieren 🛠️

```bash
sudo apt update
sudo apt install pcl-tools
```

---

## Häufige Fehler & Lösungen 🚧

### 1. `pcl_viewer` & `PCL` Installationsfehler ❌

**Fehler:**
```
ERROR: Could not find a version that satisfies the requirement pcl_viewer (from versions: none)
ERROR: No matching distribution found for pcl_viewer
```

**Lösung:**
- Entferne oder kommentiere `pcl_viewer` und `PCL` in deiner `requirements.txt` aus.
- Installiere die Anforderungen erneut und installiere die System-PCL-Tools wie oben beschrieben.

### 2. Cython-Fehler beim Build 🐍⚒️

**Fehler:**
```
ModuleNotFoundError: No module named 'Cython'
```

**Lösung:**
1. Installiere Cython in deinem Virtual Environment:
    ```bash
    pip install Cython
    ```
2. Verhindere, dass colcon dein venv durchsucht:
    ```bash
    touch ~/../venv/COLCON_IGNORE
    ```
3. Baue dein Workspace erneut:
    ```bash
    cd ~/...
    colcon build
    ```

---

## Checkliste: Überprüfung deiner Installation ✅

```bash
python3 --version     # Sollte Python 3.12.x oder deine Zielversion anzeigen
pip check             # Sollte ausgeben: "No broken requirements found."
colcon build          # Sollte das Workspace ohne Fehler bauen
```

---

## Simulation starten 🚀

Folge diesen Schritten in der Reihenfolge:

**1. QGroundControl starten 🕹️**
```bash
~/.../src$ ./QGroundControl-x86_64.AppImage
```
> _Startet das GCS für Drohnensteuerung und Telemetrie._

**2. SLAM Connector starten 🔗**
```bash
ros2 launch slam_connect test.launch.py
```

**3. Fast-LIO Mapping starten 🗺️**
```bash
ros2 launch fast_lio mapping.launch.py \
    config_path:=/home/.../.../src/match-drone/slam_connect/config \
    config_file:=slam_params.yaml \
    rviz:=false
```
> _Du kannst `slam_params.yaml` bearbeiten, um Map/SLAM-Parameter zu ändern._

**5. Drohne mit QGroundControl steuern 🎮**
- Verwende QGroundControl für alle manuellen Steuerungen und Missions-Uploads.

**Hinweis:**  
_Tastatursteuerung ist **in Kürze verfügbar**! Bleibe dran für Updates in der Parameterdatei, um schnelles Map-Switching und Tastatur-Teleoperation zu ermöglichen._ ⌨️

---

## TF-Baum Vorschau 🌳

Unten siehst du den aktuellen Transform-Baum, der von deinem System generiert wird (`tf2_tools view_frames`):

![frames_2025-10-20_19 17 52_page-0001](https://github.com/user-attachments/assets/6650c7d9-603c-4708-a161-224e366f7a78)

## Demnächst verfügbar ✨
- **setup workspace:**
- **Tastatursteuerung:** Direkte Drohnensteuerung per Tastatur für schnelle Entwicklung und Tests 
- **Parameterdatei-Verbesserungen:** Einfaches Wechseln von Maps und Konfigurationen über neue Parameter 
- **Bessere Visualisierung:** Erweiterte RViz- und Gazebo-Integration für detaillierteres Simulations-Feedback 

---
