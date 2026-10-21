# Assignment 1 — ROS 2 misie dronu

Kompletný balík `assignment1_mission` na plánovanie trasy a vykonanie vnútornej
aj vonkajšej misie cez MAVROS a ArduPilot.

- **`part1`, `part2`, `part3`**: samostatné pôvodné moduly.
- **`assignment1`**: ich zlúčená história a kód v priečinkoch `part01` až `part03`.
- **`devel`**: podporné ROS balenie, testy, skripty a CSV.
- **`main`**: spustiteľný celok vzniknutý zlúčením `assignment1` a `devel`.

Na zostavenie a spustenie používaj vetvu **`main`**.

```text
src/assignment1_mission/
├── assignment1_mission/
│   ├── part01/       # voxelová mapa, A*, príkaz plan3d
│   ├── part02/       # CSV, spoločný FlightNode, indoor misia
│   └── part03/       # GPS ↔ lokálne ENU, outdoor misia
├── test/            # existujúcich 57 testov
├── tools/           # spracovanie externých rosbagov
├── resource/
├── package.xml
├── setup.cfg
└── setup.py
scripts/             # spustenie Gazebo, SITL a MAVROS
missions/            # tri CSV misie pre indoor
```

## Prostredie a externé dáta

Používame Ubuntu s **ROS 2 Jazzy**, `colcon`, systémový Python 3, NumPy, SciPy,
pytest a ROS balíky uvedené v `package.xml` vrátane MAVROS. Simulácia vyžaduje
funkčnú školskú inštaláciu Gazebo, pluginu `ardupilot_gazebo`, ArduPilot SITL
a MAVProxy. Tieto komponenty sa týmto repozitárom neinštalujú.

```bash
git clone --branch main https://github.com/LubosValkovic/LRS.git LRS-devel
cd LRS-devel

# Absolútna cesta k existujúcemu školskému workspace s dátami:
LRS_DATA_ROOT=/home/user/gitLRS/LRS-URK
ln -s "$LRS_DATA_ROOT/maps" maps
ln -s "$LRS_DATA_ROOT/models" models
ln -s "$LRS_DATA_ROOT/worlds" worlds
```

Pripojené dáta musia obsahovať `maps/FEI_LRS_PCD/map.pcd`, modely pre dané svety
a súbory `worlds/fei_lrs_gazebo.world` a `worlds/fei_lrs_outdoor.world`.
Symlinky ignoruje Git. Mapy, point cloudy, modely, svety a rosbagy sa do vetvy
neukladajú. Samotné zostavenie balíka tieto dáta nepotrebuje.

### A note on Gazebo versions

Použi Gazebo a zostavenie pluginu `ardupilot_gazebo` z rovnakého školského
prostredia. Pri viacerých verziách Gazebo musí `GZ_SIM_SYSTEM_PLUGIN_PATH`
ukazovať na plugin zostavený pre verziu spúšťanú príkazom `gz sim`.
Skripty preberajú túto premennú z tvojho prostredia.

## Zostavenie

Z koreňa repozitára, v termináli so systémovým Python prostredím:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select assignment1_mission
source install/setup.bash
ros2 pkg executables assignment1_mission
```

Zaregistrované príkazy sú `plan3d`, `indoor` a `outdoor`. Po zmene zdrojových
súborov zopakuj zostavenie. Python prostredie pre SITL používaj v jeho vlastnom
termináli.

## Samostatné plánovanie — part01

Po zostavení a načítaní `install/setup.bash`:

```bash
ros2 run assignment1_mission plan3d \
  --map maps/FEI_LRS_PCD/map.pcd \
  --start 4.84 5.37 2.0 --goal 2.08 9.74 1.75 \
  --resolution 0.25 --safety-radius 0.45
```

Gazebo ani MAVROS netreba. Plánovač vytvorí voxelovú mapu, rozšíri prekážky
podľa bezpečnostného polomeru, nájde trasu pomocou A* a zjednoduší ju pri
zachovaní kontroly kolízií. Vypíše čas plánovania a výsledné body.

## Vnútorná misia — part02

Vo všetkých štyroch termináloch prejdi do koreňa tohto repozitára.
Spúšťaj ich v uvedenom poradí a nechaj bežať.

**Terminál 1 — Gazebo:**

```bash
source /opt/ros/jazzy/setup.bash
scripts/run_gazebo.sh
```

**Terminál 2 — ArduPilot SITL:**

```bash
# Ak používaš toto školské Python prostredie, najprv ho aktivuj:
# source ~/venv-ardupilot/bin/activate
scripts/run_sitl.sh
```

Skript predpokladá ArduPilot v `~/ardupilot`. Inú cestu nastav pomocou
`ARDUPILOT_DIR=/absolutna/cesta/ardupilot scripts/run_sitl.sh`.

**Terminál 3 — MAVROS:**

```bash
source /opt/ros/jazzy/setup.bash
scripts/run_mavros.sh
```

**Terminál 4 — misia:**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run assignment1_mission indoor --ros-args \
  -p mission_file:="$PWD/missions/defense_mission_2.csv" \
  -p map_file:="$PWD/maps/FEI_LRS_PCD/map.pcd" \
  -p resolution:=0.25 -p safety_radius:=0.45
```

Uzol čaká na pripojenie a potrebnú telemetriu. `FlightNode` zabezpečuje
prepnutie do GUIDED, arming, vzlet, posielanie setpointov a pristátie.
Zmeny stavu sa vypisujú ako `state -> ...`; úspešné dokončenie je `DONE`.

CSV má hlavičku `x,y,z,precision,task`. Súradnice sú v mapovom rámci v metroch,
`precision` je `soft` alebo `hard`. Prvý bod musí mať `takeoff`, posledný `land`;
medzi nimi možno použiť `-`, `yaw90` či iný uhol v stupňoch a `landtakeoff`.
Alternatívne CSV sú `missions/exam_example.csv` a `missions/mission_3_all.csv`.
Pri zmene cesty zmeň parameter `mission_file`. Uzol naplánuje trasy medzi bodmi
pomocou part01 pri spustení.

Pre školský indoor svet sa používa `map_launch:=[13.0,7.0,0.0]` a
`map_to_local_yaw_deg:=90.0`. Posun do MAVROS lokálneho ENU sa odmeria pri
štarte. Ak zmeníš svet alebo miesto vzletu, nastav príslušné parametre.

## Vonkajšia misia — part03

Po ukončení indoor simulácie spusti samostatnú outdoor simuláciu. Poradie
terminálov zostáva Gazebo → SITL → MAVROS → misia.

**Terminál 1:**

```bash
source /opt/ros/jazzy/setup.bash
scripts/run_gazebo.sh fei_lrs_outdoor.world
```

**Terminál 2** (s rovnakým prostredím SITL ako pri indoor):

```bash
HOME_LOCATION=48.15144444,17.07375000,150,0 scripts/run_sitl.sh
```

**Terminál 3:**

```bash
source /opt/ros/jazzy/setup.bash
scripts/run_mavros.sh
```

**Terminál 4:**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run assignment1_mission outdoor --ros-args \
  -p poi_east:=0.0 -p poi_north:=0.0 \
  -p square_side:=5.0 -p square_speed:=0.30 \
  -p figure_amplitude:=3.0 -p omega:=0.12 \
  -p flight_altitude:=5.0 -p magnetic_declination_deg:=0.0
```

Outdoor nepoužíva CSV ani voxelový plánovač. Z parametrov vytvorí uzavretý
štvorec okolo POI a plynulú osmičku, potom sa vráti na miesto vzletu a pristane.
Používa GPS, kompas a relatívnu výšku; GPS prevádza do lokálneho ENU.
Pred štartom čaká okrem iného na platný GPS fix a aspoň osem satelitov.

`poi_east` a `poi_north` sú metre od home, `square_speed` je m/s a `omega`
je rad/s. Parametre sa načítavajú pri spustení uzla; nové hodnoty zadaj cez
`--ros-args -p nazov:=hodnota` pri ďalšom spustení. Kontroly odmietnu neplatné
hodnoty a prekročenie nastavenej oblasti letu.

Konzola vypisuje stavy, GPS a ENU. Priebežné trasy sú dostupné na
`/assignment1_outdoor/target_path` a `/assignment1_outdoor/actual_path`;
v RViz ich zobraz ako `Path` s rámcom `map`.

## Prerušenie misie

V ďalšom termináli načítaj ROS a `install/setup.bash`, potom použi službu
príslušnej bežiacej misie:

```bash
ros2 service call /assignment1_indoor/abort std_srvs/srv/Trigger '{}'
# Alebo pri outdoor:
ros2 service call /assignment1_outdoor/abort std_srvs/srv/Trigger '{}'
```

Abort zastaví setpointy a požiada o RTL, ak je autopilot pripojený a stále
v GUIDED. Zmena režimu pilotom tiež zastaví setpointy. Samotné Ctrl+C službu
abort nevolá.

## Overenie a nástroje

S pripojenou externou mapou a zostaveným balíkom:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/usr/bin/python3 -m pytest -q src/assignment1_mission/test
```

Testy overujú plánovanie, CSV, súradnicové prevody a riadenie misie.
Riadiace testy používajú simulované správy a odpovede služieb; živú simuláciu
nespúšťajú.

Existujúce nástroje čítajú externé rosbagy. Potrebujú `rosbag2_py` z prostredia
ROS a Matplotlib. Napríklad porovnanie outdoor trás:

```bash
/usr/bin/python3 src/assignment1_mission/tools/plot_outdoor.py \
  /absolutna/cesta/outdoor_bag /tmp/outdoor_track.png
```

Analýza oboch záznamov:

```bash
/usr/bin/python3 src/assignment1_mission/tools/plot_evidence.py \
  --indoor /absolutna/cesta/indoor_bag \
  --outdoor /absolutna/cesta/outdoor_bag \
  --output /tmp/lrs-analysis
```

`plot_evidence.py` predpokladá indoor misiu `exam_example.csv`, mapu a pôvodné
predvolené parametre; použi zodpovedajúce záznamy. Výstupy a záznamy zostávajú
mimo verzie zdrojových kódov. Dokumentáciu tejto vetvy tvorí iba tento README.
