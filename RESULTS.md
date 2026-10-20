# A1.4 — výsledky simulácií a grafy

Jupyter verzia s uloženými grafmi a vykonateľnými bunkami:
[`RESULTS.ipynb`](RESULTS.ipynb). Prehľad dokumentov a grafov je v [README.md](README.md).

## Rozsah overenia

Merania vznikli **18. 10. 2026** spustením aktuálneho kódu v Gazebo Harmonic
8.15.0, ArduPilot SITL 4.7.0 (`1511f271`) a ROS 2 Jazzy/MAVROS. Použité boli
pôvodné svety `fei_lrs_gazebo.world` a `fei_lrs_outdoor.world`, mapa
`maps/FEI_LRS_PCD/map.pcd` a vnútorná misia `missions/exam_example.csv`.
Letová logika sa pri tvorbe grafov nemenila.

| Overenie | Výsledok |
|---|---|
| Zostavenie v pracovnom adresári | Úspešné |
| Zostavenie z čistej kópie zdrojov v samostatnom adresári | Úspešné |
| Regresné testy | 57/57 úspešných |
| Vnútorná misia vrátane `yaw180`, `landtakeoff`, `yaw90` | `DONE`, motory zaistené, 102,7 s |
| Vonkajší štvorec, osmička a návrat | `DONE`, motory zaistené, 176,6 s |
| Samostatné prerušenie počas vonkajšieho letu | `ABORT` → RTL → pristátie |

Čas misie je interval od prvej správy `SET_MODE` po `DONE`; nezahŕňa čakanie
na senzory pred štartom. Rosbagy obsahujú aj údaje pred/po tomto intervale.
Ide o jeden úplný vnútorný let, jeden úplný vonkajší let a samostatný test
prerušenia. Dva po sebe idúce úspešné lety ani reálny dron týmto overené nie sú.
Dodatočne **20. 10. 2026** prešlo zostavenie z nového Git klonu vetvy `part4`
na commite `d9dd12d0b6219d9d0d3dd0ef33336fd14cd29623` aj všetkých **57 testov**.
Záznamy sú v [`verification/git_clone_build.log`](verification/git_clone_build.log)
a [`verification/git_clone_pytest.log`](verification/git_clone_pytest.log).
Podrobnosti sú v `RUN.json`, položka `publication_verification`.

## 1. Plánovanie vo voxelovej mape

Rozlíšenie je 0,25 m, inflácia 0,45 m. Sivé objemy v grafe predstavujú
konzervatívne vyplnené regály; pre čitateľnosť je podlaha a strecha z PCD
zobrazenia vynechaná. Pri treťom úseku trasa vystúpi do 4,375 m nad regály.

![3D mapa, pôvodné A* body a skrátená trasa](figures/planning_3d.png)

| Úsek CSV | Body A* → po skrátení | Čas plánovania, medián (ms) | Dĺžka skrátenej trasy (m) |
|---|---:|---:|---:|
| W0 → W1 | 2 → 2 | 0,33 | 6,61 |
| W1 → W2 | 17 → 3 | 11,64 | 5,42 |
| W2 → W3 | 27 → 4 | 56,53 | 8,43 |
| W3 → W4 | 27 → 3 | 29,34 | 7,54 |
| W4 → W5 | 24 → 3 | 5,58 | 6,55 |
| W5 → W6 | 45 → 3 | 30,68 | 13,74 |

Čas je medián troch nových behov na rovnakom stroji, vrátane skrátenia a
kontroly segmentov, bez načítania PCD. Prvý úsek má voľnú priamu viditeľnosť,
preto nepotrebuje prehľadávanie A*. Všetky výsledné segmenty prešli kolíznou
kontrolou v nafúknutej mape. Súčet bodov cez šesť úsekov klesol zo 142 na 18
(spoločné koncové body sú v tomto súčte započítané pre každý úsek).

[Graf počtu bodov a časov plánovania](figures/planning_stats.png)
([PDF](figures/planning_stats.pdf)). Časy sa pri opakovaní menia podľa záťaže.

## 2. Vnútorný let

![Plánovaná a nameraná vnútorná trasa v mape a v 3D](figures/indoor_track.png)

Poloha MAVROS je pre graf prevedená späť do mapy inverznou rotáciou 90° a
posunom kalibrovaným zo zaznamenanej polohy pred vzletom pri mapovom štarte
`(13, 7, 0)`. W0 až W6 označujú riadky CSV. Graf výšky potvrdzuje dva vzlety
a dve pristátia; graf yaw zachytáva otočenia aj prechod cez ±180°.

| Meranie počas `NAVIGATE` | Hodnota |
|---|---:|
| Počet vzoriek polohy | 392 |
| Vzdialenosť od najbližšieho plánovaného 3D segmentu — medián | 0,072 m |
| Rovnaká vzdialenosť — 95. percentil / maximum | 0,365 / 0,612 m |
| Najväčšia odchýlka pri príchode na `hard` bod | 0,117 m |
| Vzorky v obsadených voxeloch pred infláciou | 0 |
| Vzorky v nafúknutých voxeloch | **5** |

**Limit sledovania trasy:** dron pri zrezaní zákruty prešiel bezpečnostným
okolím regálu, hoci zostal mimo pôvodných obsadených voxelov. Červené krížiky
označujú tieto vzorky. Kolízna kontrola plánovaných úsečiek preto nezaručuje,
že skutočný let vždy dodrží celú rezervu 0,45 m. Prispieva k tomu priebežná
akceptácia bodov do 0,35 m a dynamika polohového regulátora. Pre lepšie
dodržanie odstupu treba overiť spomaľovanie alebo menšiu toleranciu pri rohoch.
Kontrola diskrétnych vzoriek sama osebe nedokazuje bezkolíznosť celého pohybu.

[Výška, vzdialenosť od trasy a yaw v čase](figures/indoor_tracking.png)
([PDF](figures/indoor_tracking.pdf)). Odstup sa počíta od najbližšieho segmentu
celej plánovanej misie, nie od časovo priradeného segmentu. Vyhodnocuje sa iba
v `NAVIGATE`; pristávanie a vzlet sú z tejto metriky vynechané. Odchýlka pri
príchode používa najbližšiu vzorku polohy k udalosti `TASK`.

## 3. Vonkajší let

Parametre: strana štvorca 5 m, výška nad štartom 5 m, POI pri home,
`square_speed=0.30 m/s`, `A=3 m`, `omega=0.12 rad/s`, deklinácia 0°.

![Požadovaná a GPS odvodená vonkajšia trasa](figures/outdoor_track.png)

Zo zaznamenaných príkazov bolo overené poradie rohov **W1 → W2 → W3 → W4 →
W1**. Uzavretie štvrtej strany je teda súčasťou aktuálneho záznamu. Potom dron
preletel do stredu a vykonal osmičku za **52,35 s**. Najväčší nameraný
vodorovný odstup od POI bol **3,511 m**; plánovaná hranica je 3,536 m.
Vonkajší 3D graf zobrazuje fázy publikované cez `Path`; vzlet a pristátie sú
viditeľné v časovom grafe výšky.

| Meranie | Medián | 95. percentil | Maximum |
|---|---:|---:|---:|
| Štvorec: vzdialenosť od súčasného setpointu | 0,336 m | 0,367 m | 0,462 m |
| Osmička: vzdialenosť od súčasného setpointu | 0,385 m | 0,548 m | 0,589 m |
| Štvorec: absolútna chyba yaw voči smeru na POI | 0,57° | 1,42° | 2,57° |
| Osmička: absolútna chyba yaw voči dotyčnici | 1,29° | 4,63° | 13,09° |

Pri pohyblivej trajektórii vzdialenosť od súčasného setpointu zachytáva aj
časové oneskorenie dronu. Preto sa môže pôdorysná stopa dobre prekrývať, aj
keď táto metrika nie je nulová. Prechod zo štvorca na stred používa skok
cielovej polohy a v celkovom grafe krátko dosiahne približne 3,47 m.

[Výška, chyba polohy a chyba natočenia v čase](figures/outdoor_tracking.png)
([PDF](figures/outdoor_tracking.pdf)). Časovač je nastavený na 20 Hz; v tomto
zázname vyšla priemerná frekvencia publikovania **17,02 Hz**, medián intervalu
50 ms, 95. percentil 122 ms a maximum 650 ms. Python executor a zdieľané
prostredie neposkytujú garanciu dodržania každej periódy.

## 4. Prerušenie misie

Počas ďalšieho letu bola zavolaná služba `/assignment1_outdoor/abort`.
Odpoveď mala `success=True`, node prešiel do `ABORT` a prvá nasledujúca správa
autopilota hlásila RTL približne po **0,060 s**. Po prerušení neboli
zaznamenané ďalšie setpointy; kontrola pripúšťa 50 ms na poradie doručenia
správ. Autopilot sa vrátil a pristál, posledný stav mal `armed=False`.

RTL sa riadi parametrami autopilota: pri tomto teste dosiahlo výšku 8,86 m.
Limit 6 m v našom node kontroluje geometriu plánovanej misie pred štartom;
nie je aktívnou výškovou bariérou pre RTL.

[Záznam, odpoveď služby a namerané hodnoty](../../assignment1_mission/evidence/abort/).

## Dáta a zopakovanie grafov

- [Vnútorný rosbag, log a parametre](../../assignment1_mission/evidence/indoor/).
- [Vonkajší rosbag, log a parametre](../../assignment1_mission/evidence/outdoor/).
- [Presné hodnoty a udalosti](figures/metrics.json).
- [Prostredie a SHA-256 použitých zdrojov](RUN.json).
- [Postup záznamu](../../assignment1_mission/evidence/README.md).

Po zostavení balíka spusti z koreňa repozitára:

```bash
source install/setup.bash
/usr/bin/python3 assignment1_mission/tools/plot_evidence.py \
  --indoor assignment1_mission/evidence/indoor \
  --outdoor assignment1_mission/evidence/outdoor \
  --output assignment1_parts/04_documentation_and_defence/figures
```

Skript potrebuje systémový Matplotlib a `rosbag2_py` z ROS Jazzy. Vytvorí šesť
grafov v PNG/PDF a `metrics.json`. Je určený pre uvedený CSV súbor a predvolené
rozmery misie. Číta rosbagy priamo zo súborov; neprehráva letové príkazy.

Vonkajšia poloha pochádza z posledných kompletných histórií `Path` (480 párov
s rovnakým časom). Yaw používa najbližší kompasový záznam s rozdielom času
do 0,3 s; pri štvorci sú vynechané polohy bližšie ako 0,5 m od POI, kde je
smer na POI neurčitý. Pri kompasových a stavových správach sa používa čas
prijatia v rosbagu, pri bodoch `Path` ich časová značka. Rozdiely uhlov sú
normalizované do ±180°. Výsledky platia pre simuláciu bez pridaného vetra
a senzorového šumu nad štandardné nastavenie SITL.
