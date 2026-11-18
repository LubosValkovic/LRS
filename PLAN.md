# Assignment 2 — plán po krokoch

Téma: **vyhľadávanie a sledovanie roverov**. Pracujeme v `LRS-URK2`,
vo vetve `zadanie2-lrs`, na existujúcom balíku Assignmentu 1.

Podklady: `assignments/assignment2/rover-find-and-follow.md` a
`presentation/LRS_Assignment_2.pptx`, najmä snímky 29–36 a ich poznámky.

## Spôsob práce

V jednom kroku implementujeme jednu ucelenú časť, overíme ju a uvedieme výsledok.
Ďalší krok vychádza z overeného predchádzajúceho výsledku. Existujúce letové
riadenie používame cez `FlightNode`. Výsledky simulácie zapisujeme až po meraní.

## 0. Preštudovanie podkladov — hotovo

- Prezrieť zdrojové súbory, zadanie a prezentáciu vrátane poznámok.
- Určiť opätovne použiteľný kód a požiadavky pre dva drony.
- Označiť body na overenie: kamera, skutočná veľkosť značky, ROS namespaces,
  časovanie obrazu a telemetrie, vlastné ArUco ID a abort.

## 1. Spoločné letové riadenie pre dve inštancie — hotovo

- Umožniť, aby každý `FlightNode` komunikoval s vlastným MAVROS.
- Zachovať existujúce spúšťanie indoor a outdoor pri predvolených parametroch.
- Oddeliť telemetriu, setpointy, služby a abort pre každý dron.

**Overenie:** regresné testy Assignmentu 1 a kontrola, že dve inštancie nemajú
spoločné riadiace topicy ani služby. Tento krok nepotrebuje spustenie letu.

**Výsledok:** `FlightNode` a dodatočné outdoor odbery používajú relatívne
`mavros/...` názvy. Konštruktory prijímajú argumenty ROS uzla vrátane namespace.
Predvolené rozhrania Assignmentu 1 zostali zachované. Zostavenie cez `colcon`
prešlo; uspelo všetkých **63 testov** (57 pôvodných + 6 nových, nič nevynechané).
Nové testy overili aj skutočné doručovanie správ a volanie abort/RTL medzi
dvoma oddelenými inštanciami v testovacej ROS doméne bez autopilota.

## 2. Simulácia dvoch dronov a pripojenie kamier — hotovo

- Pripojiť externé školské modely a svet `fei_lrs_precision_landing.world`.
- Pripraviť opakovateľné spustenie dvoch SITL a dvoch MAVROS s príslušnými
  SYSID, portmi, počiatočnými polohami a oddelenými pracovnými priečinkami.
- Zistiť skutočné názvy kamerových topicov a pripojiť obraz aj `camera_info`.
- Využiť školský `scripts/randomize_rovers.py`.

**Overenie:** obe MAVROS spojenia a obe kamery majú súčasne živé správy;
uložiť zistené názvy, typy a frekvencie. Samotná existencia topicu nestačí.

**Výsledok (18. 11. 2026):** najprv opätovne prešlo všetkých 63 testov kroku 1.
Nový balík `assignment2_rover` spúšťa SITL s oddelenými stavovými priečinkami,
SYSID 1/2, FDM portmi 9002/9012 a MAVLink portmi 14550/14560. Launch spúšťa
dva MAVROS a štyri jednosmerné mosty pre obraz a `camera_info`. Telemetriu
zapína službou `MessageInterval`, vrátane opätovného nastavenia po strate
spojenia. Školský randomizér zostal zachovaný.

Zostavenie oboch balíkov a **73 testov** prešlo (63 predchádzajúcich + 10
nových). Živá kontrola s oboma dronmi a pohybujúcimi sa rovermi pri seede 42
overila všetkých desať vstupných tokov súčasne počas 20 s: heartbeat 5 Hz,
poloha a rýchlosť 10 Hz, obrazy 27,00/29,65 Hz a obe `camera_info` 30,30 Hz
reálneho času. Oba target SYSID boli správne a drony zostali nearmované.
Lokálny report je `log/step2_final_pedcbjy9/bringup.json`; postup a názvy
rozhraní sú v README. Reporty a externé dáta Git ignoruje.

## 3. Detekcia vlastnej značky na jednom drone — hotovo

- Spracovať obraz cez `cv_bridge` a OpenCV `DICT_4X4_50`.
- Spracovanie obrazu oddeliť od publikovania letových setpointov.
- Prijímať iba značku s ID zodpovedajúcim SYSID vlastného dronu.
- Potvrdzovať detekcie a zahadzovať oneskorené alebo neplatné údaje.

**Overenie:** vlastná značka sa nájde, cudzie ID sa ignoruje a obrazové
spracovanie neblokuje letové časovače. Najprv bez sledovania pohyblivého cieľa.

**Výsledok (18. 11. 2026):** `MarkerVision` poskytuje potvrdené rohy iba
vlastného ID, overeného proti `mavros/target_system_id`. Používa `cv_bridge`,
`DICT_4X4_50`, tri potvrdzovacie snímky, jednosnímkovú schránku a samostatné
callback skupiny. Kontroluje formát a čerstvosť pred aj po OpenCV; staré,
opakované a budúce snímky zahadzuje. Nový most `/clock` a `use_sim_time`
zjednocujú čas kamery a detektora. Letové setpointy zostávajú oddelené.

Prešlo **86 testov** (73 predchádzajúcich + 13 nových), vrátane skutočných
ROS doručení a behu setpointového časovača `FlightNode` počas zablokovaného
obrazového spracovania. Živá obrazová kontrola v dočasnej kópii sveta s
kamerou zafixovanou nad pôvodnými modelmi roverov detegovala obe ID 1/2
a publikovala iba rohy ID 1. Po odstránení vlastnej značky sa ID 2 ignorovalo
aj počas ďalších 3 s. Oba drony zostali nearmované. Lokálny report:
`log/step3_live_g6vrb7pm/vision.json`. Ide o overenie detekcie jedného dronu;
vyhľadávací let a spoločné letové overenie nasledujú v krokoch 5 a 6.

## 4. Prevod detekcie do lokálneho ENU

- Intrinzické parametre prevziať z `camera_info` a použiť `solvePnP`.
- Odvodiť skutočnú veľkosť detegovaného štvorca z modelu a jeho textúry.
- Zahrnúť posun a orientáciu kamery aj úplný quaternion dronu.
- Ošetriť časový odstup medzi obrazom a použitou polohou dronu.
- Zjednotiť časové bázy: kamera a detektor už používajú `/clock` z Gazebo,
  MAVROS predvolene čas počítača; tieto značky sa zatiaľ nesmú priamo porovnávať.

**Overenie:** numerické testy prevodov a kontrola pri visení nad roverom.
Zdokumentovať odhadovanú vzdialenosť a jej chybu. Ground truth môže slúžiť
oddelenému vyhodnoteniu, nie ako vstup sledovacieho riadenia.

## 5. Vyhľadávanie a sledovanie na jednom drone

- Rozšíriť `FlightNode` o vyhľadávací vzor a stavy SEARCH a TRACKING.
- Vyhľadávanie nesmie používať vopred známu polohu ani kružnicu roveru.
- Pridať nastaviteľnú výšku nad roverom, obmedzenie rýchlosti a timeout detekcie.
- Po strate značky opustiť starý cieľ a vrátiť sa k vyhľadávaniu.
- Použiť existujúci abort; bežné sledovanie neprechádza do pristátia.

**Overenie:** jeden dron nájde vlastný pohyblivý rover, sleduje ho a po
skutočnej strate obrazu prejde SEARCH → TRACKING. Skontrolovať aj abort.

## 6. Súbežné overenie dvoch dronov a dokumentácia

- Spustiť dve nezávislé inštancie rovnakého riadiaceho uzla.
- Overiť párovanie oboch ID a pohyb roverov s viacerými seedmi.
- Zaznamenať skutočný lost/found cyklus každého dronu a reakciu na abort.
- Doplniť README, diagram, parametre, odvodenia, výsledky a obmedzenia.

**Overenie:** zostavenie a testy z čistého checkoutu a zaznamenaný spoločný
beh. Splnené body zadania označiť podľa toho, čo bolo naozaj demonštrované.

## Aktuálny stav

Hotové sú kroky 0, 1, 2 a 3. Kroky 4–6 ešte nezačali.
