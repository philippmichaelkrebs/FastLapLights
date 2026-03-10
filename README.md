# FastLapLights

„Gentlemen, start your engines!“

FastLapLights bringt motorsportähnliche Start- und Streckenbeleuchtung auf deine Carrera-Bahn.
Das System nutzt den FastLapLights-Microcontroller, um Renndaten der Carrera Control Unit auszuwerten und daraus dynamische Licht- und Telemetrieeffekte zu erzeugen.

Mit adressierbaren LEDs lassen sich unter anderem Startampeln, Pit-Lane-Signale und Streckenflaggen realisieren. Animierte Lichtsequenzen sorgen dafür, dass sich der Rennstart deutlich näher anfühlt als ein einfacher Countdown auf der Control Unit.

Darüber hinaus stellt der FastLapLights-Controller gefilterte Renndaten über eine UART-Schnittstelle bereit. Damit können externe Anzeigen oder eigene Hardwareprojekte realisiert werden, zum Beispiel:

* Geschwindigkeitsanzeigen
* Scoring-Pylons
* mechanische 7-Segment-Displays
* Telemetrie-Displays
* eigene Rennstatistiken oder Analyse-Tools

Egal ob einfache Erweiterung oder komplexes Eigenbau-System – FastLapLights bietet eine solide Grundlage, um eine digitale Carrera-Bahn technisch aufzuwerten.

Unterstützte Systeme:

* Carrera Digital 124
* Carrera Digital 132

Preview:
![](https://github.com/philippmichaelkrebs/FastLapLights/blob/main/images/signal_start_proc_start_light.gif?raw=true)

---

# Funktionen

## Startampel

Realistische Startsequenzen mit WS2812B LEDs.

Die Startampel kann sowohl horizontal als auch vertikal aufgebaut werden. Beschreibungen dazu im Ordner Documents.
Die vertikale Variante erfordert etwas mehr Verdrahtung, erlaubt jedoch eine physische Trennung der einzelnen Lampen – ähnlich wie bei realen Motorsport-Startanlagen.

Das System unterstützt bis zu 40 LEDs, sodass Vorder- und Rückseite der Ampel gleichzeitig beleuchtet werden können. Dadurch entsteht nahezu eine 360-Grad-Sichtbarkeit der Startsignale.

---

## Pit-Lane-Signale

Óptional zur Startampel können LEDs als Pit-Lane-Signale verwendet werden.

Damit lassen sich Boxeneinfahrten oder Tankbereiche deutlich sichtbar kennzeichnen.
Ob die Funktion aktiv ist, hängt vom Aufbau der Hardware ab.

---

## Echtzeit-Renndaten

Der FastLapLights-Controller liest kontinuierlich die Daten der Carrera Control Unit aus.

Die wichtigsten Informationen werden gefiltert und als kompakter Datenstrom über UART ausgegeben. Dadurch können externe Systeme sehr einfach auf aktuelle Renndaten zugreifen.

Typische Anwendungen sind beispielsweise:

* Scoring-Pylons
* externe Displays
* Telemetrieanzeigen
* Rennstatistiken
* automatisierte Streckenlogik

---

# Schnellstart

1. FastLapLights-Microcontroller anschließen
3. LEDs entsprechend der gewünschten Funktion verbinden
4. Stromversorgung der Bahn anschließen
5. System starten und Rennen beginnen

---

# Dokumentation

Weitere technische Details und Hintergrundinformationen findest du in den Dokumentationsdateien:

* How the Lights Work
* Getting Started

---

# Mitmachen

Ideen für neue Features? Verbesserungen? Eigene Erweiterungen?

Beiträge sind jederzeit willkommen.
Issues und Pull Requests helfen dabei, das System weiter auszubauen und neue Funktionen zu entwickeln.

---

Developer
Philipp Krebs

Kontakt
[pmge.krebs@gmail.com](mailto:pmge.krebs@gmail.com)

GitHub
[https://github.com/philippmichaelkrebs](https://github.com/philippmichaelkrebs)

