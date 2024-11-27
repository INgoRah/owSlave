# owSlave2
Fork from https://git.smho.de/gw/?p=owSlave2.git;a=summary, License

I2C master code from an unofficial github repository for Peter Fleury's I2C library
http://homepage.hispeed.ch/peterfleury/avr-software.html#libs now cloned to
https://github.com/alx741/avr_i2c.git
License GPL v3
## Changes
Version 1.8, 2024-11-28
- Dual DS2450 (ADC) support and minor fixes
- the cfg read on the custom space seems not to be working yet
  on dual ROM

Version 1.7, 2024-11-22
- Timer based switch (supports one timer only)
- Single Pin configuration message and new custom config space (first byte = 1)
- Read out custom space by first writing one config byte with 1 then reading
- Auto toggle now signals the latch and the output

Version 1.5, 2021-11-28
- Temperature and Humidity support with DHT22 on pin PB0 (compile option)
  Humiditiy is reported in the byte #5 instead of FF

Version 1.4, 2021-10-27
- support BMP280 with dual rom with DS1820 on a soft I2C master
- report any latch change even for auto switch
- remove latch output on configured input pins

Version 1.3, 2021-01-14
- Split the files for 2408 and 1820
- In case of temperature support, regularly wake up and check alarms thresholds.
  This is using watchdog timer - power optimized.
- Temperature controlled switch (heating control, thresholds configurable
- Temperature change reporting (configurable)
- Read initial state for input switches of types CFG_PASS* and perform auto switch.
    Otherwise a thermostat might be ignored on restart and no heating active where
    it should
- per port pin intialization to reduce power if not connected (pull up not needed)

Version 1.2
- For PWM output (on ATTiny85 PB4 / PIO0 and ATTiny84 PA5) the all output latch bits
  are used for the duty cycle (bightnes), 0 for off
- PWM output level to be applied on change request (not only on switching it on)
- optimize touch sensor handling (active high, no bouncing, just rising edge with validation).
    POL and SW_ID to be implemented

Version 1.1
- Interrupt generation using the ow line low for 900 us
- Timer functions like in Arduino but on polling base within the loop. No wakeup from sleep
- Pin and push button validation
- Watchdog support:
    After 8 secs of invalid state a reset occurs (e.g. a hang in button validation).
    This also applies to pressing a button for 8 secs (force reset)
- Initial OneWire ID is initialized and copyied over to the end of EEPROM on
  first boot for reading next time.
- store new config / custom command or newid command feature

## To Do
[x] Generate alarm for latch even when auto switching
[x] Support for timer switches using upper auto switching bits [4..7]: 1 - 30 secs, 2 - 1 min, 3 - 2 mins .. 15 - 14 min.
[ ] Watchdog fires if switching PIO without polling/resetting alarm
[ ] regression testing signal generation and improvements (bus detection and delay)
[ ] Disarm interrupt on reset pulse and arm again afterwards.
    Needs check wether slave can cope with just a reset without ROM commands.
    Timeout after reset?
[ ] iButton type 2 interrupt (assert reset for long time).
    Needs master improvements detecting it and slave ASM code extending the reset low time
[ ] Dual 2408 with on PCF8473
[ ] Pin overlay for in and out using all 10 pins (reset and 1-wire not available)

## Issues
[ ] long press no longer working with outstanding changes
[ ] config save does not work sometimes
[ ] once setting PIO overides heating override ... fixed by pin change corrections?
[ ] If bad signals on the bus or power outage OW ID could be damaged. Needs protection

  # Documentation

  ## Configuration

  ### EEPROM Layout
  See eeprom.py script

  ### Configuration Definition
  CFG [1] default 0xff unused
  CFG [2] default 0xff unused
  SWx  [3..10]
    Auto switch an ouput on (short) press of this pin. Output starts with 1 (1 switches PIO0...).
    Example: SW4 = 0x01 switches PI0 on latch 4
    0x80 disables it,
    0x20 timer based switching, look up in custom config space
    0x40 reserved for long press auto switching, look up in custom config space
  CFGx [11..18] see CFG_xxx defines
  FEA [19] - Feature if bit is cleared: 
    0x1 can do temperature
    0x2 can do ADC for brightness read
  Reserved [20]
  Version [21..22]
  Type [23]

## Custom timer command

0xC5 | FEAT/PIN | TYPE | VAL1 | VAL2
read back 16 bit CRC over command and all bytes
TYPE: see TMR_TYPE_xxx definitions
VAL1: Time in 10ms or config for non static switch. If 0 use the confiugred default time.
      In case of type E3 (TMR_TYPE_BRIGHTNESS) or E5 (TMR_TYPE_THRESHOLD) 
      brightness or lower threshold value for darkness switching.
      Brightness is the inversed ligth - means 0xff dark and 0 for very bright.
      code: switch only if brightness >= cfg_custom1[1]
      Configuring threshold will also store the entire custom cfg in EEPROM
VAL2: Level 0 .. 255 for PWM output

Status represents a running timer with 0x20 or a dimming down with 0x40

# 1-wire implementation (from tm3e.de / German)

## Files

Datei  | Beschreibung |
-------- | -------- |
OWPinInterrupt.s | Interrupt für Pegeländerungen am 1-Wire Pin
OWTimerInterrupt.s | Timer Interrupt für Zeitmessungen
OWConfig.s | Allgemeine Konfiguration (Register und Speicher Belegung, Auswahl der Konfiguration für den speziellen AVR, Macros)
OWSet_ATTINYX4.s | Spezielle Konfiguration für ATTINY24 - ATTINY84
OWSet_ATTINYX5.s | Spezielle Konfiguration für ATTINY25 - ATTINY85
OWCRC8.s | CRC8 Berechnung (Wird je nach Gerät ausgewählt)
OWCRC16.s | CRC16 Berechnung (Wird je nach Gerät ausgewählt)
OWRomFunctions.s | Alle ROM-Funktionen, sowie die Funktionalität zur Änderung der OW-Pin
Diese Dateien stehen im Unterverzeichnis "common". Für jedes 1-Wire Gerät wird ein weiteres Unterverzeichnis angelegt und dort ist mindestens jeweils eine .c Datei mit dem Hauptprogramm und eine .s Datei mit den spezifischen Assemblerroutinen für das Gerät.

## Register und Variablen
Alle Registerbezeichnungen beginnen mit "r_", so dass sie nicht mit Variablen verwechselt werden können. Wenn es dazugehörige Variablen im SRAM gibt, dann stehen sie gleich dahinter.

 Bezeichnung | Nr. | Hauptfunktion |
 -------- | -------- | --- |
 r_temp | 16 | temporäre Zwischenspeicherung
 r_rwbyte / rwbyte | 17 | aktuelles Byte welches gelesen oder geschrieben wird
 r_temp2 | 18 | temporäres Zusatzregister, wenn eins nicht ausreicht
 r_bcount / bcount | 19 | Bit, welches gerade behandelt wird ist 1, Bit wird druchgeschoben. r_bcount=0 bedeutet: nächstes Byte muss behandelt werden
 r_mode / mode | 20 | Aktueller Zustand/Funktion, OW_SLEEP->Lehrlauf, Definition der ROM Commands in OWRomFunctions.s, gerätespezifische Konstanten in der .s Datei im Geräteverzeichnis. Nach dem Wert in mode werden über die Sprungtabelle (handle_stable) die entsprechenden Routinen ausgewählt.
 r_sendflag / sendflag | 21 |  0 => Slave empfängt Daten vom Master, 1=> Slave sendet Daten zum Master
 r_bytep / bytep | 22 | Pointer auf das nächste (od. aktuelle) Byte
 r_crc | 23 | Hilfsvariable für die CRC-Berechnung
 srbyte |   | Aktuelles Byte für SEARCH_ROM
 alarmflag |   | Für das Suchen von Geräten im Alarmmodus (z.B. beim DS18B20)
 owid |   | Speicher für die 1-Wire-ID
 zl / zh | 30/31 | Speicherzugriff
 reset_indicator |   | Zeigt Hauptprogramm, dass ein Resetimpuls empfangen wurde
 gcontrol |   | Kommunikation mit dem Programmteilen die nicht in der Interruptroutine laufen
Einige Register werden in der SEARCH_ROM-Routine anders verwendet, um zusätzliche Push und Pop Befehle einzusparen.

Die vom Timing aus betrachtet kritischste Sitation ist wenn der Slave eine "0" an den Master sendet. Der Master zieht die Leitung auf Low und der Slave muss möglist in 6 µs die Leitung auch auf Low ziehen. Der DS9490 lässt die Leitung unter Umständen noch schneller wieder auf High (5V) springen. Nach der 1-Wire Spezifikation muss der Slave die Leitung nach 15 µs auf Low sein, denn dann liest der Master aus. Das ist kein Problem für den AVR. Es ist aber mein Ziel, dass zwischen dem Master-Low und dem Slave-Low keine Pause ist (die Leitung nicht auf High springt).
Es muss also in der Interruptroutine schnell entschieden werden ob eine "0" gesendet werden muss. Eine normal Variable in einem SRAM-Bereich müsste dazu erst in ein Register geladen werden. Das Register müsste vorher gesichert werden. Ein schnellerer Weg ist es, wenn ein ungenutztes Bit in einem I/O-Register des AVR für die Entscheidnugn genutzt wird.

Aus diesem Grund wurde das DDR Bit des Reset Pins für diese Entscheidung zweckentfremdet. Die Reset Leitung wird normalerweise nur zum Programmieren genutzt und da stört diese Anwendung nicht. Um das Reset-Pin als vollwertiges I/O-Pin zu nutzen muss erst eine FUSE gesetzt werden. Dann geht das Programmieren aber nur noch über die Hochvoltprogrammierung.

Es gibt also noch eine weiter Variable (ZEROMAKER), die nicht direkt im Quelltext auftaucht.

Folgende Abbildungen verdeutlicht diesen Zusammenhang nocheinmal:

Variable im SRAM  -> 9,3 µs bis Bus Low
Variable im SRAM -> 9,3 µs bis Bus Low

Variable im I/O Register -> Leitung nicht zwischendurch auf High
Variable im I/O Register -> Leitung nicht zwischendurch auf High

Die beiden Verläufe sind von einem Test mit einem MOSFET (2N7000) als Pegelwandler von 5V am 1-Wirebus und 3V am ATTINY84.

## Programmablauf
Herzstück der Simulation ist die Interruptroutine die bei einer fallenden Flanke am 1-Wire Bus aufgerufen wird. In der neuen Software gibt es hier keinen unterschied zwischen den verschiedenen Zuständen wie Reset, Searchrom usw. Bei einem Reset wird ersteinmal ganz normal ein Bit gelesen. Da keine weitere fallende Flanke kommt, wird irgend wann der Timer Interrupt aufgrufen und Prüft ob die Leitung noch Low ist. Der Timer-Interrupt gibt dann entsprechend den Presents Impuls aus und schaltet auf OWM_READ_COMMAND.

Die Eigentlichen Funktionen werden bei handle_byte ausgeführt. Dort Springt das Programm anhand von einer Sprungtabelle und dem Wert in mode sehr effizient an die Stelle, wo der aktuelle Zustand bearbeitet wird.

Bei dem Searchrom-Algorithmus, wo jeweils zwei Bits gesendet (vom Slave gesndet) werden und ein Bit empfangen wird, bekommt der Bitzähler bcount nur entsprechend kleinere Werte und die handle_byte Funktion wird öfterer aufgerufen.

![Pin states](Doc/sdlpin.svg)

Beim Senden erfolgen die Berechnungen für das nächste Byte zwischen der fallenden Flanke vom Master bis zu der Zeit, in dem der Slave die Leitung wieder frei gibt, wenn denn eine 0 gesendet wird. Beim Empfangen erfolgen die Berechnungen nachdem die Leitung ausgelesen wurde bis maximal zum nächsten Low-Impuls vom Master (etwas eher sollte es schon sein).

Wichtig ist, dass nach einem handle_byte beim Empfangen das sendflag kontroliert wird. Wenn das nächste Byte gesendet wird muss der ZEROMARKER entsprechend gesetzt werden.

Im bei einem Timer Interupt wird geprüft, ob die Leitung Lange genug Low ist, damit der Impuls als Reset Impuls erkannt wird. Dazu gibt es noch die zweite Zeit OWT_RESET2. Die Zeiten wie auch OWT_READ und OWT_WRITE durch Polling eingehalten. Der Timer-Interrupt wird nur bei einem "Timeout" nach einer Fallenden Flanke ausgelöst.

![Timer States](Doc/sdltimer1.svg)

### Übersicht über die Zustände bei handle_byte
 In der Datei OWRomFunctions.s sind die grundlegenden Zustände für die 1-Wire Simulation definiert. Diese sind bei jedem Gerät gleich. Deshalb gibt es hier eine kurze Beschreibung dazu:

|Zustand (mode)|Beschreibung
| --- | ---
OW_SLEEP | Warten auf einen Resetimpuls. Alle fallenden Flanken auf dem Bus müssen dazu überprüft werden
OW_READ_ROM_COMMAND | Nach erfolgreichen Reset wird ein Befehl zur Zugriffskontrolle gelesen.
OW_MATCHROM | Empfangen einer ID und prüfen ob es die Eigene ist
OW_SEARCHROMS | Search-Rom-Algorithmus: Senden eines Bits aus der ID und danach die Negation (Complement) des Bits
OW_SEARCHROMR | Search-Rom-Algorithmus: Empfangen des Steuerbits vom Master
OW_READROM | Wenn nur ein 1-Wire Gerät am Bus ist kann die ID ausgelesen werden.
OW_WRITE_NEWID1 | Schreiben einer neuen ID in den ID-Zwischenspeicher
OW_READ_NEWID1 | Kontrolieren der ID aus den Zwischenspeicher
OW_SET_NEWID1 | Übernehmen der ID aus den Zwischenspeicher in den ID-Speicher (EEPROM)
OW_FIRST_COMMAND | Kein Zustand, enthällt die Nummer des nächsten Zustandes für die gerätespezifischen Zustände (Befehle)
1 Nur wenn die Möglichkeit zur ID-Veränderung eingeschaltet ist 

Die Zustände ensprechen nicht unbedingt immer den 1-Wire Command-Codes die behandelt werden. Diese sind deshalb nocheinmal in der folgenden Tabelle angegeben:

Code | Beschreibung
 --- | ---
0x55 | MATCH ROM: selektieren eines Gerätes anhand einer ID
0xF0 | SEARCH ROM: Suchalgorithmus für die angeschlossenen Geräte
0xCC | SKIP ROM: Überspringen von MATCH ROM wenn nur ein Gerät am Bus
0x33 | READ ROM: Lesen der ID wenn nur ein Gerät am Bus
0xEC | ALARM SEARCH: SEARCH ROM für Geräte bei denen das Alarm-Flag gesetzt ist
0x75 | WRITE NEWID: Schreiben einer neuen ID in den Zwischenspeicher
0xA7 | READ NEWID: Lesen der ID aus dem Zwischenspeicher
0x79 | SET NEWID: Übernehmen der neuen ID
Nach OW_READ_ROM_COMMAND bzw. nach OW_READ_COMMAND werden die Codes gebrüft. Dafür gibt es in der Datei OWRomFunctions.s zwei Makros. Wenn das Sprungziel im bereich von -63 und +64 Befehlen liegt kann das Makro cjmp verwendet werden. Alternativ, wenn der Linker fehler bringt, muss das aufwändigere Makro cljmp verwendt werden. Dabei wird mehr Speicher benötigt und die Ausführung dauert länger.

In der Regel wird zunächst zu einer Initialisierungsroutinge (hrc_set_[befehl]) gesprungen, die den neuen Zustand und alle anderen Parameter einstellt. In der Sprungtabelle (handle_stable) steht der Sprungbefehl zu der Routine, die den aktuellen Zustand behandlet (rjmp h_[Zustand]).