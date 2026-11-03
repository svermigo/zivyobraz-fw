# Živý obraz - firmware
[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/MultiTricker/zivyobraz-fw)

Welcome to the Živý obraz repository with firmware for e-Paper development boards based on ESP32/ESP32-S3. Live Image is used to feed ePaper/e-Ink displays with image data from a web server, whether it is a PNG or a custom basic RLE format called Z1/Z2/Z3.

  * Basic information can be found on the project website: https://zivyobraz.eu/ (Czech)
  * Specific information regarding getting things work can be found in the documentation at: https://wiki.zivyobraz.eu/ (Czech)
  * Firmware supports many boards, you can find specific hardware for Živý obraz to buy at [LáskaKit](https://www.laskakit.cz/vyhledavani/?string=%C5%BEiv%C3%BD+obraz) and [Pájeníčko](https://pajenicko.cz/vyhledavani?search=%C5%BEiv%C3%BD%20obraz).

**The documentation also includes precompiled firmware for flashing several of the most common types of ePapers, or you can use the web installer in Edge/Chrome browsers at: https://zivyobraz.eu/?page=instalace**.
**The default password for the Wi-Fi that the board transmits after uploading the firmware is: `zivyobraz`.**

----

In brief, about custom compilation and settings in the firmware code:

You will need to have the following libraries installed:
```ini
lib_deps =
    zinggjm/GxEPD2@^1.6.6
    jnthas/Improv WiFi Library@^0.0.4
    bblanchon/ArduinoJson@^7.2.1
    adafruit/Adafruit GFX Library@^1.11.9
    sensirion/Sensirion I2C SHT4x@^1.1.2
    adafruit/Adafruit BME280 Library@^2.3.0
    sparkfun/SparkFun SCD4x Arduino Library@^1.1.2
    sensirion/Sensirion I2C STCC4@^1.0.0
    kikuchan98/pngle@^1.1.0
```

In **platformio.ini**, comment out default build flags under section **common** (they are specified here for automatic compilation checks on GitHub), so you can use your own display type for compilation in next steps:
```ini
build_flags =
    -D COLOR_TYPE=BW      # Comment out this for your own display type enabled by you in display.h
    -D DISPLAY_TYPE=GDEW0154T8 # Also comment out this
```

In code **board.h** do not forget to uncomment:
1. Type of board used (ESPink_V2, ES3ink, ...)
2. Select connection type (http/https) which will be used for contacting the server by using `-DUSE_CLIENT_HTTP`
or `-DUSE_CLIENT_HTTPS` compiler flag. Default value is `https` variant.
3. **Streaming feature** is ENABLED by default for all ESP32 devices. This allows efficient buffering of image data in RAM before streaming to the display. To disable on resource-constrained devices, add `-D STREAMING_DISABLED` to your build_flags in **platformio.ini**
4. If you plan to connect one of the supported sensors via uŠup for reading temperature, humidity, and pressure/CO2 and sending the values to the server, uncomment in **sensor.h**
```c
// #define SENSOR
```
5. Display type has to be changed in **display.h** In the case of GRAYSCALE, you must remove `zinggjm/GxEPD2` from **platformio.ini** (just comment it out), otherwise there will be a library collision and the code will not work. In that case, `lib/GxEPD2_4G` will be used. For other displays (BW, 3C, 7C), leave `zinggjm/GxEPD2` active, you don't need to do anything with the 4G version.
```c
#define COLOR_TYPE=BW           // black and white
// #define COLOR_TYPE=3C        // 3 colors - black, white and red/yellow
// #define COLOR_TYPE=GRAYSCALE // grayscale - 4 colors
// #define COLOR_TYPE=7C        // 7 colors
```
6. Uncomment the definition of the specific ePaper you are putting into operation. This section begins at line `18`, and you need to select a specific display, e.g.:
```c
// BW
// #define DISPLAY_TYPE=GDEY0213B7 // 122x250, 2.13"
// #define DISPLAY_TYPE=GDEW042T2  // 400x300, 4.2"
#define DISPLAY_TYPE=GDEW075T7     // 800x480, 7.5"
```

### EPDiy parallel displays (SVERIO Paperboard)

For e-Paper displays with a parallel interface (e.g. ED060XC3, ED097TC2) driven by the [EPDiy](https://github.com/Pajenicko/epdiy) driver on the SVERIO Paperboard, dedicated build environments are already defined in **platformio.ini**. You do **not** need to manually edit `display.h` or `board.h` — simply select the matching environment to compile and use your display in definition.

```ini
[env:sverio_paperboard_epdiy]
```

These environment automatically set:
- `COLOR_TYPE=8G` (8-level grayscale)
- `BOARD_TYPE=SVERIO_PAPERBOARD_EPDIY`
- `USE_EPDIY_DRIVER` flag
- EPDiy library dependency (`https://github.com/Pajenicko/epdiy.git`)
- Both `GxEPD2` and `GxEPD2_4G` libraries are ignored (not needed for parallel displays)

If your display requires a specific VCOM voltage (might be referred on sticker placed on display), uncomment and adjust the `-D EPDIY_VCOM=1500` build flag.


### LTE instead of Wi-Fi (SIMCom A7682E)

The `sverio_paperboard_epdiy_lte` and `sverio_paperboard_spi_lte` environments replace Wi-Fi
with a SIMCom A7682E 4G modem.
Everything else (display handling, sensors, image streaming, deep sleep) behaves exactly as
in the Wi-Fi builds; OTA is off by default (see below). It is enabled by `-D USE_LTE_MODEM`; without that define the firmware is
built exactly as before.

```ini
[env:sverio_paperboard_epdiy_lte]   ; EPDiy Paperboard (parallel displays)
[env:sverio_paperboard_spi_lte]     ; SPI Paperboard (GxEPD2 displays, GDEM102T91 by default)
```

The modem board mates with the Paperboard's `U11` (SH 1.0 mm, 6 pin) through a straight 1:1 cable:

| Carrier `J3` | Paperboard `U11` | EPDiy board: silk / GPIO | SPI board: GPIO | Signal |
| ------------ | ---------------- | ------------------------ | --------------- | ------ |
| 1 | 1 | — | — | GND |
| 2 | 2 | — | — (3.3 V on U11) | not connected on the carrier |
| 3 | 3 | D9 / 10  | 6 | `MCU_RX` ← modem TX |
| 4 | 4 | D10 / 11 | 7 | `MCU_TX` → modem RX |
| 5 | 5 | D11 / 12 | 8 | `DTR` — LOW = awake, HIGH = allow sleep |
| 6 | 6 | D12 / 13 | 9 | `EN` — HIGH = modem powered, LOW = hard off |

On the EPDiy board these GPIOs are the upper half of the parallel bus and are unused by 8-bit
displays; on the SPI board GPIO10–13 drive the display, so `U11` is routed to GPIO6–9 instead.
Override them with `-D LTE_PIN_RX=…`, `-D LTE_PIN_TX=…`, `-D LTE_PIN_PWR_EN=…`, `-D LTE_PIN_DTR=…`.

Two modem boards use this connector and these semantics:

- **`lte-a7682e-carrier`** (recommended) carries the A7682E itself. `EN` switches the module's VBAT
  through a load switch, the board pulses `PWRKEY` on its own at power-up, a level shifter bridges
  the module's 1.8 V UART, and the bulk capacitance sits right at the module.
- **`lte-a7682e-daughter`** is the earlier board: `EN` enables a 5 V boost feeding a third-party
  carrier. It works, but the transmit bursts during network attach can brown the 5 V rail out unless
  it carries roughly 2 mF of bulk.

Both pull `EN` down, so a floating pad (MCU in reset, cable unplugged) leaves the modem off.

Build flags:

- `-D USE_LTE_MODEM` — enables the cellular stack (`lte_modem.cpp`, `lte_client.cpp`,
  `wireless_lte.cpp`) and compiles out Wi-Fi, WiFiManager and Improv. Communication is plain HTTP
  over a raw TCP socket on the modem; the module's TLS stack is a separate AT command set that is
  not wired up.
- `-D LTE_APN='"internet"'` — force an APN. When left out, the APN offered by the network
  (`AT+CGNAPN`) or the SIM default is used. The firmware also honours an APN in NVS under the key
  `apn` (it takes precedence), but nothing in this firmware writes one yet; the long button press
  clears it.
- `-D LTE_BAUD=115200` — the default: no `AT+IPR` switch. 460800 cuts a 30 kB image from about
  4 s to 1.3 s, but some A7682E samples reboot on the switch itself, so it is opt-in.
- `-D LTE_KEEP_POWERED` — keep the modem registered between refreshes and park it in `AT+CSCLK`
  sleep instead of cutting `EN`. Wakes faster, but the module keeps drawing a few mA, so this is
  only sensible on mains power.
- `-D LTE_FORCE_BAND=20` — pin the LTE search to a single band. A low band (20 = 800 MHz,
  8 = 900 MHz) has less path loss than 2100 MHz, so the network commands lower transmit power.
  Only useful where that band has coverage.
- `-D LTE_PREFER_LTE_ONLY=0` — skip the LTE-only step of a cold search and go straight to all
  bands including GSM. The LTE-only step is on by default because GSM's TDMA bursts are the
  harshest load the supply sees.
- `-D LTE_RADIO_SETTLE_MS=1500` — quiet time with the radio parked (`AT+CFUN=0`) before the first
  search.
- `-D LTE_POWER_OFF_GRACE_MS=2000` — time between `AT+CPOF` and cutting `EN`. The module answers
  `OK` before it has saved its state, and a hard power cut in that window can corrupt its flash.
- `-D LTE_FULL_BAND_SCOPE='"0X…,0X…,0X…"'` — the module's complete band mask (GSM, LTE, TD-SCDMA)
  that a widened search writes. Defaults to what the A7682E-LASA reports from the factory; set it
  for another module variant.
- `-D LTE_ALLOW_OTA` — enable firmware updates over LTE. **Off by default**: OTA runs over plain
  HTTP, and the LTE build reports the same `board` as the Wi-Fi Paperboard build, so a server that
  does not look at the payload's `"fwVariant": "lte"` could hand it the Wi-Fi image. Without the
  flag an `X-OTA-Update` header is logged and ignored, and the screen refreshes normally.

If the modem is already registered when bring-up reaches the network step (kept powered, or it
attached by itself while booting), that attach is kept. Otherwise bring-up walks an escalation
ladder, gentlest first: the remembered band, then every LTE band, then everything including GSM.
The operator, access technology and band of the last successful attach are cached in NVS through
the existing `Wireless::saveConnectionCache()` hook, so the server's `ForceWifiFullScan` header
clears the cellular cache too, and the next bring-up then walks the ladder even if the module
has already registered by itself on the old band. The cache is tied to the modem's IMEI and is dropped when a
different module is fitted.

If the modem attaches but the server cannot be reached, the LTE build backs off in the same steps
as for a missing link (2 min, 10 min, 30 min, 1 h, 12 h), whether the server does not answer or answers with an error: `main.cpp` resets its own failure count
once the link is up, and on LTE every retry costs a whole modem bring-up.

`begin()` logs a phase breakdown (`[4584 ms] module booted, AT responding`, …), the battery voltage
at power-up, any unexpected module restart, and — when registration fails — the signal level and
the full radio state (`CFUN`, `COPS`, `CNMP`, `CNBP`, `CPIN`, `CPSI`). With
`-D LOG_LEVEL_MINIMUM=Logger::Level::DEBUG` the whole AT conversation is logged as well.

There is no configuration portal in this build. If the modem cannot attach, the device shows the
offline screen and retries after deep sleep with the usual exponential backoff. Holding the button
for more than 6 s clears the stored APN instead of the Wi-Fi credentials.

After successfully compiling and flashing the board, continue with the documentation "Bringing your own ePaper to life":
https://wiki.zivyobraz.eu/doku.php?id=start#oziveni_vlastniho_epaperu

----

**Simple diagram of how it works**

![](how_it_works_diagram.webp)

----

## License info 

![CC BY-NC-SA 4.0](https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png)

ZivyObraz FW itself is licensed under a [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License](http://creativecommons.org/licenses/by-nc-sa/4.0/):

Attribution—Noncommercial—Share Alike  
✖ | Sharing without ATTRIBUTION  
✖ | Commercial Use  
✖ | Free Cultural Works  
✖ | Meets Open Definition
