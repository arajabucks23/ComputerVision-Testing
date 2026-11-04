# OV5640 → NUCLEO-F746ZG → Ethernet → PC

Live JPEG video from the OV5640 on the Nucleo, streamed as MJPEG over HTTP so the
CV code can open it with `cv2.VideoCapture("http://192.168.10.2/")`.

| Step | What | Status |
|---|---|---|
| 1 | Project: clocks, debug serial, Ethernet + lwIP, static IP | **done, tested** |
| 2 | Camera chip ID over I2C (0x300A/0x300B = 0x56/0x40) | **done**: reads 0x5640 (wiring self-test) |
| 3 | Sensor JPEG mode, 640x480 then 1280x720 | **640x480 done**; 1280x720 next |
| 4 | DCMI + DMA capture, find JPEG start/end (FFD8/FFD9) | **done** |
| 5 | MJPEG HTTP server on port 80 | **done**: `http://192.168.10.2/stream`, log at `/log` |
| 6 | Python receiver (preview, fps, `get_frame()`, save frames) | **done**: `receiver.py`; detector: `people_and_fire_detector.py --stm32` |

### Measured (18 Oct 2026, 640x480 JPEG, ST quality setting 0x4407 = 0x04)

| | Result |
|---|---|
| Frame rate | **17.0 fps** captured, 17.0 fps sent, 17 fps received on the PC |
| JPEG size | **41 KB average**, 44 KB max (indoor lab scene) |
| Network load | ~0.7 MB/s (≈ 6 Mbit/s of the 100 Mbit/s link) |
| Lost frames | 0 bad, 0 too big, 0 skipped, 0 DCMI errors |
| Bottleneck | the sensor: ST's 640x480 timing runs it at ~17 fps; DCMI, DMA, RAM and lwIP all keep up |

### Live video

Double-click in the project folder:
- `camera_demo.bat`: live detector window with temperature/humidity (q or Esc quits).
- `record_camera_demo.bat`: same, but records 15 s to `test_footage\stm32\demo_<date>_<time>.mp4`.
  From a terminal you can pick the length: `record_camera_demo.bat 30`.

If the board can't be reached, the launcher prints a checklist (power, cable, PC IP, reset).

Or from a terminal:

```
C:\cvenv\Scripts\python.exe stm32_camera\receiver.py                 # preview + fps
C:\cvenv\Scripts\python.exe stm32_camera\receiver.py --save captures # also save JPEGs
C:\cvenv\Scripts\python.exe people_and_fire_detector.py --stm32      # your detector on the camera
```

In your own code: `from receiver import CameraStream; frame = CameraStream().get_frame()`.
`cv_node/playback.py` opens any `http://` source this way, so YOLO always gets the newest frame
instead of a growing queue. Only one viewer can stream at a time; a second gets "503 Busy".

### Things found while bringing up capture
- ST's HAL only enables the DCMI end-of-frame interrupt after the DMA fills the whole buffer
  (it expects fixed-size frames). For JPEG, `capture.c` enables it after each start.
- On this board, the JPEG bytes arrive while **HREF is low**, so DCMI HSYNC polarity is set to
  HIGH (the idle level). This was measured by sampling the data bus against HREF.

### DHT11 (temperature / humidity)
Wired **VCC → 5V** (CN8 left column, 5th pin from the top), DATA → PE4, GND → GND.
On 3V3 it stopped answering once the camera pulled the supply down, and stayed stuck
until power-cycled. The DHT11 is rated 3.3–5.5 V. PE4 is 5 V tolerant only with its
internal pull resistor off, so `dht11.c` uses no pull (the module has its own pull-up).
Readings print once per second on the serial port and at `http://192.168.10.2/log`:
`[dht11] 24.1 C  33 % RH  checksum ok` (30 of 30 readings passed, camera streaming).

### Open issue: 3.3 V supply sags with the camera running
The STM32 measures its own supply through VREFINT: **3.26 V** with the camera off, **~2.65 V**
while streaming, on laptop USB and on a wall charger alike. The Nucleo's USB power path
(500 mA switch U4 + 3.3 V regulator) is the limit. Effects: the DHT11 stops answering, and
the Ethernet PHY is out of spec (the link dropped to 10 Mbit/s once). Fix: power the camera's
3V pin from a separate 3.3 V supply with GND tied to the Nucleo's GND.

---

## Folder layout

```
stm32_camera/
  README.md            this file
  serial_monitor.py    prints the board's debug messages (COM port, 115200)
  firmware/
    build.ps1          build, and flash with -Flash
    CMakeLists.txt     source list, compiler flags
    arm-none-eabi.cmake
    STM32F746ZG_FLASH.ld   memory layout (RAM plan below)
    Core/Inc, Core/Src     our code: main.c, ethernetif.c, lwipopts.h, ...
    st_cubef7/         ST's STM32CubeF7 package (HAL, CMSIS, lwIP, LAN8742), from GitHub
    build/             output: evac_cam.elf / .bin / .map
```

## Build, flash and watch

```
powershell -ExecutionPolicy Bypass -File stm32_camera\firmware\build.ps1 -Flash
C:\cvenv\Scripts\python.exe stm32_camera\serial_monitor.py
```

The tools come from the STM32Cube VS Code extension's bundles in `%LOCALAPPDATA%\stm32cube\bundles`:
- the GNU Arm compiler,
- CMake and Ninja,
- STM32_Programmer_CLI,
- the ST-LINK USB driver.

### Why there's no CubeMX `.ioc` file

The CubeMX that installs from the VS Code extension is **STM32CubeMX2**, which only supports newer STM32 families, not the F7. Classic CubeMX needs a separate download from st.com.

Instead, the project is built straight from ST's own code:
- the HAL,
- lwIP,
- ST's ready-made lwIP examples for this board (`STM32F746ZG-Nucleo/LwIP_HTTP_Server_Netconn_RTOS` for the pins and PHY, `STM32F769I_EVAL/LwIP_HTTP_Server_Raw` for the no-RTOS structure).

Every setting CubeMX would hold is in the code, listed below. The STM32H743 package (STM32CubeH7) has the same layout, so this carries over.

---

## Settings (what CubeMX would hold, and where it lives)

| Setting | Value | Where |
|---|---|---|
| Clock | 8 MHz from ST-LINK (HSE bypass) → PLL M8 N432 P2 Q9 → **216 MHz**, AHB 216, APB1 54, APB2 108, flash 7 WS, over-drive on | `main.c` `SystemClock_Config` |
| Caches | I-cache **on**, D-cache **off** (DMA buffers need no cache maintenance) | `main.c` `main` |
| Debug serial | USART3, PD8 TX / PD9 RX (AF7), 115200 8N1 → ST-LINK COM port | `main.c` `USART3_Init` |
| LEDs | LD1 PB0 (green, blinks 2×/s), LD2 PB7, LD3 PB14 (red = fatal error) | `main.h` |
| Camera control | PE8 = PWDN (high = off), PE7 = RESET (low = reset). Off until step 2 | `main.c` `GPIO_Init` |
| Ethernet | RMII, LAN8742 PHY: PA1 PA2 PA7 PB13 PC1 PC4 PC5 PG11 PG13 (AF11) | `ethernetif.c` `HAL_ETH_MspInit` |
| MAC address | 02:45:56:41:43:01 (locally administered) | `stm32f7xx_hal_conf.h` |
| lwIP | no RTOS (`NO_SYS 1`), raw API, static IP **192.168.10.2/24**, gateway 192.168.10.1 | `lwipopts.h`, `main.h` |
| TCP tuning | MSS 1460, send buffer 8 × MSS, window 4 × MSS, hardware checksums | `lwipopts.h` |
| I2C2 (camera SCCB) | PF0 SDA / PF1 SCL, 100 kHz | step 2 |
| DCMI | 8-bit, PCLK rising, VSYNC high, HSYNC low, JPEG mode, DMA2 Stream 1 | step 4 |

PC side: the wired Ethernet adapter is set to **192.168.10.1/24** (already done on this laptop).

---

## Pin check

Checked against the STM32F746xx datasheet (alternate-function table) and UM1974 (NUCLEO-144 user manual).

| Signal | Pin | Function (AF) | Free on the Nucleo? |
|---|---|---|---|
| DCMI D0 ← cam D2 | PC6 | DCMI_D0 (AF13) | yes |
| DCMI D1 ← cam D3 | PC7 | DCMI_D1 (AF13) | yes |
| DCMI D2 ← cam D4 | PC8 | DCMI_D2 (AF13) | yes |
| DCMI D3 ← cam D5 | PC9 | DCMI_D3 (AF13) | yes |
| DCMI D4 ← cam D6 | PC11 | DCMI_D4 (AF13) | yes |
| DCMI D5 ← cam D7 | PD3 | DCMI_D5 (AF13) | yes |
| DCMI D6 ← cam D8 | PB8 | DCMI_D6 (AF13) | yes (Arduino D15, nothing on board) |
| DCMI D7 ← cam D9 | PB9 | DCMI_D7 (AF13) | yes (Arduino D14, nothing on board) |
| PCLK ← cam PC | PA6 | DCMI_PIXCLK (AF13) | yes |
| HSYNC ← cam HS | PA4 | DCMI_HSYNC (AF13) | yes |
| VSYNC ← cam VS | PG9 | DCMI_VSYNC (AF13) | yes (Arduino D0, nothing on board) |
| SCCB SCL | PF1 | I2C2_SCL (AF4) | yes |
| SCCB SDA | PF0 | I2C2_SDA (AF4) | yes |
| cam PD (power-down) | PE8 | GPIO out, HIGH = camera off | yes |
| cam RT (reset) | PE7 | GPIO out, LOW = in reset | yes |
| INA219 SCL / SDA | PD12 / PD13 | I2C4 (AF4) | yes (not used in this task) |
| DHT11 | PE4 | GPIO | yes (not used in this task) |

Pins the board already uses (none of these are touched):

| Used by | Pins |
|---|---|
| Ethernet RMII (LAN8742 PHY) | PA1, PA2, PA7, PB13, PC1, PC4, PC5, PG11, PG13 |
| ST-LINK serial (USART3) | PD8, PD9 |
| LEDs LD1 / LD2 / LD3 | PB0, PB7, PB14 |
| User button | PC13 |
| USB OTG FS (CN13 "User USB") | PA8, PA9, PA10, PA11, PA12, PG6, PG7 |
| SWD debug | PA13, PA14, PB3 |
| Clocks (8 MHz from ST-LINK, 32 kHz crystal) | PH0, PH1, PC14, PC15 |

**Result: no conflicts.** Your wiring avoids the alternative DCMI pins that the board
already uses (PA9/PA10 = USB, PB7 = LED2, PG11 = Ethernet).

Camera breakout notes (Adafruit 5838):
- Data lines are labelled **D2 to D9**: camera D2 → PC6 (DCMI D0), and so on up to camera D9 → PB9.
- Clock jumper ships on **EXT**. For INT, cut the centre-to-EXT trace and bridge centre to INT. Leave XC unconnected.
- I2C pull-ups are already on the breakout.

### Board jumpers (Nucleo-144, UM1974)

These were wrong on this board and stopped it from powering up or being programmed:

| Jumper | Must be | Symptom when wrong |
|---|---|---|
| **JP3** "Power Source" | on **U5V** | PWR LED (LD6) off, 0.0 V on the STM32, no Ethernet link |
| **CN4** | **both** jumpers on | "No STM32 target found" although powered |
| **CN6** (SWD) | **no** jumper | can block debug connection |
| JP1 | off | |
| JP5 (IDD) | on | |

One USB cable in **CN1** (the ST-LINK end) powers everything. CN13 at the other end is the STM32's own USB, not needed.

---

## Wiring self-test (runs at every boot)

`wiring_check.c` tests every camera and DHT11 wire and prints one OK/FAIL line per wire on the serial port. Press the Nucleo's black RESET button to run it again.

| Check | How |
|---|---|
| SDA/SCL | idle high = the breakout's pull-ups are powered |
| Camera on I2C | answers at 0x3C, chip ID 0x5640 |
| RT / PD | the camera must go silent when each one is asserted |
| D2–D9, PC, HS, VS | OV5640 pad registers 0x3017/0x3018 (output enable), 0x301A/0x301B (value) and 0x301D/0x301E (select) drive one camera pin high at a time. The STM32 reads all 11 pins, which finds open, swapped and shorted wires. |
| DHT11 | full read with checksum. After that it prints once per second. |

If the camera doesn't answer, it also prints which camera pin PE7/PE8 are wired to and tries all PD/RT levels with SDA/SCL normal and swapped.

Result on 18 Oct 2026: **all connections OK**, 3 out of 3 resets. DHT11 reads ~24 °C, 29 % RH.

### Camera clock (XC)

The breakout's XCLK jumper is still in its **factory EXT position**: the Ext–centre link is uncut and Int is not soldered. With no clock the OV5640 does not answer I2C at all.

For now the STM32 supplies the clock instead:
- a wire from **XC → PA8**,
- 16 MHz from PA8 (MCO1, internal HSI oscillator),
- `CAMERA_CLOCK_FROM_MCO 1` in `main.h`.

PA8 is the USB OTG SOF pin, but nothing on the Nucleo uses it while USB is off.

The permanent fix is to cut Ext–centre, solder centre–Int, remove the XC wire, and set `CAMERA_CLOCK_FROM_MCO 0`. The breakout's own 24 MHz oscillator is the value most OV5640 register tables assume.

---

## Step 1 test (done)

1. `build.ps1 -Flash`
2. `serial_monitor.py` shows:
   ```
   [boot] EvacAI camera node, SYSCLK = 216000000 Hz
   [boot] IP 192.168.10.2, try: ping 192.168.10.2
   [eth] link up, 100 Mbit/s full duplex
   ```
3. LD1 (green) blinks twice a second.
4. `ping 192.168.10.2` gets replies in under 1 ms.

Measured on 18 Oct 2026:
- Small pings: 0% loss, < 1 ms.
- **120 full-size (1472-byte) pings in 3 parallel streams: 0% loss.**
- Small pings still worked afterwards.

**Bug found and fixed during this test.** The HAL's blocking `HAL_ETH_Transmit()` doesn't record the packet it sends, so the packet is never released.
- Every reply leaked one buffer.
- After about 16 pings the receive pool was empty and the board stopped answering.
- `ethernetif.c` now uses `HAL_ETH_Transmit_IT()` and releases finished packets from the main loop. This is what ST's own Nucleo example does.

---

## RAM plan

The F746 has 320 KB of RAM in three blocks. A 720p JPEG from the OV5640 is roughly
40–120 KB depending on the scene and the JPEG quality setting.

| Block | Address | Size | What goes in it | Used now |
|---|---|---|---|---|
| DTCM | 0x20000000 | 64 KB | variables, lwIP heap (16 KB) and pools, 2 KB C heap, 4 KB stack | 32.7 KB |
| SRAM1 | 0x20010000 | 240 KB | **two 112 KB JPEG frame buffers** (camera fills one while the other is sent), step 4 | 0 KB |
| SRAM2 | 0x2004C000 | 16 KB | Ethernet DMA descriptors + 8 receive buffers | 12.8 KB |

If a 720p frame doesn't fit in 112 KB, the smallest fix is to lower the sensor's JPEG
quality (one register). A single bigger buffer would be the fallback, at roughly half
the frame rate.
