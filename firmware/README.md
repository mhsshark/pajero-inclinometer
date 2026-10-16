# Prebuilt firmware

These are ready-to-flash images for the **Waveshare ESP32-S3-Touch-LCD-2.8C** (ESP32-S3, 16 MB flash, 8 MB octal PSRAM).
Use **one** of the two files. You don't need both.

| File | Flash address | What it contains | Use it when |
|---|---|---|---|
| `inclinometer-full-0x0.bin` | **`0x0`** | Bootloader + partition table + boot selector + app, merged into one image | First install, a blank board, or a board running other firmware |
| `inclinometer-app-0x10000.bin` | **`0x10000`** | The app only | Updating a board that already runs this inclinometer |

## Why two addresses?

The ESP32-S3 flash is laid out like this (`default_16MB.csv`):

| Address | Contents |
|---|---|
| `0x0` | Bootloader. The ROM jumps here after reset. |
| `0x8000` | Partition table |
| `0x9000` | NVS: settings, **including the saved level calibration** |
| `0xE000` | OTA data (selects which app slot boots) |
| `0x10000` | App (`app0`), which is the inclinometer firmware itself |

- **The full image starts at `0x0`.** It covers everything from the bootloader up to the end of the app, so a board
  with anything on it boots this firmware. It also passes over the NVS area and **wipes the saved level**, so set
  level again afterwards.
- **The app image goes at `0x10000`.** It replaces only the program and leaves the bootloader, partition table and
  your calibration alone. It only works if the board already has a matching bootloader and partition table, for
  example after flashing the full image once.

## Flashing

esptool (`pip install esptool`):

```bash
esptool.py --chip esp32s3 write_flash 0x0 inclinometer-full-0x0.bin
```

```bash
esptool.py --chip esp32s3 write_flash 0x10000 inclinometer-app-0x10000.bin
```

Browser flasher, such as the Espressif web flasher: add the file and set the address to `0x0` for the full image or
`0x10000` for the app image.

If the port doesn't appear, hold **BOOT**, tap **RESET**, then release **BOOT**. Press RESET again after flashing.

## After flashing the full image: set level

Park on level ground, then either:
- hold the BOOT button, or a button wired IO0 ↔ GND, for 2 s, or
- open a serial monitor (115200 baud) and send `z`.

Other serial commands are `c` to clear the level, `d` to toggle the debug log, and `?` for help.

## Checksums (SHA-256)

```
c67089888a51ae3ef3e95b9262fbf8fbf599967c4e01916c66508a72398c034c  inclinometer-full-0x0.bin
8f88ded4ec79cd93f876399b412002a8e4613ab54ee8828898f8d282f04c869d  inclinometer-app-0x10000.bin
```
