# AMS-AS7263-AS7343

Zephyr RTOS driver module for two AMS spectral sensors:

- **AS7263** — 6-channel NIR spectral sensor (610/680/730/760/810/860 nm)
- **AS7343** — 14-channel multispectral sensor (18-channel readout via SMUX Direct Chain Configuration)

Both drivers are pure C, platform-agnostic register/protocol logic wired into Zephyr via `sensor_driver_api`, `i2c_dt_spec`, and devicetree bindings. Not tied to any specific breakout board — applies to SparkFun modules or a bare die on a custom PCB.

## Layout

```
AMS-AS7263-AS7343/
├── CMakeLists.txt, Kconfig          # module root, includes drivers/
├── drivers/sensor/
│   ├── as7263/                      # AS7263 driver (.c/.h/_reg.c) + Kconfig
│   └── as7343/                      # AS7343 driver (.c/.h/_reg.c) + Kconfig
└── dts/bindings/sensor/
    ├── ams,as7263.yaml
    └── ams,as7343.yaml
```

## AS7263 — NIR spectral sensor

- I2C, fixed address `0x49`
- No direct register file — all access goes through 3 physical registers (`STATUS`/`WRITE`/`READ`) using AMS's polled virtual-register protocol (datasheet "I2C Slave Interface"). No SMUX, no bank switching, single photodiode bank.
- Channels: R (610nm), S (680nm), T (730nm), U (760nm), V (810nm), W (860nm) — 16-bit raw + optional IEEE-754 float calibrated value per channel.
- DTS properties: `gain` (0=1x/1=3.7x/2=16x/3=64x, default 3), `int-time` (× 2.8ms, default 166), `led-present`.
- Kconfig: `AS7263` (auto-enabled via `DT_HAS_AMS_AS7263_ENABLED`), `AS7263_LED` for LED_DRV attribute support.

## AS7343 — Multispectral sensor

- I2C, fixed address `0x39`, expected `ID` register value `0x81`.
- Direct register-addressed (not virtual-register like the AS7263). Uses **Direct Chain Configuration** (AN001033) for full custom 18-channel SMUX mapping across all 3 chains — not limited to the default RAM/SMUXEN method, which only reaches chain 1.
- 18-channel readout cycles through 3 SMUX chains (6 channels each): violet/blue/green/NIR/visible/flicker-detect bands (F1–F8, FZ/FY/FXL, NIR, 2×VIS, FD per cycle).
- DTS properties: `again` (0–12, default 9 = 256x), `atime` (0–255, default 29), `astep` (0–65534, default 599), `channel-count` (`"6"`/`"12"`/`"18"`, default 18), `led-present`, `int-gpios` (optional, active-low open-drain).
- Kconfig: `AS7343` (auto-enabled), plus optional `AS7343_LED`, `AS7343_GPIO` (sensor's own GPIO pin, sync/trigger), `AS7343_FLICKER` (FD support, disabled by default).

Note: `AS7343_REG_STATUS` (0x93) and `AS7343_REG_STATUS5` share the same address — this is an inherited AMS/SparkFun datasheet quirk, not a bug introduced in this driver.

## Usage

Add as a Zephyr module (west manifest or `ZEPHYR_EXTRA_MODULES`), then instantiate in a board overlay:

```dts
&i2c0 {
    as7263: as7263@49 {
        compatible = "ams,as7263";
        reg = <0x49>;
        gain = <3>;
        int-time = <166>;
    };

    as7343: as7343@39 {
        compatible = "ams,as7343";
        reg = <0x39>;
        again = <9>;
        atime = <29>;
        astep = <599>;
        channel-count = "18";
    };
};
```

Enable in `prj.conf`:

```
CONFIG_I2C=y
CONFIG_SENSOR=y
CONFIG_AS7263=y
CONFIG_AS7343=y
```

Read via the standard Zephyr sensor API (`sensor_sample_fetch` / `sensor_channel_get`).
