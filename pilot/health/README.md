# Wanderer Pilot health monitor

`wanderer-health.sh` performs the first Pilot foundation checks and drives the
common-cathode RGB status LED:

| LED | State |
|---|---|
| Blue | Health service is starting |
| Green | Wi-Fi and power checks passed |
| Yellow | Degraded: weak/unknown signal, no default route, Base unreachable, or a historical power event |
| Red | Fault: Wi-Fi unavailable/disconnected, no IPv4 address, or extremely weak signal |
| Magenta | Active undervoltage or CPU throttling |
| Off while the POWER LED is on | Linux is down or the health service is not running |

The current state is written atomically to `/run/wanderer/health.env`. State
changes are written to the system journal.

## Prerequisites

The RGB LED must be exposed using the Linux LED subsystem. Add this to
`/boot/firmware/config.txt`:

```ini
# Wanderer common-cathode RGB status LED
dtoverlay=gpio-led,gpio=17,label=wanderer-red,trigger=none,active_low=0
dtoverlay=gpio-led,gpio=27,label=wanderer-green,trigger=none,active_low=0
dtoverlay=gpio-led,gpio=22,label=wanderer-blue,trigger=none,active_low=0
```

Reboot and confirm that these paths exist:

```sh
ls /sys/class/leds/wanderer-{red,green,blue}/brightness
```

Install the health check's runtime tools:

```sh
sudo apt update
sudo apt install iw iproute2 iputils-ping rfkill
```

NetworkManager's `nmcli` is used only as a fallback when it is already
available; installing or changing the Pi's network manager is not required.
`vcgencmd` is optional. When it is available, the monitor decodes active and
historical undervoltage and throttling flags. On Debian, it is normally supplied
by the `raspi-utils` package.

## Install

Run these commands from the repository root:

```sh
sudo install -D -m 0755 pilot/health/wanderer-health.sh \
    /usr/local/libexec/wanderer-health/wanderer-health.sh
sudo install -D -m 0644 pilot/health/wanderer-health.service \
    /etc/systemd/system/wanderer-health.service
sudo install -D -m 0644 pilot/health/health.conf \
    /etc/default/wanderer-health
sudo install -D -m 0644 pilot/health/README.md \
    /usr/local/share/doc/wanderer-health/README.md

sudo systemctl daemon-reload
sudo systemctl enable --now wanderer-health.service
```

If the Base PC has a stable hostname or address, edit
`/etc/default/wanderer-health` and set `WANDERER_BASE_HOST`. Then restart:

```sh
sudo systemctl restart wanderer-health.service
```

## Use

Inspect the service, journal, and latest health state:

```sh
systemctl status wanderer-health.service
journalctl -u wanderer-health.service -f
cat /run/wanderer/health.env
```

Run one check manually without changing the LED:

```sh
sudo /usr/local/libexec/wanderer-health/wanderer-health.sh --once --no-led
echo $?
```

The one-shot exit status is:

| Status | Meaning |
|---:|---|
| 0 | OK |
| 1 | WARN |
| 2 | FAULT |

The Wi-Fi signal bands are:

| Signal | Classification |
|---:|---|
| `>= -55 dBm` | excellent |
| `-56..-68 dBm` | good |
| `-69..-76 dBm` | usable |
| `< -76 dBm` | weak |

Thresholds can be changed in `/etc/default/wanderer-health`.

## Test on a development PC

The test uses mock `iw`, `ip`, `rfkill`, `ping`, and `vcgencmd` commands, so it
does not need Raspberry Pi hardware:

```sh
pilot/health/tests/test_wanderer_health.sh
```

It covers healthy Wi-Fi, weak and unusable signal, disconnection, and active
undervoltage, including the expected RGB LED outputs.
