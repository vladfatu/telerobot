# Telerobot

VR teleoperation for dual SO-ARM101 robots using a Meta Quest headset.

## Prerequisites

- Python 3.11+
- [Poetry](https://python-poetry.org/) for dependency management
- A Meta Quest headset (e.g. Quest 3)
- SSL certificates in `ssl_cert/` (`server.crt` and `server.key`)

## Install

```bash
poetry install
```

## Usage

```bash
poetry run telerobot
```

To use a custom config file:

```bash
poetry run telerobot -c path/to/config.yaml
```

## Network Setup

**The laptop and the VR headset must be on the same Wi-Fi network.**

Find your laptop's local IP address:

```bash
# macOS
ipconfig getifaddr en0
```

## Connecting the Headset

Open the Meta Quest browser and navigate to:

```
https://<YOUR_IP>:8765
```

Replace `<YOUR_IP>` with your laptop's local IP (e.g. `192.168.1.42`).

> Since the server uses self-signed SSL certificates, the browser will show a security warning on first connection — accept it to proceed.
