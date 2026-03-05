# TeLeRobot

VR teleoperation for SO-ARM101 arms using a Meta Quest headset.

## Prerequisites

- ### A Meta Quest headset 
I only tested with Quest 3, but it should work with Quest 2 as well. If you have a different model and want to help support it, please open an issue or submit a PR!
- ### Python 3.11+
- ### [Poetry](https://python-poetry.org/) for dependency management
```bash
curl -sSL https://install.python-poetry.org | python3 -
``` 
and add Poetry's bin directory in to your PATH environment variable (as suggested by the installer) 
- ### SSL certificates (required for HTTPS/WebRTC)
```bash
mkdir -p ssl_cert && openssl req -x509 -newkey rsa:4096 -keyout ssl_cert/server.key -out ssl_cert/server.crt -days 365 -nodes -subj '/CN=localhost'
```

## Install

Move to the project directory and install dependencies:
```bash
poetry install
```

## Configuration

Telerobot requires a `config.yaml` file at the project root. Ready-made templates are provided in `examples/config/` — pick the one that matches your hardware:

| Setup | Template |
|---|---|
| Single arm | `examples/config/single-arm.yaml` |
| Dual arms (left + right) | `examples/config/dual-arms.yaml` |

Copy the appropriate template and adjust it to your setup:

```bash
# Single arm
cp examples/config/single-arm.yaml config.yaml

# Dual arms
cp examples/config/dual-arms.yaml config.yaml
```

### Port and camera index settings(mandatory)
At minimum, update the `port` field(s) under `arms` to match your robot's serial port, and the `index` field(s) under `cameras` to match your connected cameras.

#### Dataset recording settings (optional)
To record episodes to a LeRobot dataset, uncomment the `dataset` section and update the fields as follows:
- `dataset.repo_id` — the Hugging Face Hub repo where the dataset will be stored (e.g. `your-username/vr-episodes`)
- `dataset.single_task` — a string describing the task being performed in the recorded episodes (e.g. "Pick and place the cube")
- `dataset.push_to_hub` — set to true to automatically push the dataset to Hugging Face Hub on save. Run the following command to log in to your Hugging Face account before starting telerobot if you want to use this feature:
``` bash
poetry run hf auth login --token  <YOUR_HF_TOKEN> --add-to-git-credential
```

#### Other notable settings (optional):
- `regularization` — adjust to increase/decrease the smoothness of the robot's motion (higher = smoother but less responsive) A regularization of 3.0e-3 or 4.0e-3 should allow you to move the controller fast all the way to the edge of it's reach without the arm shaking, but it also means that the arm will be less responsive to small controller movements.
- `end_effector_step_sizes` — adjust to increase/decrease the robot's reach compared to the VR controller's movement (with values of { x: 0.8, y: 0.8, z: 0.8 }, the robot's end effector will move 80cm when you move the controller by 1m)

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
