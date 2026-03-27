# TeLeRobot

VR teleoperation for SO-ARM101 arms using a Meta Quest headset.

https://github.com/user-attachments/assets/666a8bd1-4923-46d0-863d-0c314d126301

## Prerequisites

- ### A Meta Quest headset 
I only tested with Quest 3, but it should work with Quest 2 as well. If you have a different model and want to help support it, please open an issue or submit a PR!
- ### Python 3.11+
- ### [Poetry](https://python-poetry.org/) for dependency management:
```bash
curl -sSL https://install.python-poetry.org | python3 -
``` 
> and add Poetry's bin directory in to your PATH environment variable (as suggested by the installer) 

## Install

Move to the project directory and install dependencies:
```bash
poetry install
```

## Generate SSL certificates (required for HTTPS/WebRTC):

```bash
mkdir -p ssl_cert && openssl req -x509 -newkey rsa:4096 -keyout ssl_cert/server.key -out ssl_cert/server.crt -days 365 -nodes -subj '/CN=localhost'
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

https://github.com/user-attachments/assets/1990d541-111a-41ab-a6a5-b5ea5f172b12


## Calibration
The first time you run telerobot, the arms will be in calibration mode. Follow the instructions in the terminal and move the arm to the middle of it's range and press enter. Then move each joint to the max and min positions and press enter. Do this for each arm.

For a more detailed explanation of the calibration process, see [this video from LeRobot](https://huggingface.co/docs/lerobot/so101#calibration-video).

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

https://github.com/user-attachments/assets/0f598147-994a-441a-a1ba-5cb857851442


## Connecting the Arms

Press the AR button to enter WEBXR mode, then press the Connect button. 
Next you can move the arms by holding the controller's grip button and moving the controller in the desired direction. The arm will follow the controller's movement with the end effector (the gripper) matching the controller's position and rotation. Move the joystick to open and close the gripper.

https://github.com/user-attachments/assets/bc44cb32-9636-479b-9f5f-b350102e4e7e

Try to move the arms around and get a feel for how they respond to the controller's movement. You can adjust the `regularization` and `end_effector_step_sizes` settings in the config file to find a balance between responsiveness and stability that works for you.

## Dataset Recording

To record an episode, press the Record button. Perform the desired task with the robot, then press the Save Episode button to save the episode to a LeRobot dataset and repeat for the next episode. After you record all the episodes you need, you can press the Save Dataset button to save the dataset and optionally push it to Hugging Face Hub if you enabled that feature in the config.


https://github.com/user-attachments/assets/0fe03e12-4e4c-484b-948e-5e66431d1ead


## Disabling Passthrough

To record good episodes it can be helpful to see only what the robot sees without the passthrough video from the headset. To do this, you can disable the Passthrough toggle in the web interface to hide the passthrough feed and only see the robot's camera feeds. It will be harder to controll the robot this way, but you should get better results during training.


https://github.com/user-attachments/assets/31be1a9b-55e2-443f-bd87-ab5510739faa


## Deleting Episodes
To delete an episode, run the following command with the list of episodes you want to delete(you need to be logged in for --push-to-hub to work):

```bash
poetry run telerobot delete-episodes --repo-id <YOUR_HF_USERNAME>/vr_test_single_arm> --episodes "[0, 2, 5]" --push-to-hub
```

