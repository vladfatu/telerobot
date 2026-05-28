import glob
import re
import subprocess
import time

import cv2


devices = sorted(
    glob.glob("/dev/video*"),
    key=lambda p: int(re.sub(r"\D", "", p) or 9999),
)


def friendly_name_for(dev):
    try:
        info = subprocess.run(
            ["v4l2-ctl", "--device", dev, "--info"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except Exception:
        return ""

    text = (info.stdout or "") + (info.stderr or "")
    for line in text.splitlines():
        if "Card type" in line and ":" in line:
            return line.split(":", 1)[1].strip().replace("\t", " ")

    return ""


def opencv_can_open(index):
    for _attempt in range(1, 4):
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)

        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

            ok, frame = cap.read()
            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
            cap.release()

            if ok and frame is not None and actual_w == 640 and actual_h == 480:
                return True

        try:
            cap.release()
        except Exception:
            pass

        time.sleep(0.7)

    return False


valid = []

for dev in devices:
    try:
        out = subprocess.run(
            ["v4l2-ctl", "--device", dev, "--list-formats-ext"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except Exception:
        continue

    text = (out.stdout or "") + (out.stderr or "")

    if "Type: Video Capture" not in text:
        continue
    if "'MJPG'" not in text:
        continue
    if "640x480" not in text:
        continue

    match = re.search(r"/dev/video(\d+)$", dev)
    if not match:
        continue

    index = int(match.group(1))

    if not opencv_can_open(index):
        print(f"SKIP\t{dev}\t{index}\tOpenCV could not open/read MJPG 640x480@30")
        continue

    friendly = friendly_name_for(dev)
    valid.append((dev, index, friendly))


for dev, index, friendly in valid:
    print(f"CAMERA\t{dev}\t{index}\t{friendly}")
