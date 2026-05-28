import json
import sys
import time
from pathlib import Path

import cv2


def can_open_camera(spec):
    index = int(spec["index"])
    width = int(spec.get("width", 640))
    height = int(spec.get("height", 480))
    fps = int(spec.get("fps", 30))
    fourcc = str(spec.get("fourcc", "MJPG"))

    for _attempt in range(1, 4):
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)

        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))

            ok, frame = cap.read()
            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
            cap.release()

            if ok and frame is not None and actual_w == width and actual_h == height:
                return True

        try:
            cap.release()
        except Exception:
            pass

        time.sleep(0.5)

    return False


def main():
    if len(sys.argv) < 2:
        print("ERROR\tmissing camera specs JSON path")
        return 2

    specs_path = Path(sys.argv[1])
    camera_specs = json.loads(specs_path.read_text(encoding="utf-8"))

    all_ok = True

    for spec in camera_specs:
        name = spec.get("name", "camera")
        index = spec.get("index", "?")

        if can_open_camera(spec):
            print(f"OK\t{name}\t{index}")
        else:
            print(f"FAIL\t{name}\t{index}")
            all_ok = False

    return 0 if all_ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
