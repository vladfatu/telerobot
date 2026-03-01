import json
import shutil
from pathlib import Path

from lerobot.datasets.image_writer import safe_stop_image_writer
from lerobot.datasets.lerobot_dataset import HF_LEROBOT_HOME, LeRobotDataset
from lerobot.datasets.pipeline_features import (
    aggregate_pipeline_dataset_features,
    create_initial_features,
)
from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts, ACTION, OBS_STR
from lerobot.processor import make_default_processors

from telerobot.logger import log_message


def setup_dataset(robot, cfg, logger) -> LeRobotDataset | None:
    """Create a LeRobotDataset for episode recording, or None if not configured."""
    if cfg.dataset is None:
        return None

    teleop_action_processor, _robot_action_processor, robot_observation_processor = (
        make_default_processors()
    )

    dataset_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=True,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=True,
        ),
    )

    # Streaming encoding is always enabled with auto HW-accelerated codec (VideoToolbox on
    # macOS, NVENC on Linux with Nvidia, etc.) for near-instant save_episode() calls.
    VCODEC = "auto"

    dataset_root = Path(cfg.dataset.root) if cfg.dataset.root is not None else HF_LEROBOT_HOME / cfg.dataset.repo_id
    info_path = dataset_root / "meta" / "info.json"
    dataset_exists = info_path.exists()

    # If the dataset dir was initialised but no frames were ever recorded (0 total_frames),
    # LeRobotDataset.__init__ will fail to find local parquet files and try to fetch from
    # the Hub → 404. Treat this the same as a missing dataset by wiping the empty directory.
    if dataset_exists:
        with open(info_path) as f:
            info = json.load(f)
        if info.get("total_frames", 0) == 0:
            shutil.rmtree(dataset_root)
            dataset_exists = False

    if dataset_exists:
        dataset = LeRobotDataset(
            repo_id=cfg.dataset.repo_id,
            root=cfg.dataset.root,
            streaming_encoding=True,
        )
        log_message(logger, f"📂 Resuming existing dataset: {cfg.dataset.repo_id} ({dataset.num_episodes} episodes so far)")
    else:
        dataset = LeRobotDataset.create(
            repo_id=cfg.dataset.repo_id,
            fps=cfg.fps,
            root=cfg.dataset.root,
            robot_type=robot.name,
            features=dataset_features,
            use_videos=True,
            vcodec=VCODEC,
            streaming_encoding=True,
        )
        log_message(logger, f"📁 Dataset recording enabled: {cfg.dataset.repo_id}")
    log_message(logger, f"🎬 Streaming video encoding active (vcodec={VCODEC})")
    return dataset


def end_active_episode(
    dataset: LeRobotDataset | None,
    logger,
) -> None:
    """Save and close the current episode if recording is active."""
    if dataset is not None:
        dataset.save_episode()
        log_message(
            logger,
            f"✅ Episode {dataset.num_episodes - 1} saved ({dataset.num_frames} total frames)",
        )

def record_step(
    dataset: LeRobotDataset | None,
    cfg,
    obs,
    action,
) -> None:
    """Record a single observation/action step into the dataset."""
    if dataset is None:
        return

    observation_frame = build_dataset_frame(
        dataset.features, obs, prefix=OBS_STR
    )
    action_frame = build_dataset_frame(
        dataset.features, action, prefix=ACTION
    )
    frame = {
        **observation_frame,
        **action_frame,
        "task": cfg.dataset.single_task,
    }
    dataset.add_frame(frame)


def finalize_dataset(dataset: LeRobotDataset | None, push_to_hub: bool, logger) -> None:
    """Gracefully stop dataset recording and background writers.
    
    Optionally pushes the dataset to the Hugging Face Hub.
    """
    if dataset is None:
        return

    # Stop the image writer only when not using streaming encoding (streaming
    # encoder threads manage their own lifecycle via finish_episode / cancel_episode).
    streaming_encoder = getattr(dataset, "_streaming_encoder", None)
    if streaming_encoder is None:
        safe_stop_image_writer(dataset)
    else:
        streaming_encoder.close()
    dataset.finalize()  # Flush buffered episode metadata to parquet files
    log_message(logger, f"📊 Total episodes recorded: {dataset.num_episodes}")

    if push_to_hub:
        if dataset.num_episodes == 0:
            log_message(logger, "⚠️ No episodes recorded — skipping push to Hub.")
            return
        try:
            log_message(logger, f"🚀 Pushing dataset '{dataset.repo_id}' to Hugging Face Hub...")
            dataset.push_to_hub()
            log_message(logger, f"✅ Dataset '{dataset.repo_id}' pushed successfully.")
        except Exception as e:
            log_message(logger, f"❌ Failed to push dataset to Hub: {e}")
            log_message(logger, "   Make sure you are logged in (run `huggingface-cli login`) or set HF_TOKEN.")
