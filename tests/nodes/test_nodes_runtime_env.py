"""dora_runtime の環境変数の読み取りのテスト (TASK-0025)。dora が無くても動く (import しない部分だけ)。"""

import pytest

from so101_vlm.nodes.dora_runtime import config_from_env, task_from_env
from so101_vlm.nodes.node_config import REAL_CONFIG_PATH, SIM_CONFIG_PATH
from so101_vlm.robot.model_paths import REPO_ROOT


def test_config_defaults_to_sim():
    assert config_from_env({}).mode == "sim"


def test_config_path_can_be_relative_to_the_repo_or_absolute():
    assert config_from_env({"CONFIG": "configs/real.yaml"}).mode == "real"
    assert config_from_env({"CONFIG": str(REAL_CONFIG_PATH)}).mode == "real"
    assert config_from_env({"CONFIG": str(SIM_CONFIG_PATH)}).mode == "sim"


def test_out_dir_overrides_the_output_dir(tmp_path):
    assert config_from_env({"OUT_DIR": str(tmp_path)}).output_dir == tmp_path
    assert config_from_env({}).output_dir == REPO_ROOT / "media" / "dataflow"


def test_missing_config_file_is_file_not_found():
    with pytest.raises(FileNotFoundError):
        config_from_env({"CONFIG": "configs/nope.yaml"})


def test_task_from_env():
    task = task_from_env({"TASK_SEED": "3", "TASK_COLOUR": "blue", "TASK_INSTRUCTION": "x"})
    assert (task.seed, task.colour, task.instruction) == (3, "blue", "x")


@pytest.mark.parametrize(
    "env",
    [
        {"TASK_COLOUR": "red", "TASK_INSTRUCTION": "x"},
        {"TASK_SEED": "a", "TASK_COLOUR": "red", "TASK_INSTRUCTION": "x"},
        {"TASK_SEED": "1", "TASK_COLOUR": "purple", "TASK_INSTRUCTION": "x"},
    ],
)
def test_bad_task_env_is_value_error(env):
    with pytest.raises(ValueError):
        task_from_env(env)
