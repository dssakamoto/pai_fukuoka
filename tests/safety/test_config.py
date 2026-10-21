"""SafetyConfig / WorkspaceBox / load_safety_config のテスト (TASK-0010, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0010/safety-testcases.md
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import copy
import dataclasses
import inspect

import pytest
import yaml

from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.robot.model_paths import REPO_ROOT
from so101_vlm.safety import config as config_module
from so101_vlm.safety.config import (
    DEFAULT_MOTION_CONFIG_PATH,
    SafetyConfig,
    WorkspaceBox,
    load_safety_config,
)
from so101_vlm.safety.limits import is_in_workspace

WS_LOWER = (0.12, -0.13, 0.00)
WS_UPPER = (0.38, 0.26, 0.17)

VALID = {
    "safety": {
        "vmax": [1.0] * 6,
        "control_dt": 0.02,
        "speed_scale": 1.0,
        "state_timeout": 0.2,
        "workspace": {"lower": [0.12, -0.13, 0.00], "upper": [0.38, 0.26, 0.17]},
    }
}


def _write(tmp_path, obj):
    """obj が str ならそのまま、それ以外は safe_dump して motion.yaml に書く。"""
    path = tmp_path / "motion.yaml"
    path.write_text(obj if isinstance(obj, str) else yaml.safe_dump(obj), encoding="utf-8")
    return path


def _with(path_keys, value):
    """VALID の深いコピーの path_keys 位置を value にした dict を返す (VALID は不変)。"""
    obj = copy.deepcopy(VALID)
    target = obj
    for key in path_keys[:-1]:
        target = target[key]
    target[path_keys[-1]] = value
    return obj


def _without(path_keys):
    obj = copy.deepcopy(VALID)
    target = obj
    for key in path_keys[:-1]:
        target = target[key]
    del target[path_keys[-1]]
    return obj


def _make(**overrides) -> SafetyConfig:
    kwargs = dict(
        vmax=(1.0,) * 6,
        control_dt=0.02,
        speed_scale=1.0,
        state_timeout=0.2,
        workspace=WorkspaceBox(lower=WS_LOWER, upper=WS_UPPER),
    )
    kwargs.update(overrides)
    return SafetyConfig(**kwargs)


# =============================================================== TC-SC
def test_safety_config_constructs_with_defaults():
    """TC-SC-01 既定値で生成でき、vmax は tuple の float 🔵"""
    cfg = _make()
    assert isinstance(cfg.vmax, tuple)  # 【確認内容】: tuple で保持
    assert all(type(v) is float for v in cfg.vmax)  # 【確認内容】: 各値 float
    assert cfg.workspace.lower == WS_LOWER
    assert cfg.workspace.upper == WS_UPPER


def test_safety_config_is_frozen():
    """TC-SC-02 frozen (入れ子の workspace も) 🔵"""
    cfg = _make()
    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.speed_scale = 0.3
    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.workspace.lower = (0.0, 0.0, 0.0)


def test_replace_speed_scale_creates_new_config():
    """TC-SC-03 replace で 0.3 倍の設定が作れ、元は不変 🔵"""
    cfg = _make()
    slow = dataclasses.replace(cfg, speed_scale=0.3)
    assert slow.speed_scale == 0.3
    assert cfg.speed_scale == 1.0  # 【確認内容】: 元の設定は変わらない


def test_lists_are_converted_to_tuples():
    """TC-SC-04 list を渡しても tuple で保持し、外の list の変更に影響されない 🟡"""
    vmax = [1.0] * 6
    lower = [0.12, -0.13, 0.0]
    box = WorkspaceBox(lower=lower, upper=[0.38, 0.26, 0.17])
    cfg = _make(vmax=vmax, workspace=box)
    vmax[0] = 9.0
    lower[0] = 9.0
    assert cfg.vmax == (1.0,) * 6
    assert isinstance(cfg.vmax, tuple)
    assert box.lower == (0.12, -0.13, 0.0)
    assert isinstance(box.lower, tuple)


@pytest.mark.parametrize(
    "overrides",
    [
        {"speed_scale": 1.0},
        {"speed_scale": 1e-6},
        {"vmax": (1e-9, 1.0, 1.0, 1.0, 1.0, 1.0)},
    ],
)
def test_boundary_values_are_accepted(overrides):
    """TC-SC-05 境界値 (scale 1.0 / 1e-6、vmax 1e-9) は受け付ける 🟡"""
    _make(**overrides)  # 【確認内容】: 例外なし


@pytest.mark.parametrize(
    "vmax",
    [
        (1.0,) * 5,
        (1.0,) * 7,
        (0.0, 1.0, 1.0, 1.0, 1.0, 1.0),
        (-1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
        (float("nan"), 1.0, 1.0, 1.0, 1.0, 1.0),
        (float("inf"), 1.0, 1.0, 1.0, 1.0, 1.0),
        (True, 1.0, 1.0, 1.0, 1.0, 1.0),
        ("1.0", 1.0, 1.0, 1.0, 1.0, 1.0),
        1.0,
    ],
)
def test_invalid_vmax_raises(vmax):
    """TC-SC-06 不正な vmax → ValueError (vmax 入り) 🟡"""
    with pytest.raises(ValueError, match="vmax"):
        _make(vmax=vmax)


@pytest.mark.parametrize("value", [0.0, -0.3, 1.01, float("nan"), float("inf"), True, "0.3"])
def test_invalid_speed_scale_raises(value):
    """TC-SC-07 不正な speed_scale → ValueError 🟡"""
    with pytest.raises(ValueError, match="speed_scale"):
        _make(speed_scale=value)


@pytest.mark.parametrize("value", [0.0, -0.02, float("nan"), float("inf"), True])
def test_invalid_control_dt_raises(value):
    """TC-SC-08 不正な control_dt → ValueError 🟡"""
    with pytest.raises(ValueError, match="control_dt"):
        _make(control_dt=value)


@pytest.mark.parametrize("value", [0.0, -0.2, float("nan"), float("inf"), False])
def test_invalid_state_timeout_raises(value):
    """TC-SC-09 不正な state_timeout → ValueError 🟡"""
    with pytest.raises(ValueError, match="state_timeout"):
        _make(state_timeout=value)


@pytest.mark.parametrize(
    ("lower", "upper"),
    [
        ((0.12, -0.13, 0.0), (0.12, 0.26, 0.17)),  # 軸 0 で等しい
        ((0.12, -0.13, 0.0), (0.38, -0.13, 0.17)),  # 軸 1
        ((0.12, -0.13, 0.0), (0.38, 0.26, 0.0)),  # 軸 2
        ((0.5, -0.13, 0.0), (0.38, 0.26, 0.17)),  # lower > upper
        ((float("nan"), -0.13, 0.0), (0.38, 0.26, 0.17)),
        ((0.12, -0.13), (0.38, 0.26, 0.17)),  # 長さ 2
        ((0.12, -0.13, 0.0, 0.0), (0.38, 0.26, 0.17)),  # 長さ 4
        ((True, -0.13, 0.0), (0.38, 0.26, 0.17)),  # bool
    ],
)
def test_invalid_workspace_box_raises(lower, upper):
    """TC-SC-10 不正な WorkspaceBox → ValueError 🟡"""
    with pytest.raises(ValueError, match="lower|upper|workspace"):
        WorkspaceBox(lower=lower, upper=upper)


def test_workspace_must_be_workspace_box():
    """TC-SC-11 workspace が dict → ValueError (workspace 入り) 🟡"""
    with pytest.raises(ValueError, match="workspace"):
        _make(workspace={"lower": WS_LOWER, "upper": WS_UPPER})


# =============================================================== TC-LC
def test_load_repository_default_file():
    """TC-LC-01 configs/motion.yaml の既定値 🟡 (Green で motion.yaml を作成する)"""
    cfg = load_safety_config()
    assert isinstance(cfg, SafetyConfig)
    assert isinstance(cfg.workspace, WorkspaceBox)
    assert cfg.vmax == (1.0,) * 6
    assert cfg.control_dt == pytest.approx(0.02)
    assert cfg.speed_scale == 1.0
    assert cfg.state_timeout == pytest.approx(0.2)
    assert cfg.workspace.lower == pytest.approx(WS_LOWER)
    assert cfg.workspace.upper == pytest.approx(WS_UPPER)


def test_default_path_is_repository_configs_motion_yaml():
    """TC-LC-02 既定パスは configs/motion.yaml 🔵"""
    assert DEFAULT_MOTION_CONFIG_PATH == REPO_ROOT / "configs" / "motion.yaml"
    assert DEFAULT_MOTION_CONFIG_PATH.is_file()
    assert load_safety_config() == load_safety_config(DEFAULT_MOTION_CONFIG_PATH)


def test_load_accepts_str_and_path(tmp_path):
    """TC-LC-03 str と Path の両方で読める 🟡"""
    path = _write(tmp_path, VALID)
    assert load_safety_config(path) == load_safety_config(str(path))


def test_load_ignores_other_top_level_keys(tmp_path):
    """TC-LC-04 safety: 以外の最上位キーは無視 🟡"""
    obj = copy.deepcopy(VALID)
    obj["approach_height"] = 0.08
    obj["pre_grasp"] = {"dz": 0.08}
    assert load_safety_config(_write(tmp_path, obj)) == load_safety_config(_write(tmp_path, VALID))


def test_load_speed_scale_override(tmp_path):
    """TC-LC-05 speed_scale を yaml で 0.3 にできる 🔵"""
    cfg = load_safety_config(_write(tmp_path, _with(["safety", "speed_scale"], 0.3)))
    assert cfg.speed_scale == 0.3


def test_load_normalizes_int_to_float(tmp_path):
    """TC-LC-06 整数値は float として保持 🟡"""
    obj = _with(["safety", "vmax"], [1, 1, 1, 1, 1, 1])
    obj["safety"]["state_timeout"] = 1
    cfg = load_safety_config(_write(tmp_path, obj))
    assert all(type(v) is float for v in cfg.vmax)
    assert type(cfg.state_timeout) is float


@pytest.mark.parametrize("key", ["vmax", "control_dt", "speed_scale", "state_timeout", "workspace"])
def test_missing_safety_key_raises(tmp_path, key):
    """TC-LC-07 safety: の欠落キー → ValueError (キー名入り) 🟡"""
    with pytest.raises(ValueError, match=key):
        load_safety_config(_write(tmp_path, _without(["safety", key])))


def test_unknown_safety_key_raises(tmp_path):
    """TC-LC-08 safety: の未知キー → ValueError 🟡"""
    with pytest.raises(ValueError, match="vmax_gripper"):
        load_safety_config(_write(tmp_path, _with(["safety", "vmax_gripper"], 2.0)))


@pytest.mark.parametrize(
    ("obj", "key"),
    [
        (_without(["safety", "workspace", "lower"]), "lower"),
        (_without(["safety", "workspace", "upper"]), "upper"),
        (_with(["safety", "workspace", "center"], [0.25, 0.065, 0.085]), "center"),
    ],
)
def test_workspace_keys_are_exact(tmp_path, obj, key):
    """TC-LC-09 workspace: の欠落・未知キー → ValueError 🟡"""
    with pytest.raises(ValueError, match=key):
        load_safety_config(_write(tmp_path, obj))


def test_missing_safety_section_raises(tmp_path):
    """TC-LC-10 safety: が無い → ValueError 🟡"""
    with pytest.raises(ValueError, match="safety"):
        load_safety_config(_write(tmp_path, {"other": 1}))


@pytest.mark.parametrize(
    "obj",
    [
        [1, 2],
        "x",
        "",
        {"safety": [1, 2]},
        _with(["safety", "workspace"], 3),
    ],
    ids=["top-list", "top-scalar", "empty", "safety-list", "workspace-scalar"],
)
def test_non_mapping_raises(tmp_path, obj):
    """TC-LC-11 mapping でない → ValueError 🟡"""
    with pytest.raises(ValueError):
        load_safety_config(_write(tmp_path, obj))


@pytest.mark.parametrize(
    "obj",
    [
        _with(["safety", "speed_scale"], 0),
        _with(["safety", "vmax"], [1.0] * 5),
        _with(["safety", "workspace", "lower"], [0.12, -0.13, 0.2]),
        _with(["safety", "control_dt"], True),
    ],
    ids=["scale0", "vmax5", "lower-z", "dt-bool"],
)
def test_invalid_values_via_yaml_raise(tmp_path, obj):
    """TC-LC-12 yaml 経由でも値の検証が働く 🟡"""
    with pytest.raises(ValueError):
        load_safety_config(_write(tmp_path, obj))


def test_missing_file_raises_file_not_found(tmp_path):
    """TC-LC-13 ファイルなし → FileNotFoundError (パス入り) 🔵"""
    with pytest.raises(FileNotFoundError, match="missing.yaml"):
        load_safety_config(tmp_path / "missing.yaml")


def test_loader_uses_safe_load(tmp_path):
    """TC-LC-14 safe_load を使い任意オブジェクトを作らない 🔵"""
    path = _write(tmp_path, '!!python/object/apply:os.system ["echo x"]')
    with pytest.raises(yaml.YAMLError):
        load_safety_config(path)
    source = inspect.getsource(config_module)
    assert "yaml.safe_load" in source
    assert "yaml.load(" not in source


# =============================================================== TC-WC
def test_default_workspace_contains_all_task_points(task_points):
    """TC-WC-01 既定の作業範囲に 172 点すべてが入る 🔵"""
    box = load_safety_config().workspace
    assert len(task_points) == 172
    assert all(is_in_workspace(p, box) for p in task_points)


def test_default_workspace_margins(task_points):
    """TC-WC-02 各面までの余裕 (x, y は 3 cm 以上、z 下側は 1.5 cm 以上) 🔵"""
    box = load_safety_config().workspace
    lo = task_points.min(axis=0) - list(box.lower)
    hi = list(box.upper) - task_points.max(axis=0)
    assert lo[0] >= 0.03 - 1e-9 and lo[1] >= 0.03 - 1e-9
    assert hi[0] >= 0.03 - 1e-9 and hi[1] >= 0.03 - 1e-9
    assert lo[2] >= 0.015 - 1e-9


def test_workspace_contains_scene_work_area_corners():
    """TC-WC-03 scene_spec の作業範囲の 4 隅 x z in {0.0, 0.17} 🟡"""
    box = load_safety_config().workspace
    for x in SCENE_SPEC.work_area_x:
        for y in SCENE_SPEC.work_area_y:
            for z in (0.0, 0.17):
                assert is_in_workspace((x, y, z), box)


def test_workspace_contains_box_corners():
    """TC-WC-04 箱の内寸の四隅 x z in {box_rim_z, 0.14} 🟡"""
    box = load_safety_config().workspace
    bx, by = SCENE_SPEC.box_center_xy
    h = SCENE_SPEC.box_inner_half_size
    for sx in (-1, 1):
        for sy in (-1, 1):
            for z in (SCENE_SPEC.box_rim_z, 0.14):
                assert is_in_workspace((bx + sx * h, by + sy * h, z), box)
