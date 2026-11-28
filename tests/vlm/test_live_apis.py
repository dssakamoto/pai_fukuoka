"""実際の API で、sim の画像から検出できるか (TASK-0019〜0021)。API キーがなければスキップする。

キーとネットワークが要り、料金がかかる (1 プロバイダ 1 回の呼び出し。結果は configs/vlm.yaml のキャッシュに残る)。
描画も要る。CI では `-m "not render and not api"` で除かれる。実行: ANTHROPIC_API_KEY などを設定して
`uv run pytest tests/vlm/test_live_apis.py -m api`
"""

import os

import pytest

from so101_vlm.env import load_dotenv

load_dotenv()  # .env にキーがあれば使う (値は表示しない)

PROVIDERS = {"anthropic": "ANTHROPIC_API_KEY", "openai": "OPENAI_API_KEY", "gemini": "GEMINI_API_KEY", "ollama": None}  # ollama はキー不要
MAX_CENTRE_ERROR_PX = 40  # 検出枠の中心と、真のブロックの中心の差の許容 [px]。VLM の枠はゆるい (暫定。実測後に見直す)


@pytest.fixture(scope="module")
def scene_image():
    """seed 2 の配置を撮影した RGB と、真のブロックを囲む枠 (OracleVlm の pick)。"""
    from so101_vlm.eval.layout import sample_layout
    from so101_vlm.eval.oracle_vlm import OracleVlm
    from so101_vlm.eval.scene_spec import SCENE_SPEC
    from so101_vlm.eval.layout import place_blocks
    from so101_vlm.nodes.camera import render_rgbd
    from so101_vlm.perception.calibration import load_calibration
    from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH
    from so101_vlm.robot.sim import SimRobot

    layout = sample_layout(2)
    with SimRobot(PICKPLACE_SCENE_PATH) as robot:
        model, data = robot.mujoco_handles()
        place_blocks(model, data, layout)
        rgb, _ = render_rgbd(model, data, SCENE_SPEC.camera_name, SCENE_SPEC.image_width, SCENE_SPEC.image_height)
    truth = OracleVlm(layout, "red", load_calibration()).plan(rgb, "x").pick.bbox_px
    return rgb, truth


def _ollama_ready() -> bool:
    """ローカルの Ollama が応答し、設定のモデルが取得済みか (/api/tags)。"""
    import json
    import urllib.request

    from so101_vlm.vlm.config import load_vlm_config

    pc = load_vlm_config().providers["ollama"]
    try:
        url = f"{str(pc.options.get('base_url', 'http://localhost:11434')).rstrip('/')}/api/tags"
        with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(url, timeout=2) as r:
            return pc.model in {m.get("name") for m in json.loads(r.read()).get("models", [])}
    except Exception:  # noqa: BLE001  起動していない・通信できない = 使えない
        return False


def _needs_key(name: str):
    """キーがなければ、描画の fixture を作る前に (収集時に) スキップする。"""
    if name == "ollama":  # キーの代わりに、Ollama が起動していてモデルが取得済みかを見る
        return pytest.param(name, marks=pytest.mark.skipif(not _ollama_ready(), reason="Ollama が起動していない、またはモデル未取得 (スキップ。レポートでは『未評価』)"))
    missing = not os.environ.get(PROVIDERS[name], "").strip()
    return pytest.param(name, marks=pytest.mark.skipif(missing, reason=f"{PROVIDERS[name]} が未設定 (実 API のテストはスキップ。レポートでは『未評価』)"))


@pytest.mark.api
@pytest.mark.render
@pytest.mark.parametrize("name", [_needs_key(n) for n in PROVIDERS])
def test_detects_the_red_block_in_the_sim_image(name, scene_image, tmp_path):
    import dataclasses

    from so101_vlm.vlm import create_client
    from so101_vlm.vlm.config import load_vlm_config

    rgb, truth = scene_image
    client = create_client(load_vlm_config(), name, cache_dir=tmp_path / "cache")
    plan = client.plan(rgb, "put the red block into the box")
    x1, y1, x2, y2 = plan.pick.bbox_px
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    tx, ty = (truth[0] + truth[2]) / 2, (truth[1] + truth[3]) / 2
    assert 0 <= x1 < x2 <= 639 and 0 <= y1 < y2 <= 479
    assert abs(cx - tx) <= MAX_CENTRE_ERROR_PX and abs(cy - ty) <= MAX_CENTRE_ERROR_PX, (
        f"{name}: 検出枠の中心 ({cx:.0f}, {cy:.0f}) が、真の位置 ({tx:.0f}, {ty:.0f}) から {MAX_CENTRE_ERROR_PX} px 以上離れている。"
        "座標の空間 (リサイズ・正規化) の取り違えを疑う。scripts/visualize_detection.py で目視確認する"
    )
    assert dataclasses.is_dataclass(plan)


@pytest.mark.api
@pytest.mark.render
@pytest.mark.skipif(not os.environ.get("OPENAI_API_KEY", "").strip(), reason="OPENAI_API_KEY が未設定 (実 API のテストはスキップ)")
def test_openai_follows_a_moved_box(tmp_path):
    """箱を動かした 3 試行で、place の枠が新しい箱を囲み、成功判定が新しい箱に追従する (SGT-014、任意。有料: 約 $0.0005) 🟡"""
    from so101_vlm.eval.box_option import parse_box_xy
    from so101_vlm.eval.eval_config import load_eval_config
    from so101_vlm.eval.geometry import get_geometry
    from so101_vlm.eval.scenario import scenario_for
    from so101_vlm.eval.trial import run_trial
    from so101_vlm.perception.calibration import load_calibration
    from so101_vlm.perception.projection import project_base_to_pixel
    from so101_vlm.vlm import create_client
    from so101_vlm.vlm.config import load_vlm_config

    g, cal = get_geometry(), load_calibration()
    box = parse_box_xy([0.20, 0.15])  # 箱を置ける窓の中 (docs/notes/box-sweep.md)
    client = create_client(load_vlm_config(), "openai", cache_dir=tmp_path / "cache")
    for seed in (2, 3, 4):
        scenario = scenario_for(seed, load_eval_config().placement, box)
        outcome = run_trial(seed, "red", "put the red block into the box", client, scenario=scenario)
        if outcome.plan is None:  # VLM が応答できなかった試行は、箱への追従を確かめられない
            continue
        u, v, _ = project_base_to_pixel(scenario.place_region(g).goal_xyz, cal.T_base_cam, cal.camera)
        x1, y1, x2, y2 = outcome.plan.place.bbox_px
        assert x1 <= u <= x2 and y1 <= v <= y2, f"seed {seed}: place の枠 {outcome.plan.place.bbox_px} が新しい箱 ({u:.0f}, {v:.0f}) を囲んでいない"
        if outcome.success:  # 成功なら、判定は新しい箱の中心からのずれで見ている
            ox, oy = outcome.run.judge.offset_xy
            assert abs(ox) < g.box_inner_half_size and abs(oy) < g.box_inner_half_size
