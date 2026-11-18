import numpy as np
import pytest
from matplotlib.backends.backend_agg import FigureCanvasAgg

from suspension.model import Vehicle
from suspension.roads import Scenario
from suspension.simulation import simulate,save_run
from suspension.viewer import Playback,ReplayRun,ReplayFigure,load_replays


@pytest.fixture
def saved_replays(tmp_path):
    scenario=Scenario("one_side",seed=5,duration=1.4)
    for controller in ("passive","mpc"):
        save_run(tmp_path/controller,*simulate(scenario,controller,"ideal"))
    return tmp_path


def test_playback_pause_seek_speed_loop_and_end():
    clock=Playback(7,speed=.5)
    assert clock.advance(2)==1
    clock.playing=False
    assert clock.advance(10)==1
    clock.seek(99);assert clock.position==7
    clock.playing=True
    assert clock.advance(2)==1
    clock.loop=False;clock.seek(6.9)
    assert clock.advance(1)==7 and not clock.playing
    clock.seek(-1);assert clock.position==0


def test_geometry_preserves_corner_order_and_physical_logs(saved_replays):
    run=ReplayRun(saved_replays/"passive")
    k=run.index(.91)
    assert run.t[k]<=.91<run.t[k+1]
    # A front/left slope must lift the front/left corners without swapping labels.
    run.a["state"][k,:3]=[.01,.02,.03]
    original=run.a["state"][k].copy()
    body,wheel,road=run.geometry(k,4)
    assert body[0]>body[1] and body[0]>body[2]
    assert body[0]-body[1]==pytest.approx(4*run.p.front_track*.03)
    np.testing.assert_array_equal(run.a["state"][k],original)
    np.testing.assert_allclose(road+.14,4*run.a["disturbance"][k,:4])
    assert run.index(-4)==0 and run.index(99)==len(run.t)-1


def test_controller_selection_render_and_unscaled_force_signals(saved_replays,tmp_path):
    runs=load_replays(saved_replays)
    assert set(runs)=={"passive","mpc"}
    scene=ReplayFigure(runs,"mpc",scale=4)
    FigureCanvasAgg(scene.fig)
    scene.draw(.91)
    np.testing.assert_array_equal(scene.plots[2].lines[0].get_ydata(),runs["mpc"].a["state"][:,14])
    scene.set_scale(8);scene.set_view("Front / roll");scene.draw(1.1)
    np.testing.assert_array_equal(scene.plots[2].lines[0].get_ydata(),runs["mpc"].a["state"][:,14])
    scene.select("passive");scene.draw(1.1)
    assert scene.view=="Front / roll" and scene.scale==8
    np.testing.assert_array_equal(scene.plots[2].lines[0].get_ydata(),0)
    scene.fig.savefig(tmp_path/"replay.png")
    assert (tmp_path/"replay.png").stat().st_size>10000


def test_mixed_experiments_are_not_presented_as_controller_comparison(saved_replays):
    import json
    path=saved_replays/"mpc"/"metadata.json"
    meta=json.loads(path.read_text());meta["scenario"]["seed"]+=1
    path.write_text(json.dumps(meta))
    with pytest.raises(ValueError,match="same scenario"):
        load_replays(saved_replays)


def test_missing_runs_give_actionable_error(tmp_path):
    with pytest.raises(ValueError,match="python -m suspension compare"):
        load_replays(tmp_path)
