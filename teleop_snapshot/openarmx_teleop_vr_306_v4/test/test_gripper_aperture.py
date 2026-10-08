import json
from pathlib import Path
import pytest
from openarmx_teleop_vr_306_v4.gripper_aperture import checked_width, width_to_motor, save_width, load_width


@pytest.mark.parametrize('width',[0,0.5,1,2,3,4,5,6,7,8,9])
def test_nominal_range(width):
    assert 10 <= width_to_motor(width) <= 360


@pytest.mark.parametrize('width',[-1,9.1,float('nan'),float('inf'),True,None,'bad'])
def test_reject_invalid(width):
    with pytest.raises((TypeError,ValueError)):checked_width(width)


def test_endpoints_and_monotonicity():
    assert width_to_motor(0)==360
    assert width_to_motor(9)==pytest.approx(28.42105263)
    assert all(width_to_motor(i/10)>width_to_motor((i+1)/10) for i in range(90))


def test_saved_width_survives_reload(tmp_path):
    path=tmp_path/'settings.json'
    assert load_width(path)==0
    save_width(5.5,path)
    assert load_width(path)==5.5
    with pytest.raises(ValueError):save_width(10,path)
    assert load_width(path)==5.5


def test_production_config_matches_recorded_pose():
    import yaml
    root=Path(__file__).resolve().parents[1]
    params=yaml.safe_load((root/'config/controller.yaml').read_text())
    p=next(iter(params.values()))['ros__parameters']
    assert p['quick_reset_left_arm_joints'][0]==pytest.approx(40.000651)
    assert p['quick_reset_right_arm_joints'][3]==pytest.approx(-129.999370)
    assert p['quick_reset_neck_joints'][1]==pytest.approx(-29.998649)
    assert p['gripper_max_position']==360
    teleop=next(iter(yaml.safe_load((root/'config/teleop.yaml').read_text()).values()))['ros__parameters']
    assert teleop['gripper_filter_alpha']==1
    assert teleop['gripper_max_step_per_cycle']>=350
