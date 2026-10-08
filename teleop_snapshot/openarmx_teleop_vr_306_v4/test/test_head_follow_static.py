from _subject import source_path


def _source(name):
    return source_path(name).read_text(encoding='utf-8')


def test_head_target_runs_before_hand_tracking_early_return():
    source = _source('vr_mapper_node')
    control_tick = source.split('    def _control_tick(self):', 1)[1]
    control_tick = control_tick.split('    def ', 1)[0]
    assert control_tick.index('self._publish_head_follow_target(now)') < (
        control_tick.index("if tracking_state != 'active':")
    )


def test_runtime_head_enable_uses_fresh_feedback_instead_of_blanket_rejection():
    source = _source('controller_node')
    handler = source.split('    def _on_head_follow_enabled', 1)[1]
    handler = handler.split('    def ', 1)[0]
    assert 'enable head follow before enabling hardware' not in handler
    assert 'joint feedback is stale' in handler
    assert 'self._head_target = None' in handler


def test_head_tracking_keeps_calibration_across_short_input_outages():
    mapper = _source('vr_mapper_node')
    publish = mapper.split('    def _publish_head_follow_target', 1)[1]
    publish = publish.split('    def ', 1)[0]
    stale_branch = publish.split('now - self._head_sample_time > head_timeout', 1)[1]
    stale_branch = stale_branch.split('measured_neck =', 1)[0]
    assert 'self._head_mapper.release()' not in stale_branch
    assert 'recalibration_required' in mapper

    config = (
        source_path('vr_mapper_node').parents[1] / 'config' / 'teleop.yaml'
    ).read_text(encoding='utf-8')
    assert 'head_tracking_mode: "relative_neutral_quaternion"' in config


def test_web_ui_allows_head_toggle_while_stably_armed():
    web_source = (
        source_path('vr_mapper_node').parents[1] / 'web' / 'vr_app.js'
    ).read_text(encoding='utf-8')
    request = web_source.split(
        'async function requestHeadFollowEnabled(enabled) {'
    )[1].split('\n}', 1)[0]
    assert 'hardwareControlIsTransitioning()' in request
    assert 'hardwareControlIsOnOrBusy()' not in request
    assert 'lastHeadRecalibrationRequired' in web_source
    assert '检测到头显坐标突变' in web_source
    assert '原有正前方基准保持不变' in web_source


def test_reset_edges_create_new_clutch_session_and_yaw_basis():
    source = _source('vr_mapper_node')
    backend = source.split('    def _on_backend_status', 1)[1]
    backend = backend.split('    def ', 1)[0]
    assert backend.count('new_clutch_session=True') >= 2
    assert 'operator_yaw_compensation_enabled' in source
    assert 'webxr_yaw_deg_from_quaternion' in source
    assert 'operator_rotation @ sample.position' in source
    assert 'operator_rotation @ body_position' in source


def test_neck_quick_reset_has_an_independent_faster_limit_vector():
    source = _source('controller_node')
    assert 'def _quick_reset_limit_vector' in source
    assert "limits[18:21]" in source
    assert source.count('quick_reset_neck_max_velocity_deg_sec') >= 3
    assert source.count('quick_reset_neck_max_acceleration_deg_sec2') >= 3
    # The position limiter has no jerk input; jerk is used by velocity servo.
    assert source.count('quick_reset_neck_max_jerk_deg_sec3') >= 2
