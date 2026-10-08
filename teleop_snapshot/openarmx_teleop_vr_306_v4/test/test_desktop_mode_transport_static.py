from _subject import source_directory, source_path


def test_desktop_keepalive_never_replaces_controller_pose():
    source = source_path('vr_web_bridge').read_text(encoding='utf-8')
    keepalive = source.index("packet.get('type') == 'desktop_keepalive'")
    replace = source.index('self._replace_latest_packet(raw_packet)')
    assert keepalive < replace


def test_same_headset_reconnect_replaces_only_its_old_owner():
    source = source_path('vr_web_bridge').read_text(encoding='utf-8')
    assert 'self._control_peer != peer' in source
    assert 'self._control_client_id != client_id' in source
    assert 'replaced_websocket = self._control_websocket' in source
    assert 'if not self._control_owner_is_current(peer, owner_session):' in source
    assert 'if current_owner:' in source


def test_vr_page_contains_head_locked_notice_and_wake_lock():
    source = (
        source_directory().parent / 'web' / 'vr_app.js'
    ).read_text(encoding='utf-8')
    assert "notice.setAttribute('position', '0 -0.25 -0.8')" in source
    assert "navigator.wakeLock.request('screen')" in source
    assert "type: 'desktop_keepalive'" in source
    assert 'showVrNotice(' in source


def test_desktop_yaw_is_captured_before_session_state_is_cleared():
    source = source_path('vr_mapper_node').read_text(encoding='utf-8')
    method = source[source.index('    def _on_desktop_mode('):]
    method = method[:method.index('\n    def ', 10)]
    capture = method.index('entry_yaw_deg = float(self._head_sample.rotation[1])')
    boundary = method.index('self._begin_session_boundary_locked(')
    assert capture < boundary
    assert 'self._desktop_position_rotation @ sample.position' in source
