(() => {
  'use strict';

  const byId = id => document.getElementById(id);
  const elements = {
    vrBadge: byId('vrBadge'),
    transportBadge: byId('transportBadge'),
    hud: byId('stateHud'),
    mode: byId('modeValue'),
    xr: byId('xrValue'),
    hands: byId('handsValue'),
    age: byId('ageValue'),
    fps: byId('fpsValue'),
    pending: byId('pendingValue'),
    offline: byId('offlineNotice'),
    clock: byId('clockValue'),
    feed: byId('cameraFeed'),
    fullscreen: byId('fullscreenButton')
  };

  const modeClass = mode => {
    if (mode === 'ESTOP') return 'error';
    if (['FAILURE_HOLD', 'EXPERT_RELEASE_REQUIRED'].includes(mode)) return 'warning';
    if (['EXPERT_READY', 'EXPERT_ACTIVE'].includes(mode)) return 'expert';
    if (['POLICY_STOPPED', 'POLICY_WARMUP', 'POLICY_ACTIVE'].includes(mode)) return 'policy';
    return '';
  };

  const formatAge = value => {
    const age = Number(value);
    return Number.isFinite(age) ? `${Math.round(age * 1000)} ms` : '--';
  };

  async function refreshStatus() {
    try {
      const [statusResponse, cameraResponse] = await Promise.all([
        fetch('/api/status', { cache: 'no-store' }),
        fetch('/api/camera/status', { cache: 'no-store' })
      ]);
      if (!statusResponse.ok) throw new Error(`status HTTP ${statusResponse.status}`);
      const status = await statusResponse.json();
      const camera = cameraResponse.ok ? await cameraResponse.json() : {};
      const teleop = status.teleop || {};
      const hg = teleop.hg_dagger || {};
      const mode = String(hg.mode || 'DISARMED');
      const connected = status.vrConnected === true;
      const hands = Array.isArray(teleop.tracked_hands) ? teleop.tracked_hands : [];

      elements.vrBadge.textContent = connected ? 'VR 已连接' : 'VR 未连接';
      elements.vrBadge.className = `badge ${connected ? 'ok' : 'warn'}`;
      elements.transportBadge.textContent = `传输 ${status.realtimeTransport || '--'}`;
      elements.mode.textContent = mode;
      elements.xr.textContent = status.xrPoseFresh === true ? '实时' : '不可用';
      elements.hands.textContent = hands.length ? hands.join(' + ') : '未追踪';
      elements.age.textContent = formatAge(status.realtimePacketAge);
      elements.pending.textContent = String(hg.collector_finalize_pending_count || 0);
      elements.fps.textContent = `${Number(
        camera.actual_fps ?? camera.fps ?? camera.server_fps ?? 0
      ).toFixed(1)} FPS`;
      elements.hud.textContent = hg.prompt || `当前：${mode}`;
      elements.hud.className = `hud ${modeClass(mode)}`.trim();
      elements.offline.classList.remove('visible');
    } catch (error) {
      elements.offline.textContent = `VR Web 状态断开：${error.message}`;
      elements.offline.classList.add('visible');
    }
  }

  function reconnectFeed() {
    window.setTimeout(() => {
      elements.feed.src = `/api/camera/stream.mjpg?retry=${Date.now()}`;
    }, 1000);
  }

  elements.feed.addEventListener('error', reconnectFeed);
  elements.fullscreen.addEventListener('click', async () => {
    try {
      if (document.fullscreenElement) await document.exitFullscreen();
      else await document.documentElement.requestFullscreen();
    } catch (_) { /* Fullscreen is optional. */ }
  });
  document.addEventListener('keydown', event => {
    if (event.key.toLowerCase() === 'f') elements.fullscreen.click();
  });

  window.setInterval(refreshStatus, 250);
  window.setInterval(() => {
    elements.clock.textContent = new Date().toLocaleString('zh-CN', { hour12: false });
  }, 1000);
  void refreshStatus();
})();
