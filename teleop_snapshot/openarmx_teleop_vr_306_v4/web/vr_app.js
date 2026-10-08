function getClientInstanceId() {
  const storageKey = 'autolife-vr-client-instance';
  try {
    let value = window.sessionStorage.getItem(storageKey);
    if (!value) {
      value = window.crypto?.randomUUID?.()
        || `${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
      window.sessionStorage.setItem(storageKey, value);
    }
    return value;
  } catch (_) {
    return `${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
  }
}

const state = {
  config: null,
  clientInstanceId: getClientInstanceId(),
  websocket: null,
  reconnectTimer: null,
  rtcPeer: null,
  realtimeChannel: null,
  realtimeReconnectTimer: null,
  realtimeGenerationCounter: 0,
  realtimeGeneration: 0,
  realtimeSequence: 0,
  realtimeLastAckSequence: -1,
  realtimeLastSendAt: 0,
  realtimeLastAckAt: 0,
  realtimeFallbackUntil: 0,
  realtimeFallbackLastSendAt: 0,
  realtimeLatestEncodedPacket: null,
  realtimeWatchdogTimer: null,
  xrSession: null,
  brandHud: null,
  headText: null,
  vrNotice: null,
  vrNoticeTimer: null,
  hgStatusHud: null,
  hgStatusHudTimer: null,
  desktopWakeLock: null,
  desktopWakeLockPending: false,
  desktopWakeLockRetryAt: 0,
  desktopKeepaliveTimer: null,
  teleop: {
    available: false,
    status: null,
    requestPending: false,
    waistFollowAvailable: false,
    waistFollowRequestPending: false,
    headFollowAvailable: false,
    headFollowRequestPending: false,
    xrPoseFresh: false,
    lastHeadFollowEnabled: null,
    lastHeadPoseFresh: null,
    lastHeadRecalibrationRequired: null,
    lastQuickResetActive: null,
    lastHgDaggerMode: null,
    lastHgDaggerHapticToken: null,
    lastHgDaggerVisualKey: null,
    desktopModeAvailable: false,
    desktopModeRequestPending: false,
    desktopModeLastReady: null,
    holdTimer: null,
    holdTriggered: false,
    pollTimer: null,
    pollInFlight: false,
    controlsInitialized: false,
    lastRequestMessage: '',
    enableAfterFreshVr: false,
    enableAfterFreshVrDeadline: 0
  },
  image: {
    enabled: false,
    cameras: [],
    states: new Map(),
    panels: new Map(),
    opacity: 1.0,
    renderStarted: false,
    mode: 'passthrough',
    toggleButton: 'B',
    lastToggleAt: -Infinity,
    modeIndicator: null,
    unifiedStream: null,
    unifiedPeer: null,
    unifiedGeneration: 0,
    clientStatusTimer: null,
    cameraWatchdogTimer: null
  }
};

function setStatus(message) {
  const statusText = document.getElementById('statusText');
  if (statusText) statusText.textContent = message;
}

function ensureVrNotice() {
  if (state.vrNotice) return state.vrNotice;
  const cameraEl = document.querySelector('a-scene')?.camera?.el;
  if (!cameraEl) return null;
  const notice = document.createElement('a-text');
  notice.setAttribute('id', 'desktop-mode-vr-notice');
  notice.setAttribute('align', 'center');
  notice.setAttribute('anchor', 'center');
  notice.setAttribute('baseline', 'center');
  notice.setAttribute('color', '#7DFFCF');
  notice.setAttribute('width', '1.35');
  notice.setAttribute('wrap-count', '28');
  notice.setAttribute('position', '0 -0.25 -0.8');
  notice.setAttribute('visible', false);
  cameraEl.appendChild(notice);
  state.vrNotice = notice;
  return notice;
}

function showVrNotice(message, durationMs = 3500, color = '#7DFFCF') {
  setStatus(message);
  const notice = ensureVrNotice();
  if (!notice) return;
  if (state.vrNoticeTimer) window.clearTimeout(state.vrNoticeTimer);
  notice.setAttribute('color', color);
  notice.setAttribute('value', String(message));
  notice.setAttribute('visible', true);
  state.vrNoticeTimer = window.setTimeout(() => {
    if (state.vrNotice === notice) notice.setAttribute('visible', false);
    state.vrNoticeTimer = null;
  }, Math.max(1000, Number(durationMs) || 3500));
}

const HG_DAGGER_MODE_LABELS = {
  DISARMED: '未使能',
  POLICY_STOPPED: 'VLA 已停止 · 长按 X 1.2 秒后松开开始',
  POLICY_ACTIVE: 'VLA 控制',
  FAILURE_HOLD: '请求接管 · 机器人保持',
  EXPERT_RELEASE_REQUIRED: '请完全松开双 Grip',
  EXPERT_READY: '请再次按下 Grip',
  EXPERT_ACTIVE: 'VR 接管中',
  POLICY_WARMUP: 'VLA 预热中',
  ESTOP: '安全停止'
};

function hgDaggerVisualState(hgDagger) {
  const mode = String(hgDagger?.mode || 'DISARMED');
  const notice = hgDagger?.notice;
  if (notice?.text) {
    const colors = {
      success: '#7DFFCF',
      error: '#FF6B6B',
      warning: '#FFD166',
      info: '#79C7FF'
    };
    return {
      key: `NOTICE:${notice.token ?? notice.text}`,
      label: String(notice.text),
      color: colors[String(notice.level || 'info')] || colors.info,
      notice: true
    };
  }
  const resetActive = hgDagger?.quick_reset?.pending === true
    || hgDagger?.quick_reset?.active === true;
  const paused = mode === 'EXPERT_ACTIVE' && hgDagger?.expert_paused === true;
  return {
    key: resetActive ? 'HG_RESET_ACTIVE' : (paused ? `${mode}:paused` : mode),
    label: resetActive ? '机器人复位中 · 请松开 X+A'
      : (paused ? 'VR 已暂停 · 按 Grip 继续' : (HG_DAGGER_MODE_LABELS[mode] || mode)),
    color: mode === 'ESTOP' ? '#FF6B6B'
      : resetActive ? '#FFD166'
      : ['FAILURE_HOLD', 'EXPERT_RELEASE_REQUIRED'].includes(mode) ? '#FFB86B'
        : ['EXPERT_READY', 'EXPERT_ACTIVE'].includes(mode) ? '#7DFFCF'
          : mode === 'POLICY_ACTIVE' || mode === 'POLICY_WARMUP' ? '#79C7FF'
            : '#D7E0EA',
    notice: false
  };
}

function drawHgStatusHud(message, color) {
  const hud = state.hgStatusHud;
  if (!hud) return;
  const { canvas, context, texture } = hud;
  const width = canvas.width;
  const height = canvas.height;
  context.clearRect(0, 0, width, height);
  context.fillStyle = 'rgba(5, 10, 16, 0.76)';
  context.beginPath();
  if (typeof context.roundRect === 'function') {
    context.roundRect(4, 4, width - 8, height - 8, 28);
  } else {
    context.rect(4, 4, width - 8, height - 8);
  }
  context.fill();
  context.fillStyle = color;
  context.fillRect(4, 28, 10, height - 56);
  context.textAlign = 'center';
  context.textBaseline = 'middle';
  context.font = '600 38px sans-serif';
  context.fillText(String(message), width / 2 + 4, height / 2 + 1, width - 72);
  texture.needsUpdate = true;
}

function ensureHgStatusHud() {
  if (state.hgStatusHud) return state.hgStatusHud;
  const cameraEl = document.querySelector('a-scene')?.camera?.el;
  if (!cameraEl?.object3D || typeof THREE === 'undefined') return null;

  const canvas = document.createElement('canvas');
  canvas.width = 1024;
  canvas.height = 128;
  const context = canvas.getContext('2d');
  if (!context) return null;
  const texture = new THREE.CanvasTexture(canvas);
  if ('colorSpace' in texture && THREE.SRGBColorSpace) {
    texture.colorSpace = THREE.SRGBColorSpace;
  } else if (THREE.sRGBEncoding) {
    texture.encoding = THREE.sRGBEncoding;
  }
  const material = new THREE.MeshBasicMaterial({
    map: texture,
    transparent: true,
    depthTest: false,
    depthWrite: false
  });
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(0.76, 0.095), material);
  mesh.name = 'hg-dagger-status-hud';
  mesh.position.set(0, 0.34, -0.82);
  mesh.renderOrder = 10000;
  mesh.visible = Boolean(state.xrSession);
  cameraEl.object3D.add(mesh);
  state.hgStatusHud = { canvas, context, texture, material, mesh };
  drawHgStatusHud('当前：未使能', '#D7E0EA');
  return state.hgStatusHud;
}

function updateHgStatusHud(hgDagger, { forceCurrent = false } = {}) {
  const hud = ensureHgStatusHud();
  if (!hud || !hgDagger) return;
  hud.mesh.visible = Boolean(state.xrSession);
  const next = hgDaggerVisualState(hgDagger);
  const previousKey = state.teleop.lastHgDaggerVisualKey;
  const changed = previousKey !== null && previousKey !== next.key;
  if ((changed || forceCurrent) && state.hgStatusHudTimer) {
    window.clearTimeout(state.hgStatusHudTimer);
    state.hgStatusHudTimer = null;
  }
  if (next.notice || String(previousKey || '').startsWith('NOTICE:')) {
    drawHgStatusHud(next.notice ? next.label : `当前：${next.label}`, next.color);
  } else if (changed && !forceCurrent) {
    const previous = hgDaggerVisualState({
      mode: previousKey.split(':')[0],
      expert_paused: previousKey.endsWith(':paused')
    });
    drawHgStatusHud(`${previous.label}  →  ${next.label}`, next.color);
    state.hgStatusHudTimer = window.setTimeout(() => {
      drawHgStatusHud(`当前：${next.label}`, next.color);
      state.hgStatusHudTimer = null;
    }, 2200);
  } else if (forceCurrent || previousKey !== next.key) {
    drawHgStatusHud(`当前：${next.label}`, next.color);
  }
  state.teleop.lastHgDaggerVisualKey = next.key;
}

function pulseVrControllers(intensity = 0.7, durationMs = 180) {
  for (const source of Array.from(state.xrSession?.inputSources || [])) {
    const gamepad = source?.gamepad;
    const actuator = gamepad?.hapticActuators?.[0]
      || gamepad?.vibrationActuator;
    try {
      if (typeof actuator?.pulse === 'function') {
        void actuator.pulse(intensity, durationMs);
      } else if (typeof actuator?.playEffect === 'function') {
        void actuator.playEffect('dual-rumble', {
          duration: durationMs,
          strongMagnitude: intensity,
          weakMagnitude: intensity * 0.6
        });
      }
    } catch (_) { /* Haptics are optional on WebXR runtimes. */ }
  }
}

function setServerUrl() {
  const serverUrl = document.getElementById('serverUrl');
  if (serverUrl) serverUrl.textContent = window.location.origin;
}

function hardwareControlIsOnOrBusy() {
  const teleop = state.teleop.status;
  const backendState = String(teleop?.state || '').toLowerCase();
  return Boolean(
    teleop?.hardware_enabled
    || teleop?.hardware_enable_pending
    || teleop?.quick_reset?.active
    || ['enabling', 'enabling_arms', 'resetting', 'resetting_arms', 'active', 'armed'].includes(backendState)
  );
}

function hardwareControlIsTransitioning() {
  const teleop = state.teleop.status;
  const backendState = String(teleop?.state || '').toLowerCase();
  return Boolean(
    teleop?.hardware_enable_pending
    || teleop?.quick_reset?.active
    || ['enabling', 'enabling_arms', 'resetting', 'resetting_arms'].includes(backendState)
  );
}

function vrInputIsFresh(teleop = state.teleop.status) {
  const age = Number(teleop?.vr_age);
  const trackedHands = teleop?.tracked_hands;
  const configuredTimeout = Number(state.config?.vr?.input_fresh_timeout_seconds);
  const maximumAge = Number.isFinite(configuredTimeout) && configuredTimeout > 0
    ? configuredTimeout
    : 0.8;
  return Boolean(
    teleop?.vr_input_fresh === true
    && Array.isArray(trackedHands)
    && trackedHands.length > 0
    && Number.isFinite(age)
    && age >= 0
    && age <= maximumAge
  );
}

function cancelDeferredHardwareEnable(message = '') {
  state.teleop.enableAfterFreshVr = false;
  state.teleop.enableAfterFreshVrDeadline = 0;
  if (message) state.teleop.lastRequestMessage = message;
}

function setHardwareButtonText(button, text) {
  if (button) button.innerHTML = `<span>${text}</span>`;
}

function waistFollowEnabledFromStatus() {
  const teleop = state.teleop.status;
  const backend = teleop?.backend || teleop;
  return backend?.waist_follow_enabled === true;
}

function headFollowEnabledFromStatus() {
  const teleop = state.teleop.status;
  const backend = teleop?.backend || teleop;
  return backend?.head_follow_enabled === true;
}

function desktopModeEnabledFromStatus() {
  return state.teleop.status?.desktop_mode?.enabled === true;
}

async function acquireDesktopWakeLock() {
  if (
    !desktopModeEnabledFromStatus()
    || state.desktopWakeLock
    || state.desktopWakeLockPending
  ) return Boolean(state.desktopWakeLock);
  if (Date.now() < state.desktopWakeLockRetryAt) return false;
  if (!navigator.wakeLock?.request) {
    state.desktopWakeLockRetryAt = Number.POSITIVE_INFINITY;
    showVrNotice(
      '浏览器不支持防休眠；请保持头显唤醒并露出追踪摄像头',
      5000,
      '#FFD166'
    );
    return false;
  }
  state.desktopWakeLockPending = true;
  try {
    const lock = await navigator.wakeLock.request('screen');
    state.desktopWakeLock = lock;
    lock.addEventListener('release', () => {
      if (state.desktopWakeLock === lock) state.desktopWakeLock = null;
      if (
        desktopModeEnabledFromStatus()
        && document.visibilityState === 'visible'
      ) {
        window.setTimeout(() => void acquireDesktopWakeLock(), 500);
      }
    }, { once: true });
    return true;
  } catch (error) {
    state.desktopWakeLockRetryAt = Date.now() + 10000;
    showVrNotice(`防休眠请求失败：${error.message}`, 5000, '#FFD166');
    return false;
  } finally {
    state.desktopWakeLockPending = false;
  }
}

async function releaseDesktopWakeLock() {
  const lock = state.desktopWakeLock;
  state.desktopWakeLock = null;
  state.desktopWakeLockRetryAt = 0;
  if (lock) {
    try { await lock.release(); } catch (_) { /* already released */ }
  }
}

function startDesktopKeepalive() {
  if (state.desktopKeepaliveTimer) return;
  state.desktopKeepaliveTimer = window.setInterval(() => {
    if (!desktopModeEnabledFromStatus()) return;
    sendReliablePacket({
      type: 'desktop_keepalive',
      tracking: Boolean(state.xrSession),
      timestamp: Date.now()
    });
  }, 2000);
}

function renderHardwareControl() {
  const card = document.getElementById('hardwareControl');
  const label = document.getElementById('hardwareStatusLabel');
  const detail = document.getElementById('hardwareStatusDetail');
  const button = document.getElementById('hardwareToggleButton');
  const waistToggle = document.getElementById('waistFollowToggle');
  const headToggle = document.getElementById('headFollowToggle');
  if (!card || !label || !detail || !button) return;

  card.classList.remove('is-off', 'is-busy', 'is-on', 'is-error');
  button.classList.remove('is-disable-action');
  const teleop = state.teleop.status;
  const backendState = String(teleop?.state || '').toLowerCase();
  const websocketReady = state.websocket?.readyState === WebSocket.OPEN;
  const controlling = hardwareControlIsOnOrBusy();
  const backendReason = teleop?.backend?.reason || teleop?.backend?.detail || '';

  if (!teleop) {
    card.classList.add('is-error');
    label.textContent = '遥操节点状态未连接';
    detail.textContent = state.teleop.lastRequestMessage || '请确认遥操功能包已经启动。';
    setHardwareButtonText(button, '等待遥操节点');
  } else if (teleop.dry_run) {
    card.classList.add('is-off');
    label.textContent = '当前为模拟模式';
    detail.textContent = 'dry_run:=true，不会向真机发送指令。';
    setHardwareButtonText(button, '模拟模式不可使能');
  } else if (state.teleop.enableAfterFreshVr) {
    card.classList.add('is-busy');
    label.textContent = '等待 VR 手柄实时姿态';
    detail.textContent = '使能授权已确认。正在进入 VR；唤醒手柄后才会安全使能真机。';
    setHardwareButtonText(button, '取消等待使能');
  } else if (
    teleop.hardware_enable_pending
    || teleop.quick_reset?.active
    || ['enabling', 'enabling_arms', 'resetting', 'resetting_arms'].includes(backendState)
  ) {
    card.classList.add('is-busy');
    const enabling = teleop.hardware_enable_pending || backendState.startsWith('enabling');
    label.textContent = enabling ? '正在进入 SYNC 并到达默认姿态' : '正在快速复位';
    detail.textContent = backendReason || teleop.detail || '正在处理，请保持机器人周围安全。';
    button.classList.add('is-disable-action');
    setHardwareButtonText(button, '立即关闭真机遥操');
  } else if (teleop.hardware_enabled) {
    card.classList.add('is-on');
    label.textContent = ['active', 'armed'].includes(backendState) ? '真机遥操中' : '真机遥操已使能';
    detail.textContent = teleop.detail || '先松开双手握把，再按住需要控制的一侧。';
    button.classList.add('is-disable-action');
    setHardwareButtonText(button, '关闭真机遥操');
  } else if (['fault', 'e_stop'].includes(backendState)) {
    card.classList.add('is-error');
    label.textContent = backendState === 'e_stop' ? '急停保护已锁定' : '真机使能失败';
    detail.textContent = state.teleop.lastRequestMessage || backendReason || teleop.detail;
    setHardwareButtonText(button, '排除故障后按住 1 秒重试');
  } else if (!vrInputIsFresh(teleop)) {
    card.classList.add('is-off');
    label.textContent = '等待 VR 手柄';
    detail.textContent = state.teleop.lastRequestMessage
      || '当前只有网页连接，没有实时手柄姿态。长按后会进入 VR，检测到手柄才使能。';
    setHardwareButtonText(button, '按住 1 秒，到达默认姿态后使能');
  } else {
    card.classList.add('is-off');
    label.textContent = '真机遥操已关闭';
    detail.textContent = state.teleop.lastRequestMessage || backendReason || teleop.detail || '机械臂不会跟随 VR 手柄。';
    setHardwareButtonText(button, '按住 1 秒，到达默认姿态后使能');
  }

  button.disabled = Boolean(
    state.teleop.requestPending
    || state.teleop.waistFollowRequestPending
    || state.teleop.headFollowRequestPending
    || !state.teleop.available
    || !websocketReady
    || teleop?.dry_run
  );
  if (waistToggle) {
    if (!state.teleop.waistFollowRequestPending) {
      waistToggle.checked = waistFollowEnabledFromStatus();
    }
    waistToggle.disabled = Boolean(
      state.teleop.requestPending
      || state.teleop.waistFollowRequestPending
      || controlling
      || !state.teleop.waistFollowAvailable
      || teleop?.dry_run
    );
  }
  if (headToggle) {
    if (!state.teleop.headFollowRequestPending) {
      headToggle.checked = headFollowEnabledFromStatus();
    }
    headToggle.disabled = Boolean(
      state.teleop.requestPending
      || state.teleop.headFollowRequestPending
      || hardwareControlIsTransitioning()
      || !state.teleop.headFollowAvailable
      || teleop?.dry_run
    );
  }
  if (state.teleop.requestPending) {
    setHardwareButtonText(button, controlling ? '正在关闭……' : '正在请求使能……');
  } else if (!websocketReady && !controlling) {
    setHardwareButtonText(button, '等待 VR 数据连接');
  } else if (!state.teleop.available && !controlling) {
    setHardwareButtonText(button, '真机使能服务未连接');
  }
}

async function refreshHardwareStatus() {
  if (state.teleop.pollInFlight) return;
  state.teleop.pollInFlight = true;
  const controller = typeof AbortController === 'function'
    ? new AbortController()
    : null;
  const timeout = window.setTimeout(() => controller?.abort(), 1500);
  try {
    const response = await fetch('/api/status', {
      cache: 'no-store',
      signal: controller?.signal
    });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const payload = await response.json();
    state.teleop.available = Boolean(payload.hardwareControlAvailable);
    state.teleop.waistFollowAvailable = Boolean(
      payload.waistFollowControlAvailable
    );
    state.teleop.headFollowAvailable = Boolean(
      payload.headFollowControlAvailable
    );
    state.teleop.xrPoseFresh = payload.xrPoseFresh === true;
    state.teleop.desktopModeAvailable = Boolean(
      payload.desktopModeControlAvailable
    );
    state.teleop.status = payload.teleop || null;
    const hgDagger = state.teleop.status?.hg_dagger;
    const hgMode = String(hgDagger?.mode || '');
    const hapticToken = hgDagger?.haptic_token;
    updateHgStatusHud(hgDagger);
    if (
      hgMode
      && (
        state.teleop.lastHgDaggerMode !== hgMode
        || state.teleop.lastHgDaggerHapticToken !== hapticToken
      )
    ) {
      const warningModes = new Set([
        'FAILURE_HOLD', 'EXPERT_RELEASE_REQUIRED', 'ESTOP'
      ]);
      showVrNotice(
        hgDagger.prompt || `HG-DAgger: ${hgMode}`,
        hgMode === 'EXPERT_RELEASE_REQUIRED' ? 7000 : 4500,
        warningModes.has(hgMode) ? '#FFB86B' : '#7DFFCF'
      );
      if (
        state.xrSession
        && ['EXPERT_RELEASE_REQUIRED', 'EXPERT_READY'].includes(hgMode)
      ) {
        pulseVrControllers(
          hgMode === 'EXPERT_RELEASE_REQUIRED' ? 0.9 : 0.55,
          hgMode === 'EXPERT_RELEASE_REQUIRED' ? 260 : 140
        );
      }
    }
    state.teleop.lastHgDaggerMode = hgMode || null;
    state.teleop.lastHgDaggerHapticToken = hapticToken ?? null;
    const headFollow = state.teleop.status?.head_follow;
    const headEnabled = headFollow?.enabled === true
      || headFollowEnabledFromStatus();
    const headPoseFresh = headFollow?.pose_fresh === true
      && state.teleop.xrPoseFresh;
    if (
      state.xrSession
      && state.teleop.lastHeadFollowEnabled !== null
      && state.teleop.lastHeadFollowEnabled !== headEnabled
    ) {
      showVrNotice(
        headEnabled
          ? '头部跟随已开启\n当前方向已设为正前方，机器人头部正在回中'
          : '头部跟随已关闭\n机器人头部保持当前位置',
        3500,
        headEnabled ? '#7DFFCF' : '#FFD166'
      );
    }
    if (
      state.xrSession
      && headEnabled
      && state.teleop.lastHeadPoseFresh !== null
      && state.teleop.lastHeadPoseFresh !== headPoseFresh
    ) {
      showVrNotice(
        headPoseFresh
          ? '头姿追踪已恢复\n原有正前方基准保持不变'
          : '头姿数据中断\n机器人头部已安全保持',
        3500,
        headPoseFresh ? '#7DFFCF' : '#FFB86B'
      );
    }
    state.teleop.lastHeadFollowEnabled = headEnabled;
    state.teleop.lastHeadPoseFresh = headPoseFresh;
    const headRecalibrationRequired = headFollow?.recalibration_required === true;
    if (
      state.xrSession
      && state.teleop.lastHeadRecalibrationRequired !== null
      && state.teleop.lastHeadRecalibrationRequired !== headRecalibrationRequired
      && headRecalibrationRequired
    ) {
      showVrNotice(
        '检测到头显坐标突变\n头部已保持，请使用 X+A 或关闭再开启头部跟随进行校准',
        6500,
        '#FFB86B'
      );
    }
    state.teleop.lastHeadRecalibrationRequired = headRecalibrationRequired;
    const quickResetActive = state.teleop.status?.quick_reset?.active === true;
    if (
      state.xrSession
      && state.teleop.lastQuickResetActive !== null
      && state.teleop.lastQuickResetActive !== quickResetActive
    ) {
      showVrNotice(
        quickResetActive
          ? '正在快速复位\n请松开左右腰键'
          : '快速复位完成\n松开再按腰键，当前身体朝向将自动设为新前方',
        quickResetActive ? 3000 : 5000,
        quickResetActive ? '#FFD166' : '#7DFFCF'
      );
    }
    state.teleop.lastQuickResetActive = quickResetActive;
    const desktop = state.teleop.status?.desktop_mode;
    const desktopReady = desktop?.enabled === true && desktop?.ready === true;
    if (desktop?.enabled === true) {
      if (state.teleop.desktopModeLastReady !== desktopReady) {
        showVrNotice(
          desktopReady
            ? '桌面双臂模式已就绪\n松开再按左右腰键接管'
            : '桌面双臂模式稳定检测中\n请放稳头显和双手柄',
          desktopReady ? 4500 : 3000,
          desktopReady ? '#7DFFCF' : '#FFD166'
        );
      }
      void acquireDesktopWakeLock();
    } else if (state.teleop.desktopModeLastReady !== null) {
      void releaseDesktopWakeLock();
    }
    state.teleop.desktopModeLastReady = desktop?.enabled === true
      ? desktopReady
      : null;
  } catch (error) {
    state.teleop.available = false;
    state.teleop.waistFollowAvailable = false;
    state.teleop.headFollowAvailable = false;
    state.teleop.xrPoseFresh = false;
    state.teleop.desktopModeAvailable = false;
    state.teleop.status = null;
    state.teleop.lastRequestMessage = `状态读取失败：${error.message}`;
  } finally {
    window.clearTimeout(timeout);
    state.teleop.pollInFlight = false;
  }
  renderHardwareControl();
  void maybeEnableAfterFreshVr();
}

async function maybeEnableAfterFreshVr() {
  if (
    !state.teleop.enableAfterFreshVr
    || state.teleop.requestPending
    || hardwareControlIsOnOrBusy()
  ) return;
  if (Date.now() > state.teleop.enableAfterFreshVrDeadline) {
    cancelDeferredHardwareEnable('等待 VR 手柄超时，真机未使能。请重新长按并进入 VR。');
    setStatus('未检测到实时 VR 手柄，真机保持关闭。');
    renderHardwareControl();
    return;
  }
  if (!vrInputIsFresh()) return;
  cancelDeferredHardwareEnable('已检测到实时 VR 手柄，正在执行安全使能。');
  await requestHardwareEnabled(true);
}

async function requestHardwareEnabled(enabled) {
  if (state.teleop.requestPending) return;
  if (!enabled) cancelDeferredHardwareEnable();
  state.teleop.requestPending = true;
  state.teleop.lastRequestMessage = '';
  renderHardwareControl();
  try {
    const response = await fetch('/api/hardware-control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        enabled,
        waist_follow_enabled: enabled
          ? Boolean(document.getElementById('waistFollowToggle')?.checked)
          : undefined,
        head_follow_enabled: enabled
          ? Boolean(document.getElementById('headFollowToggle')?.checked)
          : undefined
      })
    });
    let payload = {};
    try {
      payload = await response.json();
    } catch (_) {
      payload = { message: `HTTP ${response.status}` };
    }
    state.teleop.lastRequestMessage = payload.message || (enabled ? '使能请求已发送' : '关闭请求已发送');
    if (!response.ok || !payload.success) {
      throw new Error(state.teleop.lastRequestMessage);
    }
  } catch (error) {
    state.teleop.lastRequestMessage = error.message;
    setStatus(`真机遥操切换失败：${error.message}`);
  } finally {
    state.teleop.requestPending = false;
    await refreshHardwareStatus();
  }
}

async function requestWaistFollowEnabled(enabled) {
  if (
    state.teleop.waistFollowRequestPending
    || state.teleop.requestPending
    || hardwareControlIsOnOrBusy()
  ) return;
  state.teleop.waistFollowRequestPending = true;
  renderHardwareControl();
  try {
    const response = await fetch('/api/waist-follow', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enabled: Boolean(enabled) })
    });
    const payload = await response.json();
    state.teleop.lastRequestMessage = payload.message || '';
    if (!response.ok || !payload.success) {
      throw new Error(payload.message || `HTTP ${response.status}`);
    }
  } catch (error) {
    state.teleop.lastRequestMessage = `腰部跟随切换失败：${error.message}`;
    setStatus(state.teleop.lastRequestMessage);
  } finally {
    state.teleop.waistFollowRequestPending = false;
    await refreshHardwareStatus();
  }
}

async function requestHeadFollowEnabled(enabled) {
  if (
    state.teleop.headFollowRequestPending
    || state.teleop.requestPending
    || hardwareControlIsTransitioning()
  ) return;
  state.teleop.headFollowRequestPending = true;
  renderHardwareControl();
  try {
    const response = await fetch('/api/head-follow', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enabled: Boolean(enabled) })
    });
    const payload = await response.json();
    state.teleop.lastRequestMessage = payload.message || '';
    if (!response.ok || !payload.success) {
      throw new Error(payload.message || `HTTP ${response.status}`);
    }
    showVrNotice(
      enabled
        ? '头部跟随已开启\n保持当前朝向，随后可直接转头控制'
        : '头部跟随已关闭\n机器人头部保持当前位置',
      3500,
      enabled ? '#7DFFCF' : '#FFD166'
    );
  } catch (error) {
    state.teleop.lastRequestMessage = `头部跟随切换失败：${error.message}`;
    setStatus(state.teleop.lastRequestMessage);
  } finally {
    state.teleop.headFollowRequestPending = false;
    await refreshHardwareStatus();
  }
}

async function requestDesktopMode(enabled) {
  if (
    state.teleop.desktopModeRequestPending
    || !state.teleop.desktopModeAvailable
  ) return false;
  state.teleop.desktopModeRequestPending = true;
  showVrNotice(
    enabled
      ? '正在进入桌面双臂模式\n机器人保持当前位置'
      : '正在恢复佩戴头显模式\n机器人保持当前位置',
    3000
  );
  try {
    const response = await fetch('/api/desktop-mode', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enabled: Boolean(enabled) })
    });
    let payload = {};
    try {
      payload = await response.json();
    } catch (_) {
      payload = { message: `HTTP ${response.status}` };
    }
    state.teleop.lastRequestMessage = payload.message || '';
    if (!response.ok || !payload.success) {
      throw new Error(payload.message || `HTTP ${response.status}`);
    }
    showVrNotice(
      enabled
        ? '桌面双臂模式已开启\n现在放稳头显和双手柄'
        : '已恢复佩戴头显模式\n松开再按腰键重新接管',
      4500
    );
    if (enabled) {
      void acquireDesktopWakeLock();
    } else {
      void releaseDesktopWakeLock();
    }
    return true;
  } catch (error) {
    showVrNotice(
      `桌面双臂模式切换失败：${error.message}`,
      5000,
      '#FF7B89'
    );
    return false;
  } finally {
    state.teleop.desktopModeRequestPending = false;
    await refreshHardwareStatus();
  }
}

function cancelHardwareEnableHold() {
  if (state.teleop.holdTimer !== null) {
    window.clearTimeout(state.teleop.holdTimer);
    state.teleop.holdTimer = null;
  }
  const button = document.getElementById('hardwareToggleButton');
  if (button) button.classList.remove('is-holding');
  if (!state.teleop.requestPending) renderHardwareControl();
}

function abortHardwareEnableHold() {
  const confirmed = state.teleop.holdTriggered;
  state.teleop.holdTriggered = false;
  cancelHardwareEnableHold();
  if (confirmed) {
    cancelDeferredHardwareEnable('使能确认已取消，真机保持关闭。');
    renderHardwareControl();
  }
}

function finishHardwareEnableHold(event) {
  const confirmed = state.teleop.holdTriggered;
  cancelHardwareEnableHold();
  if (!confirmed) return;
  if (event) event.preventDefault();
  if (vrInputIsFresh()) {
    cancelDeferredHardwareEnable('已检测到实时 VR 手柄，正在执行安全使能。');
    void requestHardwareEnabled(true);
    return;
  }
  setStatus('授权已确认；正在进入 VR，检测到手柄实时姿态后自动使能。');
  void enterVr();
}

function beginHardwareEnableHold(event) {
  const button = document.getElementById('hardwareToggleButton');
  if (
    !button
    || button.disabled
    || hardwareControlIsOnOrBusy()
    || state.teleop.enableAfterFreshVr
  ) return;
  event.preventDefault();
  if (state.teleop.holdTimer !== null) return;
  state.teleop.holdTriggered = false;
  button.classList.remove('is-holding');
  void button.offsetWidth;
  button.classList.add('is-holding');
  setHardwareButtonText(button, '继续按住以确认使能');
  state.teleop.holdTimer = window.setTimeout(() => {
    state.teleop.holdTimer = null;
    state.teleop.holdTriggered = true;
    state.teleop.enableAfterFreshVr = true;
    state.teleop.enableAfterFreshVrDeadline = Date.now() + 15000;
    state.teleop.lastRequestMessage = '使能授权已确认，等待实时 VR 手柄姿态。';
    button.classList.remove('is-holding');
    renderHardwareControl();
  }, 1000);
}

function setupHardwareControl() {
  const button = document.getElementById('hardwareToggleButton');
  const waistToggle = document.getElementById('waistFollowToggle');
  const headToggle = document.getElementById('headFollowToggle');
  if (!button) return;
  if (state.teleop.controlsInitialized) return;
  state.teleop.controlsInitialized = true;
  if (waistToggle) {
    waistToggle.addEventListener('change', () => {
      void requestWaistFollowEnabled(waistToggle.checked);
    });
  }
  if (headToggle) {
    headToggle.addEventListener('change', () => {
      void requestHeadFollowEnabled(headToggle.checked);
    });
  }
  button.addEventListener('pointerdown', beginHardwareEnableHold);
  button.addEventListener('pointerup', finishHardwareEnableHold);
  button.addEventListener('pointercancel', abortHardwareEnableHold);
  button.addEventListener('pointerleave', event => {
    if (event.buttons) abortHardwareEnableHold();
  });
  button.addEventListener('keydown', event => {
    if (event.key === 'Enter' || event.key === ' ') beginHardwareEnableHold(event);
  });
  button.addEventListener('keyup', event => {
    if (event.key === 'Enter' || event.key === ' ') finishHardwareEnableHold(event);
  });
  button.addEventListener('click', event => {
    if (state.teleop.holdTriggered) {
      state.teleop.holdTriggered = false;
      event.preventDefault();
      return;
    }
    if (state.teleop.enableAfterFreshVr) {
      cancelDeferredHardwareEnable('等待使能已取消，真机保持关闭。');
      renderHardwareControl();
      return;
    }
    if (hardwareControlIsOnOrBusy()) {
      requestHardwareEnabled(false);
      return;
    }
    event.preventDefault();
    state.teleop.lastRequestMessage = '需要持续按住按钮 1 秒才能使能。';
    renderHardwareControl();
  });
  refreshHardwareStatus();
  if (state.teleop.pollTimer === null) {
    state.teleop.pollTimer = window.setInterval(refreshHardwareStatus, 400);
  }
}

async function loadConfig() {
  const response = await fetch('/api/config');
  state.config = await response.json();
  const imageConfig = state.config?.vr_images || state.config?.vr_image || {};
  state.image.enabled = Boolean(imageConfig.enabled);
  state.image.opacity = Number.isFinite(Number(imageConfig.opacity)) ? Number(imageConfig.opacity) : 0.82;
  state.image.cameras = imageConfig.cameras || (imageConfig.image_key ? [{ id: 'front', ...imageConfig }] : []);
  state.image.mode = imageConfig.default_mode === 'rgbd' ? 'rgbd' : 'passthrough';
  state.image.toggleButton = String(imageConfig.toggle_button || 'B').toUpperCase();
  return state.config;
}

function websocketUrl() {
  const path = state.config?.network?.websocket_path || '/ws';
  const url = new URL(`wss://${window.location.host}${path}`);
  url.searchParams.set('client_id', state.clientInstanceId);
  return url.toString();
}

function connectWebSocket() {
  if (state.websocket && [WebSocket.CONNECTING, WebSocket.OPEN].includes(state.websocket.readyState)) return;
  const url = websocketUrl();
  const websocket = new WebSocket(url);
  state.websocket = websocket;
  websocket.onopen = () => {
    if (state.websocket !== websocket) return;
    if (state.reconnectTimer) window.clearTimeout(state.reconnectTimer);
    state.reconnectTimer = null;
    setStatus(`VR data connected: ${url}`);
    refreshHardwareStatus();
    void connectRealtimeChannel();
  };
  websocket.onerror = () => {
    if (state.websocket === websocket) setStatus('VR data connection error');
  };
  websocket.onclose = () => {
    // A delayed close from a replaced socket must never retire the active
    // socket's DataChannel or schedule a second reconnect loop.
    if (state.websocket !== websocket) return;
    state.websocket = null;
    closeRealtimeChannel();
    setStatus('VR data disconnected; reconnecting...');
    state.teleop.lastRequestMessage = 'VR 数据连接暂时中断，机器人保持当前位置；正在自动重连。';
    renderHardwareControl();
    if (!state.reconnectTimer) {
      state.reconnectTimer = window.setTimeout(() => {
        state.reconnectTimer = null;
        connectWebSocket();
      }, 1000);
    }
  };
}

function scheduleRealtimeReconnect() {
  if (
    state.realtimeReconnectTimer
    || state.rtcPeer
    || state.websocket?.readyState !== WebSocket.OPEN
  ) return;
  state.realtimeReconnectTimer = window.setTimeout(() => {
    state.realtimeReconnectTimer = null;
    void connectRealtimeChannel();
  }, 2000);
}

function realtimeConnectionIsCurrent(peer, channel, generation) {
  return Boolean(
    state.rtcPeer === peer
    && state.realtimeChannel === channel
    && state.realtimeGeneration === generation
  );
}

function closeRealtimeChannel(expectedGeneration = null) {
  if (
    expectedGeneration !== null
    && state.realtimeGeneration !== expectedGeneration
  ) return false;
  if (state.realtimeReconnectTimer) {
    window.clearTimeout(state.realtimeReconnectTimer);
    state.realtimeReconnectTimer = null;
  }
  const channel = state.realtimeChannel;
  const peer = state.rtcPeer;
  state.realtimeChannel = null;
  state.rtcPeer = null;
  state.realtimeGeneration = 0;
  useLatestJpegFallback('real-time WebRTC connection is reconnecting');
  if (channel) {
    channel.onclose = null;
    try { channel.close(); } catch (_) { /* already closed */ }
  }
  if (peer) {
    peer.onconnectionstatechange = null;
    try { peer.close(); } catch (_) { /* already closed */ }
  }
  return Boolean(channel || peer);
}

function markRealtimeFallback(durationMs = 3000) {
  state.realtimeFallbackUntil = Math.max(
    state.realtimeFallbackUntil,
    performance.now() + durationMs
  );
}

function sendLatestRealtimeOverWebSocket(now = performance.now()) {
  if (
    !state.realtimeLatestEncodedPacket
    || state.websocket?.readyState !== WebSocket.OPEN
  ) return false;
  try {
    state.websocket.send(state.realtimeLatestEncodedPacket);
    state.realtimeFallbackLastSendAt = now;
    return true;
  } catch (_) {
    return false;
  }
}

function retireRealtimeConnection(peer, channel, generation, reconnect = true) {
  if (!realtimeConnectionIsCurrent(peer, channel, generation)) return false;
  markRealtimeFallback();
  // Do not wait for ICE/SCTP teardown: put the freshest pose onto the reliable
  // path now, then retire and recreate the stale peer asynchronously.
  sendLatestRealtimeOverWebSocket();
  closeRealtimeChannel(generation);
  if (reconnect) scheduleRealtimeReconnect();
  return true;
}

function startRealtimeWatchdog() {
  if (state.realtimeWatchdogTimer) return;
  state.realtimeWatchdogTimer = window.setInterval(() => {
    const channel = state.realtimeChannel;
    const peer = state.rtcPeer;
    if (!channel || channel.readyState !== 'open') return;
    const peerBad = peer && ['failed', 'disconnected', 'closed'].includes(peer.connectionState);
    const bufferedStuck = channel.bufferedAmount > 16384;
    const now = performance.now();
    const activelySending = state.xrSession && now - state.realtimeLastSendAt < 500;
    const acknowledgementStale = activelySending && now - state.realtimeLastAckAt > 800;
    const cameraStalled = (
      state.image.mode === 'rgbd'
      && document.visibilityState === 'visible'
      && state.xrSession
      && Array.from(state.image.states.values()).some(cameraState => (
        cameraState.transportMode === 'unified-webrtc'
        && now - cameraState.unifiedStartedAt > 2000
        && now - cameraState.lastVideoFrameAt > 1200
      ))
    );
    if (!peerBad && !bufferedStuck && !acknowledgementStale && !cameraStalled) return;
    // A WebRTC DataChannel can remain `open` after the UDP path has stopped
    // delivering. Immediately fall back to the live WebSocket and recreate
    // the peer instead of black-holing every future Grip/X+A frame.
    retireRealtimeConnection(
      peer,
      channel,
      state.realtimeGeneration
    );
  }, 250);
}

async function connectRealtimeChannel() {
  if (
    state.config?.network?.webrtc_enabled !== true
    || state.websocket?.readyState !== WebSocket.OPEN
    || state.rtcPeer
  ) return;
  const generation = ++state.realtimeGenerationCounter;
  const peer = new RTCPeerConnection({ iceServers: [] });
  const channel = peer.createDataChannel('teleop', {
    ordered: false,
    maxRetransmits: 0
  });
  state.rtcPeer = peer;
  state.realtimeChannel = channel;
  state.realtimeGeneration = generation;
  state.realtimeLastAckSequence = -1;
  if (state.image.enabled) {
    peer.addTransceiver('video', { direction: 'recvonly' });
    peer.ontrack = event => {
      if (!realtimeConnectionIsCurrent(peer, channel, generation)) return;
      if (event.track?.kind !== 'video') return;
      const stream = event.streams[0] || new MediaStream([event.track]);
      attachUnifiedCameraStream(peer, generation, stream);
    };
  }
  channel.bufferedAmountLowThreshold = 0;
  channel.onopen = () => {
    if (!realtimeConnectionIsCurrent(peer, channel, generation)) return;
    state.realtimeLastAckAt = performance.now();
    // Probe the newly opened channel exclusively. If it is half-open, the ACK
    // watchdog restores WebSocket delivery within 800 ms.
    state.realtimeFallbackUntil = 0;
    setStatus('VR real-time UDP channel connected');
  };
  channel.onmessage = event => {
    if (!realtimeConnectionIsCurrent(peer, channel, generation)) return;
    try {
      const packet = JSON.parse(event.data);
      const ackSequence = Number(packet?.sequence);
      if (
        packet?.type === 'teleop_ack'
        && Number(packet.generation) === generation
        && Number.isSafeInteger(ackSequence)
        && ackSequence > state.realtimeLastAckSequence
        && ackSequence <= state.realtimeSequence
      ) {
        state.realtimeLastAckSequence = ackSequence;
        state.realtimeLastAckAt = performance.now();
        state.realtimeFallbackUntil = 0;
      }
    } catch (_) { /* Ignore non-control channel messages. */ }
  };
  channel.onclose = () => {
    retireRealtimeConnection(peer, channel, generation);
  };
  channel.onerror = () => {
    if (!realtimeConnectionIsCurrent(peer, channel, generation)) return;
    setStatus('UDP real-time channel unavailable; using WebSocket fallback');
    retireRealtimeConnection(peer, channel, generation);
  };
  peer.onconnectionstatechange = () => {
    if (['failed', 'disconnected', 'closed'].includes(peer.connectionState)) {
      retireRealtimeConnection(peer, channel, generation);
    }
  };
  try {
    const offer = await peer.createOffer();
    await peer.setLocalDescription(offer);
    await waitForIceGatheringComplete(peer);
    const path = state.config?.network?.realtime_offer_path || '/api/realtime/offer';
    const response = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        sdp: peer.localDescription.sdp,
        type: peer.localDescription.type,
        generation
      })
    });
    if (!response.ok) throw new Error(await response.text());
    const answer = await response.json();
    if (!realtimeConnectionIsCurrent(peer, channel, generation)) {
      try { peer.close(); } catch (_) { /* superseded while negotiating */ }
      return;
    }
    if (Number(answer.generation) !== generation) {
      throw new Error('stale WebRTC answer generation');
    }
    await peer.setRemoteDescription(answer);
  } catch (error) {
    if (!realtimeConnectionIsCurrent(peer, channel, generation)) return;
    markRealtimeFallback();
    closeRealtimeChannel(generation);
    console.warn('WebRTC setup failed; using WebSocket fallback:', error);
    scheduleRealtimeReconnect();
  }
}

function sendRealtimePacket(payload) {
  payload.sequence = ++state.realtimeSequence;
  if (!payload.timestamp) payload.timestamp = Date.now();
  const encoded = JSON.stringify(payload);
  state.realtimeLatestEncodedPacket = encoded;
  const channel = state.realtimeChannel;
  const now = performance.now();
  if (now - state.realtimeLastSendAt > 500) {
    // First frame after an idle/non-immersive period gets a full ACK timeout;
    // do not mistake the intentionally idle channel for a half-open path.
    state.realtimeLastAckAt = now;
  }
  state.realtimeLastSendAt = now;
  let sentWebrtc = false;
  const fallbackActive = now < state.realtimeFallbackUntil;
  if (
    !fallbackActive
    && channel?.readyState === 'open'
    && state.rtcPeer
    && !['failed', 'disconnected', 'closed'].includes(state.rtcPeer.connectionState)
  ) {
    // Never build a local queue of stale motion.  A future frame is more
    // valuable than any pose that could not be sent immediately.
    if (channel.bufferedAmount <= 16384) {
      try {
        channel.send(encoded);
        sentWebrtc = true;
      } catch (_) {
        markRealtimeFallback();
      }
    } else {
      markRealtimeFallback();
    }
  }
  // During normal operation send only UDP. If that route is unhealthy, send
  // the same sequenced latest frame over WebSocket at 30 Hz until WebRTC is
  // recreated. The bridge de-duplicates by sequence across both transports.
  const fallbackDue = fallbackActive || !sentWebrtc;
  if (
    fallbackDue
    && now - state.realtimeFallbackLastSendAt >= 33
    && state.websocket?.readyState === WebSocket.OPEN
  ) {
    state.websocket.send(encoded);
    state.realtimeFallbackLastSendAt = now;
  }
}

function sendReliablePacket(payload) {
  if (!payload.timestamp) payload.timestamp = Date.now();
  if (state.websocket?.readyState === WebSocket.OPEN) {
    state.websocket.send(JSON.stringify(payload));
  }
}

function attachRigToCamera() {
  const rig = document.getElementById('cameraRig');
  const cameraEl = document.querySelector('a-scene')?.camera?.el;
  if (!rig || !cameraEl) return;
  if (rig.parentNode !== cameraEl) cameraEl.appendChild(rig);
  rig.setAttribute('position', '0 -0.08 -1.85');
}

function createTextHud() {
  const cameraEl = document.querySelector('a-scene')?.camera?.el;
  if (!cameraEl || state.headText) return;

  const text = document.createElement('a-text');
  text.setAttribute('value', 'Head: waiting...');
  text.setAttribute('position', '0 -0.38 -0.75');
  text.setAttribute('align', 'center');
  text.setAttribute('color', '#EAF4FF');
  text.setAttribute('width', '0.9');
  text.setAttribute('baseline', 'center');
  text.setAttribute('anchor', 'center');
  cameraEl.appendChild(text);
  state.headText = text;
}

function createBrandHud() {
  const rig = document.getElementById('cameraRig');
  if (!rig || state.brandHud) return;

  const group = document.createElement('a-entity');
  group.setAttribute('id', 'hgm-brand-hud');
  group.setAttribute('position', '0.72 0.88 0');

  const mark = document.createElement('a-text');
  mark.setAttribute('value', 'AUTOLIFE 306 V4');
  mark.setAttribute('align', 'center');
  mark.setAttribute('anchor', 'center');
  mark.setAttribute('baseline', 'center');
  mark.setAttribute('color', '#05070A');
  mark.setAttribute('width', '1.45');
  mark.setAttribute('position', '0 0 0');
  group.appendChild(mark);

  const subtitle = document.createElement('a-text');
  subtitle.setAttribute('value', 'Dual Arm Teleoperation');
  subtitle.setAttribute('align', 'center');
  subtitle.setAttribute('anchor', 'center');
  subtitle.setAttribute('baseline', 'center');
  subtitle.setAttribute('color', '#05070A');
  subtitle.setAttribute('width', '0.86');
  subtitle.setAttribute('position', '0 -0.12 0');
  group.appendChild(subtitle);

  rig.appendChild(group);
  state.brandHud = group;
}

function wait(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

async function waitForIceGatheringComplete(pc) {
  if (pc.iceGatheringState === 'complete') return;

  await new Promise(resolve => {
    const checkState = () => {
      if (pc.iceGatheringState === 'complete') {
        pc.removeEventListener('icegatheringstatechange', checkState);
        resolve();
      }
    };
    pc.addEventListener('icegatheringstatechange', checkState);
    window.setTimeout(() => {
      pc.removeEventListener('icegatheringstatechange', checkState);
      resolve();
    }, 2000);
  });
}

function cameraId(cameraConfig) {
  return cameraConfig.id || cameraConfig.image_key || 'front';
}

function panelLayout(cameraIdValue) {
  if (cameraIdValue === 'rgbd_head_color') return { x: 0, y: 0.02, width: 2.3, height: 1.725 };
  if (cameraIdValue === 'front') return { x: 0, y: 0.16, width: 1.5, height: 1.12 };
  if (cameraIdValue === 'left_wrist') return { x: -1.1, y: -0.08, width: 0.76, height: 0.57 };
  if (cameraIdValue === 'right_wrist') return { x: 1.1, y: -0.08, width: 0.76, height: 0.57 };
  return { x: 0, y: -0.08, width: 0.76, height: 0.57 };
}

function createHiddenImageState(cameraConfig) {
  const id = cameraId(cameraConfig);
  const img = document.createElement('img');
  img.id = `${id}-camera-stream`;
  img.alt = `${id} ZMQ camera stream`;
  img.crossOrigin = 'anonymous';
  img.decoding = 'async';
  img.style.cssText = 'position:fixed;right:0;bottom:0;width:2px;height:2px;opacity:0.01;pointer-events:none;';

  const canvas = document.createElement('canvas');
  canvas.id = `${id}-camera-canvas`;
  canvas.width = cameraConfig.width || 640;
  canvas.height = cameraConfig.height || 480;
  canvas.style.cssText = img.style.cssText;

  const video = document.createElement('video');
  video.id = `${id}-camera-video`;
  video.autoplay = true;
  video.muted = true;
  video.playsInline = true;
  video.setAttribute('autoplay', '');
  video.setAttribute('muted', '');
  video.setAttribute('playsinline', '');
  video.setAttribute('webkit-playsinline', '');
  video.style.cssText = img.style.cssText;

  document.body.appendChild(img);
  document.body.appendChild(canvas);
  document.body.appendChild(video);

  state.image.states.set(id, {
    id,
    config: cameraConfig,
    img,
    video,
    canvas,
    ctx: canvas.getContext('2d', { alpha: false, desynchronized: true }),
    texture: null,
    textureKind: null,
    material: null,
    transportMode: 'initializing',
    peerConnection: null,
    cameraConnectionGeneration: 0,
    cameraConnectInFlight: false,
    cameraReconnectTimer: null,
    cameraReconnectAttempt: 0,
    cameraConnectStartedAt: 0,
    cameraConnectedAt: 0,
    loading: false,
    lastFrameRequestMs: -Infinity,
    frameSeq: 0,
    drawnFrameSeq: 0,
    snapshotLoopRunning: false,
    snapshotStopRequested: false,
    videoPumpGeneration: 0,
    unifiedStartedAt: 0,
    lastVideoFrameAt: 0,
    decodedFrameCount: 0,
    drawnFrameCount: 0,
    clientFps: 0,
    fpsWindowStartedAt: 0,
    fpsWindowFrames: 0,
    fallbackReason: '',
    frameIntervalMs: 1000 / Math.max(1, Math.min(30, Number(cameraConfig.fps || 30)))
  });
}

function disposeCameraTexture(cameraState) {
  if (!cameraState?.texture) return;
  try {
    cameraState.texture.dispose();
  } catch (error) {
    console.warn('Failed to dispose camera texture:', error);
  }
  cameraState.texture = null;
  cameraState.textureKind = null;
  cameraState.material = null;
}

function setPanelTexture(id, texture, kind) {
  const cameraState = state.image.states.get(id);
  const panel = state.image.panels.get(id);
  if (!cameraState || !panel) return;

  const mesh = panel.screen.getObject3D('mesh');
  if (!mesh) return;

  if (cameraState.texture !== texture || cameraState.textureKind !== kind || !cameraState.material) {
    cameraState.material = new THREE.MeshBasicMaterial({
      map: texture,
      side: THREE.DoubleSide,
      toneMapped: false,
      transparent: state.image.opacity < 1,
      opacity: state.image.opacity,
      depthWrite: state.image.opacity >= 1
    });
    mesh.material = cameraState.material;
    mesh.material.needsUpdate = true;
    cameraState.texture = texture;
    cameraState.textureKind = kind;
  }
}

function applyCanvasTexture(id) {
  const cameraState = state.image.states.get(id);
  if (!cameraState) return;

  if (!cameraState.texture || cameraState.textureKind !== 'canvas') {
    disposeCameraTexture(cameraState);
    cameraState.texture = new THREE.CanvasTexture(cameraState.canvas);
    cameraState.texture.minFilter = THREE.LinearFilter;
    cameraState.texture.magFilter = THREE.LinearFilter;
    cameraState.texture.generateMipmaps = false;
    if ('colorSpace' in cameraState.texture && THREE.SRGBColorSpace) {
      cameraState.texture.colorSpace = THREE.SRGBColorSpace;
    }
    cameraState.textureKind = 'canvas';
    cameraState.material = null;
  }

  setPanelTexture(id, cameraState.texture, 'canvas');
  cameraState.texture.needsUpdate = true;
}

function setCameraLatestJpeg(cameraState, reason = '') {
  if (!cameraState || cameraState.config.transport !== 'unified-webrtc') return;
  cameraState.videoPumpGeneration += 1;
  cameraState.transportMode = 'latest-jpeg';
  cameraState.fallbackReason = reason;
  cameraState.unifiedStartedAt = 0;
  try { cameraState.video.pause(); } catch (_) { /* already paused */ }
  cameraState.video.srcObject = null;
  updateCameraPanel(cameraState.id);
  void startLatestJpegStream(cameraState.id);
}

function useLatestJpegFallback(reason = '') {
  state.image.unifiedStream = null;
  state.image.unifiedPeer = null;
  state.image.unifiedGeneration = 0;
  state.image.states.forEach(cameraState => {
    setCameraLatestJpeg(cameraState, reason);
  });
}

function pumpUnifiedVideoFrames(cameraState, peer, generation) {
  const token = ++cameraState.videoPumpGeneration;
  const video = cameraState.video;
  const onFrame = () => {
    if (
      token !== cameraState.videoPumpGeneration
      || state.rtcPeer !== peer
      || state.realtimeGeneration !== generation
      || video.srcObject !== state.image.unifiedStream
    ) return;
    cameraState.lastVideoFrameAt = performance.now();
    cameraState.decodedFrameCount += 1;
    cameraState.fpsWindowFrames += 1;
    if (cameraState.fpsWindowStartedAt <= 0) {
      cameraState.fpsWindowStartedAt = cameraState.lastVideoFrameAt;
    } else if (cameraState.lastVideoFrameAt - cameraState.fpsWindowStartedAt >= 1000) {
      cameraState.clientFps = 1000 * cameraState.fpsWindowFrames
        / (cameraState.lastVideoFrameAt - cameraState.fpsWindowStartedAt);
      cameraState.fpsWindowStartedAt = cameraState.lastVideoFrameAt;
      cameraState.fpsWindowFrames = 0;
      updateCameraPanel(cameraState.id);
    }
    if (
      state.image.mode === 'rgbd'
      && video.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA
      && video.videoWidth > 0
      && video.videoHeight > 0
    ) {
      if (
        cameraState.canvas.width !== video.videoWidth
        || cameraState.canvas.height !== video.videoHeight
      ) {
        cameraState.canvas.width = video.videoWidth;
        cameraState.canvas.height = video.videoHeight;
      }
      cameraState.ctx.drawImage(
        video, 0, 0, cameraState.canvas.width, cameraState.canvas.height
      );
      cameraState.drawnFrameCount += 1;
      applyCanvasTexture(cameraState.id);
    }
    video.requestVideoFrameCallback(onFrame);
  };
  video.requestVideoFrameCallback(onFrame);
}

function attachUnifiedCameraStream(peer, generation, stream) {
  if (state.rtcPeer !== peer || state.realtimeGeneration !== generation) return;
  state.image.unifiedStream = stream;
  state.image.unifiedPeer = peer;
  state.image.unifiedGeneration = generation;
  state.image.states.forEach(cameraState => {
    if (cameraState.config.transport !== 'unified-webrtc') return;
    cameraState.video.srcObject = stream;
    cameraState.transportMode = 'unified-webrtc';
    cameraState.unifiedStartedAt = performance.now();
    cameraState.lastVideoFrameAt = cameraState.unifiedStartedAt;
    cameraState.fallbackReason = '';
    updateCameraPanel(cameraState.id);
    cameraState.video.play().then(() => {
      if (
        state.rtcPeer !== peer
        || state.realtimeGeneration !== generation
        || cameraState.video.srcObject !== stream
      ) return;
      if (typeof cameraState.video.requestVideoFrameCallback !== 'function') {
        setCameraLatestJpeg(cameraState, 'video frame callback is unavailable');
        return;
      }
      pumpUnifiedVideoFrames(cameraState, peer, generation);
    }).catch(error => {
      setCameraLatestJpeg(cameraState, `video playback failed: ${error.message}`);
    });
  });
}

function applyVideoTexture(id) {
  const cameraState = state.image.states.get(id);
  if (!cameraState || !cameraState.video.srcObject) return;

  if (!cameraState.texture || cameraState.textureKind !== 'video') {
    disposeCameraTexture(cameraState);
    cameraState.texture = new THREE.VideoTexture(cameraState.video);
    cameraState.texture.minFilter = THREE.LinearFilter;
    cameraState.texture.magFilter = THREE.LinearFilter;
    cameraState.texture.generateMipmaps = false;
    if ('colorSpace' in cameraState.texture && THREE.SRGBColorSpace) {
      cameraState.texture.colorSpace = THREE.SRGBColorSpace;
    }
    cameraState.textureKind = 'video';
    cameraState.material = null;
  }

  setPanelTexture(id, cameraState.texture, 'video');
}

function drawCameraFrame(id) {
  const cameraState = state.image.states.get(id);
  if (!cameraState || !cameraState.ctx) return;

  if (
    cameraState.transportMode === 'webrtc'
    || cameraState.transportMode === 'unified-webrtc'
  ) {
    return;
  }

  if (!cameraState.img.complete || !cameraState.img.naturalWidth || !cameraState.img.naturalHeight) return;
  if (cameraState.canvas.width !== cameraState.img.naturalWidth || cameraState.canvas.height !== cameraState.img.naturalHeight) {
    cameraState.canvas.width = cameraState.img.naturalWidth;
    cameraState.canvas.height = cameraState.img.naturalHeight;
  }

  try {
    cameraState.ctx.drawImage(cameraState.img, 0, 0, cameraState.canvas.width, cameraState.canvas.height);
    applyCanvasTexture(id);
  } catch (error) {
    console.warn(`Could not draw MJPEG frame for ${id}:`, error);
  }
}

function createCameraPanel(cameraConfig) {
  const rig = document.getElementById('cameraRig');
  if (!rig) return;

  const id = cameraId(cameraConfig);
  const layout = panelLayout(id);
  const panel = document.createElement('a-entity');
  panel.setAttribute('id', `${id}-camera-panel`);
  panel.setAttribute('position', `${layout.x} ${layout.y} 0`);
  panel.setAttribute('visible', state.image.mode === 'rgbd');

  const border = document.createElement('a-plane');
  border.setAttribute('width', (layout.width + 0.04).toFixed(2));
  border.setAttribute('height', (layout.height + 0.04).toFixed(2));
  border.setAttribute('color', '#202836');
  border.setAttribute('position', '0 0 -0.01');
  border.setAttribute('material', 'shader: flat; side: double');
  panel.appendChild(border);

  const screen = document.createElement('a-plane');
  screen.setAttribute('width', layout.width);
  screen.setAttribute('height', layout.height);
  screen.setAttribute('color', '#111111');
  screen.setAttribute('material', 'shader: flat; side: double');
  panel.appendChild(screen);

  const label = document.createElement('a-text');
  label.setAttribute('value', cameraConfig.name || id);
  label.setAttribute('align', 'center');
  label.setAttribute('color', '#FFFFFF');
  label.setAttribute('width', '1.6');
  label.setAttribute('position', `0 ${(layout.height / 2 + 0.1).toFixed(2)} 0`);
  panel.appendChild(label);

  const meta = document.createElement('a-text');
  meta.setAttribute('value', 'waiting...');
  meta.setAttribute('align', 'center');
  meta.setAttribute('color', '#B8C7D9');
  meta.setAttribute('width', '1.2');
  meta.setAttribute('position', `0 ${(-layout.height / 2 - 0.1).toFixed(2)} 0`);
  panel.appendChild(meta);

  rig.appendChild(panel);
  state.image.panels.set(id, { panel, screen, label, meta });
}

function updateCameraPanel(id) {
  const cameraState = state.image.states.get(id);
  const panel = state.image.panels.get(id);
  if (!cameraState || !panel || !cameraState.status) return;

  panel.label.setAttribute('value', cameraState.status.name || id);
  const text = [
    `${cameraState.status.width || 0}x${cameraState.status.height || 0}`,
    `${cameraState.status.fps || 0} FPS`,
    `transport: ${cameraState.transportMode}`,
    `client: ${Number(cameraState.clientFps || 0).toFixed(1)} FPS`,
    `frame: ${cameraState.status.frame_version ?? 0}`,
    cameraState.status.image_key || id
  ];
  if (cameraState.fallbackReason) text.push(`fallback: ${cameraState.fallbackReason}`);
  if (cameraState.status.last_error) text.push(`error: ${cameraState.status.last_error}`);
  panel.meta.setAttribute('value', text.join(' | '));
}

async function refreshCameraStatus(id) {
  const cameraState = state.image.states.get(id);
  if (!cameraState) return;
  try {
    cameraState.status = await fetch(`/api/camera/status?camera=${encodeURIComponent(id)}`).then(response => response.json());
    updateCameraPanel(id);
  } catch (error) {
    console.warn(`Could not load ZMQ camera status for ${id}:`, error);
  }
}

async function reportCameraClientStatus() {
  const now = performance.now();
  const cameras = {};
  state.image.states.forEach((cameraState, id) => {
    cameras[id] = {
      mode: state.image.mode,
      transport: cameraState.transportMode,
      client_fps: Number(cameraState.clientFps.toFixed(1)),
      decoded_frames: cameraState.decodedFrameCount,
      drawn_frames: cameraState.drawnFrameCount,
      video_age_ms: cameraState.lastVideoFrameAt > 0
        ? Math.round(now - cameraState.lastVideoFrameAt) : null,
      fallback_reason: cameraState.fallbackReason
    };
  });
  try {
    await fetch('/api/camera/client-status', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        page: 'v28',
        client_id: state.clientInstanceId,
        cameras
      }),
      cache: 'no-store',
      keepalive: true
    });
  } catch (_) { /* Status reporting must never affect video playback. */ }
}

function updateCameraModeUi() {
  const rgbMode = state.image.mode === 'rgbd';
  state.image.panels.forEach(panel => {
    panel.panel.setAttribute('visible', rgbMode);
  });
  const button = document.getElementById('cameraModeButton');
  if (button) {
    button.textContent = rgbMode
      ? '图像模式：深度相机 RGB（按 B 返回透视）'
      : '图像模式：头显透视（按 B 查看相机）';
    button.classList.toggle('is-camera', rgbMode);
  }
  if (state.image.modeIndicator) {
    state.image.modeIndicator.setAttribute(
      'value', rgbMode ? 'RGB-D CAMERA  |  B: PASSTHROUGH' : ''
    );
    state.image.modeIndicator.setAttribute('visible', rgbMode);
  }
  setStatus(rgbMode ? '已切换到头部深度相机 RGB 图像' : '已切换到头显透视模式');
}

function setCameraMode(mode) {
  if (!state.image.enabled) return false;
  state.image.mode = mode === 'rgbd' ? 'rgbd' : 'passthrough';
  if (
    state.image.mode === 'rgbd'
    && state.image.unifiedStream
    && state.image.unifiedPeer === state.rtcPeer
  ) {
    attachUnifiedCameraStream(
      state.rtcPeer,
      state.realtimeGeneration,
      state.image.unifiedStream
    );
  }
  updateCameraModeUi();
  return true;
}

function toggleCameraMode() {
  const now = performance.now();
  if (!state.image.enabled || now - state.image.lastToggleAt < 300) return false;
  state.image.lastToggleAt = now;
  return setCameraMode(state.image.mode === 'rgbd' ? 'passthrough' : 'rgbd');
}

function createCameraModeIndicator() {
  if (state.image.modeIndicator) return;
  const rig = document.getElementById('cameraRig');
  if (!rig) return;
  const indicator = document.createElement('a-text');
  indicator.setAttribute('id', 'camera-mode-indicator');
  indicator.setAttribute('value', '');
  indicator.setAttribute('align', 'center');
  indicator.setAttribute('color', '#7DFFCF');
  indicator.setAttribute('width', '1.8');
  indicator.setAttribute('position', '0 -1.08 0.03');
  indicator.setAttribute('visible', false);
  rig.appendChild(indicator);
  state.image.modeIndicator = indicator;
}

function startMjpegStream(id) {
  const cameraState = state.image.states.get(id);
  if (!cameraState) return;

  cameraState.transportMode = 'mjpeg-stream';
  const oldPeer = cameraState.peerConnection;
  cameraState.peerConnection = null;
  if (oldPeer) {
    try {
      oldPeer.close();
    } catch (error) {
      console.warn(`Failed to close ${id} WebRTC peer:`, error);
    }
  }
  cameraState.video.pause();
  cameraState.video.srcObject = null;

  const loadStream = () => {
    cameraState.img.src = `/api/camera/stream.mjpg?camera=${encodeURIComponent(id)}&ts=${Date.now()}`;
  };
  cameraState.img.onload = () => updateCameraPanel(id);
  cameraState.img.onerror = () => {
    cameraState.transportMode = 'mjpeg-retrying';
    updateCameraPanel(id);
    window.setTimeout(loadStream, 500);
  };
  loadStream();
  updateCameraPanel(id);
}

function delay(milliseconds) {
  return new Promise(resolve => window.setTimeout(resolve, milliseconds));
}

async function startLatestJpegStream(id) {
  const cameraState = state.image.states.get(id);
  if (!cameraState || cameraState.snapshotLoopRunning) return;
  cameraState.snapshotLoopRunning = true;
  cameraState.snapshotStopRequested = false;
  cameraState.transportMode = 'latest-jpeg';
  updateCameraPanel(id);

  while (!cameraState.snapshotStopRequested) {
    if (
      state.image.mode !== 'rgbd'
      || cameraState.transportMode === 'unified-webrtc'
    ) {
      await delay(80);
      continue;
    }
    try {
      const baseUrl = cameraState.config.frame_url
        || `/api/camera/frame.jpg?camera=${encodeURIComponent(id)}`;
      const separator = baseUrl.includes('?') ? '&' : '?';
      const response = await fetch(
        `${baseUrl}${separator}after=${cameraState.frameSeq}&ts=${Date.now()}`,
        { cache: 'no-store' }
      );
      if (response.status === 204) continue;
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const nextFrame = Number(response.headers.get('X-Frame-Id'));
      const blob = await response.blob();
      if (cameraState.transportMode === 'unified-webrtc') continue;
      const bitmap = await createImageBitmap(blob);
      try {
        if (
          cameraState.canvas.width !== bitmap.width
          || cameraState.canvas.height !== bitmap.height
        ) {
          cameraState.canvas.width = bitmap.width;
          cameraState.canvas.height = bitmap.height;
        }
        cameraState.ctx.drawImage(
          bitmap, 0, 0, cameraState.canvas.width, cameraState.canvas.height
        );
      } finally {
        bitmap.close();
      }
      if (Number.isSafeInteger(nextFrame)) cameraState.frameSeq = nextFrame;
      cameraState.drawnFrameSeq = cameraState.frameSeq;
      applyCanvasTexture(id);
    } catch (error) {
      cameraState.transportMode = 'latest-jpeg-retrying';
      updateCameraPanel(id);
      console.warn(`Latest JPEG frame failed for ${id}:`, error);
      await delay(250);
      cameraState.transportMode = 'latest-jpeg';
    }
  }
  cameraState.snapshotLoopRunning = false;
}

function cancelCameraReconnect(cameraState) {
  if (!cameraState?.cameraReconnectTimer) return;
  window.clearTimeout(cameraState.cameraReconnectTimer);
  cameraState.cameraReconnectTimer = null;
}

function scheduleCameraReconnect(cameraState, reason = 'video connection interrupted') {
  if (!cameraState || cameraState.cameraReconnectTimer) return;
  cameraState.cameraReconnectAttempt = Math.min(
    4, cameraState.cameraReconnectAttempt + 1
  );
  const delayMs = Math.min(
    4000, 500 * (2 ** Math.max(0, cameraState.cameraReconnectAttempt - 1))
  );
  cameraState.transportMode = 'webrtc-reconnecting';
  cameraState.fallbackReason = reason;
  updateCameraPanel(cameraState.id);
  cameraState.cameraReconnectTimer = window.setTimeout(() => {
    cameraState.cameraReconnectTimer = null;
    const peer = cameraState.peerConnection;
    if (
      peer
      && peer.connectionState === 'connected'
      && performance.now() - cameraState.lastVideoFrameAt < 2000
    ) {
      cameraState.cameraReconnectAttempt = 0;
      cameraState.transportMode = 'webrtc';
      cameraState.fallbackReason = '';
      updateCameraPanel(cameraState.id);
      return;
    }
    void startCameraFeed(cameraState.id);
  }, delayMs);
}

function pumpIndependentVideoFrames(cameraState, peer, generation) {
  const token = ++cameraState.videoPumpGeneration;
  const video = cameraState.video;
  let lastCurrentTime = -1;

  const onFrame = () => {
    if (
      token !== cameraState.videoPumpGeneration
      || cameraState.peerConnection !== peer
      || cameraState.cameraConnectionGeneration !== generation
      || video.srcObject === null
    ) return;
    const now = performance.now();
    cameraState.lastVideoFrameAt = now;
    cameraState.decodedFrameCount += 1;
    cameraState.fpsWindowFrames += 1;
    cameraState.cameraReconnectAttempt = 0;
    if (cameraState.fpsWindowStartedAt <= 0) {
      cameraState.fpsWindowStartedAt = now;
    } else if (now - cameraState.fpsWindowStartedAt >= 1000) {
      cameraState.clientFps = 1000 * cameraState.fpsWindowFrames
        / (now - cameraState.fpsWindowStartedAt);
      cameraState.fpsWindowStartedAt = now;
      cameraState.fpsWindowFrames = 0;
      updateCameraPanel(cameraState.id);
    }
    if (state.image.mode === 'rgbd') {
      cameraState.drawnFrameCount += 1;
      cameraState.drawnFrameSeq += 1;
    }
    if (typeof video.requestVideoFrameCallback === 'function') {
      video.requestVideoFrameCallback(onFrame);
    } else {
      window.requestAnimationFrame(pollCurrentTime);
    }
  };

  const pollCurrentTime = () => {
    if (
      token !== cameraState.videoPumpGeneration
      || cameraState.peerConnection !== peer
      || cameraState.cameraConnectionGeneration !== generation
    ) return;
    if (video.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA
        && video.currentTime !== lastCurrentTime) {
      lastCurrentTime = video.currentTime;
      onFrame();
      return;
    }
    window.requestAnimationFrame(pollCurrentTime);
  };

  if (typeof video.requestVideoFrameCallback === 'function') {
    video.requestVideoFrameCallback(onFrame);
  } else {
    window.requestAnimationFrame(pollCurrentTime);
  }
}

function startCameraWatchdog() {
  if (state.image.cameraWatchdogTimer) return;
  state.image.cameraWatchdogTimer = window.setInterval(() => {
    const now = performance.now();
    state.image.states.forEach(cameraState => {
      const peer = cameraState.peerConnection;
      if (!peer || cameraState.cameraConnectInFlight) return;
      if (
        cameraState.transportMode === 'webrtc'
        && cameraState.cameraConnectedAt > 0
        && now - cameraState.cameraConnectedAt > 3000
        && now - cameraState.lastVideoFrameAt > 2500
      ) {
        scheduleCameraReconnect(cameraState, 'video frames stalled');
      } else if (
        cameraState.transportMode === 'webrtc-connecting'
        && now - cameraState.cameraConnectStartedAt > 8000
      ) {
        scheduleCameraReconnect(cameraState, 'video connection timed out');
      }
    });
  }, 500);
}

async function startCameraFeed(id) {
  const cameraState = state.image.states.get(id);
  if (!cameraState || cameraState.cameraConnectInFlight) return;
  cameraState.img.setAttribute('referrerpolicy', 'no-referrer');
  cameraState.img.decoding = 'async';

  if (cameraState.config.transport === 'mjpeg') {
    startMjpegStream(id);
    return;
  }
  if (cameraState.config.transport === 'latest-jpeg') {
    void startLatestJpegStream(id);
    return;
  }
  if (cameraState.config.transport === 'unified-webrtc') {
    setCameraLatestJpeg(cameraState, 'waiting for real-time WebRTC video');
    if (
      state.image.unifiedStream
      && state.image.unifiedPeer === state.rtcPeer
      && state.image.unifiedGeneration === state.realtimeGeneration
    ) {
      attachUnifiedCameraStream(
        state.rtcPeer,
        state.realtimeGeneration,
        state.image.unifiedStream
      );
    }
    return;
  }

  cancelCameraReconnect(cameraState);
  cameraState.cameraConnectInFlight = true;
  const generation = ++cameraState.cameraConnectionGeneration;
  const oldPeer = cameraState.peerConnection;
  cameraState.peerConnection = null;
  if (oldPeer) {
    try { oldPeer.close(); } catch (_) { /* already closed */ }
  }

  try {
    cameraState.transportMode = 'webrtc-connecting';
    cameraState.cameraConnectStartedAt = performance.now();
    updateCameraPanel(id);
    const peer = new RTCPeerConnection({ iceServers: [] });
    cameraState.peerConnection = peer;
    peer.addTransceiver('video', { direction: 'recvonly' });
    peer.ontrack = async event => {
      if (
        cameraState.peerConnection !== peer
        || cameraState.cameraConnectionGeneration !== generation
      ) return;
      const stream = event.streams[0] || new MediaStream([event.track]);
      cameraState.video.srcObject = stream;
      cameraState.video.onloadedmetadata = () => {
        applyVideoTexture(id);
        updateCameraPanel(id);
      };
      try {
        await cameraState.video.play();
      } catch (playError) {
        console.warn(`Autoplay retry required for ${id}:`, playError);
      }
      if (
        cameraState.peerConnection !== peer
        || cameraState.cameraConnectionGeneration !== generation
      ) return;
      cameraState.transportMode = 'webrtc';
      cameraState.cameraConnectedAt = performance.now();
      cameraState.lastVideoFrameAt = cameraState.cameraConnectedAt;
      cameraState.fallbackReason = '';
      cameraState.cameraReconnectAttempt = 0;
      applyVideoTexture(id);
      updateCameraPanel(id);
      pumpIndependentVideoFrames(cameraState, peer, generation);
    };
    peer.onconnectionstatechange = () => {
      if (
        cameraState.peerConnection !== peer
        || cameraState.cameraConnectionGeneration !== generation
      ) return;
      if (peer.connectionState === 'connected') {
        cancelCameraReconnect(cameraState);
      } else if (['failed', 'closed'].includes(peer.connectionState)) {
        scheduleCameraReconnect(cameraState, `video ${peer.connectionState}`);
      } else if (peer.connectionState === 'disconnected') {
        scheduleCameraReconnect(cameraState, 'video temporarily disconnected');
      }
    };

    const offer = await peer.createOffer();
    await peer.setLocalDescription(offer);
    await waitForIceGatheringComplete(peer);

    const response = await fetch('/api/camera/webrtc/offer', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        camera_id: id,
        sdp: peer.localDescription.sdp,
        type: peer.localDescription.type
      })
    });
    const answer = await response.json();
    if (!response.ok || !answer.sdp) throw new Error(answer.error || `WebRTC offer failed for ${id}`);
    if (
      cameraState.peerConnection !== peer
      || cameraState.cameraConnectionGeneration !== generation
    ) return;
    await peer.setRemoteDescription(answer);
  } catch (error) {
    if (cameraState.cameraConnectionGeneration !== generation) return;
    console.warn(`Camera WebRTC startup failed for ${id}; reconnecting:`, error);
    scheduleCameraReconnect(cameraState, `video setup failed: ${error.message}`);
  } finally {
    if (cameraState.cameraConnectionGeneration === generation) {
      cameraState.cameraConnectInFlight = false;
    }
  }
}

function createAxisPart(type, attributes) {
  const el = document.createElement(type);
  Object.entries(attributes).forEach(([key, value]) => el.setAttribute(key, value));
  return el;
}

function addControllerAxes(handEl) {
  if (!handEl || handEl.querySelector('[data-telegrip-axis="true"]')) return;

  const axisLength = 0.11;
  const radius = 0.004;
  const tipHeight = 0.025;
  const tipRadius = 0.011;
  const axes = [
    {
      color: '#ff3b30',
      cylinder: { position: `${axisLength / 2} 0 0`, rotation: '0 0 90' },
      cone: { position: `${axisLength} 0 0`, rotation: '0 0 90' }
    },
    {
      color: '#34c759',
      cylinder: { position: `0 ${axisLength / 2} 0`, rotation: '0 0 0' },
      cone: { position: `0 ${axisLength} 0`, rotation: '0 0 0' }
    },
    {
      color: '#0a84ff',
      cylinder: { position: `0 0 ${axisLength / 2}`, rotation: '90 0 0' },
      cone: { position: `0 0 ${axisLength}`, rotation: '90 0 0' }
    }
  ];

  axes.forEach(axis => {
    const cylinder = createAxisPart('a-cylinder', {
      'data-telegrip-axis': 'true',
      height: axisLength,
      radius,
      color: axis.color,
      position: axis.cylinder.position,
      rotation: axis.cylinder.rotation
    });
    const cone = createAxisPart('a-cone', {
      'data-telegrip-axis': 'true',
      height: tipHeight,
      'radius-bottom': tipRadius,
      'radius-top': 0,
      color: axis.color,
      position: axis.cone.position,
      rotation: axis.cone.rotation
    });
    handEl.appendChild(cylinder);
    handEl.appendChild(cone);
  });
}

function renderCameraFrames() {
  if (!state.image.enabled) return;

  state.image.states.forEach((_, id) => drawCameraFrame(id));

  window.requestAnimationFrame(renderCameraFrames);
}

async function setupOptionalCameraPanels() {
  if (!state.image.enabled) {
    setStatus('ZMQ image display disabled; VR data ready');
    return;
  }

  const cameras = state.image.cameras.filter(camera => camera.enabled !== false);
  createCameraModeIndicator();
  cameras.forEach(camera => {
    createHiddenImageState(camera);
    createCameraPanel(camera);
  });

  await Promise.all(cameras.map(async camera => {
    const id = cameraId(camera);
    await refreshCameraStatus(id);
    await startCameraFeed(id);
  }));
  startCameraWatchdog();

  if (!state.image.renderStarted) {
    state.image.renderStarted = true;
    window.requestAnimationFrame(renderCameraFrames);
  }

  updateCameraModeUi();

  window.setInterval(() => {
    cameras.forEach(camera => refreshCameraStatus(cameraId(camera)));
  }, 5000);
  if (!state.image.clientStatusTimer) {
    state.image.clientStatusTimer = window.setInterval(
      reportCameraClientStatus, 1000
    );
    void reportCameraClientStatus();
  }
}

function controllerData(handEl, hand, buttons, webxrPose = null) {
  const payload = {
    hand,
    position: null,
    rotation: null,
    quaternion: null,
    gripActive: state.teleop.desktopModeRequestPending ? false : buttons.grip,
    gripValue: Math.min(1, Math.max(0, Number(buttons.gripValue) || 0)),
    trigger: Math.min(1, Math.max(0, Number(buttons.trigger) || 0)),
    thumbstick: buttons.thumbstick || { x: 0, y: 0, pressed: 0 }
  };

  if (hand === 'left') {
    payload.xButton = buttons.x ? 1 : 0;
    payload.yButton = buttons.y ? 1 : 0;
  } else {
    payload.aButton = buttons.a ? 1 : 0;
    payload.bButton = buttons.b ? 1 : 0;
  }

  if (webxrPose) {
    payload.position = webxrPose.position;
    payload.rotation = webxrPose.rotation;
    payload.quaternion = webxrPose.quaternion;
    return payload;
  }

  if (!handEl?.object3D?.visible) return payload;

  const pos = handEl.object3D.position;
  const rot = handEl.object3D.rotation;
  const quat = handEl.object3D.quaternion;

  payload.position = { x: pos.x, y: pos.y, z: pos.z };
  payload.rotation = {
    x: THREE.MathUtils.radToDeg(rot.x),
    y: THREE.MathUtils.radToDeg(rot.y),
    z: THREE.MathUtils.radToDeg(rot.z)
  };
  payload.quaternion = { x: quat.x, y: quat.y, z: quat.z, w: quat.w };
  return payload;
}

AFRAME.registerComponent('telegrip-vr-bridge', {
  init: function () {
    this.leftHand = document.getElementById('leftHand');
    this.rightHand = document.getElementById('rightHand');
    this.leftButtons = { grip: false, gripValue: 0, trigger: 0, x: false, y: false, thumbstick: { x: 0, y: 0, pressed: 0 } };
    this.rightButtons = { grip: false, gripValue: 0, trigger: 0, a: false, b: false, thumbstick: { x: 0, y: 0, pressed: 0 } };
    this.rightBPressedAt = null;
    this.desktopChordStartedAt = null;
    this.desktopChordConsumed = false;

    this.el.renderer.xr.addEventListener('sessionstart', () => {
      this.releaseAllControllerButtons();
      this.resetDesktopModeChord();
      state.xrSession = this.el.renderer.xr.getSession();
      ensureVrNotice();
      ensureHgStatusHud();
      updateHgStatusHud(state.teleop.status?.hg_dagger, { forceCurrent: true });
      setCameraMode('passthrough');
      setStatus('VR 已进入，正在检测手柄实时姿态。');
      attachRigToCamera();
      // Keep the operator's VR view clean by default.  The diagnostic HUD can
      // still be enabled explicitly in the fetched VR configuration.
      if (state.config?.vr?.show_hud === true) {
        createBrandHud();
        createTextHud();
      }
    });
    this.el.renderer.xr.addEventListener('sessionend', () => {
      this.releaseAllControllerButtons();
      this.resetDesktopModeChord();
      state.xrSession = null;
      if (state.hgStatusHud) state.hgStatusHud.mesh.visible = false;
      if (desktopModeEnabledFromStatus()) {
        setStatus(
          '头显已退出沉浸模式；机器人保持当前位置。重新进入 VR 后可继续接管。'
        );
      }
      if (state.teleop.enableAfterFreshVr) {
        cancelDeferredHardwareEnable('VR 会话已退出，真机未使能。');
        renderHardwareControl();
      }
    });

    this.bindControllerEvents(this.leftHand, 'left', this.leftButtons);
    this.bindControllerEvents(this.rightHand, 'right', this.rightButtons);

    if (state.config?.vr?.controller_axes?.enabled !== false) {
      addControllerAxes(this.leftHand);
      addControllerAxes(this.rightHand);
    }
  },

  sendButtonEvent: function (hand, button, pressed) {
    sendReliablePacket({
      type: pressed ? 'button_press' : 'button_release',
      hand,
      button,
      pressed,
      timestamp: Date.now()
    });
  },

  sendReleaseEvent: function (hand, releaseKey) {
    sendReliablePacket({
      hand,
      [releaseKey]: true
    });
  },

  releaseControllerButtons: function (buttons, hand) {
    // Reliable release events unwind mapper state even when there will be no
    // following XR pose frame (session end or an input source disappearing).
    this.sendReleaseEvent(hand, 'gripReleased');
    this.sendReleaseEvent(hand, 'triggerReleased');
    if (hand === 'left') {
      this.sendButtonEvent('left', 'X', false);
      this.sendButtonEvent('left', 'Y', false);
    } else {
      this.sendButtonEvent('right', 'A', false);
      this.sendButtonEvent('right', 'B', false);
      this.rightBPressedAt = null;
    }
    this.clearControllerButtons(buttons, hand);
  },

  releaseAllControllerButtons: function () {
    this.releaseControllerButtons(this.leftButtons, 'left');
    this.releaseControllerButtons(this.rightButtons, 'right');
  },

  resetDesktopModeChord: function () {
    this.desktopChordStartedAt = null;
    this.desktopChordConsumed = false;
  },

  handleCameraModeBDown: function () {
    if (this.rightBPressedAt === null) {
      this.rightBPressedAt = performance.now();
    }
  },

  handleCameraModeBUp: function () {
    const startedAt = this.rightBPressedAt;
    this.rightBPressedAt = null;
    // B is shared with HG-DAgger policy stop. Preserve the legacy camera
    // switch only for an unambiguous short tap; a deliberate hold is reserved
    // exclusively for stopping VLA inference.
    if (startedAt !== null && performance.now() - startedAt < 700) {
      toggleCameraMode();
    }
  },

  checkDesktopModeGesture: function (tracked) {
    const bothTracked = Boolean(tracked?.left && tracked?.right);
    const bothPressed = Boolean(
      this.leftButtons.thumbstick?.pressed
      && this.rightButtons.thumbstick?.pressed
    );
    if (!bothTracked || !bothPressed) {
      this.resetDesktopModeChord();
      return;
    }
    if (this.desktopChordConsumed || state.teleop.desktopModeRequestPending) {
      return;
    }
    const now = performance.now();
    if (this.desktopChordStartedAt === null) {
      this.desktopChordStartedAt = now;
      showVrNotice(
        '继续按住左右摇杆按键\n1 秒后切换桌面双臂模式',
        1800,
        '#FFD166'
      );
      return;
    }
    if (now - this.desktopChordStartedAt < 1000) return;
    this.desktopChordConsumed = true;
    // Stop both arms before the HTTP request crosses threads into ROS. The
    // mapper service repeats the same hold and starts a new clutch session.
    this.releaseAllControllerButtons();
    void requestDesktopMode(!desktopModeEnabledFromStatus());
  },

  bindControllerEvents: function (handEl, hand, buttons) {
    if (!handEl) return;
    handEl.addEventListener('gripdown', () => { buttons.grip = true; });
    handEl.addEventListener('gripup', () => {
      buttons.grip = false;
      buttons.gripValue = 0;
      // Reliable notification starts the mapper's short debounce window.  A
      // following active realtime frame cancels it, so a browser threshold
      // glitch cannot tear down the clutch anchor.
      this.sendReleaseEvent(hand, 'gripReleased');
    });
    handEl.addEventListener('triggerdown', () => { buttons.trigger = 1; });
    handEl.addEventListener('triggerup', () => {
      buttons.trigger = 0;
      this.sendReleaseEvent(hand, 'triggerReleased');
    });

    if (hand === 'left') {
      handEl.addEventListener('xbuttondown', () => {
        buttons.x = true;
        this.sendButtonEvent('left', 'X', true);
      });
      handEl.addEventListener('xbuttonup', () => {
        buttons.x = false;
        this.sendButtonEvent('left', 'X', false);
      });
      handEl.addEventListener('ybuttondown', () => {
        buttons.y = true;
        this.sendButtonEvent('left', 'Y', true);
      });
      handEl.addEventListener('ybuttonup', () => {
        buttons.y = false;
        this.sendButtonEvent('left', 'Y', false);
      });
    } else {
      handEl.addEventListener('abuttondown', () => {
        buttons.a = true;
        this.sendButtonEvent('right', 'A', true);
      });
      handEl.addEventListener('abuttonup', () => {
        buttons.a = false;
        this.sendButtonEvent('right', 'A', false);
      });
      handEl.addEventListener('bbuttondown', () => {
        if (!buttons.b) this.handleCameraModeBDown();
        buttons.b = true;
        this.sendButtonEvent('right', 'B', true);
      });
      handEl.addEventListener('bbuttonup', () => {
        if (buttons.b) this.handleCameraModeBUp();
        buttons.b = false;
        this.sendButtonEvent('right', 'B', false);
      });
    }
  },

  updateThumbsticks: function () {
    if (!state.xrSession) return { left: false, right: false };
    const deadzone = 0.05;
    const seen = { left: false, right: false };

    for (const source of state.xrSession.inputSources) {
      if (!source.gamepad || !source.handedness) continue;
      if (source.handedness === 'left' || source.handedness === 'right') {
        seen[source.handedness] = true;
      }
      const axes = source.gamepad.axes || [];
      const buttons = source.gamepad.buttons || [];
      const x = Math.abs(axes[2] || 0) < deadzone ? 0 : axes[2] || 0;
      const y = Math.abs(axes[3] || 0) < deadzone ? 0 : axes[3] || 0;
      if (source.handedness === 'left') {
        this.leftButtons.trigger = Number.isFinite(buttons[0]?.value)
          ? buttons[0].value
          : (buttons[0]?.pressed ? 1 : 0);
        this.updateGripState(this.leftButtons, buttons[1]);
        this.leftButtons.x = Boolean(buttons[4]?.pressed);
        this.leftButtons.y = Boolean(buttons[5]?.pressed);
        this.leftButtons.thumbstick = {
          x,
          y,
          pressed: (buttons[3]?.pressed || (!buttons[3] && buttons[2]?.pressed)) ? 1 : 0
        };
      }
      if (source.handedness === 'right') {
        this.rightButtons.trigger = Number.isFinite(buttons[0]?.value)
          ? buttons[0].value
          : (buttons[0]?.pressed ? 1 : 0);
        this.updateGripState(this.rightButtons, buttons[1]);
        this.rightButtons.a = Boolean(buttons[4]?.pressed);
        const previousB = this.rightButtons.b;
        const nextB = Boolean(buttons[5]?.pressed);
        if (nextB && !previousB) this.handleCameraModeBDown();
        if (!nextB && previousB) this.handleCameraModeBUp();
        this.rightButtons.b = nextB;
        this.rightButtons.thumbstick = {
          x,
          y,
          pressed: (buttons[3]?.pressed || (!buttons[3] && buttons[2]?.pressed)) ? 1 : 0
        };
      }
    }
    // WebXR can temporarily remove an input source while the render loop keeps
    // running. Never carry X/A or Grip from the last visible frame into that
    // gap; pose tracking recovery will re-establish the current state.
    if (!seen.left && this.controllerButtonsActive(this.leftButtons, 'left')) {
      this.releaseControllerButtons(this.leftButtons, 'left');
    }
    if (!seen.right && this.controllerButtonsActive(this.rightButtons, 'right')) {
      this.releaseControllerButtons(this.rightButtons, 'right');
    }
    return seen;
  },

  controllerButtonsActive: function (buttons, hand) {
    return Boolean(
      buttons.grip
      || buttons.gripValue
      || buttons.trigger
      || buttons.thumbstick?.pressed
      || (hand === 'left' ? (buttons.x || buttons.y) : (buttons.a || buttons.b))
    );
  },

  clearControllerButtons: function (buttons, hand) {
    buttons.grip = false;
    buttons.gripValue = 0;
    buttons.trigger = 0;
    buttons.thumbstick = { x: 0, y: 0, pressed: 0 };
    if (hand === 'left') {
      buttons.x = false;
      buttons.y = false;
    } else {
      buttons.a = false;
      buttons.b = false;
    }
  },

  updateGripState: function (stateButtons, gripButton) {
    // Quest exposes the side squeeze as an analogue value.  Browser
    // `pressed` can flicker around its single threshold, so use a Schmitt
    // trigger: squeeze firmly to engage, but do not release until it is
    // substantially relaxed.  The mapper adds a short second-stage debounce.
    const value = Number.isFinite(gripButton?.value)
      ? Math.min(1, Math.max(0, gripButton.value))
      : (gripButton?.pressed ? 1 : 0);
    stateButtons.gripValue = value;
    const engageThreshold = 0.55;
    const releaseThreshold = 0.25;
    stateButtons.grip = stateButtons.grip
      ? value > releaseThreshold
      : value >= engageThreshold;
  },

  inputSourceForHand: function (hand) {
    if (!state.xrSession) return null;
    return Array.from(state.xrSession.inputSources || []).find(source => source.handedness === hand) || null;
  },

  webxrPoseForHand: function (hand) {
    const source = this.inputSourceForHand(hand);
    const frame = this.el.renderer.xr.getFrame?.();
    const referenceSpace = this.el.renderer.xr.getReferenceSpace?.();
    if (!source || !frame || !referenceSpace) return null;
    // Some WebXR runtimes keep gripSpace registered while temporarily
    // returning no grip pose, even though targetRaySpace remains valid.
    // Try both spaces before declaring the controller untracked.
    const poseSpaces = [source.gripSpace, source.targetRaySpace].filter(Boolean);
    let pose = null;
    for (const poseSpace of poseSpaces) {
      pose = frame.getPose(poseSpace, referenceSpace);
      if (pose) break;
    }
    if (!pose) return null;
    const position = pose.transform.position;
    const quaternion = pose.transform.orientation;
    const euler = new THREE.Euler().setFromQuaternion(
      new THREE.Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
      'XYZ'
    );
    return {
      position: { x: position.x, y: position.y, z: position.z },
      rotation: {
        x: THREE.MathUtils.radToDeg(euler.x),
        y: THREE.MathUtils.radToDeg(euler.y),
        z: THREE.MathUtils.radToDeg(euler.z)
      },
      quaternion: { x: quaternion.x, y: quaternion.y, z: quaternion.z, w: quaternion.w }
    };
  },

  controllerForHand: function (hand, handEl, buttons) {
    return controllerData(handEl, hand, buttons, this.webxrPoseForHand(hand));
  },

  inputSourceSummary: function () {
    const sources = Array.from(state.xrSession?.inputSources || []);
    if (!sources.length) return 'none';
    return sources.map(source => {
      const profile = source.profiles?.[0] || (source.hand ? 'hand-tracking' : 'unknown');
      return `${source.handedness || 'unknown'}:${profile}`;
    }).join(', ');
  },

  webxrHeadPose: function () {
    const frame = this.el.renderer.xr.getFrame?.();
    const referenceSpace = this.el.renderer.xr.getReferenceSpace?.();
    if (!frame || !referenceSpace || typeof frame.getViewerPose !== 'function') return null;
    const viewerPose = frame.getViewerPose(referenceSpace);
    if (!viewerPose?.transform) return null;
    const position = viewerPose.transform.position;
    const quaternion = viewerPose.transform.orientation;
    const values = [
      position?.x, position?.y, position?.z,
      quaternion?.x, quaternion?.y, quaternion?.z, quaternion?.w
    ];
    if (!values.every(Number.isFinite)) return null;
    const euler = new THREE.Euler().setFromQuaternion(
      new THREE.Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
      'XYZ'
    );
    return {
      position: { x: position.x, y: position.y, z: position.z },
      rotation: {
        x: THREE.MathUtils.radToDeg(euler.x),
        y: THREE.MathUtils.radToDeg(euler.y),
        z: THREE.MathUtils.radToDeg(euler.z)
      },
      quaternion: { x: quaternion.x, y: quaternion.y, z: quaternion.z, w: quaternion.w },
      source: 'webxr_viewer_pose'
    };
  },

  headData: function (leftController, rightController) {
    // In immersive mode the A-Frame camera's local Euler angles can stay at
    // zero even while the headset moves. Prefer the native WebXR viewer pose.
    const viewerHead = this.webxrHeadPose();
    if (viewerHead) {
      if (state.headText) {
        const serverReady = state.websocket?.readyState === WebSocket.OPEN;
        state.headText.setAttribute(
          'value',
          `AUTOLIFE 306 V4 / QUEST MIDDLE\n` +
          `Server: ${serverReady ? 'CONNECTED' : 'DISCONNECTED'} | Head: WEBXR READY\n` +
          `Head Follow: ${headFollowEnabledFromStatus() ? 'ON' : 'OFF'} | Pose: ${state.teleop.xrPoseFresh ? 'LIVE' : 'STALE'}\n` +
          `Inputs: ${this.inputSourceSummary()}\n` +
          `Head Pos: ${viewerHead.position.x.toFixed(2)} ${viewerHead.position.y.toFixed(2)} ${viewerHead.position.z.toFixed(2)}\n` +
          `RESET ARMS: hold LEFT X + RIGHT A for 1.0s`
        );
      }
      return viewerHead;
    }

    const headObject = this.el.camera?.el?.object3D;
    if (!headObject) return { position: null, rotation: null, quaternion: null };

    const pos = headObject.position;
    const rot = headObject.rotation;
    const quat = headObject.quaternion;
    const head = {
      position: { x: pos.x, y: pos.y, z: pos.z },
      rotation: {
        x: THREE.MathUtils.radToDeg(rot.x),
        y: THREE.MathUtils.radToDeg(rot.y),
        z: THREE.MathUtils.radToDeg(rot.z)
      },
      quaternion: { x: quat.x, y: quat.y, z: quat.z, w: quat.w }
    };

    if (state.headText) {
      const serverReady = state.websocket?.readyState === WebSocket.OPEN;
      const leftReady = Boolean(leftController?.position);
      const rightReady = Boolean(rightController?.position);
      state.headText.setAttribute(
        'value',
        `AUTOLIFE 306 V4 / QUEST MIDDLE\n` +
        `Server: ${serverReady ? 'CONNECTED' : 'DISCONNECTED'} | ` +
        `Left: ${leftReady ? 'READY' : 'MISSING'} | Right: ${rightReady ? 'READY' : 'MISSING'}\n` +
        `Head Follow: ${headFollowEnabledFromStatus() ? 'ON' : 'OFF'} | Pose: ${state.teleop.xrPoseFresh ? 'LIVE' : 'STALE'}\n` +
        `Inputs: ${this.inputSourceSummary()}\n` +
        `Head Pos: ${head.position.x.toFixed(2)} ${head.position.y.toFixed(2)} ${head.position.z.toFixed(2)}\n` +
        `Grip L:${this.leftButtons.grip ? 'ON' : 'OFF'} R:${this.rightButtons.grip ? 'ON' : 'OFF'} ` +
        `Trigger L:${this.leftButtons.trigger.toFixed(2)} R:${this.rightButtons.trigger.toFixed(2)}\n` +
        `RESET ARMS: hold LEFT X + RIGHT A for 1.0s`
      );
    }
    return head;
  },

  tick: function () {
    if (!state.xrSession) return;

    const tracked = this.updateThumbsticks();
    this.checkDesktopModeGesture(tracked);
    const leftController = this.controllerForHand('left', this.leftHand, this.leftButtons);
    const rightController = this.controllerForHand('right', this.rightHand, this.rightButtons);
    const packet = {
      packetType: 'pose',
      timestamp: Date.now(),
      head: this.headData(leftController, rightController),
      leftController,
      rightController
    };
    sendRealtimePacket(packet);
  }
});

async function enterVr() {
  const scene = document.querySelector('a-scene');
  if (!scene) return;

  const button = document.getElementById('startVrButton');
  if (button) {
    button.disabled = true;
    button.textContent = 'Starting...';
  }

  try {
    await scene.enterVR(true);
  } catch (error) {
    console.error('Failed to enter VR:', error);
    setStatus(`Failed to enter VR: ${error.message}`);
    if (state.teleop.enableAfterFreshVr) {
      cancelDeferredHardwareEnable(`进入 VR 失败：${error.message}`);
      renderHardwareControl();
    }
    if (button) {
      button.disabled = false;
      button.textContent = 'Start VR';
    }
  }
}

async function init() {
  setServerUrl();
  setStatus('Loading configuration...');
  await loadConfig();

  const scene = document.querySelector('a-scene');
  if (scene.hasLoaded) {
    scene.setAttribute('telegrip-vr-bridge', '');
  } else {
    scene.addEventListener('loaded', () => scene.setAttribute('telegrip-vr-bridge', ''), { once: true });
  }

  connectWebSocket();
  startRealtimeWatchdog();
  setupHardwareControl();
  startDesktopKeepalive();
  attachRigToCamera();
  createBrandHud();
  await setupOptionalCameraPanels();

  const cameraModeButton = document.getElementById('cameraModeButton');
  if (cameraModeButton) cameraModeButton.addEventListener('click', toggleCameraMode);

  const startButton = document.getElementById('startVrButton');
  if (startButton) startButton.addEventListener('click', enterVr);

  scene.addEventListener('enter-vr', () => {
    const launchPanel = document.getElementById('launchPanel');
    if (launchPanel) launchPanel.style.display = 'none';
  });
  scene.addEventListener('exit-vr', () => {
    const launchPanel = document.getElementById('launchPanel');
    const startButton = document.getElementById('startVrButton');
    if (launchPanel) launchPanel.style.display = 'flex';
    if (startButton) {
      startButton.disabled = false;
      startButton.textContent = 'Start VR';
    }
  });
}

document.addEventListener('visibilitychange', () => {
  if (
    document.visibilityState === 'visible'
    && desktopModeEnabledFromStatus()
  ) {
    connectWebSocket();
    void acquireDesktopWakeLock();
  }
});

document.addEventListener('DOMContentLoaded', () => {
  init().catch(error => {
    console.error('Telegrip startup failed:', error);
    setStatus(`Startup failed: ${error.message}`);
  });
});
