// Same WebXR actuator path as the inspected colleague's app. Receipt events
// arrive over one SSE stream, independent of pose transport and control RPCs.
(() => {
  const seen = new Set();
  const diagnostic = state.collectorFeedback = {build: '20261107-camera-reset', received: 0,
    last_event: null, haptics: [], audio_state: 'locked'};
  const labels = {start: '开始录制', mark: '子任务已标记', saving: '后台保存中，尚未落盘',
    save: '本条已保存', discarding: '正在丢弃', discard: '本条已丢弃',
    resetting: '正在复位，请松开握持键', reset: '复位完成',
    takeover: '人工接管已生效', cancel: '已取消', error: '操作失败，请查看启动器'};
  let audio;
  function unlockAudio() {
    try {
      const Audio = window.AudioContext || window.webkitAudioContext;
      if (!audio && Audio) audio = new Audio();
      if (audio?.state === 'suspended') void audio.resume().then(() => {
        diagnostic.audio_state = audio.state;
      }).catch(() => {});
      diagnostic.audio_state = audio?.state || 'unavailable';
    } catch (_) {}
  }
  window.addEventListener('pointerdown', unlockAudio);
  function tone(event) {
    if (!audio || audio.state !== 'running') return;
    const count = event === 'save' ? 2 : 1;
    for (let i = 0; i < count; ++i) {
      const oscillator = audio.createOscillator(), gain = audio.createGain();
      const start = audio.currentTime + i * 0.18;
      oscillator.frequency.value = event === 'error' ? 260 : event === 'save' ? 880 : 550;
      gain.gain.setValueAtTime(0, start);
      gain.gain.linearRampToValueAtTime(0.07, start + 0.01);
      gain.gain.exponentialRampToValueAtTime(0.001, start + 0.12);
      oscillator.connect(gain); gain.connect(audio.destination);
      oscillator.start(start); oscillator.stop(start + 0.13);
      oscillator.onended = () => { oscillator.disconnect(); gain.disconnect(); };
    }
  }
  function speak(text, event) {
    // Browser/headset output works even when the robot speaker is unavailable.
    try {
      tone(event);
      if (!window.speechSynthesis || !window.SpeechSynthesisUtterance) return;
      const utterance = new window.SpeechSynthesisUtterance(text);
      utterance.lang = 'zh-CN'; utterance.rate = 1.1;
      window.speechSynthesis.cancel();  // Never queue stale save announcements.
      window.speechSynthesis.speak(utterance);
    } catch (_) {}
  }
  let generation = 0;
  const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
  async function pulse(event) {
    const current = ++generation;
    const session = state.xrSession;
    const durations = event.pulses_ms;
    if (!Array.isArray(durations) || durations.length > 4) return;
    for (const duration of durations) {
      if (current !== generation || !session || state.xrSession !== session) return;
      if (!Number.isFinite(duration) || duration < 1 || duration > 500) return;
      // Keep the actual browser result: receiving a takeover is not proof of vibration.
      const sources = Array.from(session.inputSources || []);
      diagnostic.haptics = sources.map(source => ({side: source.handedness, result: 'unavailable'}));
      for (let index = 0; index < sources.length; ++index) {
        const gamepad = sources[index].gamepad;
        const actuator = gamepad?.hapticActuators?.[0] || gamepad?.vibrationActuator;
        const result = diagnostic.haptics[index];
        try {
          let request;
          if (typeof actuator?.pulse === 'function') request = actuator.pulse(0.65, duration);
          else if (typeof actuator?.playEffect === 'function') request = actuator.playEffect('dual-rumble', {
            duration, strongMagnitude: 0.65, weakMagnitude: 0.39
          });
          else continue;
          result.result = 'requested';
          void Promise.resolve(request).then(value => {
            result.result = value === false || value === 'preempted' ? 'rejected' : 'completed';
          }, error => { result.result = 'failed'; result.error = String(error).slice(0, 100); });
        } catch (error) { result.result = 'failed'; result.error = String(error).slice(0, 100); }
      }
      if (!diagnostic.haptics.some(item => item.result !== 'unavailable')) {
        showVrNotice('接管已生效；浏览器未提供手柄震动，请检查头显页面。', 6000, '#FFCB6B');
        return;
      }
      await delay(duration + 90);
    }
  }
  const events = new EventSource('/collector_events');
  function receive(event, silent = false) {
    try {
      if (!event || typeof event.id !== 'string' || seen.has(event.id)) return;
      seen.add(event.id);
      if (seen.size > 128) seen.delete(seen.values().next().value);
      if (silent) return;
      diagnostic.received += 1;
      diagnostic.last_event = {id: event.id, event: event.event};
      const text = typeof event.text === 'string' ? event.text.slice(0, 200) : labels[event.event];
      if (text) showVrNotice(text, event.event === 'save' || event.event === 'error' ? 6000 : 2500,
        event.event === 'error' ? '#FF6B6B' : '#7DFFCF');
      if (text) speak(text, event.event);
      if (state.xrSession && event.event === 'takeover') void pulse(event);
    } catch (_) { /* Optional feedback must not interrupt teleoperation. */ }
  }
  events.onmessage = message => {
    try { receive(JSON.parse(message.data)); } catch (_) {}
  };
  // A dropped SSE event must not lose takeover/save confirmation. Replayed
  // status on page entry is silent; subsequent new IDs share the SSE dedup.
  if (collectorWebMode === 'dagger' && typeof renderHardwareControl === 'function') {
    const render = renderHardwareControl;
    let initialized = false;
    renderHardwareControl = function () {
      render();
      const history = state.teleop.status?.hg_dagger?.collector_compat?.feedback_events;
      if (!Array.isArray(history)) return;
      history.forEach(event => receive(event, !initialized));
      initialized = true;
    };
  }
  window.addEventListener('pagehide', () => { ++generation; events.close(); });
  if (collectorWebMode !== 'dagger') {
    document.title = `CB Collector | ${collectorWebMode}`;
    const guide = document.querySelector('.dagger-guide');
    const text = '按住 GL+GR：A 开始，B 保存，X 仅复位，Y 仅丢弃。'
      + '先保存或丢弃，再按 X 并松开双握持键复位；终端退出。'
      + (collectorWebMode === 'subtask' ? 'B 短按标记，最后一次标记自动保存；B 长按提前保存。' : 'B 无长按功能。');
    if (guide) guide.textContent = text;
    const title = document.querySelector('.title');
    if (title) title.textContent = document.title;
  }
})();
