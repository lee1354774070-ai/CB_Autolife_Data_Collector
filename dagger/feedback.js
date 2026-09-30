// Same WebXR actuator path as the inspected colleague's app. Receipt events
// arrive over one SSE stream, independent of pose transport and control RPCs.
(() => {
  const seen = new Set();
  const labels = {start: 'Recording started', mark: 'Subtask marked', saving: 'Saving...',
    save: 'Episode saved', discarding: 'Discarding...', discard: 'Episode discarded',
    resetting: 'Resetting: release both Grips', reset: 'Reset confirmed',
    takeover: 'Human control selected', cancel: 'Cancelled', error: 'Error: check terminal'};
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
      pulseVrControllers(0.65, duration);
      await delay(duration + 90);
    }
  }
  const events = new EventSource('/collector_events');
  events.onmessage = message => {
    try {
      const event = JSON.parse(message.data);
      if (!event || typeof event.id !== 'string' || seen.has(event.id)) return;
      seen.add(event.id);
      if (seen.size > 128) seen.delete(seen.values().next().value);
      if (labels[event.event]) showVrNotice(labels[event.event], 2000,
        event.event === 'error' ? '#FF6B6B' : '#7DFFCF');
      if (state.xrSession) void pulse(event);
    } catch (_) { /* Optional feedback must not interrupt teleoperation. */ }
  };
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
