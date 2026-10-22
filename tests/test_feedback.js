// Run: node --test tests/test_feedback.js. No browser or hardware required.
const test = require('node:test');
const assert = require('node:assert/strict');
const vm = require('node:vm');
const fs = require('node:fs');
const path = require('node:path');
test('only takeover vibrates; all status notices remain', async () => {
  let stream;
  const pulses = [], notices = [];
  const context = {
    Set, Promise, Number, Array, JSON,
    state: {xrSession: {}}, collectorWebMode: 'dagger',
    EventSource: class { constructor() { stream = this; } close() {} },
    window: {addEventListener() {}},
    setTimeout: callback => { callback(); },
    pulseVrControllers: (...args) => pulses.push(args),
    showVrNotice: text => notices.push(text)
  };
  vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../dagger/feedback.js'), 'utf8'), context);
  const send = event => stream.onmessage({data: JSON.stringify({
    id: event, event, pulses_ms: [300, 70]
  })});
  for (const event of ['start','mark','saving','save','discarding','discard','resetting','reset','cancel','error']) {
    send(event);
  }
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(pulses.length, 0);
  assert.equal(notices.length, 10);
  send('takeover');
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(pulses.length, 2);
  assert.equal(notices.length, 11);
  send('takeover');
  assert.equal(pulses.length, 2);
});

test('headset guide and status distinguish Y discard, failure saving and storage errors', () => {
  const lines = [];
  const context = vm.createContext({
    HG_DAGGER_MODE_LABELS: {}, state: {teleop: {}},
    collectorRobotTitle: 'test', document: {title: 'test', querySelector: () => null},
    updateHgStatusHud() {}, drawHgPanelBackground() {},
    createHgCanvasHud: () => ({context: {fillText: text => lines.push(text)}, texture: {}})
  });
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../dagger/ui_overrides.js'), 'utf8'), context);
  vm.runInContext('ensureHgControlsHud()', context);
  assert.match(lines.join(' '), /Y 短按松开：丢弃.*按住 1 秒：失败保存/);
  assert.doesNotMatch(lines.join(' '), /右摇杆/);
  for (const [text, level] of [['正在保存：任务失败', 'info'], ['本条已保存：任务失败', 'success'],
    ['本条未保存，请检查录制错误', 'error']]) {
    context.status = {mode: 'DISARMED', notice: {text, level, token: 1}};
    const shown = vm.runInContext('hgDaggerVisualState(status)', context);
    assert.equal(shown.label, text);
    assert.equal(shown.color, level === 'error' ? '#FF6B6B' : '#7DFFCF');
  }
});
