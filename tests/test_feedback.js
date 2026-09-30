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
