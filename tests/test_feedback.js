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
    state: {xrSession: {inputSources:[{handedness:'right',gamepad:{hapticActuators:[{pulse:(...args)=>pulses.push(args)}]}}]}}, collectorWebMode: 'dagger',
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
    panelLayout() {}, updateCameraPanel() {}, setCameraMode() {}, applyCanvasTexture() {}, HG_DAGGER_MODE_LABELS: {}, state: {teleop: {}},
    collectorRobotTitle: 'test', document: {title: 'test', querySelector: () => null},
    renderHardwareControl() {}, updateHgStatusHud() {}, drawHgPanelBackground() {},
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


test('controller failure is shown before the subsequent recording failure', () => {
  const context = vm.createContext({
    panelLayout() {}, updateCameraPanel() {}, setCameraMode() {}, applyCanvasTexture() {},
    HG_DAGGER_MODE_LABELS: {}, state: {teleop: {}}, collectorRobotTitle: 'test',
    document: {title: 'test', querySelector: () => null}, renderHardwareControl() {}, updateHgStatusHud() {}
  });
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../dagger/ui_overrides.js'), 'utf8'), context);
  context.status = {mode: 'ESTOP', controller_reason: 'tracking error 23.7 deg exceeds limit',
    trial: {invalid_reason: 'missing_stale_or_noncausal_action'}};
  const shown = vm.runInContext('hgDaggerVisualState(status)', context);
  assert.match(shown.label, /^控制停止：tracking error.*本条录制失效：missing_stale/);
  assert.equal(shown.color, '#FF6B6B');
});

test('hardware enabled never masquerades as expert control; VR entry stays available', () => {
  const nodes = Object.fromEntries(['hardwareControl','hardwareStatusLabel','hardwareStatusDetail','startVrButton']
    .map(id => [id, {classList: {remove() {}, add(value) { this.value = value; }}}]));
  const state = {teleop: {status: {hardware_enabled: true, hg_dagger: {mode:'POLICY_ACTIVE'}}},
                 websocket:{readyState:1}, xrSession:null};
  const context = vm.createContext({panelLayout() {}, updateCameraPanel() {}, setCameraMode() {}, applyCanvasTexture() {}, HG_DAGGER_MODE_LABELS:{}, state, WebSocket:{OPEN:1},
    collectorRobotTitle:'test', document:{title:'test',querySelector:()=>null,getElementById:id=>nodes[id]},
    updateHgStatusHud(){},renderHardwareControl(){},vrInputIsFresh:()=>true});
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../dagger/ui_overrides.js'),'utf8'),context);
  vm.runInContext('renderHardwareControl()',context);
  assert.match(nodes.hardwareStatusLabel.textContent,/模型控制/);
  assert.notEqual(nodes.hardwareControl.classList.value,'is-on');
  assert.match(nodes.hardwareStatusDetail.textContent,/尚未进入/);
  assert.equal(nodes.startVrButton.disabled,false);
  state.xrSession={};state.teleop.status.hg_dagger.mode='EXPERT_ACTIVE';
  vm.runInContext('renderHardwareControl()',context);
  assert.equal(nodes.hardwareControl.classList.value,'is-on');
  state.websocket=null;
  vm.runInContext('renderHardwareControl()',context);
  assert.notEqual(nodes.hardwareControl.classList.value,'is-on');
  assert.match(nodes.hardwareStatusDetail.textContent,/未连接/);
});


test('status backs up dropped SSE; reconnect is silent and save speaks once', async () => {
  let stream;
  const pulses = [], spoken = [], notices = [];
  const state = {xrSession:{inputSources:[{handedness:'right',gamepad:{hapticActuators:[{pulse:(...args)=>pulses.push(args)}]}}]}, teleop:{status:{hg_dagger:{collector_compat:{feedback_events:[]}}}}};
  const context = vm.createContext({state, collectorWebMode:'dagger', Set, Promise, Number, Array, JSON,
    EventSource:class {constructor(){stream=this;} close(){}},
    window:{addEventListener(){}, speechSynthesis:{cancel(){},speak:u=>spoken.push(u.text)},
      SpeechSynthesisUtterance:class {constructor(text){this.text=text;}}},
    renderHardwareControl(){}, setTimeout:fn=>fn(),
    pulseVrControllers:(...args)=>pulses.push(args), showVrNotice:text=>notices.push(text)});
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../dagger/feedback.js'),'utf8'),context);
  const history=state.teleop.status.hg_dagger.collector_compat.feedback_events;
  history.push({id:'old',event:'takeover',pulses_ms:[300,70]});
  vm.runInContext('renderHardwareControl()',context);
  assert.equal(pulses.length,0);assert.equal(spoken.length,0);
  history.push({id:'live',event:'takeover',pulses_ms:[300,70]});
  vm.runInContext('renderHardwareControl()',context);
  await new Promise(resolve=>setImmediate(resolve));
  assert.equal(pulses.length,2);
  const saved={id:'save1',event:'save',text:'第1条已保存',pulses_ms:[220]};
  stream.onmessage({data:JSON.stringify(saved)});history.push(saved);
  vm.runInContext('renderHardwareControl()',context);
  assert.equal(spoken.filter(x=>x==='第1条已保存').length,1);
  assert.equal(pulses.length,2);assert.ok(notices.includes('第1条已保存'));
});


test('browser haptic rejection is observable without interrupting the takeover notice', async () => {
  let stream; const notices=[];
  const state={xrSession:{inputSources:[{handedness:'left',gamepad:{hapticActuators:[{
    pulse:()=>Promise.reject(new Error('device disconnected'))
  }]}}]}};
  const context=vm.createContext({state,collectorWebMode:'dagger',Set,Promise,Number,Array,JSON,String,
    EventSource:class {constructor(){stream=this;}close(){}},window:{addEventListener(){}},
    setTimeout:fn=>fn(),showVrNotice:text=>notices.push(text)});
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../dagger/feedback.js'),'utf8'),context);
  stream.onmessage({data:JSON.stringify({id:'new',event:'takeover',pulses_ms:[300]})});
  await new Promise(resolve=>setImmediate(resolve));
  assert.equal(state.collectorFeedback.haptics[0].result,'failed');
  assert.match(state.collectorFeedback.haptics[0].error,/device disconnected/);
  assert.ok(notices.includes('人工接管已生效'));
});

test('XR sessionstart cannot hide head RGB or pause the actual JPEG decode loop', async () => {
  const native=fs.readFileSync(path.join(__dirname,'../dagger/assets/web/vr_app.js'),'utf8');
  let fetches=0,draws=0,uploads=0;
  const camera={config:{frame_url:'/frame'},transportMode:'latest-jpeg',frameSeq:0,drawnFrameSeq:0,
    snapshotLoopRunning:false,snapshotStopRequested:false,decodedFrameCount:0,drawnFrameCount:0,
    fpsWindowFrames:0,fpsWindowStartedAt:0,canvas:{width:640,height:480},ctx:{drawImage(){draws++;}}};
  const state={xrSession:{},teleop:{},image:{enabled:true,mode:'passthrough',states:new Map([['head',camera]])}};
  const context=vm.createContext({state,Map,Number,Math,JSON,Promise,Date,
    document:{title:'test',querySelector:()=>null},collectorRobotTitle:'test',HG_DAGGER_MODE_LABELS:{},
    panelLayout(){},updateCameraPanel(){},renderHardwareControl(){},updateHgStatusHud(){},updateCameraModeUi(){},
    performance:{now:()=>1500},console,
    fetch:async()=>{fetches++;return {status:200,ok:true,headers:{get:()=>1},blob:async()=>({})};},
    createImageBitmap:async()=>({width:640,height:480,close(){}}),
    applyCanvasTexture(){uploads++;camera.snapshotStopRequested=true;},
    delay:async()=>{throw new Error('camera polling incorrectly paused');}});
  vm.runInContext(native.slice(native.indexOf('function setCameraMode('),native.indexOf('function toggleCameraMode(')),context);
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../dagger/ui_overrides.js'),'utf8'),context);
  vm.runInContext("setCameraMode('passthrough')",context); // Exact native sessionstart call.
  assert.equal(state.image.mode,'rgbd');
  vm.runInContext(native.slice(native.indexOf('async function startLatestJpegStream('),native.indexOf('function cancelCameraReconnect(')),context);
  await vm.runInContext("startLatestJpegStream('head')",context);
  assert.equal(fetches,1);assert.equal(draws,1);assert.equal(uploads,1);
  assert.equal(camera.decodedFrameCount,1);assert.equal(camera.drawnFrameCount,1);
  assert.equal(camera.lastVideoFrameAt,1500);
});


test('head RGB stays centered and fits 4:3 or 16:9 without stretching', () => {
  const camera={canvas:{width:640,height:480}};
  const state={teleop:{},image:{states:new Map([['rgbd_head_color',camera]])}};
  const context=vm.createContext({state,Math,Number,Map,HG_DAGGER_MODE_LABELS:{},
    collectorRobotTitle:'test',document:{title:'test',querySelector:()=>null},
    panelLayout:()=>({x:1,y:2,width:.5,height:.5}),updateCameraPanel(){},
    renderHardwareControl(){},updateHgStatusHud(){},setCameraMode(){},applyCanvasTexture(){}});
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../dagger/ui_overrides.js'),'utf8'),context);
  for (const [w,h] of [[640,480],[1280,720],[480,640]]) {
    camera.canvas={width:w,height:h};
    const layout=vm.runInContext("panelLayout('rgbd_head_color')",context);
    assert.equal(layout.x,0);assert.equal(layout.y,.08);
    assert.ok(Math.abs(layout.width/layout.height-w/h)<1e-10);
    assert.ok(layout.width<=1.8 && layout.height<=1.1+1e-10);
  }
});
