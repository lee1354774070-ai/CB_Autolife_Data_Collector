'use strict';

// Event identity, never text similarity, determines whether a cue is new.
class SpeechCursor {
  constructor(sequence) { this.sequence = sequence; }
  accept(item) {
    if (!item || !Number.isSafeInteger(item.seq) || item.seq < 0) throw new Error('无效的状态事件');
    if (item.kind === 'error') throw new Error(item.text || '采集状态连接异常');
    if (item.seq <= this.sequence) return false;
    if (item.seq !== this.sequence + 1) throw new Error('状态事件不连续，请检查终端后重新开启播报');
    if (!['speech', 'connected', 'ended'].includes(item.kind)) throw new Error('未知的状态事件');
    if (item.kind === 'speech' && (typeof item.text !== 'string' || !item.text)) throw new Error('播报文字为空');
    this.sequence = item.seq;
    return true;
  }
}

function mount() {
  const $ = id => document.getElementById(id);
  const token = location.hash.slice(1);
  let source = null, active = false, run = 0, voices = [];
  const synth = window.speechSynthesis;
  function status(text, bad = false) {
    $('hint').textContent = text;
    $('hint').className = bad ? 'hint error' : 'hint';
    $('badge').textContent = bad ? '需要检查' : active ? '正在播报' : '等待开启';
    $('badge').className = bad ? 'badge error' : active ? 'badge live' : 'badge';
  }
  function close(cancel = true) {
    active = false; run++;
    if (source) source.close();
    source = null;
    if (cancel && synth) synth.cancel();
    $('start').disabled = false; $('stop').disabled = true;
  }
  function fail(text) { close(); status(text, true); }
  function say(text) {
    const voice = voices[Number($('voice').value)];
    if (!voice) throw new Error('未找到本机中文语音，请检查电脑系统的中文语音设置');
    const utterance = new SpeechSynthesisUtterance(text);
    utterance.voice = voice; utterance.lang = voice.lang; utterance.rate = 1.1;
    utterance.onerror = event => {
      if (active && !['canceled', 'interrupted'].includes(event.error)) fail('声音播放失败：' + event.error + '。文字反馈不代表声音已播放。');
    };
    synth.speak(utterance);
  }
  function listVoices() {
    if (!synth) return;
    const chosen = voices[Number($('voice').value)]?.voiceURI;
    voices = synth.getVoices().filter(voice => voice.localService && /^zh(?:-|_)/i.test(voice.lang));
    $('voice').replaceChildren();
    voices.forEach((voice, i) => {
      const option = document.createElement('option'); option.value = String(i); option.textContent = voice.name;
      $('voice').append(option);
      if (voice.voiceURI === chosen) $('voice').value = String(i);
    });
    if (!voices.length) {
      const option = document.createElement('option'); option.textContent = '没有检测到本机中文语音'; $('voice').append(option);
    }
  }
  function display(text) {
    $('current').textContent = text;
    const line = document.createElement('li'), stamp = document.createElement('time'), body = document.createElement('span');
    stamp.textContent = new Date().toLocaleTimeString(); body.textContent = text;
    line.append(stamp, body); $('events').prepend(line);
    while ($('events').children.length > 18) $('events').lastChild.remove();
  }
  $('start').onclick = async () => {
    if (active) return;
    if (!token) { fail('链接不完整，请从采集终端复制包含 # 后内容的完整链接'); return; }
    if (!synth || !window.SpeechSynthesisUtterance) { fail('浏览器不支持语音，请使用电脑上的Chrome或Edge'); return; }
    listVoices();
    if (!voices.length) { fail('未检测到本机中文语音，请确认Chrome/Edge能使用系统中文声音'); return; }
    active = true;
    const currentRun = ++run;
    $('start').disabled = true; $('stop').disabled = false;
    status('正在连接当前采集状态…');
    try {
      synth.cancel();
      say('正在连接采集播报'); // Start in the click handler to satisfy browser activation.
      const response = await fetch('/state?token=' + encodeURIComponent(token), {cache: 'no-store'});
      if (!response.ok) throw new Error('连接未获授权或采集已退出，请使用本次采集的新链接');
      const state = await response.json();
      if (!active || currentRun !== run) return;
      if (state.closed) throw new Error('本次状态来源已结束，请检查采集终端');
      const cursor = new SpeechCursor(state.seq);
      $('task').textContent = state.base_dir || '等待采集状态源就绪';
      source = new EventSource('/events?token=' + encodeURIComponent(token) + '&after=' + state.seq);
      source.onopen = () => { if (currentRun === run) status('已连接。可以切回采集终端，请保持本页打开。'); };
      source.onmessage = event => {
        if (!active || currentRun !== run) return;
        try {
          const item = JSON.parse(event.data);
          if (!cursor.accept(item)) return;
          if (item.kind === 'speech') { display(item.text); say(item.text); }
          else if (item.kind === 'connected') $('task').textContent = item.base_dir;
          else if (item.kind === 'ended') {
            display('采集状态源已结束，请检查原终端的最终保存结果');
            say('状态来源已结束，请检查采集终端');
            close(false); status('已停止跟随。原终端的保存结果为准。');
          }
        } catch (error) { fail(error.message); }
      };
      // Do not silently reconnect and replay stale countdowns or confirmations.
      source.onerror = () => { if (currentRun === run) fail('播报连接已断开，采集状态未知。检查终端后点开启播报重新连接。'); };
    } catch (error) { if (currentRun === run) fail(error.message); }
  };
  $('stop').onclick = () => { close(); status('电脑播报已停止，采集仍由原终端控制。'); };
  window.addEventListener('pagehide', () => close());
  if (synth) { listVoices(); synth.addEventListener('voiceschanged', listVoices); }
  if (!token) status('请从采集终端打开完整的电脑播报链接', true);
}

if (typeof module !== 'undefined') module.exports = {SpeechCursor};
if (typeof document !== 'undefined') mount();
