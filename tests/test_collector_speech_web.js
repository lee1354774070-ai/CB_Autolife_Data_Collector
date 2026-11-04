'use strict';
const assert = require('node:assert/strict');
const {SpeechCursor} = require('../web/collector_speech/app.js');
const cursor = new SpeechCursor(0), spoken = [];
for (let i=1;i<=8;i++) {
  const item = {kind:'speech',seq:i,text:'下一步'};
  if (cursor.accept(item)) spoken.push(item.text);
  assert.equal(cursor.accept(item), false);
}
for (let i=9;i<=10;i++) {
  const item = {kind:'speech',seq:i,text:`第${i+30}条已保存`};
  if (cursor.accept(item)) spoken.push(item.text);
}
assert.equal(spoken.length, 10);
assert.deepEqual(spoken.slice(-2),['第39条已保存','第40条已保存']);
assert.throws(()=>cursor.accept({kind:'speech',seq:12,text:'保存成功'}),/不连续/);
assert.throws(()=>cursor.accept({kind:'error',seq:10,text:'数据源中断'}),/数据源中断/);
assert.throws(()=>cursor.accept({kind:'speech',seq:11,text:''}),/文字为空/);
assert.equal(cursor.accept({kind:'ended',seq:11}),true);
console.log('PASS browser event sequence: repeated marks, distinct saves, dedup, gaps, errors, and EOF');
