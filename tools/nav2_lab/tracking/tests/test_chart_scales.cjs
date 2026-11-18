'use strict';
const {test}=require('node:test');
const assert=require('node:assert/strict');
const {niceStep,scales,label}=require('../web/chart_scales.js');

test('axis ticks are numeric, ordered, and have explicit simulated seconds',()=>{
  const a=scales([{points:[
    {t:0,e:.2},{t:8.9,e:-.45},{t:13.2,e:.05}
  ]}]);
  assert.equal(a.xMin,0);
  assert.ok(a.xMax>=13.2);
  assert.ok(a.xTicks.includes(0));
  assert.ok(a.xTicks.length>=3);
  assert.ok(a.yMin<=-.45&&a.yMax>=.2);
  assert.ok(a.yTicks.some(v=>Math.abs(v)<1e-9));
  assert.ok(a.yTicks.every((v,i,arr)=>!i||v>arr[i-1]));
  assert.ok(a.xTicks.every((v,i,arr)=>!i||v>arr[i-1]));
});

test('scales of multiple traces are shared rather than separately normalized',()=>{
  const a=scales([
    {points:[{t:0,e:.5},{t:11.8,e:-1.12}]},
    {points:[{t:0,e:-.01},{t:49.8,e:.62}]}
  ]);
  assert.ok(a.xMax>=49.8);
  assert.ok(a.yMin<=-1.12);
  assert.ok(a.yMax>=.62);
  assert.ok(a.yTicks.length<15);
  assert.ok(a.xTicks.length<15);
});

test('empty or zero error data produces a finite meaningful chart',()=>{
  const blank=scales([]);
  assert.ok(blank.xMax>0);
  assert.ok(blank.yMax>0&&blank.yMin<0);
  const zero=scales([{points:[{t:0,e:0},{t:2,e:0}]}]);
  assert.ok(zero.yMin<0&&zero.yMax>0);
  assert.ok(zero.yTicks.every(Number.isFinite));
});

test('nice tick labels preserve small cross-track-error resolution',()=>{
  assert.equal(niceStep(10),2);
  assert.equal(niceStep(0),1);
  assert.match(label(.025,.01),/^0\.025/);
  assert.ok(!label(0,.02).startsWith('-'));
});

test('nonfinite/negative-time trace points are not included in scale',()=>{
  const a=scales([{points:[
    {t:-500,e:900},{t:0,e:.1},{t:1,e:Infinity},{t:2,e:-.2}
  ]}]);
  assert.ok(a.xMax<10);
  assert.ok(a.yMax<2&&a.yMin>-2);
});
