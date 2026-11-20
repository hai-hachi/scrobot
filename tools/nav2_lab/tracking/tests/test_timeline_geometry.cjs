'use strict';
const {test}=require('node:test');
const assert=require('node:assert/strict');
const {geometry,niceStep}=require('../web/timeline_geometry.js');

test('XY data units have exactly the same screen scale',()=>{
  const points=[[-5,-2],[0,0],[4,1],[7,3]];
  const frames=[
    {x:-4,y:-1,ref:{x:-3.5,y:-.5,radius:.6}},
    {x:2,y:2,ref:{x:2.4,y:2.3,radius:1.2}}
  ];
  const g=geometry(points,frames);
  const pxPerMetre=g.x(1)-g.x(0);
  const pyPerMetre=g.y(0)-g.y(1);
  assert.ok(pxPerMetre>0);
  assert.ok(Math.abs(pxPerMetre-pyPerMetre)<1e-9);
  assert.ok(Math.abs(g.radius(1.2)-1.2*pxPerMetre)<1e-9);
  assert.equal(g.width,900);
  assert.equal(g.height,450);
});

test('path, current/historical robot and full circles fit inside plot area',()=>{
  const points=[[-6,-3],[8,-3],[8,6]];
  const frames=[
    {x:-5,y:-1,ref:{x:-4.6,y:-1.4,radius:.6}},
    {x:8,y:6,ref:{x:8.6,y:5.5,radius:1.8}}
  ];
  const g=geometry(points,frames);
  const [left,right,top,bottom]=[
    g.box.left,g.width-g.box.right,
    g.box.top,g.height-g.box.bottom
  ];
  const contains=(x,y,r=0)=>{
    assert.ok(g.x(x)-g.radius(r)>=left-1e-7);
    assert.ok(g.x(x)+g.radius(r)<=right+1e-7);
    assert.ok(g.y(y)-g.radius(r)>=top-1e-7);
    assert.ok(g.y(y)+g.radius(r)<=bottom+1e-7);
  };
  for(const p of points)contains(p[0],p[1]);
  for(const f of frames){
    contains(f.x,f.y,f.ref.radius);
    contains(f.ref.x,f.ref.y);
  }
  assert.ok(g.xTicks.length>1);
  assert.ok(g.yTicks.length>1);
  assert.ok(g.xTicks.every((v,i,a)=>i===0||v>a[i-1]));
  assert.ok(g.yTicks.every((v,i,a)=>i===0||v>a[i-1]));
  assert.ok(g.xTicks.every(Number.isFinite));
});

test('very short path, empty data and outlier samples remain finite',()=>{
  for(const g of [
    geometry(),geometry([[0,0],[0.001,0.002]],[]),
    geometry([[0,0],[1,1]],[{x:10,y:-12,ref:{x:10,y:-11,radius:.8}}])
  ]){
    assert.ok(Number.isFinite(g.scale)&&g.scale>0);
    assert.ok(Number.isFinite(g.x(0))&&Number.isFinite(g.y(0)));
    assert.ok(g.step>0);
    assert.ok(g.format(0)==='0'||g.format(0)==='0.0'||
      g.format(0)==='0.00'||g.format(0)==='0.000');
  }
  assert.equal(niceStep(10),2);
});

test('range circle stays circular through the same scale as metre ticks',()=>{
  const g=geometry([[0,0],[8,0]],[
    {x:0,y:.3,ref:{x:.5,y:0,radius:.6}}
  ]);
  const radius=g.radius(.6);
  assert.equal(radius,.6*g.scale);
  assert.ok(g.x(0)+radius<g.x(2));
  assert.ok(Math.abs((g.x(2)-g.x(0))+(g.y(2)-g.y(0)))<1e-8);
});
