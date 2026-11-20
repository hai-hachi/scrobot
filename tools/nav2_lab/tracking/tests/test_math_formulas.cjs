'use strict';
const {test}=require('node:test');
const assert=require('node:assert/strict');
const {definitions}=require('../web/math_formulas.js');

test('complete source-faithful one- and two-surface full laws are displayed',()=>{
  const required=[
    'smc1-s','smc1-omega','smc1-v','smc1-reaching',
    'smc2-s','smc2-omega','smc2-v','smc2-reaching',
    'gx','q','sat-definition','denominator','switch-enable',
    'body-x','body-y','body-heading','carrot-radius','carrot-window',
    'carrot-curvature','reference-speed','reference-omega',
    'limit-angular','limit-accel','limit-wheel'
  ];
  for(const id of required){
    assert.ok(definitions[id],id);
    assert.ok(definitions[id].tex.length>10,id);
    assert.ok(definitions[id].mml.length>15,id);
  }
  assert.match(definitions['smc1-s'].tex,/\\lambda_x e_x/);
  assert.match(definitions['smc1-omega'].tex,/-\\lambda_x g_x/);
  assert.match(definitions['smc2-s'].tex,/s_1=e_x/);
  assert.match(definitions['smc2-omega'].tex,/\\omega_R/);
  assert.doesNotMatch(definitions['smc2-omega'].tex,/\\lambda_x/);
  assert.match(definitions['smc1-v'].tex,/\\cos/);
  assert.match(definitions['smc2-v'].tex,/\\cos/);
});

test('tunable reaching, guard, saturation and actuator symbols all appear',()=>{
  const all=Object.values(definitions).map(x=>x.tex).join('\n');
  for(const term of [
    '\\lambda_y','\\lambda_x','k_s','k_x','\\eta_x','\\eta',
    '\\phi_x','\\phi','\\delta_D','\\chi_{\\mathrm{sat}}',
    '\\omega_{\\max}','a_{\\max}','\\alpha_{\\max}',
    '\\operatorname{sat}','\\kappa_R','v_R','\\omega_R'
  ])assert.ok(all.includes(term),term);
  assert.match(definitions['sat-definition'].mml,/<mtable>/);
  assert.match(definitions['smc1-omega'].mml,/<mfrac>/);
  assert.match(definitions['smc2-omega'].mml,/<mfrac>/);
  assert.match(definitions['smc1-s'].mml,/<msub>/);
});

test('static MathML fallback is local and every formula is validly wrapped',()=>{
  for(const [key,x] of Object.entries(definitions)){
    assert.ok(x.mml.startsWith('<mrow>'),key);
    assert.ok(x.mml.endsWith('</mrow>'),key);
    const opens=(x.mml.match(/<mrow>/g)||[]).length;
    const closes=(x.mml.match(/<\/mrow>/g)||[]).length;
    assert.equal(opens,closes,key+' mrow balance');
  }
});
