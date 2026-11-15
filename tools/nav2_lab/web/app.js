'use strict';
const $ = id => document.getElementById(id);
let config = null, status = null, busy = false, runHistory = [];

async function api(route, data) {
  const options = data === undefined ? {} : {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify(data)
  };
  let response;
  try { response = await fetch(route, {...options, cache:'no-store'}); }
  catch (error) { throw new Error('Lab service unavailable: '+error.message); }
  const json = await response.json();
  if (!response.ok) throw new Error(json.error || 'HTTP '+response.status);
  return json;
}
function notice(message, error=false) {
  const n = $('notice'); n.hidden=false; n.textContent=message; n.classList.toggle('error', error);
  clearTimeout(notice.timer); notice.timer=setTimeout(()=>n.hidden=true,7500);
}
function populate(cfg) {
  config=structuredClone(cfg);
  $('planner').value=cfg.planner; $('controller').value=cfg.controller;
  $('x').value=cfg.goal.x; $('y').value=cfg.goal.y; $('yaw').value=cfg.goal.yaw_deg;
  $('speed').value=cfg.limits.cruise_speed;
  $('timeout').value=cfg.experiment.timeout_s; $('rate').value=cfg.experiment.sample_hz;
  $('kpi-planner').textContent=cfg.planner;
  $('kpi-controller').textContent=cfg.controller;
}
function readConfig(mode) {
  const next=structuredClone(config);
  next.planner=$('planner').value; next.controller=$('controller').value;
  next.mode=mode || next.mode;
  next.goal.x=Number($('x').value); next.goal.y=Number($('y').value);
  next.goal.yaw_deg=Number($('yaw').value);
  next.limits.cruise_speed=Number($('speed').value);
  next.experiment.timeout_s=Number($('timeout').value);
  next.experiment.sample_hz=Number($('rate').value);
  return next;
}
async function save(mode) {
  const result=await api('/api/save',readConfig(mode));
  config=result.config; populate(config);
  return config;
}
function setBusy(value) { busy=value; updateButtons(); }
async function perform(fn) {
  if (busy) return;
  setBusy(true);
  try { await fn(); await refresh(); }
  catch (e) { notice(e.message,true); }
  finally { setBusy(false); }
}
function updateButtons() {
  const live=Boolean(status?.stack_running), running=Boolean(status?.experiment_running);
  $('stack-start').disabled=busy||live;
  $('stack-stop').disabled=busy||!live;
  $('plan').disabled=busy||!live||running;
  $('track').disabled=busy||!live||running;
  $('cancel').disabled=busy||!running;
  $('save').disabled=busy;
}
function fmt(value, precision=3) {
  return typeof value==='number'&&Number.isFinite(value)?value.toFixed(precision):'—';
}
function renderSummary(summary) {
  if(!summary) { $('summary').textContent='No completed experiment yet.'; return; }
  const metrics=[
    ['Status', summary.status==='success'?'PASS':summary.error||'ERROR'],
    ['Planner',summary.planner],['Controller',summary.controller],
    ['Planned distance',fmt(summary.path_length_m)+' m'],
    ['Planner wall time',fmt(summary.planning_wall_s)+' s'],
    ['Nav2 compute time',fmt(summary.planning_nav2_s)+' s'],
    ['Tracking time',fmt(summary.tracking_duration_s)+' s'],
    ['Truth cross-track RMSE',fmt(summary.ground_truth_tracking_rmse_m)+' m'],
    ['Truth goal position error',fmt(summary.ground_truth_goal_position_error_m)+' m'],
    ['Cross-track RMSE',fmt(summary.tracking_rmse_m)+' m'],
    ['Maximum cross-track',fmt(summary.tracking_max_error_m)+' m'],
    ['Goal position error',fmt(summary.goal_position_error_m)+' m'],
    ['Travel distance',fmt(summary.travel_distance_m)+' m'],
    ['Wheel limit fraction',typeof summary.wheel_limit_fraction==='number'?
      fmt(100*summary.wheel_limit_fraction,1)+' %':'—'],
    ['Command variation',fmt(summary.command_total_variation)],
    ['Tracked samples',summary.tracked_samples??'—']
  ];
  $('summary').replaceChildren(...metrics.map(([name,value])=>{
    const block=document.createElement('div');block.className='stat';
    const label=document.createElement('small');label.textContent=name;
    const number=document.createElement('strong');number.textContent=value;
    block.append(label,number); return block;
  }));
}
function drawPlot(s) {
  const canvas=$('plot'), ctx=canvas.getContext('2d');
  const width=canvas.width,height=canvas.height;
  ctx.fillStyle='#0c1927';ctx.fillRect(0,0,width,height);
  const planned=s?.path?.points||[], measured=s?.tracked||[], truth=s?.ground_truth_aligned||[];
  const all=[...planned,...measured,...truth];
  if (!all.length) {
    ctx.fillStyle='#7794ac';ctx.font='15px sans-serif';
    ctx.textAlign='center';ctx.fillText('No path available — start a planner experiment',width/2,height/2);return;
  }
  const xs=all.map(p=>p[0]),ys=all.map(p=>p[1]);
  const bounds={minx:Math.min(...xs),maxx:Math.max(...xs),
                miny:Math.min(...ys),maxy:Math.max(...ys)};
  const dx=Math.max(bounds.maxx-bounds.minx,.8),dy=Math.max(bounds.maxy-bounds.miny,.8);
  const scale=Math.min((width-90)/dx,(height-70)/dy);
  const mx=(bounds.maxx+bounds.minx)/2,my=(bounds.maxy+bounds.miny)/2;
  const map=p=>[width/2+(p[0]-mx)*scale,height/2-(p[1]-my)*scale];
  ctx.lineWidth=1;ctx.strokeStyle='#1d3346';
  for(let x=40;x<width;x+=40){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,height);ctx.stroke();}
  for(let y=30;y<height;y+=40){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(width,y);ctx.stroke();}
  function draw(points,color,dash=[]) {
    if(!points.length)return;
    ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=2.5;ctx.setLineDash(dash);
    points.forEach((p,i)=>{const [x,y]=map(p);i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);});
    ctx.stroke();ctx.setLineDash([]);
  }
  draw(planned,'#76aaf8');draw(measured,'#4bd8b1');draw(truth,'#f2ba67',[6,3]);
  function mark(p,color,r,label) {
    const [x,y]=map(p);ctx.fillStyle=color;ctx.beginPath();ctx.arc(x,y,r,0,2*Math.PI);ctx.fill();
    ctx.font='12px sans-serif';ctx.fillText(label,x+10,y-10);
  }
  if(planned.length){mark(planned[0],'#f3c16e',5,'Start');mark(planned[planned.length-1],'#f3c16e',6,'Goal');}
  ctx.fillStyle='#728da4';ctx.font='12px monospace';ctx.textAlign='left';
  ctx.fillText('ODOM frame / meters',20,22);
  ctx.fillText(`X ${fmt(bounds.minx,1)} … ${fmt(bounds.maxx,1)} | Y ${fmt(bounds.miny,1)} … ${fmt(bounds.maxy,1)}`,20,height-14);
}
async function refresh() {
  try {
    status=await api('/api/status');
    $('conn').textContent='Local service connected';
    $('stack-led').classList.toggle('live',status.stack_running);
    $('stack-label').textContent=status.stack_running?'Stack running':'Stack stopped';
    $('kpi-status').textContent=status.experiment_running?'Running':status.summary?.status||'Idle';
    $('run-name').textContent=status.run_name||'No run';
    $('action-status').textContent=status.stack_running?
      (status.experiment_running?'Experiment in progress':'Nav2 stack ready for actions'):
      'Stack stopped · click Start simulation';
    $('logs').textContent=status.experiment_running ?
      (status.runner_log_tail||'Waiting for Nav2 action server…') :
      (status.summary?.error||status.runner_log_tail||status.stack_log_tail||'No output available');
    renderSummary(status.summary);
    drawPlot(status);
    updateButtons();
  } catch (e) {
    $('conn').textContent='Disconnected';
    $('action-status').textContent=e.message;
  }
}
async function refreshRuns() {
  try {
    runHistory=await api('/api/runs');
    if(!runHistory.length){$('history').textContent='No experiments yet.';return;}
    const table=document.createElement('table');table.className='history';
    const top=document.createElement('tr');
    ['Run','Planner','Controller','Mode','Status','Plan (m)','RMSE (m)','Time (s)'].forEach(t=>{
      const h=document.createElement('th');h.textContent=t;top.append(h);
    });table.append(top);
    for(const row of runHistory) {
      const tr=document.createElement('tr'), s=row.summary;
      const values=[row.name,s.planner,s.controller,s.mode,s.status,
                    fmt(s.path_length_m),fmt(s.tracking_rmse_m),fmt(s.tracking_duration_s,1)];
      values.forEach((v,i)=>{const td=document.createElement('td');td.textContent=v;
        if(i===4)td.className=s.status==='success'?'ok':'failed';tr.append(td);});
      table.append(tr);
    }
    $('history').replaceChildren(table);
  } catch(e) {}
}
for (const tab of document.querySelectorAll('.tab')) {
  tab.addEventListener('click',()=>{
    document.querySelectorAll('.tab').forEach(el=>el.classList.toggle('active',el===tab));
    document.querySelectorAll('.pane').forEach(el=>el.classList.toggle('active',el.id===tab.dataset.tab));
    if(tab.dataset.tab==='results')refreshRuns();
  });
}
$('planner').addEventListener('change',()=>$('kpi-planner').textContent=$('planner').value);
$('controller').addEventListener('change',()=>$('kpi-controller').textContent=$('controller').value);
$('save').onclick=()=>perform(async()=>{
  await save();notice('Saved lab YAML. Restart the stack to apply changed cruise speed.');
});
$('stack-start').onclick=()=>perform(async()=>{
  await save();await api('/api/start-stack',{});notice('Nav2 Lab launch started. Wait for planner and controller action servers to become active before running.');
});
$('stack-stop').onclick=()=>perform(async()=>{
  await api('/api/stop-stack',{});notice('Stopping Nav2 Lab and Gazebo.');
});
async function run(mode) {
  await save(mode);
  await api('/api/run',{});
  notice(mode==='plan'?'Planner-only action started; robot will not drive.':
    'Plan + FollowPath started; robot will move in Gazebo through the normal control pipeline.');
}
$('plan').onclick=()=>perform(()=>run('plan'));
$('track').onclick=()=>perform(async()=>{
  if(!confirm('Execute Nav2 FollowPath in the isolated Gazebo simulation?\nConfirm no real robot controller shares this ROS_DOMAIN_ID.'))return;
  await run('track');
});
$('cancel').onclick=()=>perform(async()=>{await api('/api/cancel',{});notice('Cancellation requested.');});
(async function initialize(){
  try {populate(await api('/api/config'));await refresh();await refreshRuns();}
  catch(e){notice(e.message,true);}
  setInterval(refresh,2000);
  setInterval(refreshRuns,12000);
})();
