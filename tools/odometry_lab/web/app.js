'use strict';
const $ = id => document.getElementById(id);
const COLORS = {ground_truth:'#4bd8b1',wheel_odom:'#69adff',ekf_odom:'#bc9df8',encoder_odom:'#f1b66e'};
const ORDER = ['ground_truth','wheel_odom','ekf_odom','encoder_odom'];
const STREAM_NAMES = {ground_truth:'Gazebo truth',wheel_odom:'Wheel odom',ekf_odom:'EKF',encoder_odom:'Lab encoder',joint_states:'Joint states',requested_cmd:'Requested cmd',final_cmd:'Final cmd'};
const state = {config:null,draft:null,dirty:false,tab:'overview',status:null,samples:[],history:[],pending:false,lastError:''};
const defs = {
  publishing:[
    ['rates.encoder_sample_hz','Encoder sampling','Hz',1,200,1],
    ['rates.estimator_hz','Estimator update','Hz',1,200,1],
    ['rates.publish_hz','Lab odometry publish','Hz',1,200,1],
    ['rates.gui_hz','Monitor refresh','Hz',1,30,1],
    ['motion.command_publish_hz','Motion command rate','Hz',1,100,1],
    ['qos.depth','QoS history depth','messages',1,50,1],
    ['qos.reliability','Lab stream reliability','',null,null,null,'choice',['best_effort','reliable']]
  ],
  performance:[['stress.cpu_duty','Synthetic CPU duty','fraction',0,.75,.05]],
  hardware:[
    ['hardware.encoder_ticks_per_motor_rev','Motor encoder resolution','ticks/rev',1,10000,1],
    ['hardware.gear_ratio','Motor / wheel gear ratio','ratio',.1,200,.1],
    ['hardware.wheel_radius_m','Wheel radius','m',.01,.15,.001],
    ['hardware.wheel_separation_m','Wheel separation','m',.1,1,.005],
    ['hardware.left_radius_multiplier','Left radius multiplier','×',.8,1.2,.005],
    ['hardware.right_radius_multiplier','Right radius multiplier','×',.8,1.2,.005],
    ['hardware.encoder_noise_ticks_std','Encoder noise','ticks',0,20,.1],
    ['hardware.encoder_drop_probability','Dropped-sample probability','fraction',0,1,.01],
    ['hardware.feedback_delay_ms','Feedback delay','ms',0,500,1],
    ['hardware.feedback_jitter_ms','Feedback jitter','ms',0,250,1]
  ]
};

async function api(url,body){
  const opts=body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)};
  let response;
  try{response=await fetch(url,{...opts,cache:'no-store'});}catch(e){throw new Error('Local web service unavailable: '+e.message);}
  const payload=await response.json();
  if(!response.ok){const error=new Error(payload.error||`HTTP ${response.status}`);error.httpStatus=response.status;throw error;}
  return payload;
}
function getPath(obj,path){return path.split('.').reduce((v,k)=>v?.[k],obj);}
function setPath(obj,path,value){const parts=path.split('.');obj[parts[0]][parts[1]]=value;}
function fmt(x,dec=2){return typeof x==='number'&&Number.isFinite(x)?x.toFixed(dec):'—';}
function esc(s){return String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#039;'}[c]));}
function message(text,error=false){const el=$('notice');el.textContent=text;el.classList.remove('hidden');el.classList.toggle('error',error);clearTimeout(message.timer);message.timer=setTimeout(()=>el.classList.add('hidden'),error?11000:5500);}
function dirty(){state.dirty=true;$('unsaved').classList.remove('hidden');}
function clearDirty(){state.dirty=false;$('unsaved').classList.add('hidden');}

function renderFields(){
  for(const [section,entries] of Object.entries(defs)){
    const root=$('fields-'+section);root.replaceChildren();
    for(const [path,label,unit,min,max,step,kind,choices] of entries){
      const value=getPath(state.draft,path);
      const row=document.createElement('div');row.className='field';
      const lab=document.createElement('label');lab.textContent=label;row.append(lab);
      if(kind==='choice'){
        const select=document.createElement('select');
        for(const ch of choices){const option=document.createElement('option');option.value=ch;option.textContent=ch;select.append(option);}
        select.value=value;
        select.addEventListener('change',()=>{setPath(state.draft,path,select.value);dirty();});
        row.append(select);root.append(row);continue;
      }
      const range=document.createElement('input');range.type='range';range.min=min;range.max=max;range.step=step;range.value=value;
      const number=document.createElement('input');number.type='number';number.min=min;number.max=max;number.step=step;number.value=value;
      number.setAttribute('aria-label',`${label} (${unit})`);range.setAttribute('aria-label',label);
      range.addEventListener('input',()=>{number.value=range.value;setPath(state.draft,path,Number(range.value));dirty();});
      number.addEventListener('input',()=>{
        if(number.value==='')return;
        const v=Number(number.value);if(Number.isFinite(v)){
          range.value=Math.min(max,Math.max(min,v));setPath(state.draft,path,v);dirty();
        }
      });
      row.append(range,number);root.append(row);
    }
  }
  $('scenario').value=state.draft.scenario;
}
async function loadConfig(){const cfg=await api('/api/config');state.config=cfg;state.draft=structuredClone(cfg);renderFields();clearDirty();}
async function save(){
  try{
    if(!state.draft)return false;
    state.draft.scenario=$('scenario').value;
    const result=await api('/api/apply',state.draft);
    state.config=result.config;state.draft=structuredClone(result.config);clearDirty();
    message(result.message);return true;
  }catch(e){message(e.message,true);return false;}
}
async function start(){
  if(state.pending)return;
  if(state.dirty && !(await save()))return;
  const scenario=$('scenario').value;
  if(scenario!=='none' && !confirm(`Start ${scenario.toUpperCase()} motion in Gazebo?\n\nThe existing controller will receive motion commands through the normal manual-control path. Verify simulation is active and the robot work area is safe.`))return;
  state.pending=true;setButtons();
  try{
    const data=await api('/api/start',{});
    state.history=[];state.samples=[];state.status=data.status;
    message(scenario==='none'?'Passive recording started.':'Motion test starting through SCROBOT control stack.');
    updateStatus(data.status);
  }catch(e){message(e.message,true);}finally{state.pending=false;setButtons();}
}
async function stop(){
  if(state.pending)return;
  state.pending=true;setButtons();
  try{const data=await api('/api/stop',{});message(data.message);}catch(e){message(e.message,true);}
  finally{state.pending=false;setButtons();}
}
function setButtons(){const running=Boolean(state.status?.running);const checking=Boolean(state.status?.preflight_in_progress);$('start').disabled=running||checking||state.pending;$('stop').disabled=!running||state.pending;}
function setTab(tab){state.tab=tab;document.querySelectorAll('.nav').forEach(n=>n.classList.toggle('active',n.dataset.tab===tab));document.querySelectorAll('.tab-panel').forEach(n=>n.classList.toggle('hidden',n.id!=='tab-'+tab));drawAll();}

function metrics(s){
  const live=s?.live||{};const t=live.timings||{};const wheel=t.wheel_odom||{};const accuracy=s?.summary?.accuracy?.wheel_odom||{};
  $('metric-rate').textContent=fmt(wheel.wall_hz,1);
  $('metric-stamp').textContent='Stamp rate '+fmt(wheel.stamp_hz,1)+' Hz';
  $('metric-rtf').textContent=fmt(live.rtf,2);
  $('metric-rtf-note').textContent='Simulation time / wall time';
  $('metric-jitter').textContent=fmt(wheel.wall_period_p95_ms,1);
  $('metric-rmse').textContent=fmt(accuracy.position_rmse_m,3);
}
function timingTable(live){
  const t=live?.timings||{};const names=['ground_truth','wheel_odom','ekf_odom','encoder_odom','joint_states','requested_cmd','final_cmd'];
  const row=key=>{const v=t[key]||{};return `<tr><td>${STREAM_NAMES[key]}</td><td>${fmt(v.wall_hz,1)}</td><td>${fmt(v.stamp_hz,1)}</td><td>${fmt(v.wall_period_p95_ms,1)}</td><td>${fmt(v.age_sim_mean_ms,1)}</td><td>${v.repeated_stamps??'—'}</td></tr>`;};
  const html='<table class="data-table"><thead><tr><th>Stream</th><th>Wall Hz</th><th>Stamp Hz</th><th>P95 ms</th><th>Age ms*</th><th>Repeat</th></tr></thead><tbody>'+names.map(row).join('')+'</tbody></table>';
  $('timing-overview').innerHTML=html;$('timing-detailed').innerHTML=html;
}
function perf(live){
  const metrics=[
    ['Gazebo RTF',fmt(live?.rtf,3)+' ×'],
    ['System CPU',fmt(live?.system_cpu_percent,1)+' %'],
    ['Gazebo CPU estimate',fmt(live?.gazebo_process_cpu_percent_estimate,1)+' %'],
    ['Monitor CPU',fmt(live?.lab_cpu_percent,1)+' %'],
    ['Monitor RAM',fmt(live?.lab_memory_mib,1)+' MiB'],
    ['Pending encoder samples',live?.pending_samples??'—'],
    ['Estimator updates',live?.encoder_updates??'—'],
    ['Publish callback P95',fmt(live?.callback_runtime_ms?.publisher?.p95,2)+' ms']
  ];
  $('perf-details').innerHTML=metrics.map(([n,v])=>`<div class="stat"><div class="name">${n}</div><div class="value">${v}</div></div>`).join('');
}
function accuracyReport(report){
  const acc=report?.accuracy||{};
  const keys=[['position_rmse_m','Position RMSE (m)'],['heading_rmse_deg','Heading RMSE (°)'],['endpoint_position_error_m','Endpoint error (m)'],['drift_per_meter','Drift / path (m/m)'],['vx_rmse_mps','Linear vel RMSE (m/s)'],['wz_rmse_rps','Yaw rate RMSE (rad/s)']];
  const names=['wheel_odom','ekf_odom','encoder_odom'];
  $('accuracy-report').innerHTML=Object.keys(acc).length?'<table class="data-table"><thead><tr><th>Metric</th>'+names.map(n=>`<th>${STREAM_NAMES[n]}</th>`).join('')+'</tr></thead><tbody>'+keys.map(([key,name])=>`<tr><td>${name}</td>`+names.map(n=>`<td>${fmt(acc[n]?.[key],key.includes('deg')?2:4)}</td>`).join('')+'</tr>').join('')+'</tbody></table>':'<div class="empty">Complete a test to generate time-aligned accuracy metrics.</div>';
  const w=acc.wheel_odom;
  $('report-summary').textContent=w?`Wheel odometry: ${fmt(w.position_rmse_m,3)} m position RMSE, ${fmt(w.heading_rmse_deg,2)}° yaw RMSE over ${w.matched_samples} matched samples.`:'No completed report yet. Live data is displayed during recording.';
}
function exports(s){
  const names=['summary.json','summary.csv','samples.csv','trajectory.svg','config_snapshot.yaml'];
  const html=s?.run_name?names.map(n=>`<a href="/api/export?file=${encodeURIComponent(n)}" download>${esc(n)}</a>`).join(''):'';
  $('exports-overview').innerHTML=html;$('exports-accuracy').innerHTML=html;
}
function updateStatus(s){
  state.status=s;
  const live=s?.live||{};
  $('run-dot').classList.toggle('on',s?.running||false);
  $('run-badge').textContent=s?.preflight_in_progress?'CHECKING ROS':s?.running?'RUNNING':s?.run_name?'COMPLETED / STOPPED':'IDLE';
  $('run-name').textContent=s?.run_name||'No experiment selected';
  $('run-scenario').textContent=(live?.state||s?.scenario||'—').replaceAll('_',' ');
  $('connection-label').textContent='Local service online';
  metrics(s);timingTable(live);perf(live);accuracyReport(s?.summary);exports(s);setButtons();
  if(s?.running){
    const hist={t:performance.now(),rtf:live.rtf,
      wheel:live.timings?.wheel_odom?.wall_hz,ekf:live.timings?.ekf_odom?.wall_hz,
      encoder:live.timings?.encoder_odom?.wall_hz};
    state.history.push(hist);if(state.history.length>150)state.history.shift();
  }
  drawAll();
}
function canvas2d(canvas){
  if(!canvas)return null;
  const rect=canvas.getBoundingClientRect();const w=Math.max(200,Math.floor(rect.width));
  const h=Number(canvas.getAttribute('height'))||300;
  const ratio=window.devicePixelRatio||1;
  if(canvas.width!==Math.round(w*ratio)||canvas.height!==Math.round(h*ratio)){
    canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);
  }
  const ctx=canvas.getContext('2d');ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,w,h);
  return {ctx,w,h};
}
function baseAxes(ctx,w,h,left=49,bottom=28){
  const right=w-14, top=12, low=h-bottom;
  ctx.strokeStyle='#273b4b';ctx.lineWidth=1;
  for(let i=0;i<=4;i++){
    const y=top+(low-top)*i/4;ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();
  }
  return {left,right,top,low};
}
function placeholder(ctx,w,h,label){ctx.fillStyle='#758da2';ctx.font='12px system-ui';ctx.textAlign='center';ctx.fillText(label,w/2,h/2);ctx.textAlign='left';}
function plotTime(id,channels,yLabel){
  const canvas=$(id);if(canvas.closest('.hidden'))return;
  const c=canvas2d(canvas);if(!c)return;const {ctx,w,h}=c;const a=baseAxes(ctx,w,h);
  const data=state.history;
  if(data.length<2){placeholder(ctx,w,h,'Live data appears after monitoring starts');return;}
  const values=data.flatMap(d=>channels.map(([key])=>d[key]).filter(Number.isFinite));
  if(!values.length){placeholder(ctx,w,h,'Waiting for measurements');return;}
  let hi=Math.max(...values,1)*1.15,lo=Math.min(0,...values);if(hi-lo<.01)hi=lo+1;
  ctx.fillStyle='#7992a5';ctx.font='10px ui-monospace,monospace';
  for(let k=0;k<=4;k++){const n=hi-(hi-lo)*k/4;ctx.fillText(n.toFixed(hi>20?0:2),4,a.top+(a.low-a.top)*k/4+3);}
  ctx.fillText(yLabel,a.left,a.low+19);
  const tx=i=>a.left+i/(data.length-1)*(a.right-a.left);
  const ty=v=>a.low-(v-lo)/(hi-lo)*(a.low-a.top);
  for(const [key,color] of channels){
    ctx.strokeStyle=color;ctx.lineWidth=2;ctx.lineJoin='round';ctx.beginPath();let drawing=false;
    data.forEach((p,i)=>{if(!Number.isFinite(p[key])){drawing=false;return;}const x=tx(i),y=ty(p[key]);if(!drawing){ctx.moveTo(x,y);drawing=true;}else ctx.lineTo(x,y);});ctx.stroke();
  }
}
function relativePoints(rows){
  if(!rows.length)return [];
  const x0=rows[0][2],y0=rows[0][3],a=rows[0][4],ca=Math.cos(a),sa=Math.sin(a);
  return rows.map(r=>{const dx=r[2]-x0,dy=r[3]-y0;return [ca*dx+sa*dy,-sa*dx+ca*dy];});
}
function plotTrajectory(id){
  const canvas=$(id);if(canvas.closest('.hidden'))return;
  const c=canvas2d(canvas);if(!c)return;const {ctx,w,h}=c;
  const selected=(id==='trajectory-main'?['ground_truth','wheel_odom','encoder_odom']:ORDER);
  const streams={};const gathered=[];
  for(const name of selected){const records=state.samples.filter(s=>s[0]===name).sort((a,b)=>a[1]-b[1]);streams[name]=relativePoints(records);gathered.push(...streams[name]);}
  const a=baseAxes(ctx,w,h,48,30);
  if(gathered.length<2){placeholder(ctx,w,h,'Trajectory will appear after Gazebo publishes odometry');return;}
  let minX=Math.min(...gathered.map(v=>v[0])),maxX=Math.max(...gathered.map(v=>v[0]));
  let minY=Math.min(...gathered.map(v=>v[1])),maxY=Math.max(...gathered.map(v=>v[1]));
  const width=a.right-a.left,height=a.low-a.top;
  let scale=Math.min(width/Math.max(.5,(maxX-minX)*1.22),height/Math.max(.5,(maxY-minY)*1.22));
  const cx=(minX+maxX)/2,cy=(minY+maxY)/2;
  const tx=x=>(a.left+a.right)/2+(x-cx)*scale;
  const ty=y=>(a.top+a.low)/2-(y-cy)*scale;
  if(minX<0&&maxX>0){ctx.strokeStyle='#335063';ctx.setLineDash([4,5]);ctx.beginPath();ctx.moveTo(tx(0),a.top);ctx.lineTo(tx(0),a.low);ctx.stroke();ctx.setLineDash([]);}
  if(minY<0&&maxY>0){ctx.strokeStyle='#335063';ctx.setLineDash([4,5]);ctx.beginPath();ctx.moveTo(a.left,ty(0));ctx.lineTo(a.right,ty(0));ctx.stroke();ctx.setLineDash([]);}
  for(const name of selected){
    const points=streams[name];if(points.length<2)continue;
    ctx.strokeStyle=COLORS[name];ctx.lineWidth=name==='ground_truth'?2.8:1.8;
    ctx.beginPath();points.forEach(([x,y],i)=>{if(i===0)ctx.moveTo(tx(x),ty(y));else ctx.lineTo(tx(x),ty(y));});ctx.stroke();
    const [xe,ye]=points[points.length-1];ctx.fillStyle=COLORS[name];ctx.beginPath();ctx.arc(tx(xe),ty(ye),3,0,Math.PI*2);ctx.fill();
  }
  ctx.fillStyle='#8ca2b6';ctx.font='10px ui-monospace,monospace';ctx.fillText('X (m)',a.right-30,a.low+18);ctx.fillText(`Scale ${fmt(60/scale,2)} m / 60 px`,a.left,a.low+18);
}
function drawAll(){
  if(state.tab==='overview'){plotTrajectory('trajectory-main');plotTime('rate-main',[['wheel',COLORS.ground_truth],['ekf',COLORS.wheel_odom],['encoder',COLORS.encoder_odom]],'Hz · wall receive');}
  if(state.tab==='publishing')plotTime('rate-detailed',[['wheel',COLORS.ground_truth],['ekf',COLORS.wheel_odom],['encoder',COLORS.encoder_odom]],'Hz · wall receive');
  if(state.tab==='performance')plotTime('rtf-chart',[['rtf',COLORS.ground_truth]],'Gazebo RTF');
  if(state.tab==='accuracy')plotTrajectory('trajectory-accuracy');
}
async function poll(){
  try{
    const s=await api('/api/status');updateStatus(s);
    state.lastError='';
  }catch(e){
    $('connection-label').textContent=e.httpStatus?'Server error':'Server disconnected';
    if(state.lastError!==e.message){message(e.message,true);state.lastError=e.message;}
  }
}
async function pollSamples(){
  try{const r=await api('/api/samples?limit=2200');state.samples=r.samples||[];drawAll();}catch(e){/* Status poll surfaces connectivity errors. */}
}
function init(){
  document.querySelectorAll('.nav').forEach(n=>n.addEventListener('click',()=>setTab(n.dataset.tab)));
  $('scenario').addEventListener('change',()=>{if(state.draft){state.draft.scenario=$('scenario').value;dirty();}});
  $('apply').addEventListener('click',save);$('start').addEventListener('click',start);$('stop').addEventListener('click',stop);
  window.addEventListener('resize',drawAll);
  $('local-clock').textContent=new Date().toLocaleTimeString();setInterval(()=>$('local-clock').textContent=new Date().toLocaleTimeString(),1000);
  loadConfig().catch(e=>message(e.message,true));
  poll();pollSamples();setInterval(poll,1100);setInterval(pollSamples,2100);
}
document.addEventListener('DOMContentLoaded',init);
