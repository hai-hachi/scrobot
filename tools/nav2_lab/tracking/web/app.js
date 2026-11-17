'use strict';
// SCROBOT Tracker UI. All side effects require explicit server actions.
const $=id=>document.getElementById(id);
const ns='http://www.w3.org/2000/svg';
const API='/api/tracker';
const TOPICS={
  '/clock':'Gazebo clock',
  '/evaluation/ground_truth_odom':'Independent ground truth',
  '/diff_drive_controller/odom':'Wheel odometry',
  '/joint_states':'Physical wheel joints',
  '/tracker/cmd_vel_safe':'Saturated controller command'
};
let settings=null,defaultSettings=null,preview=null,live=null,lastStatus=null,busy=false;
let lastHistoryRefresh=0,lastPreviewKey='',initializing=true,drag=null,previewSequence=0;
let shownControllerError=null;
let shownFallbackDomain=null;

async function api(path,body){
  const opts=body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},
                                body:JSON.stringify(body)};
  const resp=await fetch(API+path,{...opts,cache:'no-store'});
  const data=await resp.json();
  if(!resp.ok)throw new Error(data.error||'HTTP '+resp.status);
  return data;
}
const fmt=(x,d=2)=>typeof x==='number'&&Number.isFinite(x)?x.toFixed(d):'—';
const num=id=>Number($(id).value);
function notify(message,error=false){
  const n=$('notice');n.textContent=message;n.hidden=false;
  n.classList.toggle('error',error);
  clearTimeout(notify.timer);
  notify.timer=setTimeout(()=>n.hidden=true,error?15000:8500);
}
async function operate(task){
  if(busy)return;
  busy=true;setButtons();
  try{await task();await refresh();}
  catch(e){notify(e.message,true);}
  finally{busy=false;setButtons();}
}
function setButtons(){
  const stack=Boolean(lastStatus?.stack_running),trial=Boolean(lastStatus?.trial_running);
  const controls={
    'save-settings':!trial,'load-settings':!trial,
    'reset-shared':!trial,'reset-rpp':!trial,'reset-smc':!trial,
    'preview-button':!trial,'new-seed':!trial,
    'start':!stack&&!trial,'run':!trial&&!lastStatus?.controller_error,'cancel':trial,
    'stop':stack,'reset':!trial,
    'gz-open':stack&&!lastStatus?.gazebo_gui?.running,
    'gz-close':stack&&Boolean(lastStatus?.gazebo_gui?.running),
    'rviz-open':stack&&!lastStatus?.rviz_running,
    'rviz-close':stack&&Boolean(lastStatus?.rviz_running),
    'clear-history':!trial
  };
  for(const [id,available] of Object.entries(controls))$(id).disabled=busy||!available;
}
function readForm(){
  if(!settings)return null;
  return {
    path:$('path-kind').value,
    controller:$('controller-select').value,
    spawn_mode:$('spawn-mode').value,
    progress:num('start-progress'),
    lateral_m:num('lateral-offset'),heading_deg:num('heading-error'),
    seed:num('seed'),random_lateral_m:num('random-lateral'),
    random_heading_deg:num('random-heading'),
    pose:{x:num('pose-x'),y:num('pose-y'),yaw_deg:num('pose-yaw')},
    shared:{
      desired_speed_mps:num('desired-speed'),
      max_sim_duration_s:num('max-duration'),
      goal_xy_tolerance_m:num('goal-xy'),
      goal_yaw_tolerance_deg:num('goal-yaw'),
      progress_radius_m:num('progress-radius'),
      progress_timeout_sim_s:num('progress-timeout')
    },
    rpp:{
      lookahead_dist:num('rpp-lookahead'),
      min_lookahead_dist:num('rpp-lookahead-min'),
      max_lookahead_dist:num('rpp-lookahead-max'),
      lookahead_time:num('rpp-lookahead-time'),
      use_velocity_scaled_lookahead_dist:$('rpp-velocity-lookahead').checked,
      use_regulated_linear_velocity_scaling:$('rpp-curvature-slow').checked,
      regulated_linear_scaling_min_radius:num('rpp-min-radius'),
      regulated_linear_scaling_min_speed:num('rpp-min-speed'),
      use_rotate_to_heading:$('rpp-rotate').checked,
      rotate_to_heading_angular_vel:num('rpp-rotate-rate'),
      rotate_to_heading_min_angle_deg:num('rpp-rotate-threshold'),
      max_angular_accel:num('rpp-angular-accel'),
      min_approach_linear_velocity:num('rpp-min-approach')
    },
    smc:{
      cross_gain:num('smc-cross'),heading_gain:num('smc-heading'),
      surface_lambda:num('smc-lambda'),switch_gain:num('smc-switch'),
      boundary_layer:num('smc-boundary'),lookahead_m:num('smc-lookahead'),
      feedforward_gain:num('smc-feedforward'),
      goal_heading_gain:num('smc-goal-heading'),
      goal_align_threshold_m:num('smc-goal-threshold'),
      max_angular_cmd:num('smc-max-angular')
    }
  };
}
function setForm(x){
  settings=structuredClone(x);
  const shared=x.shared,rpp=x.rpp,smc=x.smc;
  const fields={
    'path-kind':x.path,'controller-select':x.controller,
    'spawn-mode':x.spawn_mode,'start-progress':x.progress,
    'lateral-offset':x.lateral_m,'heading-error':x.heading_deg,
    'seed':x.seed,'random-lateral':x.random_lateral_m,
    'random-heading':x.random_heading_deg,
    'pose-x':x.pose.x,'pose-y':x.pose.y,'pose-yaw':x.pose.yaw_deg,
    'desired-speed':shared.desired_speed_mps,
    'max-duration':shared.max_sim_duration_s,
    'goal-xy':shared.goal_xy_tolerance_m,
    'goal-yaw':shared.goal_yaw_tolerance_deg,
    'progress-radius':shared.progress_radius_m,
    'progress-timeout':shared.progress_timeout_sim_s,
    'rpp-lookahead':rpp.lookahead_dist,
    'rpp-lookahead-min':rpp.min_lookahead_dist,
    'rpp-lookahead-max':rpp.max_lookahead_dist,
    'rpp-lookahead-time':rpp.lookahead_time,
    'rpp-min-radius':rpp.regulated_linear_scaling_min_radius,
    'rpp-min-speed':rpp.regulated_linear_scaling_min_speed,
    'rpp-rotate-rate':rpp.rotate_to_heading_angular_vel,
    'rpp-rotate-threshold':rpp.rotate_to_heading_min_angle_deg,
    'rpp-angular-accel':rpp.max_angular_accel,
    'rpp-min-approach':rpp.min_approach_linear_velocity,
    'smc-cross':smc.cross_gain,'smc-heading':smc.heading_gain,
    'smc-lambda':smc.surface_lambda,'smc-switch':smc.switch_gain,
    'smc-boundary':smc.boundary_layer,'smc-lookahead':smc.lookahead_m,
    'smc-feedforward':smc.feedforward_gain,
    'smc-goal-heading':smc.goal_heading_gain,
    'smc-goal-threshold':smc.goal_align_threshold_m,
    'smc-max-angular':smc.max_angular_cmd
  };
  for(const [id,v] of Object.entries(fields))$(id).value=v;
  $('rpp-velocity-lookahead').checked=rpp.use_velocity_scaled_lookahead_dist;
  $('rpp-curvature-slow').checked=rpp.use_regulated_linear_velocity_scaling;
  $('rpp-rotate').checked=rpp.use_rotate_to_heading;
  spawnVisible();
  highlightActiveController();
}
function highlightActiveController(){
  const selected=$('controller-select').value.toLowerCase();
  for(const name of ['rpp','smc']){
    $('card-'+name).classList.toggle('is-active',selected===name);
    $('card-'+name).classList.toggle('is-inactive',selected!==name);
  }
}
function spawnVisible(){
  $('relative-fields').hidden=$('spawn-mode').value!=='relative';
  $('random-fields').hidden=$('spawn-mode').value!=='random';
  $('manual-fields').hidden=$('spawn-mode').value!=='manual';
}
function shape(name,attrs){
  const item=document.createElementNS(ns,name);
  for(const[k,v]of Object.entries(attrs))item.setAttribute(k,String(v));
  return item;
}
const px=x=>450+x*52;
const py=y=>225-y*35;
const wx=x=>(x-450)/52;
const wy=y=>(225-y)/35;
function addLine(g,a,b,color,width=1,dash=''){
  const el=shape('line',{x1:a[0],y1:a[1],x2:b[0],y2:b[1],
      stroke:color,'stroke-width':width});
  if(dash)el.setAttribute('stroke-dasharray',dash);
  g.append(el);
}
function draw(){
  const grid=$('map-grid'),p=$('map-path'),g=$('map-trace'),a=$('map-start');
  for(const item of [grid,p,g,a])item.replaceChildren();
  for(let x=-7;x<=7;x++)addLine(grid,[px(x),10],[px(x),440],'#203444',.8);
  for(let y=-5;y<=5;y++)addLine(grid,[10,py(y)],[890,py(y)],'#203444',.8);
  addLine(grid,[px(0),10],[px(0),440],'#3b596a',1.2);
  addLine(grid,[10,py(0)],[890,py(0)],'#3b596a',1.2);
  const pts=preview?.points||[];
  if(pts.length){
    const data=pts.map(q=>px(q[0])+','+py(q[1])).join(' ');
    p.append(shape('polyline',{points:data,fill:'none',stroke:'#42d0c2',
      'stroke-width':4,'stroke-linejoin':'round','stroke-linecap':'round'}));
    for(const [i,col]of [[0,'#67e4cb'],[pts.length-1,'#e6f2fc']]){
      p.append(shape('circle',{cx:px(pts[i][0]),cy:py(pts[i][1]),
        r:6,fill:col,stroke:'#17333a','stroke-width':2}));
    }
  }
  const trace=live?.trace||[];
  if(trace.length>=2)g.append(shape('polyline',{
    points:trace.map(q=>px(q.x)+','+py(q.y)).join(' '),
    fill:'none',stroke:'#f4bb66','stroke-width':3.5,'stroke-linecap':'round'}));
  if(preview?.pose){
    const init=preview.pose,yaw=init.yaw_deg*Math.PI/180;
    const x=px(init.x),y=py(init.y);
    a.append(shape('circle',{cx:x,cy:y,r:13,
      fill:'#7da9ff','fill-opacity':.14,stroke:'#7da9ff',
      'stroke-width':1.6,'stroke-dasharray':'4 3'}));
    addLine(a,[x,y],[x+24*Math.cos(yaw),y-24*Math.sin(yaw)],'#7da9ff',3);
  }
  if(live?.last_truth){
    const r=live.last_truth,x=px(r.x),y=py(r.y);
    a.append(shape('circle',{cx:x,cy:y,r:8,
      fill:'#f4bb66',stroke:'#fff1d6','stroke-width':1.5}));
    addLine(a,[x,y],[x+18*Math.cos(r.yaw),y-18*Math.sin(r.yaw)],'#f4bb66',3);
  }
}
async function refreshPreview(){
  const x=readForm();if(!x)return;
  const key=JSON.stringify(x);
  if(key===lastPreviewKey)return;
  const sequence=++previewSequence;
  const response=await api('/preview',x);
  if(sequence!==previewSequence)return;
  preview=response;lastPreviewKey=key;
  const p=response.pose;
  $('pose-label').textContent=
    'Resolved initial pose: x '+fmt(p.x,3)+' m · y '+fmt(p.y,3)+
    ' m · yaw '+fmt(p.yaw_deg,1)+'° · '+response.points.length+
    ' reference waypoints. The same seed reproduces this exact pose.';
  draw();
}
function guardedPreview(){
  clearTimeout(guardedPreview.timer);
  guardedPreview.timer=setTimeout(async()=>{
    try{await refreshPreview();}
    catch(e){$('pose-label').textContent=e.message;}
  },100);
}
function renderChart(trace){
  const grid=$('error-grid'),plot=$('error-series');
  grid.replaceChildren();plot.replaceChildren();
  for(let i=0;i<=4;i++){
    const y=20+i*40;
    addLine(grid,[25,y],[825,y],'#33495a',1,'4 4');
  }
  if(!trace||trace.length<2)return;
  const end=Math.max(1,trace[trace.length-1].t);
  const bound=Math.max(.1,...trace.map(q=>Math.abs(q.e)));
  const xy=trace.map(q=>[25+800*q.t/end,
    100-78*q.e/bound]);
  plot.append(shape('polyline',{points:xy.map(q=>q.join(',')).join(' '),
    fill:'none',stroke:'#f4bb66','stroke-width':2.6,'stroke-linecap':'round'}));
  addLine(plot,[25,100],[825,100],'#53aa9f',1.1);
}
function showStatus(s){
  lastStatus=s;
  const running=s.stack_running,trial=s.trial_running;
  $('stack-dot').classList.toggle('on',running);
  $('stack-state').textContent=running?'Simulation running':'Simulation stopped';
  $('action-state').textContent=s.controller_error?
    'NAV2 CONTROLLER FAILED · Stop and restart Tracker':
    trial?'Tracking '+s.run_name+' · ROS '+(s.ros_domain_id??'—'):
    running?'Gazebo ready · ROS domain '+(s.ros_domain_id??'—')+
      ' · start a FollowPath trial':'Tracking simulator stopped';
  if(running&&s.ros_domain_id!==78&&
      s.ros_domain_id!==shownFallbackDomain){
    shownFallbackDomain=s.ros_domain_id;
    const busy=Object.keys(s.occupied_ros_domains||{}).join(', ');
    notify('A previous ROS participant was found in domain '+(busy||'78')+
      '. This Tracker session is isolated in domain '+s.ros_domain_id+
      '. Existing processes were not terminated; stop old sessions to free resources.');
  }
  if(!running)shownFallbackDomain=null;
  if(s.controller_error&&s.controller_error!==shownControllerError){
    shownControllerError=s.controller_error;
    notify(s.controller_error,true);
  }
  if(!s.controller_error)shownControllerError=null;
  $('run-name').textContent=s.run_name||'No experiment';
  $('trial-state').textContent=s.controller_error?'Controller error':
    trial?'Running':s.summary?.status||'Idle';
  $('stack-log').textContent=s.stack_log||'No stack log.';
  $('run-log').textContent=s.trial_log||'No tracking log.';
  $('gazebo-log').textContent=s.gazebo_log||'Gazebo UI closed.';
  $('rviz-log').textContent=s.rviz_log||'RViz closed.';
  const health=s.health||{};
  $('rtf-current').textContent=fmt(health.rtf,2)+'×';
  $('rtf-caption').textContent='Applied cap '+fmt(s.applied_max_rtf,2)+'×';
  $('perf-rtf').textContent=fmt(health.rtf,3)+'×';
  $('perf-clock').textContent=fmt(health.simulation_time_s,1)+' s';
  const current=s.summary?.status==='success'||s.summary?.status==='error'||
    s.summary?.status==='cancelled'?s.summary:s.live||s.summary;
  live=s.live||null;
  const result=current?.results||{};
  $('rmse-value').textContent=fmt(result.cross_track_rmse_m,4);
  $('heading-value').textContent=fmt(result.heading_rmse_deg,2)+'°';
  $('max-error').textContent=fmt(result.max_abs_cross_track_m,3)+' m';
  $('travel-dist').textContent=fmt(result.distance_traveled_m,2)+' m';
  $('duration').textContent=fmt(result.duration_sim_s,2)+' s';
  $('violations').textContent=result.wheel_envelope_violations??'—';
  $('trial-details').textContent=current?
    'Status: '+current.status+
    (current.error?' · '+current.error:'')+
    ' · Ground-truth samples: '+(current.ground_truth_samples??0)+
    ' · Wheel-odom samples: '+(current.wheel_odom_samples??0)+
    (current.status==='initializing'?
      ' · FollowPath endpoint: '+(current.action_server_ready?'available':'unavailable')+
      ' · Controller lifecycle: '+(current.controller_lifecycle_state||'unknown')+
      ' · Ready to dispatch: '+(current.controller_active&&
        current.action_server_ready?'yes':'no'):'')+
    (current.goal_rejections?' · Rejected goal attempts: '+current.goal_rejections:'')+
    ' · Raw commands requiring wheel-envelope clipping: '+
    (current.raw_limit_requests??0)+
    ' · Path distance: '+fmt(result.reference_length_m,2)+' m':
    'Run a controller to record ground-truth error, path and heading metrics.';
  draw();renderChart(live?.trace);
  const resources=s.resources||{};
  $('perf-cpu').textContent=fmt(resources?.totals?.cpu_percent_one_core,1)+'%';
  $('perf-ram').textContent=fmt(resources?.totals?.rss_mb,1)+' MB';
  const tbody=$('topic-body');tbody.replaceChildren();
  for(const [topic,label]of Object.entries(TOPICS)){
    const record=health.topics?.[topic]||{};
    addRow(tbody,[topic+' · '+label,fmt(record.wall_hz,1),
      fmt(record.sim_hz,1),record.last_age_s==null?'—':fmt(record.last_age_s,1)+' s']);
  }
  const rp=$('resources-body');rp.replaceChildren();
  for(const row of resources.processes||[])
    addRow(rp,[row.name,fmt(row.cpu_percent,1),fmt(row.rss_mb,1),
               (row.pids||[]).join(', ')]);
  if(!rp.children.length)addRow(rp,['No processes sampled.','—','—','—']);
  setButtons();
}
function addRow(body,values){
  const tr=document.createElement('tr');
  for(const value of values){const td=document.createElement('td');td.textContent=value;tr.append(td);}
  body.append(tr);return tr;
}
async function history(){
  const rows=await api('/history');
  const table=$('history-body');table.replaceChildren();
  for(const item of rows){
    const s=item.summary||{},r=s.results||{};
    const tr=addRow(table,[item.name,s.path||'—',s.controller||'—',
      s.seed??'—',s.status||'—',fmt(r.cross_track_rmse_m,4),
      fmt(r.heading_rmse_deg,2),fmt(r.duration_sim_s,1)]);
    const td=document.createElement('td'),b=document.createElement('button');
    b.className='btn small danger';b.textContent='Delete';
    b.disabled=busy||Boolean(lastStatus?.trial_running);
    b.onclick=()=>{
      if(!confirm('Permanently delete tracking experiment '+item.name+'?'))return;
      operate(async()=>{
        await api('/history/delete',{name:item.name,confirm:true});
        await history();notify('Experiment deleted.');
      });
    };
    td.append(b);tr.append(td);
  }
  if(!rows.length)addRow(table,['No completed or running experiments yet.']);
}
async function refresh(){
  const data=await api('/status');
  showStatus(data);
}
function activate(view){
  for(const b of document.querySelectorAll('.section-tabs [data-view]'))
    b.classList.toggle('active',b.dataset.view===view);
  for(const area of document.querySelectorAll('.tracker-view'))
    area.hidden=area.dataset.view!==view;
}
for(const button of document.querySelectorAll('.section-tabs [data-view]'))
  button.onclick=()=>activate(button.dataset.view);
$('spawn-mode').onchange=()=>{
  spawnVisible();lastPreviewKey='';guardedPreview();
};
for(const id of ['path-kind','controller-select','start-progress','lateral-offset',
 'heading-error','seed','random-lateral','random-heading','pose-x','pose-y','pose-yaw',
 'desired-speed','max-duration','goal-xy','goal-yaw','progress-radius','progress-timeout',
 'rpp-lookahead','rpp-lookahead-min','rpp-lookahead-max','rpp-lookahead-time',
 'rpp-velocity-lookahead','rpp-curvature-slow','rpp-min-radius','rpp-min-speed',
 'rpp-rotate','rpp-rotate-rate','rpp-rotate-threshold','rpp-angular-accel',
 'rpp-min-approach','smc-cross','smc-heading','smc-lambda','smc-switch',
 'smc-boundary','smc-lookahead','smc-feedforward','smc-goal-heading',
 'smc-goal-threshold','smc-max-angular']){
  $(id).addEventListener('change',()=>{
    if(id==='controller-select')highlightActiveController();
    lastPreviewKey='';guardedPreview();
  });
}
$('new-seed').onclick=()=>{
  $('seed').value=Math.floor(Math.random()*2147483647);
  lastPreviewKey='';guardedPreview();
};
$('preview-button').onclick=()=>{lastPreviewKey='';guardedPreview();};
// Drag from the clicked spawn coordinate to set yaw, matching Planner Lab.
function eventPoint(ev){
  const rect=$('tracker-map').getBoundingClientRect();
  return [ev.clientX-rect.left,ev.clientY-rect.top].map(
    (x,i)=>x/(i===0?rect.width:rect.height)*(i===0?900:450));
}
$('tracker-map').addEventListener('pointerdown',ev=>{
  const [x,y]=eventPoint(ev);
  const worldX=Math.max(-7,Math.min(7,wx(x)));
  const worldY=Math.max(-6,Math.min(6,wy(y)));
  $('spawn-mode').value='manual';
  spawnVisible();
  $('pose-x').value=worldX.toFixed(3);
  $('pose-y').value=worldY.toFixed(3);
  drag={x,y};
  $('tracker-map').setPointerCapture(ev.pointerId);
  lastPreviewKey='';guardedPreview();
});
$('tracker-map').addEventListener('pointermove',ev=>{
  if(!drag)return;
  const [x,y]=eventPoint(ev);
  const dx=x-drag.x,dy=drag.y-y;
  if(Math.hypot(dx,dy)>5){
    $('pose-yaw').value=(Math.atan2(dy,dx)*180/Math.PI).toFixed(1);
    lastPreviewKey='';guardedPreview();
  }
});
for(const event of ['pointerup','pointercancel'])
  $('tracker-map').addEventListener(event,()=>{drag=null;});
for(const name of ['shared','rpp','smc']){
  $('reset-'+name).onclick=()=>{
    if(!defaultSettings){notify('Tracker defaults have not loaded.',true);return;}
    const current=readForm();
    if(!current)return;
    // Reset exactly one category, leaving the other two, path, initial pose
    // and controller selection unchanged.
    current[name]=structuredClone(defaultSettings[name]);
    setForm(current);
    lastPreviewKey='';
    guardedPreview();
    notify('Reset '+name.toUpperCase()+
      ' parameters to baseline. Save Lab YAML to keep these values.');
  };
}
$('save-settings').onclick=()=>operate(async()=>{
  await api('/save',readForm());
  notify('Saved tracking_lab.yaml (independent of production simulation).');
});
$('load-settings').onclick=()=>operate(async()=>{
  setForm(await api('/config'));
  lastPreviewKey='';await refreshPreview();
  notify('Loaded saved Tracker Lab YAML.');
});
$('start').onclick=()=>operate(async()=>{
  await api('/start',readForm());
  notify('Starting headless tracking world and calibrated SCROBOT wheel physics.');
});
$('reset').onclick=()=>operate(async()=>{
  await api('/reset',readForm());
  notify('Respawning robot at the selected initial pose and resetting Gazebo physics.');
});
$('run').onclick=()=>operate(async()=>{
  await api('/run',readForm());
  activate('results');
  notify('Started real Nav2 FollowPath with '+$('controller-select').value+
    '. Each comparison starts from an identical fresh world.');
});
$('cancel').onclick=()=>operate(async()=>{
  await api('/abort',{});notify('Requested cancellation; robot command stops.');
});
$('stop').onclick=()=>operate(async()=>{
  await api('/stop',{});notify('Tracker simulation stopped.');
});
for(const [id,path,message]of [
  ['gz-open','/gazebo/open','Opening Gazebo UI.'],
  ['gz-close','/gazebo/close','Gazebo UI closed.'],
  ['rviz-open','/rviz/open','Opening RViz with odom fixed frame.'],
  ['rviz-close','/rviz/close','RViz closed.']
]){
  $(id).onclick=()=>operate(async()=>{await api(path,{});notify(message);});
}
$('clear-history').onclick=()=>{
  if(!confirm('Permanently delete ALL saved Tracker Lab experiments?'))return;
  operate(async()=>{const r=await api('/history/clear',{confirm:true});
    await history();notify('Deleted '+r.deleted+' tracking experiments.');});
};
(async()=>{
  try{
    defaultSettings=await api('/defaults');
    const x=await api('/config');setForm(x);
    await refreshPreview();
    await refresh();await history();
  }catch(e){notify(e.message,true);}
  initializing=false;
  setInterval(async()=>{
    if(busy)return;
    try{await refresh();}catch(e){$('action-state').textContent=e.message;}
  },1800);
  setInterval(async()=>{if(!busy)try{await history();}catch(e){}},9000);
})();
