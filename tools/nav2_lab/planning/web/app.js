'use strict';
const $=id=>document.getElementById(id);
let saved=null, status=null, busy=false, activePane='setup';
let calibration=null;

function showNotice(msg,error=false){
  const el=$('notice');el.hidden=false;el.textContent=msg;el.classList.toggle('error',error);
  clearTimeout(showNotice.timer);
  showNotice.timer=setTimeout(()=>el.hidden=true,error?12000:6500);
}
async function api(url,body){
  const opts=body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)};
  let response;
  try{response=await fetch(url,{...opts,cache:'no-store'});}
  catch(e){throw new Error('Cannot connect to Nav2 Lab service: '+e.message);}
  const data=await response.json();
  if(!response.ok)throw new Error(data.error||('HTTP '+response.status));
  return data;
}
const FIELDS={
  planner:'planner',timeout_s:'timeout',depth_obstacles:'depth-enabled',resolution_m:'resolution'
};
function populate(config){
  saved=structuredClone(config);
  const p=config.planning;
  $('planner').value=p.planner;
  $('timeout').value=p.timeout_s;
  $('depth-enabled').checked=p.depth_obstacles;
  $('resolution').value=String(p.resolution_m);
  for(const kind of ['start','goal']){
    $(''+kind+'-x').value=p[kind].x;
    $(''+kind+'-y').value=p[kind].y;
    $(''+kind+'-yaw').value=p[kind].yaw_deg;
  }
  $('domain').textContent=config.simulation.ros_domain_id;
}
function formConfig(){
  const p=structuredClone(saved.planning);
  p.planner=$('planner').value;
  p.timeout_s=Number($('timeout').value);
  p.depth_obstacles=$('depth-enabled').checked;
  p.resolution_m=Number($('resolution').value);
  for(const kind of ['start','goal']){
    p[kind].x=Number($(''+kind+'-x').value);
    p[kind].y=Number($(''+kind+'-y').value);
    p[kind].yaw_deg=Number($(''+kind+'-yaw').value);
  }
  return {planning:p};
}
async function save(){
  const response=await api('/api/save',formConfig());
  populate(response.config);
}
function updateButtons(){
  const live=Boolean(status?.stack_running),trial=Boolean(status?.trial_running);
  for(const [name,enabled] of Object.entries({
    start:!live,stop:live,plan:live&&!trial,cancel:trial,
    'rviz-open':live&&!status?.rviz_running,
    'rviz-close':live&&status?.rviz_running,
    'add-obstacle':live&&!trial,'clear-obstacles':live&&!trial,save:true
  })){
    $(name).disabled=busy||!enabled;
  }
}
async function operation(task){
  if(busy)return;
  busy=true;updateButtons();
  try{await task();await refresh();}
  catch(e){showNotice(e.message,true);}
  finally{busy=false;updateButtons();}
}
const format=(v,precision=2)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(precision):'—';
function renderResults(data){
  const rows=data?.summary?.results||{};
  const tbody=$('results');tbody.replaceChildren();
  for(const [name,result] of Object.entries(rows)){
    const row=document.createElement('tr');
    const values=[
      name,result.status,
      format(typeof result.planning_nav2_s==='number'?result.planning_nav2_s*1000:NaN,1),
      format(typeof result.planning_wall_s==='number'?result.planning_wall_s*1000:NaN,1),
      format(result.path_length_m,2),
      format(result.min_costmap_lethal_clearance_m,2),
      format(typeof result.planner_process_cpu_s==='number'?result.planner_process_cpu_s*1000:NaN,1),
      format(result.planner_process_rss_mb,1),
      format(result.planner_process_rss_delta_mb,2)
    ];
    values.forEach((value,index)=>{
      const cell=document.createElement('td');
      cell.textContent=value;
      if(index===1)cell.className=result.status==='success'?'pass':'error';
      row.append(cell);
    });
    tbody.append(row);
    if(result.error){
      const explain=document.createElement('tr');
      const message=document.createElement('td');
      message.colSpan=9;message.className='error';message.textContent=name+': '+result.error;
      explain.append(message);tbody.append(explain);
    }
  }
  $('run-name').textContent=data?.run_name||'None';
  const summary=data?.summary;
  if(summary){
    $('result-note').textContent='Run: '+summary.status+
      ' · Frame: '+(summary.map_frame||'waiting')+
      ' · depth messages seen: '+(summary.camera_scan_messages_seen??0)+
      (summary.costmap_consistent===false?' · WARNING: costmap changed between planners':'');
  }else $('result-note').textContent='No results yet.';
}
function renderCalibration(data){
  calibration=data;
  const container=$('calibration-list');
  container.replaceChildren();
  const labels={
    lighting:'Lighting',camera:'Camera / D435i',apriltags:'AprilTags',
    mobility:'Mobility',odometry:'Odometry'
  };
  const details={
    lighting:data.lighting.light_rows+'×'+data.lighting.light_cols+
      ' lights · '+data.lighting.shadow_count+' shadows · venue from saved YAML',
    camera:data.camera.color+' @ '+data.camera.color_fps+
      ' Hz RGB · '+data.camera.depth+' @ '+data.camera.depth_fps+' Hz depth',
    apriltags:'Edge '+data.apriltags.size_m+' m · mount height '+data.apriltags.height_m+' m',
    mobility:'Wheel contact '+data.mobility.physics.wheel_collision_width+
      ' m · motor '+data.mobility.motor.model,
    odometry:'Wheel '+data.odometry.wheel_odom_hz+' Hz · IMU '+
      data.odometry.imu_hz+' Hz · EKF '+data.odometry.ekf_hz+' Hz'
  };
  for(const [key,title] of Object.entries(labels)){
    const origin=data.sources[key];
    const row=document.createElement('div');row.className='calibration-row';
    const left=document.createElement('div');
    const heading=document.createElement('strong');heading.textContent=title;
    const desc=document.createElement('small');desc.textContent=details[key];
    left.append(heading,desc);
    const badge=document.createElement('span');
    badge.className='calibration-origin'+(origin.source==='saved_lab_yaml'?'':' fallback');
    badge.textContent=origin.source==='saved_lab_yaml'?'Saved YAML':'Published fallback';
    badge.title=origin.path;
    row.append(left,badge);container.append(row);
  }
  $('depth-mode').textContent=data.camera.depth;
  $('depth-rate').textContent=data.camera.depth_fps+' FPS · saved Camera Lab · ~90.5° HFOV';
  const warning=[...(data.warnings||[])];
  if(data.restart_required)warning.unshift(
    'A source lab YAML has changed. Restart the simulation to apply the new calibration.');
  $('calibration-warning').textContent=warning.join(' ');
}
async function refreshCalibration(){
  try{renderCalibration(await api('/api/calibration'));}
  catch(e){
    $('calibration-list').textContent='Cannot load saved lab configuration: '+e.message;
    $('calibration-warning').textContent='Save the required Lighting, Camera and AprilTags YAMLs in their owning labs first.';
  }
}
function renderObstacles(obstacles){
  const list=$('obstacle-list');list.replaceChildren();
  if(!obstacles?.length){
    const p=document.createElement('p');p.className='muted';p.textContent='No active obstacles.';list.append(p);return;
  }
  for(const obstacle of obstacles){
    const row=document.createElement('div');row.className='obstacle-row';
    const summary=document.createElement('div');
    const strong=document.createElement('strong');strong.textContent=obstacle.name;
    const details=document.createElement('small');
    details.textContent=obstacle.kind+' · ('+format(obstacle.x,2)+', '+format(obstacle.y,2)+')'+
      (obstacle.moving?' · walking route':'');
    summary.append(strong,details);
    const remove=document.createElement('button');
    remove.className='btn small danger';remove.textContent='Remove';
    remove.disabled=busy||Boolean(status?.trial_running);
    remove.addEventListener('click',()=>operation(async()=>{
      await api('/api/obstacles/remove',{name:obstacle.name});showNotice('Obstacle removed');
    }));
    row.append(summary,remove);list.append(row);
  }
}
async function refresh(){
  try{
    status=await api('/api/status');
    $('connection').textContent='Local service connected';
    $('stack-dot').classList.toggle('on',status.stack_running);
    $('stack-state').textContent=status.stack_running?'Simulation running':'Simulation stopped';
    $('action-state').textContent=status.trial_running?'Planning experiment in progress':
      status.stack_running?'Planner stack running · ready for a request':'Simulation stopped';
    $('log').textContent=(status.trial_running?status.trial_log:status.stack_log)||'No logs yet.';
    if(status.summary?.error)$('log').textContent+='\n\nLast error: '+status.summary.error;
    $('move-error').textContent=status.moving_obstacle_error?('Moving obstacle: '+status.moving_obstacle_error):'';
    renderResults(status);
    renderObstacles(status.obstacles);
    updateButtons();
  }catch(e){
    $('connection').textContent='Service disconnected';
    $('action-state').textContent=e.message;
  }
}
async function refreshHistory(){
  try{
    const records=await api('/api/history');
    const dest=$('history');dest.replaceChildren();
    if(!records.length){
      const p=document.createElement('p');p.className='muted';p.textContent='No saved experiments.';dest.append(p);return;
    }
    const table=document.createElement('table');
    const head=document.createElement('tr');
    for(const t of ['Run','Status','Successful plans','Plan time range (ms)','Costmap stable']){
      const th=document.createElement('th');th.textContent=t;head.append(th);
    }
    table.append(head);
    for(const record of records){
      const s=record.summary;
      const results=Object.values(s.results||{});
      const times=results.filter(x=>x.status==='success').map(x=>x.planning_nav2_s*1000);
      const values=[record.name,s.status,
        results.filter(x=>x.status==='success').length+'/'+results.length,
        times.length?format(Math.min(...times),1)+' – '+format(Math.max(...times),1):'—',
        s.costmap_consistent===false?'No':'Yes / not applicable'];
      const row=document.createElement('tr');
      for(const value of values){const cell=document.createElement('td');cell.textContent=value;row.append(cell);}
      table.append(row);
    }
    dest.append(table);
  }catch(e){}
}
for(const button of document.querySelectorAll('.tab')){
  button.onclick=()=>{
    activePane=button.dataset.pane;
    document.querySelectorAll('.tab').forEach(e=>e.classList.toggle('active',e===button));
    document.querySelectorAll('.pane').forEach(e=>e.classList.toggle('active',e.id===activePane));
    if(activePane==='benchmarks')refreshHistory();
  };
}
$('obstacle-kind').onchange=()=>{
  const human=$('obstacle-kind').value==='human';
  $('obstacle-moving').disabled=!human;
  if(!human)$('obstacle-moving').checked=false;
};
$('obstacle-kind').onchange();
$('save').onclick=()=>operation(async()=>{
  await save();await refreshCalibration();
  showNotice('Saved Nav2 Lab YAML separately under tools/configs/labs/. Restart simulation after upstream calibration changes.');
});
$('start').onclick=()=>operation(async()=>{
  await save();await api('/api/start',{});await refreshCalibration();
  showNotice('Planner simulation launching with snapshots of saved Lighting, Camera, AprilTags and available Mobility/Odometry calibrations.');
});
$('stop').onclick=()=>operation(async()=>{
  await api('/api/stop',{});await refreshCalibration();
  showNotice('Stopping isolated simulation.');
});
$('rviz-open').onclick=()=>operation(async()=>{await api('/api/rviz/open',{});showNotice('RViz opening with planner path and costmap displays.');});
$('rviz-close').onclick=()=>operation(async()=>{await api('/api/rviz/close',{});showNotice('Closing RViz.');});
$('plan').onclick=()=>operation(async()=>{
  await save();
  await api('/api/plan',{});
  showNotice('ComputePathToPose benchmark started. The robot will NOT move.');
});
$('cancel').onclick=()=>operation(async()=>{await api('/api/cancel',{});showNotice('Planner cancellation requested.');});
$('add-obstacle').onclick=()=>operation(async()=>{
  const kind=$('obstacle-kind').value;
  await api('/api/obstacles/add',{
    kind,x:Number($('obstacle-x').value),y:Number($('obstacle-y').value),
    moving:kind==='human'&&$('obstacle-moving').checked
  });
  showNotice('Obstacle spawned in Gazebo. Check RViz depth scan and costmap for visibility.');
});
$('clear-obstacles').onclick=()=>operation(async()=>{await api('/api/obstacles/clear',{});showNotice('Lab obstacles removed.');});
(async()=>{
  try{populate(await api('/api/config'));await refresh();await refreshHistory();await refreshCalibration();}
  catch(e){showNotice(e.message,true);}
  setInterval(refresh,2200);
  setInterval(refreshHistory,15000);
  setInterval(refreshCalibration,12000);
})();
