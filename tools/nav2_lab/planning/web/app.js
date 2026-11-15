'use strict';
const $=id=>document.getElementById(id);
const SVG_NS='http://www.w3.org/2000/svg';
let saved=null,scene=null,status=null,calibration=null,busy=false;
let venue={x:21,y:12},historyReady=false;

async function api(route,body){
  const opts=body===undefined?{}:{method:'POST',
    headers:{'Content-Type':'application/json'},body:JSON.stringify(body)};
  const response=await fetch(route,{...opts,cache:'no-store'});
  const data=await response.json();
  if(!response.ok)throw new Error(data.error||('HTTP '+response.status));
  return data;
}
function notify(text,error=false){
  const n=$('notice');n.hidden=false;n.textContent=text;n.classList.toggle('error',error);
  clearTimeout(notify.timer);notify.timer=setTimeout(()=>n.hidden=true,error?14000:7500);
}
const n=id=>Number($(id).value);
const fmt=(v,d=2)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(d):'—';
function readRobot(){return{x:n('robot-x'),y:n('robot-y'),yaw_deg:n('robot-yaw')};}
function populateScene(payload){
  scene=structuredClone(payload.scene||payload);
  $('robot-x').value=scene.robot.x;$('robot-y').value=scene.robot.y;
  $('robot-yaw').value=scene.robot.yaw_deg;
  showScene();
}
function populatePlan(config){
  saved=structuredClone(config);const p=saved.planning;
  $('planner').value=p.planner;$('timeout').value=p.timeout_s;
  $('depth-enabled').checked=p.depth_obstacles;
  $('resolution').value=String(p.resolution_m);
  $('goal-x').value=p.goal.x;$('goal-y').value=p.goal.y;
  $('goal-yaw').value=p.goal.yaw_deg;
  $('domain').textContent=config.simulation.ros_domain_id;
}
function formPlan(){
  const p=structuredClone(saved.planning);
  p.planner=$('planner').value;p.timeout_s=n('timeout');
  p.depth_obstacles=$('depth-enabled').checked;p.resolution_m=n('resolution');
  p.goal={x:n('goal-x'),y:n('goal-y'),yaw_deg:n('goal-yaw')};
  return {planning:p};
}
async function savePlan(){
  const result=await api('/api/save',formPlan());
  saved=result.config;
}
function setBusy(v){busy=v;setButtons();}
async function operation(callback){
  if(busy)return;
  setBusy(true);
  try{await callback();await refresh();}
  catch(e){notify(e.message,true);}
  finally{setBusy(false);}
}
function setButtons(){
  const live=Boolean(status?.stack_running),running=Boolean(status?.trial_running);
  const buttons={
    start:!live,stop:live,plan:live&&!running,cancel:running,save:true,
    'scene-save':!running,'scene-load':!live,
    'place-robot':!running,
    'gui-open':live&&!status?.gazebo_gui?.running,
    'gui-close':live&&Boolean(status?.gazebo_gui?.running),
    'rviz-open':live&&!status?.rviz_running,
    'rviz-close':live&&status?.rviz_running,
    'add-box':!running,'add-human':!running,'clear-obstacles':!running
  };
  for(const[key,enabled]of Object.entries(buttons))$(key).disabled=busy||!enabled;
}
function tag(name,attrs={},text=''){
  const element=document.createElementNS(SVG_NS,name);
  for(const[k,v]of Object.entries(attrs))element.setAttribute(k,String(v));
  if(text)element.textContent=text;
  return element;
}
function worldToPixel(x,y){
  return [380+680*x/venue.x,215-340*y/venue.y];
}
function pixelToWorld(x,y){
  return [+(venue.x*(x-380)/680).toFixed(2),+(venue.y*(215-y)/340).toFixed(2)];
}
function plotCircle(g,x,y,r,fill,stroke='#f7f7f7'){
  const [px,py]=worldToPixel(x,y);
  g.append(tag('circle',{cx:px,cy:py,r,fill,stroke,'stroke-width':1.3}));
  return [px,py];
}
function plotLabel(g,x,y,text,color='#c9dce8'){
  const [px,py]=worldToPixel(x,y);
  g.append(tag('text',{x:px+9,y:py-9,fill:color,'font-size':13,
                       'font-weight':600},text));
}
function createCourt(){
  const g=$('courtSurface');g.replaceChildren();
  g.append(tag('rect',{x:40,y:45,width:680,height:340,rx:7,fill:'#1d3a35',
                       stroke:'#6b9a88','stroke-width':2}));
  const [left,top]=worldToPixel(-7.7,3.6);
  const width=15.4*680/venue.x,height=7.2*340/venue.y;
  g.append(tag('rect',{x:left,y:top,width,height,fill:'#24584a',stroke:'#c7e0d1',
                       'stroke-width':2}));
  // Standard official court and service lines, selector only (not Nav2 map).
  for(const y of [-3.05,0,3.05]){
    const a=worldToPixel(-7.7,y),b=worldToPixel(7.7,y);
    g.append(tag('line',{x1:a[0],y1:a[1],x2:b[0],y2:b[1],
                         stroke:'#b1e0cc','stroke-opacity':.65,'stroke-width':1}));
  }
  for(const x of [-7.7,-5.94,-1.98,0,1.98,5.94,7.7]){
    const a=worldToPixel(x,-3.6),b=worldToPixel(x,3.6);
    g.append(tag('line',{x1:a[0],y1:a[1],x2:b[0],y2:b[1],
                         stroke:'#9dcbb8','stroke-opacity':.42,'stroke-width':1}));
  }
  for(const y of [-3.05,3.05])plotCircle(g,0,y,4,'#f1b77b','#815730');
  const [a,b]=worldToPixel(-venue.x/2,-venue.y/2);
  g.append(tag('text',{x:47,y:38,fill:'#a2cbbd','font-size':12},
               'Court-fixed map coordinates (+Y up)'));
}
function showScene(){
  if(!scene)return;
  createCourt();
  const obstacles=$('courtObstacles'),draft=$('courtDraft');
  obstacles.replaceChildren();draft.replaceChildren();
  const active=scene.obstacles||[];
  for(const [index,item]of active.entries()){
    if(item.kind==='box'){
      const [px,py]=worldToPixel(item.x,item.y);
      obstacles.append(tag('rect',{x:px-item.size*680/venue.x/2,
        y:py-item.size*340/venue.y/2,width:item.size*680/venue.x,
        height:item.size*340/venue.y,fill:'#ba8154',stroke:'#f6d1a3',
        'stroke-width':2}));
      plotLabel(obstacles,item.x,item.y,'Cube '+(index+1));
    }else{
      const [ax,ay]=worldToPixel(item.point_a.x,item.point_a.y);
      const [bx,by]=worldToPixel(item.point_b.x,item.point_b.y);
      obstacles.append(tag('line',{x1:ax,y1:ay,x2:bx,y2:by,
        stroke:'#7aabfa','stroke-width':3,'stroke-dasharray':'8 5'}));
      plotCircle(obstacles,item.point_a.x,item.point_a.y,5,'#7aabfa');
      plotCircle(obstacles,item.point_b.x,item.point_b.y,5,'#7aabfa');
      plotLabel(obstacles,item.point_a.x,item.point_a.y,'Human '+(index+1));
    }
  }
  // Show editable robot pose instead of an unrelated imaginary start position.
  const robot=readRobot();
  if(Number.isFinite(robot.x)&&Number.isFinite(robot.y)&&Number.isFinite(robot.yaw_deg)){
    const [x,y]=plotCircle(draft,robot.x,robot.y,8,'#f2bf71');
    const yaw=robot.yaw_deg*Math.PI/180;
    draft.append(tag('line',{x1:x,y1:y,x2:x+22*Math.cos(yaw),
      y2:y-22*Math.sin(yaw),stroke:'#f5cb82','stroke-width':4,'stroke-linecap':'round'}));
    plotLabel(draft,robot.x,robot.y,'Robot');
  }
  plotCircle(draft,n('goal-x'),n('goal-y'),6,'#5bd7b6');
  plotLabel(draft,n('goal-x'),n('goal-y'),'Goal');
  const [cx,cy]=worldToPixel(n('obstacle-x'),n('obstacle-y'));
  const size=n('box-size');
  draft.append(tag('rect',{x:cx-size*680/venue.x/2,y:cy-size*340/venue.y/2,
    width:size*680/venue.x,height:size*340/venue.y,fill:'none',
    stroke:'#f0c19b','stroke-width':1.5,'stroke-dasharray':'4 4'}));
  const a=worldToPixel(n('human-ax'),n('human-ay'));
  const b=worldToPixel(n('human-bx'),n('human-by'));
  draft.append(tag('line',{x1:a[0],y1:a[1],x2:b[0],y2:b[1],
    stroke:'#9ac0f8','stroke-width':1.5,'stroke-dasharray':'5 6'}));
  plotCircle(draft,n('human-ax'),n('human-ay'),5,'#7aabfa');
  plotCircle(draft,n('human-bx'),n('human-by'),5,'#7aabfa');
}
function picked(x,y){
  const mode=$('court-pick').value;
  const ids={
    robot:['robot-x','robot-y'],goal:['goal-x','goal-y'],
    box:['obstacle-x','obstacle-y'],
    'human-a':['human-ax','human-ay'],'human-b':['human-bx','human-by']
  }[mode];
  if(!ids)return;
  $(ids[0]).value=x;$(ids[1]).value=y;
  showScene();
}
$('courtMap').addEventListener('click',event=>{
  const svg=$('courtMap'),matrix=svg.getScreenCTM();
  if(!matrix)return;
  const point=svg.createSVGPoint();point.x=event.clientX;point.y=event.clientY;
  const local=point.matrixTransform(matrix.inverse());
  if(local.x<40||local.x>720||local.y<45||local.y>385)return;
  picked(...pixelToWorld(local.x,local.y));
});
$('courtMap').addEventListener('keydown',event=>{
  if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(event.key))return;
  event.preventDefault();
  const ids={
    robot:['robot-x','robot-y'],goal:['goal-x','goal-y'],
    box:['obstacle-x','obstacle-y'],
    'human-a':['human-ax','human-ay'],'human-b':['human-bx','human-by']
  }[$('court-pick').value];
  if(!ids)return;
  let x=n(ids[0]),y=n(ids[1]);
  if(event.key==='ArrowLeft')x-=.1;
  if(event.key==='ArrowRight')x+=.1;
  if(event.key==='ArrowUp')y+=.1;
  if(event.key==='ArrowDown')y-=.1;
  picked(+x.toFixed(2),+y.toFixed(2));
});
for(const id of ['robot-x','robot-y','robot-yaw','goal-x','goal-y','goal-yaw',
   'obstacle-x','obstacle-y','human-ax','human-ay','human-bx','human-by']){
  $(id).addEventListener('input',showScene);
}
for(const [slider,label]of [['box-size','box-size-value'],['human-speed','human-speed-value']]){
  $(slider).addEventListener('input',()=>{
    $(label).textContent=fmt(n(slider),2)+' m'+(slider==='human-speed'?'/s':'');
    showScene();
  });
}
function renderObstacles(items){
  const list=$('obstacle-list');list.replaceChildren();
  if(!items?.length){const p=document.createElement('p');
    p.className='muted';p.textContent='No obstacles yet. Add a cube or walking human.';list.append(p);return;}
  items.forEach((o,i)=>{
    const row=document.createElement('div');row.className='obstacle-row';
    const info=document.createElement('div');
    const title=document.createElement('strong');
    title.textContent=o.kind==='box'?'Cube '+(i+1):'Human '+(i+1);
    const detail=document.createElement('small');
    detail.textContent=o.kind==='box'?
      '('+fmt(o.x)+', '+fmt(o.y)+') · '+fmt(o.size)+' m cube':
      'A ('+fmt(o.point_a.x)+', '+fmt(o.point_a.y)+') → B ('+
      fmt(o.point_b.x)+', '+fmt(o.point_b.y)+') · '+fmt(o.speed_mps)+' m/s';
    info.append(title,detail);
    const remove=document.createElement('button');
    remove.className='btn small danger';remove.textContent='Remove';
    remove.disabled=busy||Boolean(status?.trial_running);
    remove.onclick=()=>operation(async()=>{
      const updated=await api('/api/obstacles/remove',{index:i});
      scene=updated.scene;renderObstacles(scene.obstacles);showScene();
    });
    row.append(info,remove);list.append(row);
  });
}
function renderCalibration(data){
  calibration=data;
  const block=$('calibration-list');block.replaceChildren();
  const labels={
    lighting:['Lighting',data.lighting.light_rows+'×'+data.lighting.light_cols+
      ' fixtures · '+data.lighting.shadow_count+' shadows'],
    camera:['Camera',data.camera.color+' @ '+data.camera.color_fps+' Hz RGB · '+
      data.camera.depth+' @ '+data.camera.depth_fps+' Hz depth'],
    apriltags:['AprilTags','Edge '+data.apriltags.size_m+' m · height '+data.apriltags.height_m+' m'],
    mobility:['Mobility','Wheel contact '+data.mobility.physics.wheel_collision_width+' m'],
    odometry:['Odometry','Wheel '+data.odometry.wheel_odom_hz+' Hz · IMU '+
      data.odometry.imu_hz+' Hz · EKF '+data.odometry.ekf_hz+' Hz']
  };
  for(const [key,value]of Object.entries(labels)){
    const row=document.createElement('div');row.className='calibration-row';
    const name=document.createElement('div');const title=document.createElement('strong');
    title.textContent=value[0];const desc=document.createElement('small');
    desc.textContent=value[1];name.append(title,desc);
    const badge=document.createElement('span');badge.className='calibration-origin';
    badge.textContent=data.sources[key].source==='saved_lab_yaml'?'Saved YAML':'Fallback';
    badge.title=data.sources[key].path;
    if(data.sources[key].source!=='saved_lab_yaml')badge.classList.add('fallback');
    row.append(name,badge);block.append(row);
  }
  $('depth-mode').textContent=data.camera.depth;
  $('depth-rate').textContent=data.camera.depth_fps+' FPS · Camera Lab';
  venue={x:15.4+2*data.lighting.venue.clearance_x,
         y:7.2+2*data.lighting.venue.clearance_y};
  $('court-coordinates').textContent='Venue '+fmt(venue.x,2)+' × '+fmt(venue.y,2)+
    ' m · click to place in '+$('court-pick').selectedOptions[0].text;
  const warn=[...(data.warnings||[])];
  if(data.restart_required)warn.unshift('A saved upstream calibration changed: restart simulation.');
  $('calibration-warning').textContent=warn.join(' ');
  showScene();
}
async function refreshCalibration(){
  try{renderCalibration(await api('/api/calibration'));}
  catch(error){$('calibration-warning').textContent=error.message;}
}
function updateMapStatus(health){
  const label=(id,value)=>{$(id).textContent=value?'Receiving':'Waiting';};
  if(!health){
    for(const id of ['map-state','costmap-state','tf-state'])$(id).textContent='Waiting';
    $('map-hint').textContent='Map diagnostics will appear after launching Nav2.';
    return;
  }
  const m=health.static_map||{},c=health.global_costmap||{};
  label('map-state',Boolean(m.received));
  label('costmap-state',Boolean(c.received));
  label('tf-state',Boolean(health.tf?.map_to_base_footprint));
  $('map-meta').textContent=m.received?
    m.width+' × '+m.height+' cells · '+m.occupied_cells+' occupied':
    'No /map message received';
  $('costmap-meta').textContent=c.received?
    c.width+' × '+c.height+' cells · '+c.occupied_cells+' occupied':
    'No global costmap message received';
  $('map-hint').textContent=health.hint||'Checking ROS topics…';
}
function renderResults(summary){
  const rows=$('results');rows.replaceChildren();
  for(const [name,result]of Object.entries(summary?.results||{})){
    const row=document.createElement('tr');
    const values=[name,result.status,
      fmt(typeof result.planning_nav2_s==='number'?result.planning_nav2_s*1000:NaN,1),
      fmt(typeof result.planning_wall_s==='number'?result.planning_wall_s*1000:NaN,1),
      fmt(result.path_length_m),fmt(result.min_costmap_lethal_clearance_m),
      fmt(typeof result.planner_process_cpu_s==='number'?result.planner_process_cpu_s*1000:NaN,1),
      fmt(result.planner_process_rss_mb,1),fmt(result.planner_process_rss_delta_mb)];
    values.forEach((v,i)=>{const td=document.createElement('td');td.textContent=v;
      if(i===1)td.className=result.status==='success'?'pass':'error';row.append(td);});
    rows.append(row);
    if(result.error){const extra=document.createElement('tr'),td=document.createElement('td');
      td.colSpan=9;td.className='error';td.textContent=name+': '+result.error;
      extra.append(td);rows.append(extra);}
  }
  $('result-note').textContent=summary?
    'Status '+summary.status+' · map frame '+(summary.map_frame||'—')+
    ' · depth scans received '+(summary.camera_scan_messages_seen??0)+
    (summary.costmap_consistent===false?' · costmap changed between planners':''):
    'No planner benchmark started.';
}
async function refreshHistory(){
  try{
    const records=await api('/api/history');
    const area=$('history');area.replaceChildren();
    if(!records.length){area.textContent='No saved experiments yet.';return;}
    const table=document.createElement('table'),header=document.createElement('tr');
    ['Run','Status','Plans passed','Planning time (ms)'].forEach(v=>{
      const th=document.createElement('th');th.textContent=v;header.append(th);
    });table.append(header);
    for(const row of records){
      const results=Object.values(row.summary.results||{});
      const times=results.filter(v=>v.status==='success').map(v=>1000*v.planning_nav2_s);
      const vals=[row.name,row.summary.status,
        results.filter(v=>v.status==='success').length+'/'+results.length,
        times.length?fmt(Math.min(...times),1)+'…'+fmt(Math.max(...times),1):'—'];
      const tr=document.createElement('tr');
      vals.forEach(v=>{const td=document.createElement('td');td.textContent=v;tr.append(td);});
      table.append(tr);
    }
    area.append(table);
  }catch(e){}
}
async function refresh(){
  try{
    status=await api('/api/status');
    $('connection').textContent='Local service connected';
    $('stack-dot').classList.toggle('on',status.stack_running);
    $('stack-state').textContent=status.stack_running?'Simulation running':'Simulation stopped';
    $('action-state').textContent=status.trial_running?'Planning…':
      status.stack_running?'Simulation active · robot stays stationary':'Simulation stopped';
    $('scene-warning').textContent=status.scene_restart_required?
      'Robot coordinates changed since Gazebo launch. Use Place Robot to restart and align TF.':'';
    $('move-error').textContent=status.moving_obstacle_error||'';
    $('log').textContent=status.stack_log||'No stack output.';
    $('navTrialLog').textContent=status.trial_log||'No trial output.';
    $('navGuiLog').textContent=status.gui_log||'Gazebo UI not opened.';
    $('navRvizLog').textContent=status.rviz_log||'RViz not opened.';
    $('run-name').textContent=status.run_name||'No experiment';
    updateMapStatus(status.map_status);
    renderResults(status.summary);
    setButtons();
  }catch(e){$('connection').textContent='Disconnected';$('action-state').textContent=e.message;}
}
$('save').onclick=()=>operation(async()=>{
  await savePlan();notify('Saved Nav2 planner settings to shared lab YAML.');
});
$('scene-save').onclick=()=>operation(async()=>{
  const data=await api('/api/scene/save',{robot:readRobot()});
  scene=data.scene;showScene();
  notify('Scene saved: tools/configs/labs/nav2_scene.yaml. Restart if robot pose has changed.');
});
$('scene-load').onclick=()=>operation(async()=>{
  const data=await api('/api/scene/load',{});
  populateScene(data);notify('Saved scene loaded; restart to apply it.');
});
$('place-robot').onclick=()=>operation(async()=>{
  const data=await api('/api/scene/place-robot',{robot:readRobot()});
  scene=data.scene||scene;await refresh();
  notify(status?.stack_running?'Robot placed; restarting simulation to reset TF and odometry.':
    'Robot starting pose set. Start simulation to spawn it.');
});
$('add-box').onclick=()=>operation(async()=>{
  const result=await api('/api/obstacles/add',{obstacle:{
    kind:'box',x:n('obstacle-x'),y:n('obstacle-y'),size:n('box-size')}});
  scene=result.scene;renderObstacles(scene.obstacles);showScene();
  notify('Cube added to '+(status?.stack_running?'Gazebo':'the staged scene')+'.');
});
$('add-human').onclick=()=>operation(async()=>{
  const result=await api('/api/obstacles/add',{obstacle:{
    kind:'human',point_a:{x:n('human-ax'),y:n('human-ay')},
    point_b:{x:n('human-bx'),y:n('human-by')},speed_mps:n('human-speed')}});
  scene=result.scene;renderObstacles(scene.obstacles);showScene();
  notify('Moving human added to '+(status?.stack_running?'Gazebo':'the staged scene')+'.');
});
$('clear-obstacles').onclick=()=>operation(async()=>{
  const result=await api('/api/obstacles/clear',{});
  scene=result.scene;renderObstacles(scene.obstacles);showScene();notify('All obstacles cleared.');
});
$('start').onclick=()=>operation(async()=>{
  // Apply editable robot pose without silently writing the scene YAML.
  await api('/api/scene/place-robot',{robot:readRobot()});
  await savePlan();await api('/api/start',{});
  await refreshCalibration();
  notify('Planner-only simulation launching from the selected court pose.');
});
$('stop').onclick=()=>operation(async()=>{await api('/api/stop',{});notify('Stopping lab simulation.');});
$('plan').onclick=()=>operation(async()=>{
  await savePlan();await api('/api/plan',{});
  notify('Planner comparison started. No drive commands will be sent.');
});
$('cancel').onclick=()=>operation(async()=>{await api('/api/cancel',{});notify('Cancel requested.');});
$('gui-open').onclick=()=>operation(async()=>{await api('/api/gazebo-gui/open',{});notify('Opening Gazebo GUI.');});
$('gui-close').onclick=()=>operation(async()=>{await api('/api/gazebo-gui/close',{});notify('Closed Gazebo GUI.');});
$('rviz-open').onclick=()=>operation(async()=>{await api('/api/rviz/open',{});notify('RViz opening with court-fixed map frame.');});
$('rviz-close').onclick=()=>operation(async()=>{await api('/api/rviz/close',{});notify('RViz closed.');});
(async()=>{
  try{
    populatePlan(await api('/api/config'));
    populateScene(await api('/api/scene'));
    renderObstacles(scene.obstacles);
    await refreshCalibration();await refresh();await refreshHistory();
  }catch(e){notify(e.message,true);}
  setInterval(refresh,2300);
  setInterval(refreshHistory,15000);
  setInterval(refreshCalibration,15000);
})();
