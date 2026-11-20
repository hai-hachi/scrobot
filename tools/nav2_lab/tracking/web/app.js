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
const plotRuns=new Map();
const pendingPlotRuns=new Set();
const MAX_PLOT_RUNS=8;
const PLOT_PALETTE=['#55C9F6','#B89BFF','#50DDB9','#FF86A2',
  '#E8D46C','#8CACFF','#DE96CE','#82E3E6'];
const LIVE_PLOT_COLOR='#F4BB66';
let timelineData=null;
let timelineName='';
let physicalLimits=null;


function showPhysicalLimits(limits){
  physicalLimits=limits;
  $('physical-wheel-diameter').textContent=limits.wheel_diameter_m.toFixed(3)+' m';
  $('physical-wheelbase').textContent=limits.wheel_separation_m.toFixed(3)+' m';
  $('physical-max-rpm').textContent=limits.wheel_max_rpm.toFixed(1)+' RPM';
  $('physical-max-linear').textContent=limits.max_linear_mps.toFixed(4)+' m/s';
  $('physical-max-angular').textContent=limits.max_angular_radps.toFixed(4)+' rad/s';
  const h=limits.wheel_half_separation_m.toFixed(3);
  const v=limits.max_wheel_mps.toFixed(4);
  $('physical-coupled-envelope').textContent=
    'Each wheel is individually limited to 95 RPM: |v − '+h+'ω| ≤ '+v+
    ' and |v + '+h+'ω| ≤ '+v+' m/s. '+
    'Equivalently, |v| + '+h+'|ω| ≤ '+v+' m/s.';
  for(const id of ['desired-speed','rpp-min-speed','rpp-min-approach']){
    $(id).max=String(limits.max_linear_mps);
  }
  for(const id of ['rpp-rotate-rate','smc1-max-angular-cmd',
                   'smc2-max-angular-cmd']){
    $(id).max=String(limits.max_angular_radps);
  }
  updateTurningAllowance();
}
function updateTurningAllowance(){
  if(!physicalLimits)return;
  const v=Math.abs(num('desired-speed'));
  const remaining=physicalLimits.max_wheel_mps-v;
  const omega=Math.max(0,remaining)/physicalLimits.wheel_half_separation_m;
  $('physical-turning-allowance').textContent=
    'At requested forward speed '+fmt(v,3)+' m/s, wheel RPM allows '+
    fmt(omega,3)+' rad/s simultaneous rotation. '+
    'At v = 0, the physical maximum is '+
    fmt(physicalLimits.max_angular_radps,3)+' rad/s. '+
    'The coupled wheel limiter scales both v and ω when necessary.';
}

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
    'reset-shared':!trial,'reset-rpp':!trial,'reset-smc_1s':!trial,'reset-smc_2s':!trial,
    'preview-button':!trial,'new-seed':!trial,
    'start':!stack&&!trial,'run':!trial&&!lastStatus?.controller_error,'cancel':trial,
    'stop':stack,'reset':!trial,
    'gz-open':stack&&!lastStatus?.gazebo_gui?.running,
    'gz-close':stack&&Boolean(lastStatus?.gazebo_gui?.running),
    'rviz-open':stack&&!lastStatus?.rviz_running,
    'rviz-close':stack&&Boolean(lastStatus?.rviz_running),
    'clear-history':!trial,
    'clear-plot':plotRuns.size>0||pendingPlotRuns.size>0
  };
  for(const [id,available] of Object.entries(controls))$(id).disabled=busy||!available;
}
const smcKeys=['lambda_y','k_s','k_x','eta','phi','eta_x','phi_x',
  'singular_guard','max_angular_cmd','max_linear_accel','max_angular_accel'];
function smcInputId(group,key){
  return 'smc'+group+'-'+key.replaceAll('_','-');
}
function readFullSMC(group){
  const keys=group==='1'?[...smcKeys,'lambda_x']:smcKeys;
  const out=Object.fromEntries(keys.map(key=>[key,num(smcInputId(group,key))]));
  out.use_saturation=$(smcInputId(group,'use_saturation')).checked;
  return out;
}
function setFullSMC(group,config){
  for(const [key,value] of Object.entries(config)){
    const element=$(smcInputId(group,key));
    if(key==='use_saturation')element.checked=value;
    else element.value=value;
  }
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
      progress_timeout_sim_s:num('progress-timeout'),
      lookahead_radius_m:num('carrot-radius'),
      curvature_window_m:num('curvature-window')
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
    smc_1s:readFullSMC('1'),
    smc_2s:readFullSMC('2')
  };
}
function setForm(x){
  settings=structuredClone(x);
  const shared=x.shared,rpp=x.rpp;
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
    'carrot-radius':shared.lookahead_radius_m,
    'curvature-window':shared.curvature_window_m,
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
  };
  for(const [id,v] of Object.entries(fields))$(id).value=v;
  $('rpp-velocity-lookahead').checked=rpp.use_velocity_scaled_lookahead_dist;
  $('rpp-curvature-slow').checked=rpp.use_regulated_linear_velocity_scaling;
  $('rpp-rotate').checked=rpp.use_rotate_to_heading;
  setFullSMC('1',x.smc_1s);
  setFullSMC('2',x.smc_2s);
  window.TrackerMath?.refresh();
  updateTurningAllowance();
  spawnVisible();
  highlightActiveController();
}
function highlightActiveController(){
  const selected=$('controller-select').value.toLowerCase();
  for(const name of ['rpp','smc_1s','smc_2s']){
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
function chartText(group,content,attrs){
  const node=shape('text',attrs);
  node.textContent=content;
  group.append(node);
}
function plotLines(){
  const lines=[];
  const currentName=lastStatus?.run_name;
  // Avoid drawing the latest experiment twice if its complete saved trace
  // has already been chosen from the history table.
  if($('plot-live').checked&&!plotRuns.has(currentName)&&live?.trace?.length>=2){
    lines.push({
      name:'Current / latest '+(lastStatus?.summary?.controller||'trial'),
      path:lastStatus?.summary?.path||null,
      color:LIVE_PLOT_COLOR,
      points:live.trace.map(p=>({t:p.t,e:p.e}))
    });
  }
  for(const [name,run] of plotRuns){
    lines.push({
      name:run.controller+' · '+run.path+' · '+name+
        (typeof run.cross_track_rmse_m==='number'?
          ' · RMSE '+run.cross_track_rmse_m.toFixed(3)+' m':''),
      path:run.path,color:run.color,points:run.points
    });
  }
  return lines;
}
function renderChart(){
  const grid=$('error-grid'),plot=$('error-series');
  const legend=$('plot-legend');
  grid.replaceChildren();plot.replaceChildren();legend.replaceChildren();
  const lines=plotLines();
  const scales=window.TrackerChartScales.scales(lines);
  const fmtAxis=window.TrackerChartScales.label;
  // SVG dimensions are fixed in viewBox; the browser scales it responsively.
  const L=77,R=824,T=18,B=196;
  const xTo=t=>L+(t-scales.xMin)/(scales.xMax-scales.xMin)*(R-L);
  const yTo=e=>B-(e-scales.yMin)/(scales.yMax-scales.yMin)*(B-T);
  // Explicit numeric ticks in simulated seconds and signed metres.
  for(const t of scales.xTicks){
    const x=xTo(t);
    addLine(grid,[x,T],[x,B],'#293F50',.8,'3 5');
    chartText(grid,fmtAxis(t,scales.xStep),
      {x,y:B+20,'text-anchor':'middle',class:'tick-label'});
  }
  for(const e of scales.yTicks){
    const y=yTo(e);
    addLine(grid,[L,y],[R,y],
      Math.abs(e)<1e-9?'#6B929F':'#293F50',Math.abs(e)<1e-9?1.5:.8,
      Math.abs(e)<1e-9?'':'3 5');
    chartText(grid,fmtAxis(e,scales.yStep),
      {x:L-10,y:y+4,'text-anchor':'end',class:'tick-label'});
  }
  addLine(grid,[L,T],[L,B],'#839CAB',1.4);
  addLine(grid,[L,B],[R,B],'#839CAB',1.4);
  chartText(grid,'Elapsed simulation time (s)',
    {x:(L+R)/2,y:248,'text-anchor':'middle',class:'axis-label'});
  chartText(grid,'Cross-track error (m)',
    {transform:'translate(16 110) rotate(-90)',
     x:0,y:0,'text-anchor':'middle',class:'axis-label'});

  if(!lines.length){
    chartText(plot,'Select saved runs below or start a tracking trial',
      {x:(L+R)/2,y:(T+B)/2,'text-anchor':'middle',class:'empty-chart'});
  }
  for(const line of lines){
    const points=(line.points||[]).filter(p=>
      Number.isFinite(p.t)&&Number.isFinite(p.e)&&p.t>=0);
    if(points.length>=2){
      plot.append(shape('polyline',{
        points:points.map(p=>xTo(p.t).toFixed(2)+','+yTo(p.e).toFixed(2)).join(' '),
        fill:'none',stroke:line.color,'stroke-width':2.7,
        'stroke-linecap':'round','stroke-linejoin':'round'
      }));
    }
    const item=document.createElement('span');
    item.className='plot-legend-item';
    const swatch=document.createElement('span');
    swatch.className='plot-legend-swatch';
    swatch.style.borderColor=line.color;
    const label=document.createElement('span');
    label.className='plot-legend-name';
    label.textContent=line.name;
    item.append(swatch,label);
    legend.append(item);
  }
  if(!lines.length)legend.textContent='No curves visible.';
  const paths=new Set(lines.map(line=>line.path).filter(Boolean));
  const saved=[...plotRuns.values()];
  const poseKeys=new Set(saved.filter(p=>p.start_pose).map(p=>
    JSON.stringify([p.start_pose.x,p.start_pose.y,p.start_pose.yaw_deg])));
  const speeds=new Set(saved.filter(p=>Number.isFinite(p.requested_speed_mps))
    .map(p=>Number(p.requested_speed_mps).toFixed(4)));
  const cautions=[];
  if(paths.size>1)cautions.push('Different paths');
  if(poseKeys.size>1)cautions.push('different start poses');
  if(speeds.size>1)cautions.push('different reference speeds');
  $('plot-status').textContent=plotRuns.size+' saved run'+
    (plotRuns.size===1?'':'s')+' selected (max '+MAX_PLOT_RUNS+
    '). '+(cautions.length?
      'Comparison caution: '+cautions.join(', ')+'. ':
      '')+
    'Curves share the same axes and begin at t = 0; no time normalization.';
}
function nextPlotColor(){
  const used=new Set([...plotRuns.values()].map(run=>run.color));
  return PLOT_PALETTE.find(color=>!used.has(color))||PLOT_PALETTE[0];
}
function syncPlotCheckboxes(){
  const reached=plotRuns.size+pendingPlotRuns.size>=MAX_PLOT_RUNS;
  for(const checkbox of document.querySelectorAll('.plot-checkbox')){
    const name=checkbox.dataset.plotName;
    checkbox.checked=plotRuns.has(name)||pendingPlotRuns.has(name);
    checkbox.disabled=!checkbox.dataset.hasTrace||
      pendingPlotRuns.has(name)||(reached&&!checkbox.checked);
    checkbox.closest('tr')?.classList.toggle('plot-selected',checkbox.checked);
  }
  $('clear-plot').disabled=busy||(!plotRuns.size&&!pendingPlotRuns.size);
}
async function selectSavedTrace(name,checked){
  if(!checked){
    pendingPlotRuns.delete(name);
    plotRuns.delete(name);
    syncPlotCheckboxes();renderChart();
    return;
  }
  if(plotRuns.has(name)||pendingPlotRuns.has(name))return;
  if(plotRuns.size+pendingPlotRuns.size>=MAX_PLOT_RUNS){
    notify('Only '+MAX_PLOT_RUNS+' comparison runs can be plotted at once.',true);
    syncPlotCheckboxes();
    return;
  }
  pendingPlotRuns.add(name);
  syncPlotCheckboxes();
  try{
    const trace=await api('/history/trace?name='+encodeURIComponent(name));
    if(!pendingPlotRuns.has(name))return; // cleared during an in-flight fetch
    if(!Array.isArray(trace.points)||trace.points.length<2)
      throw new Error('Saved experiment has no usable trace samples');
    plotRuns.set(name,{...trace,color:nextPlotColor()});
  }catch(e){
    notify('Could not load '+name+': '+e.message,true);
  }finally{
    pendingPlotRuns.delete(name);
    syncPlotCheckboxes();
    renderChart();
  }
}

function phaseAxes(svg,xlo,xhi,ylo,yhi,xLabel,yLabel){
  svg.replaceChildren();
  const L=39,R=311,T=17,B=189;
  const X=x=>L+(x-xlo)/(xhi-xlo)*(R-L);
  const Y=y=>B-(y-ylo)/(yhi-ylo)*(B-T);
  for(let i=0;i<=4;i++){
    const x=xlo+(xhi-xlo)*i/4,y=ylo+(yhi-ylo)*i/4;
    addLine(svg,[X(x),T],[X(x),B],'#304552',.55,'3 4');
    addLine(svg,[L,Y(y)],[R,Y(y)],'#304552',.55,'3 4');
    chartText(svg,x.toFixed(2),{x:X(x),y:204,'text-anchor':'middle',class:'tick-label'});
    chartText(svg,y.toFixed(2),{x:35,y:Y(y)+2,'text-anchor':'end',class:'tick-label'});
  }
  chartText(svg,xLabel,{x:172,y:218,'text-anchor':'middle',class:'axis-label'});
  chartText(svg,yLabel,{transform:'translate(8 105) rotate(-90)',
    x:0,y:0,'text-anchor':'middle',class:'axis-label'});
  if(xlo<=0&&xhi>=0)addLine(svg,[X(0),T],[X(0),B],'#68949C',.8);
  if(ylo<=0&&yhi>=0)addLine(svg,[L,Y(0)],[R,Y(0)],'#68949C',.8);
  return {X,Y,L,R,T,B};
}
function phaseTrace(svg,coords,axes,color='#F4BB66'){
  const points=coords.filter(p=>Number.isFinite(p[0])&&Number.isFinite(p[1]));
  if(points.length<2)return;
  svg.append(shape('polyline',{
    points:points.map(p=>axes.X(p[0]).toFixed(2)+','+
      axes.Y(p[1]).toFixed(2)).join(' '),
    fill:'none',stroke:color,'stroke-width':2
  }));
  const last=points[points.length-1];
  svg.append(shape('circle',{cx:axes.X(last[0]),cy:axes.Y(last[1]),
    r:3.4,fill:color}));
}
function polylineEquation(svg,xlo,xhi,axes,equation,color,dash=''){
  const coords=Array.from({length:151},(_,i)=>{
    const x=xlo+(xhi-xlo)*i/150;return [x,equation(x)];
  });
  const path=shape('polyline',{
    points:coords.map(([x,y])=>axes.X(x).toFixed(2)+','+
      axes.Y(y).toFixed(2)).join(' '),
    fill:'none',stroke:color,'stroke-width':1.5
  });
  if(dash)path.setAttribute('stroke-dasharray',dash);
  svg.append(path);
}
function verticalBoundary(svg,axes,min,max){
  const x1=axes.X(min),x2=axes.X(max);
  svg.insertBefore(shape('rect',{
    x:Math.min(x1,x2),y:axes.T,width:Math.abs(x2-x1),
    height:axes.B-axes.T,fill:'#59C9AE','fill-opacity':.10
  }),svg.firstChild);
}
function sat(x){return Math.max(-1,Math.min(1,x));}
function renderPhasePortraits(){
  const frames=timelineData?.frames||[];
  const which=timelineData?.controller||$('controller-select').value;
  const name=which==='SMC_2S'?'smc_2s':'smc_1s';
  const params=timelineData?.config?.[name]||readForm()?.[name]||
    defaultSettings?.[name];
  if(!params)return;
  const errors=frames.map(f=>f.errors||{});
  const xs=errors.map(e=>e.ex).filter(Number.isFinite);
  const ys=errors.map(e=>e.ey).filter(Number.isFinite);
  const hs=errors.map(e=>e.etheta).filter(Number.isFinite);
  const bound=Math.max(.6,...ys.map(Math.abs),...hs.map(Math.abs));
  const pxy=phaseAxes($('phase-ey'),-bound,bound,-bound,bound,'ey (m)','eθ (rad)');
  // Display s=0 slice at ex=0 and the angular boundary layer.
  const yline=x=>-params.lambda_y*x;
  const lines=Array.from({length:100},(_,i)=>{
    const x=-bound+2*bound*i/99;
    const base=yline(x);
    return [x,base+params.phi,x,base-params.phi];
  });
  if(which!=='RPP'){
    const band=shape('polygon',{
      points:[...lines.map(v=>[pxy.X(v[0]),pxy.Y(v[1])]),
        ...lines.slice().reverse().map(v=>[pxy.X(v[2]),pxy.Y(v[3])])]
        .map(v=>v.join(',')).join(' '),
      fill:'#59C9AE','fill-opacity':.12
    });
    $('phase-ey').insertBefore(band,$('phase-ey').firstChild);
    polylineEquation($('phase-ey'),-bound,bound,pxy,yline,'#55C9B7','5 3');
  }
  phaseTrace($('phase-ey'),errors.map(e=>[e.ey,e.etheta]),pxy);
  const pexey=phaseAxes($('phase-exey'),-bound,bound,-bound,bound,
    'ex (m)','ey (m)');
  phaseTrace($('phase-exey'),errors.map(e=>[e.ex,e.ey]),pexey);
  if(which==='RPP'){
    for(const [id,xl,yl] of [
      ['phase-s','s','ds/dt'],['phase-ex','ex (m)','dex/dt'],
      ['sat-plot','z','sat(z)']
    ]){
      const svg=$(id);
      phaseAxes(svg,-1,1,-1,1,xl,yl);
      chartText(svg,'SMC diagnostic · not applicable to RPP',
        {x:176,y:98,'text-anchor':'middle',class:'empty-chart'});
    }
    return;
  }

  const angular=frames.map(f=>({
    t:f.t,s:which==='SMC_2S'?f.surfaces?.s2:f.surfaces?.s1
  })).filter(f=>Number.isFinite(f.s));
  const dx=frames.map(f=>({t:f.t,x:f.errors?.ex}))
    .filter(f=>Number.isFinite(f.x));
  const deriv=data=>data.slice(1).map((b,i)=>{
    const a=data[i],dt=b.t-a.t;
    return dt>1e-6?[(a.s??a.x),((b.s??b.x)-(a.s??a.x))/dt]:
      [NaN,NaN];
  });
  const sRange=Math.max(.6,params.phi*2,...angular.map(f=>Math.abs(f.s)));
  const q=s=>params.k_s*s+(params.use_saturation?params.eta*sat(s/params.phi):0);
  const rateRange=Math.max(1,...deriv(angular).map(p=>Math.abs(p[1])),
    q(sRange),q(-sRange));
  const ps=phaseAxes($('phase-s'),-sRange,sRange,-rateRange,rateRange,
    'angular surface s','ds/dt (1/s)');
  verticalBoundary($('phase-s'),ps,-params.phi,params.phi);
  polylineEquation($('phase-s'),-sRange,sRange,ps,s=>-q(s),'#59DDB9','5 3');
  phaseTrace($('phase-s'),deriv(angular),ps);

  const eRange=Math.max(.6,params.phi_x*2,...dx.map(f=>Math.abs(f.x)));
  const gx=x=>params.k_x*x+
    (params.use_saturation?params.eta_x*sat(x/params.phi_x):0);
  const eRate=Math.max(1,...deriv(dx).map(p=>Math.abs(p[1])),
    gx(eRange),gx(-eRange));
  const pe=phaseAxes($('phase-ex'),-eRange,eRange,-eRate,eRate,
    'ex (m)','dex/dt (m/s)');
  verticalBoundary($('phase-ex'),pe,-params.phi_x,params.phi_x);
  polylineEquation($('phase-ex'),-eRange,eRange,pe,x=>-gx(x),'#59DDB9','5 3');
  phaseTrace($('phase-ex'),deriv(dx),pe);
  const psat=phaseAxes($('sat-plot'),-2,2,-1.2,1.2,'z = s / φ','sat(z)');
  verticalBoundary($('sat-plot'),psat,-1,1);
  polylineEquation($('sat-plot'),-2,2,psat,sat,'#F4BB66');
}
function drawTimeline(){
  const grid=$('timeline-grid'),pathGroup=$('timeline-path'),
    traceGroup=$('timeline-trace'),carrotGroup=$('timeline-carrot');
  for(const el of [grid,pathGroup,traceGroup,carrotGroup])el.replaceChildren();
  if(!timelineData?.frames?.length)return;

  const frames=timelineData.frames;
  const points=timelineData.reference?.points||[];
  const world=window.TrackerTimelineGeometry.geometry(points,frames);
  const {box,x,y,scale,format}=world;
  const left=box.left,right=world.width-box.right;
  const top=box.top,bottom=world.height-box.bottom;

  // Metre rulers, with the same world->pixel distance on BOTH axes.
  // The plot border is visible even when world origin (0,0) is offscreen.
  for(const value of world.xTicks){
    const at=x(value);
    addLine(grid,[at,top],[at,bottom],
      Math.abs(value)<1e-9?'#607985':'#263C4C',
      Math.abs(value)<1e-9?1.3:.7,'3 5');
    addLine(grid,[at,bottom],[at,bottom+4],'#A0BCCB',1);
    chartText(grid,format(value),{
      x:at,y:bottom+17,'text-anchor':'middle',class:'tick-label'});
  }
  for(const value of world.yTicks){
    const at=y(value);
    addLine(grid,[left,at],[right,at],
      Math.abs(value)<1e-9?'#607985':'#263C4C',
      Math.abs(value)<1e-9?1.3:.7,'3 5');
    addLine(grid,[left-4,at],[left,at],'#A0BCCB',1);
    chartText(grid,format(value),{
      x:left-8,y:at+3,'text-anchor':'end',class:'tick-label'});
  }
  addLine(grid,[left,top],[left,bottom],'#95AFBF',1.3);
  addLine(grid,[left,bottom],[right,bottom],'#95AFBF',1.3);
  chartText(grid,'X position (m)',{
    x:(left+right)/2,y:world.height-8,
    'text-anchor':'middle',class:'axis-label'});
  chartText(grid,'Y position (m)',{
    transform:'translate(18 '+(top+bottom)/2+') rotate(-90)',
    x:0,y:0,'text-anchor':'middle',class:'axis-label'});
  const frameIndex=Math.min(Math.max(0,Number($('timeline-slider').value)||0),
    frames.length-1);
  const frame=frames[frameIndex];
  const ref=frame.ref;
  if(points.length>1)pathGroup.append(shape('polyline',{
    points:points.map(p=>x(p[0])+','+y(p[1])).join(' '),
    fill:'none',stroke:'#42d0c2','stroke-width':3
  }));
  const trail=frames.slice(0,frameIndex+1);
  if(trail.length>1)traceGroup.append(shape('polyline',{
    points:trail.map(f=>x(f.x)+','+y(f.y)).join(' '),
    fill:'none',stroke:'#F4BB66','stroke-width':2.5
  }));
  const rx=x(frame.x),ry=y(frame.y),cx=x(ref.x),cy=y(ref.y);
  const radius=Number.isFinite(ref.radius)&&ref.radius>0?
    world.radius(ref.radius):0;
  // Equal scale is now guaranteed; this is a true SVG CIRCLE.
  carrotGroup.append(shape('circle',{
    cx:rx,cy:ry,r:radius,stroke:'#E5A94A',
    'stroke-width':1.4,'stroke-dasharray':'6 4',fill:'none'
  }));
  addLine(carrotGroup,[rx,ry],[cx,cy],'#E5A94A',1.5,'5 3');
  carrotGroup.append(shape('circle',{cx,cy,r:6,fill:'#42d0c2'}));
  carrotGroup.append(shape('circle',{cx:rx,cy:ry,r:8,fill:'#F4BB66'}));
  addLine(carrotGroup,[rx,ry],
    [rx+24*Math.cos(frame.yaw),ry-24*Math.sin(frame.yaw)],'#F4BB66',2.5);
  addLine(carrotGroup,[cx,cy],
    [cx+20*Math.cos(ref.yaw),cy-20*Math.sin(ref.yaw)],'#42d0c2',2);
  chartText(carrotGroup,'Robot',{x:rx+10,y:ry-10,class:'map-tag'});
  chartText(carrotGroup,'Carrot',{x:cx+9,y:cy-10,class:'map-tag'});
  $('timeline-time').textContent=fmt(frame.t,2)+' s';
  const e=frame.errors||{},surface=frame.surfaces||{};
  $('timeline-fields').textContent=
    'Robot ('+fmt(frame.x,2)+', '+fmt(frame.y,2)+') m · '+
    'carrot ('+fmt(ref.x,2)+', '+fmt(ref.y,2)+') m · '+
    'range '+fmt(ref.radius,3)+' m · '+
    'nominal vR '+fmt(ref.v_ref,3)+' m/s · nominal ωR '+
    fmt(ref.w_ref,3)+' rad/s · '+
    'eₓ '+fmt(e.ex,3)+' m · eᵧ '+fmt(e.ey,3)+' m · '+
    'eθ '+fmt(e.etheta,3)+' rad · '+
    's₁ '+fmt(surface.s1,3)+' · s₂ '+fmt(surface.s2,3)+
    ' · D '+fmt(surface.D,3)+
    ' · applied v '+fmt(frame.v,3)+' m/s · ω '+fmt(frame.w,3)+
    ' rad/s · carrot source: '+(ref.source||'unknown')+
    (ref.fallback?' (intersection fallback)':'')+
    ' · equal-scale axes: 1 m on X = 1 m on Y';
}
async function loadTimeline(name){
  timelineName=name;
  timelineData=null;
  $('timeline-run').value=name;
  $('timeline-slider').disabled=true;
  $('timeline-fields').textContent=name?'Loading detailed timeline…':
    'Select a saved run with detailed telemetry.';
  drawTimeline();
  if(!name){renderPhasePortraits();return;}
  try{
    const response=await api('/history/timeline?name='+encodeURIComponent(name));
    if(timelineName!==name)return;
    timelineData=response;
    $('timeline-slider').min='0';
    $('timeline-slider').max=String(response.frames.length-1);
    $('timeline-slider').value='0';
    $('timeline-slider').disabled=false;
    drawTimeline();renderPhasePortraits();
  }catch(e){
    if(timelineName===name){
      $('timeline-fields').textContent=e.message;
      notify('Could not open timeline: '+e.message,true);
    }
  }
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
  draw();renderChart();
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
    const data=item.summary||{},result=data.results||{};
    const tr=addRow(table,[item.name,data.path||'—',data.controller||'—',
      data.seed??'—',data.status||'—',fmt(result.cross_track_rmse_m,4),
      fmt(result.heading_rmse_deg,2),fmt(result.duration_sim_s,1)]);
    const checkCell=document.createElement('td');
    checkCell.className='plot-checkbox-cell';
    const checkbox=document.createElement('input');
    checkbox.type='checkbox';checkbox.className='plot-checkbox';
    checkbox.dataset.plotName=item.name;
    if(item.has_trace)checkbox.dataset.hasTrace='yes';
    checkbox.title=item.has_trace?
      'Add/remove this saved experiment from the comparison plot':
      'No finalized trace yet; this run cannot be plotted';
    checkbox.setAttribute('aria-label','Plot saved experiment '+item.name);
    checkbox.checked=plotRuns.has(item.name)||pendingPlotRuns.has(item.name);
    checkbox.addEventListener('change',()=>selectSavedTrace(item.name,checkbox.checked));
    checkCell.append(checkbox);tr.insertBefore(checkCell,tr.firstChild);
    const replayCell=document.createElement('td');
    const replayButton=document.createElement('button');
    replayButton.type='button';
    replayButton.textContent='Inspect';
    replayButton.className='btn small outline';
    replayButton.disabled=!item.has_timeline;
    replayButton.title=item.has_timeline?
      'Load this saved run into the timeline inspector':
      'Detailed timeline unavailable: recorded before the telemetry update';
    replayButton.onclick=()=>{
      loadTimeline(item.name);
      $('timeline-run').scrollIntoView({behavior:'smooth',block:'center'});
    };
    replayCell.append(replayButton);
    tr.insertBefore(replayCell,tr.children[1]);

    const td=document.createElement('td'),b=document.createElement('button');
    b.className='btn small danger';b.textContent='Delete';
    b.disabled=busy||Boolean(lastStatus?.trial_running);
    b.onclick=()=>{
      if(!confirm('Permanently delete tracking experiment '+item.name+'?'))return;
      operate(async()=>{
        await api('/history/delete',{name:item.name,confirm:true});
        plotRuns.delete(item.name);pendingPlotRuns.delete(item.name);
        if(timelineName===item.name)await loadTimeline('');
        await history();renderChart();
        notify('Experiment deleted.');
      });
    };
    td.append(b);tr.append(td);
  }
  if(!rows.length){
    const tr=document.createElement('tr'),td=document.createElement('td');
    td.colSpan=11;td.textContent='No saved tracking runs yet.';
    tr.append(td);table.append(tr);
  }
  syncPlotCheckboxes();
  const picker=$('timeline-run');
  const previous=timelineName;
  picker.replaceChildren();
  const empty=document.createElement('option');
  empty.value='';empty.textContent='Choose a recorded experiment…';
  picker.append(empty);
  for(const item of rows){
    if(!item.has_timeline)continue;
    const option=document.createElement('option');
    option.value=item.name;
    option.textContent=(item.summary?.controller||'—')+
      ' · '+(item.summary?.path||'—')+' · '+item.name;
    picker.append(option);
  }
  picker.value=rows.some(x=>x.name===previous&&x.has_timeline)?
    previous:'';
  if(previous&&!picker.value)loadTimeline('');
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
for(const id of [
  'path-kind','controller-select','start-progress','lateral-offset',
  'heading-error','seed','random-lateral','random-heading','pose-x','pose-y','pose-yaw',
  ...Array.from(document.querySelectorAll('.tuning-fields input')).map(el=>el.id)
]){
  $(id).addEventListener('change',()=>{
    if(id==='controller-select')highlightActiveController();
    window.TrackerMath?.refresh();
    if(id==='desired-speed')updateTurningAllowance();
    lastPreviewKey='';guardedPreview();
  });
}
$('desired-speed').addEventListener('input',updateTurningAllowance);
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
for(const name of ['shared','rpp','smc_1s','smc_2s']){
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
$('plot-live').onchange=renderChart;
$('timeline-run').onchange=()=>loadTimeline($('timeline-run').value);
$('timeline-slider').oninput=drawTimeline;
$('clear-plot').onclick=()=>{
  plotRuns.clear();pendingPlotRuns.clear();
  syncPlotCheckboxes();renderChart();
};
$('clear-history').onclick=()=>{
  if(!confirm('Permanently delete ALL saved Tracker Lab experiments?'))return;
  operate(async()=>{const r=await api('/history/clear',{confirm:true});
    plotRuns.clear();pendingPlotRuns.clear();
    await loadTimeline('');
    await history();renderChart();
    notify('Deleted '+r.deleted+' tracking experiments.');});
};
(async()=>{
  try{
    defaultSettings=await api('/defaults');
    showPhysicalLimits(await api('/physical-limits'));
    const x=await api('/config');setForm(x);
    await refreshPreview();
    await refresh();await history();
    renderPhasePortraits();
  }catch(e){notify(e.message,true);}
  initializing=false;
  setInterval(async()=>{
    if(busy)return;
    try{await refresh();}catch(e){$('action-state').textContent=e.message;}
  },1800);
  setInterval(async()=>{if(!busy)try{await history();}catch(e){}},9000);
})();
