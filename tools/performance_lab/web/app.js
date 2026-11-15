'use strict';
const $=id=>document.getElementById(id);
const fmt=(n,d=1)=>typeof n==='number'&&Number.isFinite(n)?n.toFixed(d):'—';
let status=null,loading=false,settingsLoaded=false,historyBusy=false;
let rtfPoints=[],lastClockReset=null;

async function api(route,body){
  const response=await fetch(route,{method:body===undefined?'GET':'POST',
    headers:body===undefined?{}:{'Content-Type':'application/json'},
    body:body===undefined?undefined:JSON.stringify(body),cache:'no-store'});
  let data={};
  try{data=await response.json()}catch(e){}
  if(!response.ok)throw Error(data.error||'HTTP '+response.status);
  return data;
}
function alertText(msg,isError=false){
  const box=$('notice');box.hidden=false;box.textContent=msg;
  box.classList.toggle('error',isError);
}
async function action(callback){
  if(loading)return;
  loading=true;buttons();
  try{await callback();await refresh();}
  catch(e){alertText(e.message,true)}
  finally{loading=false;buttons()}
}
function setTab(id){
  document.querySelectorAll('.tab').forEach(button=>{
    button.classList.toggle('active',button.dataset.tab===id)});
  document.querySelectorAll('.tab-pane').forEach(pane=>
    pane.classList.toggle('active',pane.id==='tab-'+id));
}
document.querySelectorAll('.tab').forEach(button=>
  button.addEventListener('click',()=>setTab(button.dataset.tab)));

const settingIds=['mode','profile','max_rtf','duration_s','warmup_s',
 'ros_domain_id','shuttle_count','shuttle_seed','yolo_device','model_path'];
const integerKeys=new Set(['duration_s','warmup_s','ros_domain_id','shuttle_count','shuttle_seed']);
function setInputs(v){
  settingIds.forEach(key=>{if(v[key]!==undefined)$(key).value=v[key]});
  $('rtf-choice').textContent=Number($('max_rtf').value).toFixed(2)+'×';
  settingsLoaded=true;
}
function readInputs(){
  const result={};
  for(const key of settingIds){
    const value=$(key).value;
    if(integerKeys.has(key))result[key]=Number(value);
    else if(key==='max_rtf')result[key]=Number(value);
    else result[key]=value;
  }
  for(const key of integerKeys)
    if(!Number.isInteger(result[key]))throw Error(key+' must be an integer');
  return result;
}
$('max_rtf').addEventListener('input',()=>
  $('rtf-choice').textContent=Number($('max_rtf').value).toFixed(2)+'×');

function buttons(){
  const live=Boolean(status?.stack_running),active=Boolean(status?.benchmark);
  const permissions={
    start:!live,stop:live,save:true,'bench-start':live&&!active,
    'bench-cancel':active,'gazebo-open':live&&!status?.gazebo_gui?.running,
    'gazebo-close':Boolean(status?.gazebo_gui?.running),
    'rviz-open':live&&!status?.rviz_running,'rviz-close':Boolean(status?.rviz_running),
    'audit-start':live&&status?.audit?.status!=='running',
    'history-refresh':true,
  };
  for(const[key,enabled]of Object.entries(permissions)){
    $(key).disabled=loading||!enabled;
  }
  document.querySelectorAll('#history-rows .delete-run').forEach(button=>{
    button.disabled=loading||active;
  });
}

function emptyRow(colspan,msg){
  const tr=document.createElement('tr'),td=document.createElement('td');
  td.colSpan=colspan;td.textContent=msg;tr.append(td);return tr;
}
function cell(tr,value,kind=''){
  const td=document.createElement('td');td.textContent=value;
  if(kind)td.className=kind;
  tr.append(td);return td;
}
function updateTopicStatus(item,topic){
  const info=item?.topics?.[topic];
  return info?(fmt(info.wall_hz)+' / '+fmt(info.sim_hz)+' Hz'):'—';
}
function rateStatus(row){
  return ({
    pass:['Pass','ok'],
    paused:['Paused','warn'],
    sim_rate_low:['Low (sim time)','bad'],
    sim_rate_high:['High (sim time)','bad'],
    rtf_low:['RTF low','warn'],
    unknown:['Unknown','warn'],
  })[row.status]||['Unknown','warn'];
}
function renderTopics(monitor){
  const records=monitor?.topics||{};
  const body=$('topic-rows');body.replaceChildren();
  if(!Object.keys(records).length){
    body.append(emptyRow(7,'Start simulation and wait for live ROS /clock and topics.'));
    return;
  }
  for(const[topic,data]of Object.entries(records)){
    const tr=document.createElement('tr');
    cell(tr,topic);
    cell(tr,data.expected_sim_hz==null?'—':fmt(data.expected_sim_hz));
    cell(tr,fmt(data.wall_hz));
    cell(tr,fmt(data.sim_hz));
    cell(tr,fmt(data.header_age_sim_s,3));
    cell(tr,data.header_sample_count>1?String(data.header_large_gap_count):'—');
    const [txt,kind]=rateStatus(data);
    cell(tr,txt,kind);
    body.append(tr);
  }
  $('simclock').textContent=fmt(monitor.sim_now,2)+' s';
  $('clock-state').textContent=monitor.paused?'Paused / unavailable':
    'Advancing · '+fmt(monitor.rtf,2)+'×';
  $('clock-jumps').textContent=String(monitor.clock_jumps||0);
  const clues=[];
  for(const [topic,data]of Object.entries(records))
    if(data.status==='sim_rate_low'||data.status==='sim_rate_high')
      clues.push(data.label+': '+data.status.replace('_',' '));
  $('clock-note').textContent=clues.length?
    clues.slice(0,4).join('; ')+'. Check rates, timestamps, queues and logs.':
    'Wall Hz and simulated Hz are independent measurements. Low RTF by itself is not a simulated-time rate failure.';
}

function renderChart(rtf){
  if(lastClockReset!==(status?.monitor?.clock_jumps??null)){
    rtfPoints=[];lastClockReset=status?.monitor?.clock_jumps??null;
  }
  if(typeof rtf==='number'&&Number.isFinite(rtf)){
    rtfPoints.push(rtf);if(rtfPoints.length>120)rtfPoints.shift();
  }
  if(rtfPoints.length<2){$('rtf-line').setAttribute('d','');return;}
  const coords=rtfPoints.map((v,i)=>{
    const x=i*850/(rtfPoints.length-1);
    const y=135-Math.min(1.5,Math.max(0,v))*90;
    return (i===0?'M':'L')+x.toFixed(1)+','+y.toFixed(1);
  });
  $('rtf-line').setAttribute('d',coords.join(' '));
}
function renderResources(resources,gpu){
  const total=resources?.totals||{},system=resources?.system||{};
  $('cpu').textContent=fmt(total.cpu_percent_one_core,0)+'%';
  $('proc-cpu').textContent=fmt(total.cpu_percent_one_core,1)+'%';
  $('proc-ram').textContent=fmt(total.rss_mb,0)+' MiB';
  $('host-cpu').textContent=fmt(system.cpu_percent)+'%';
  $('host-ram').textContent=fmt(system.ram_percent)+'%';
  const devices=gpu?.devices||[];
  $('gpu').textContent=devices.length?fmt(devices[0].util_percent,0)+'%':'—';
  $('gpu-details').replaceChildren();
  if(!devices.length){
    $('gpu-details').textContent='No NVIDIA GPU stats available; no GPU performance conclusion is possible.';
  }else{
    for(const item of devices){
      const line=document.createElement('div');
      line.className='calibration-item';
      line.textContent='GPU '+item.index+' · '+item.name+
        ' · '+fmt(item.util_percent)+'% utilization · '+
        fmt(item.memory_used_mib,0)+' / '+fmt(item.memory_total_mib,0)+' MiB VRAM · '+
        fmt(item.temperature_c,0)+'°C';
      $('gpu-details').append(line);
    }
  }
  const tbody=$('process-rows');tbody.replaceChildren();
  if(!resources?.processes?.length){
    tbody.append(emptyRow(4,resources?.hint||'No process metrics available.'));
    return;
  }
  for(const p of resources.processes){
    const tr=document.createElement('tr');
    cell(tr,p.name);cell(tr,fmt(p.cpu_percent));cell(tr,fmt(p.rss_mb));
    cell(tr,(p.pids||[]).join(', '));tbody.append(tr);
  }
}
function renderAudit(audit){
  const lines=$('audit-rows');lines.replaceChildren();
  $('audit-note').textContent=
    (audit?.status||'not_run')+' · '+
    (audit?.nodes?.length||0)+' nodes checked'+
    (audit?.truncated?' · first 48 nodes only':'')+
    (audit?.error?' · '+audit.error:'')+
    '. Sim-time parameter alone cannot reveal hidden wall timers or callback overload.';
  if(!audit?.nodes?.length){
    lines.append(emptyRow(3,'Press Check use_sim_time parameters when the stack is ready.'));
    return;
  }
  for(const item of audit.nodes){
    const tr=document.createElement('tr');
    cell(tr,item.node);
    cell(tr,item.status,item.status==='wall_time'?'bad':
      item.status==='sim_time'?'ok':'warn');
    cell(tr,item.detail||'—','wrapped');lines.append(tr);
  }
}
function renderProvenance(provenance){
  const list=$('calibration');list.replaceChildren();
  if(!provenance){list.textContent='Start the simulation to snapshot inputs.';return;}
  const keys=['production_world','installed_camera','installed_mobility',
    'installed_odometry','production_camera','production_mobility',
    'production_odometry','saved_lighting','saved_camera',
    'saved_mobility','saved_odometry','saved_yolo'];
  for(const name of keys){
    const info=provenance[name];
    if(!info)continue;
    const wrap=document.createElement('div');wrap.className='calibration-item';
    const h=document.createElement('strong');h.textContent=name.replaceAll('_',' ');
    const p=document.createElement('small');
    p.textContent=info.error?'Unavailable: '+info.error:
      (info.sha256||'—').slice(0,16)+'… · '+info.path;
    wrap.append(h,p);list.append(wrap);
  }
}

function renderSummary(data){
  const report=data?.last_result;
  const area=$('run-summary');
  const bench=data?.benchmark;
  $('bench-status').textContent=bench?
    bench.phase+' · '+fmt(bench.elapsed_s,0)+' s / '+
    (bench.duration_s+bench.warmup_s)+' s':'No benchmark running';
  if(!report){area.textContent='No completed benchmark from this server session.';return;}
  if(report.error){area.textContent='Benchmark error: '+report.error;return;}
  const r=report.summary||{};
  const title=document.createElement('strong');
  title.textContent=report.run_id+' · '+report.outcome;
  const p=document.createElement('p');
  p.textContent='Average RTF '+fmt(r.avg_rtf,3)+'× · Sim-time cadence '+
    (r.cadence_pass?'PASS':'NOT VERIFIED / BELOW TARGET')+
    (r.real_time_rate_pass===null?' · Real-time status NOT assessed':
      ' · Real-time rate '+(r.real_time_rate_pass?'PASS':'FAIL'))+
    '. Results are topic-rate checks, not an end-to-end mission acceptance certificate.';
  area.replaceChildren(title,p);
}

async function refreshHistory(){
  if(historyBusy)return;
  historyBusy=true;
  try{
    const items=await api('/api/history');
    const tbody=$('history-rows');tbody.replaceChildren();
    if(!items.length){tbody.append(emptyRow(8,'No saved performance experiments.'));return;}
    for(const row of items){
      const tr=document.createElement('tr');
      cell(tr,row.run_id);cell(tr,row.profile);cell(tr,row.mode);
      cell(tr,fmt(row.summary?.avg_rtf,3)+'×');
      cell(tr,row.summary?.cadence_pass?'Pass':'Not verified',
           row.summary?.cadence_pass?'ok':'warn');
      cell(tr,row.outcome);
      const artifact=document.createElement('td');
      const link=document.createElement('a');
      link.href='/api/history/'+encodeURIComponent(row.run_id+'.json');
      link.target='_blank';link.rel='noopener';link.textContent='View JSON';
      link.style.color='#7acacf';artifact.append(link);tr.append(artifact);
      const actions=document.createElement('td');
      const button=document.createElement('button');
      button.type='button';button.className='btn small danger delete-run';
      button.textContent='Delete';
      button.disabled=Boolean(status?.benchmark);
      button.addEventListener('click',()=>{
        if(!window.confirm('Permanently delete '+row.run_id+'?'))return;
        action(async()=>{
          await api('/api/history/delete',{name:row.run_id+'.json'});
          await refreshHistory();
          alertText('Deleted '+row.run_id);
        });
      });
      actions.append(button);tr.append(actions);
      tbody.append(tr);
    }
  }catch(e){alertText(e.message,true)}
  finally{historyBusy=false}
}

async function refresh(){
  try{
    const result=await api('/api/status');
    status=result;
    $('connection').textContent='Local server connected';
    const active=result.stack_running;
    $('top-status').textContent=active?(
      'Full stack running · '+result.launch_settings?.mode):
      'Simulation stopped';
    const meter=result.monitor;
    $('rtf').textContent=meter?.rtf==null?'—':fmt(meter.rtf,3)+'×';
    $('rtf-sub').textContent=meter?.paused?'Clock paused / unavailable':
      active?'Target cap '+fmt(result.launch_settings?.max_rtf,2)+'×':'Waiting for /clock';
    $('rgb-rate').textContent=updateTopicStatus(meter,'/camera/camera/color/image_raw');
    $('depth-rate').textContent=updateTopicStatus(meter,'/camera/camera/depth/points');
    $('odom-rate').textContent=updateTopicStatus(meter,'/diff_drive_controller/odom');
    $('depth-note').textContent='Wall / sim Hz; target '+fmt(
      meter?.topics?.['/camera/camera/depth/points']?.expected_sim_hz,0)+' sim Hz';
    renderChart(meter?.rtf);
    renderTopics(meter);
    renderResources(result.resources,result.gpu);
    renderAudit(result.audit);
    renderSummary(result);
    renderProvenance(result.calibration);
    $('stack-log').textContent=result.logs?.stack||'No output.';
    $('observer-log').textContent=result.logs?.observer||'No observer output.';
    $('gazebo-log').textContent=result.logs?.gazebo||'Gazebo GUI not opened.';
    $('rviz-log').textContent=result.logs?.rviz||'RViz not opened.';
    $('run-info').textContent=result.benchmark?
      'Benchmark '+result.benchmark.phase+' · '+
      fmt(result.benchmark.elapsed_s,0)+' s elapsed · '+result.benchmark.sample_count+
      ' measured samples':active?
      'Running '+result.launch_settings?.profile+
      '. Saved settings changed during runtime apply only after restart.':
      'Start simulation to profile complete robot software.';
    buttons();
  }catch(e){
    $('connection').textContent='Disconnected';
    $('top-status').textContent=e.message;
  }
}

$('save').onclick=()=>action(async()=>{
  const result=await api('/api/save',{settings:readInputs()});
  alertText('Saved '+result.file+
    (result.restart_required?' · Restart simulation to apply changed settings.':''));
});
$('start').onclick=()=>action(async()=>{
  await api('/api/settings',{settings:readInputs()});
  const result=await api('/api/start',{});
  alertText('Full stack launching (PID '+result.pid+
    '). Allow sensors and controllers to initialize before benchmarking.');
});
$('stop').onclick=()=>action(async()=>{
  await api('/api/stop',{});
  alertText('Full-stack simulation stopped.');
});
$('bench-start').onclick=()=>action(async()=>{
  const r=await api('/api/benchmark/start',{});
  alertText('Benchmark started · '+r.warmup_s+' s warm-up, '+
    r.duration_s+' s measurement. All rates retain their configured simulated-time targets.');
  setTab('results');
});
$('bench-cancel').onclick=()=>action(async()=>{
  await api('/api/benchmark/cancel',{});
  alertText('Benchmark cancellation requested; partial run will be recorded.');
});
$('audit-start').onclick=()=>action(async()=>{
  await api('/api/audit/start',{});
  alertText('ROS node clock audit started. Results will update automatically.');
  setTab('synchronization');
});
$('history-refresh').onclick=refreshHistory;
$('gazebo-open').onclick=()=>action(async()=>{
  await api('/api/gazebo/open',{});alertText('Opened Gazebo GUI (extra render load may reduce RTF).');
});
$('gazebo-close').onclick=()=>action(async()=>{
  await api('/api/gazebo/close',{});alertText('Closed Gazebo GUI.');
});
$('rviz-open').onclick=()=>action(async()=>{
  await api('/api/rviz/open',{});alertText('RViz opened; measured rendering load may rise.');
});
$('rviz-close').onclick=()=>action(async()=>{
  await api('/api/rviz/close',{});alertText('RViz closed.');
});

(async()=>{
  try{
    setInputs(await api('/api/settings'));
    await refresh();
    await refreshHistory();
  }catch(e){alertText(e.message,true)}
  setInterval(refresh,2500);
  setInterval(refreshHistory,20000);
})();
