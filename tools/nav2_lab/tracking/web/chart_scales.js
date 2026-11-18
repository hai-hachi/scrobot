'use strict';
/* Pure numerical chart scale helpers. Shared by the browser and Node tests.
   Time is simulated seconds from the beginning of EACH selected experiment;
   signed lateral error is in metres. No paths are normalized or rescaled. */
(function(root) {
  function niceStep(span,desiredIntervals=5) {
    if (!Number.isFinite(span) || span<=0) return 1;
    const raw=span/desiredIntervals;
    const power=10**Math.floor(Math.log10(raw));
    const scaled=raw/power;
    const basis=scaled<=1?1:scaled<=2?2:scaled<=5?5:10;
    return basis*power;
  }
  function ticks(from,to,step) {
    const count=Math.min(20,Math.round((to-from)/step));
    return Array.from({length:count+1},(_,i)=>+(from+i*step).toPrecision(12));
  }
  function scales(lines) {
    const values=[];
    for (const line of lines || []) {
      for (const p of line.points || []) {
        if (p && Number.isFinite(p.t) && Number.isFinite(p.e) && p.t>=0)
          values.push(p);
      }
    }
    const maxT=values.length?Math.max(...values.map(p=>p.t)):10;
    let minE=values.length?Math.min(0,...values.map(p=>p.e)):-.5;
    let maxE=values.length?Math.max(0,...values.map(p=>p.e)):.5;
    if (maxE-minE<1e-9) {minE=-.1;maxE=.1;}
    const xStep=niceStep(Math.max(1,maxT),5);
    const xMax=Math.max(xStep,Math.ceil(maxT/xStep)*xStep);
    const margin=Math.max(.005,(maxE-minE)*.08);
    const yStep=niceStep(maxE-minE+2*margin,5);
    let yMin=Math.floor((minE-margin)/yStep)*yStep;
    let yMax=Math.ceil((maxE+margin)/yStep)*yStep;
    if (Math.abs(yMax-yMin)<1e-12) {yMin=-yStep;yMax=yStep;}
    yMin=+yMin.toPrecision(12);
    yMax=+yMax.toPrecision(12);
    return {
      xMin:0,xMax,yMin,yMax,
      xStep,yStep,
      xTicks:ticks(0,xMax,xStep),
      yTicks:ticks(yMin,yMax,yStep)
    };
  }
  function label(value,step) {
    if (!Number.isFinite(value)) return '—';
    const digits=Math.max(0,Math.min(5,Math.ceil(-Math.log10(step))+1));
    return Object.is(value,-0)?(0).toFixed(digits):value.toFixed(digits);
  }
  const api={niceStep,scales,label};
  if (typeof module !== 'undefined' && module.exports)
    module.exports=api;
  if (root)root.TrackerChartScales=api;
})(typeof window !== 'undefined'?window:null);
