'use strict';
// True equal-aspect world-to-SVG mapping for historical Tracker playback.
// X and Y are in metres; every metre uses exactly the same SVG distance.
(function(root){
  const WIDTH=900,HEIGHT=450;
  const BOX={left:70,right:20,top:20,bottom:50};
  const valid=n=>Number.isFinite(n);
  const niceStep=(span,count=6)=>{
    const raw=Math.max(span,1e-8)/count;
    const magnitude=10**Math.floor(Math.log10(raw));
    const n=raw/magnitude;
    return (n<=1?1:n<=2?2:n<=5?5:10)*magnitude;
  };
  function geometry(pathPoints=[],frames=[]){
    const coords=[];
    const include=(x,y,r=0)=>{
      if(!valid(x)||!valid(y))return;
      const radius=valid(r)&&r>0?r:0;
      coords.push([x-radius,y-radius],[x+radius,y+radius]);
    };
    for(const point of pathPoints){
      if(Array.isArray(point)&&point.length>=2)include(point[0],point[1]);
    }
    for(const f of frames){
      include(f.x,f.y);
      const ref=f.ref||{};
      include(ref.x,ref.y);
      // A radius is centred on the ROBOT, not the carrot.
      if(valid(f.x)&&valid(f.y))include(f.x,f.y,ref.radius);
    }
    if(!coords.length)include(0,0,1);
    const lowX=Math.min(...coords.map(v=>v[0]));
    const highX=Math.max(...coords.map(v=>v[0]));
    const lowY=Math.min(...coords.map(v=>v[1]));
    const highY=Math.max(...coords.map(v=>v[1]));
    const spanX=Math.max(.4,highX-lowX);
    const spanY=Math.max(.4,highY-lowY);
    const centerX=(highX+lowX)/2,centerY=(highY+lowY)/2;
    const plotW=WIDTH-BOX.left-BOX.right;
    const plotH=HEIGHT-BOX.top-BOX.bottom;
    // 8% margin: never truncate either endpoint or the complete circle.
    const scale=Math.min(plotW/(spanX*1.16),plotH/(spanY*1.16));
    const originX=BOX.left+plotW/2,originY=BOX.top+plotH/2;
    const x=x=>originX+(x-centerX)*scale;
    const y=y=>originY-(y-centerY)*scale;
    const visible={
      minX:centerX-plotW/(2*scale),maxX:centerX+plotW/(2*scale),
      minY:centerY-plotH/(2*scale),maxY:centerY+plotH/(2*scale)
    };
    const step=niceStep(Math.min(visible.maxX-visible.minX,
                                visible.maxY-visible.minY),5);
    const ticks=(low,high)=>{
      const first=Math.ceil((low-1e-10)/step);
      const last=Math.floor((high+1e-10)/step);
      return Array.from({length:Math.max(0,Math.min(50,last-first+1))},
        (_,i)=>+(step*(first+i)).toPrecision(12));
    };
    const format=n=>{
      const digits=step<1?Math.min(3,Math.ceil(-Math.log10(step))):0;
      return (Math.abs(n)<step*1e-7?0:n).toFixed(digits);
    };
    return {
      width:WIDTH,height:HEIGHT,box:BOX,scale,centerX,centerY,visible,
      x,y,radius:metres=>metres*scale,step,format,
      xTicks:ticks(visible.minX,visible.maxX),
      yTicks:ticks(visible.minY,visible.maxY)
    };
  }
  const api={geometry,niceStep};
  if(typeof module!=='undefined'&&module.exports)module.exports=api;
  if(root)root.TrackerTimelineGeometry=api;
})(typeof window!=='undefined'?window:null);
