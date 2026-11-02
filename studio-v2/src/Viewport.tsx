import { useEffect, useMemo, useRef, useState } from 'react';
import type { Point,Sample } from './types';
import type { Field } from './Plot';
import { interpolate } from './engineering';

function project(n:number,e:number,h:number,az:number,el:number,scale:number,cx:number,cy:number){
  const x=e,y=n,z=h;
  const a=x*Math.cos(az)-y*Math.sin(az),b=x*Math.sin(az)+y*Math.cos(az);
  return {x:cx+a*scale,y:cy+(b*Math.sin(el)-z*Math.cos(el))*scale,depth:b*Math.cos(el)+z*Math.sin(el)};
}
const col=(t:number)=>{
  const u=Math.min(1,Math.max(0,t));
  return 'hsl('+(196-175*u)+', 85%, '+(62-7*u)+'%)';
};
export default function Viewport({points,samples,cursor,field}:{points:Point[];samples:Sample[];cursor:number;field:Field}){
  const ref=useRef<HTMLCanvasElement>(null);
  const [camera,setCamera]=useState({az:-0.65,el:0.57,zoom:1});
  const [size,setSize]=useState({width:800,height:480});
  const drag=useRef<{x:number;y:number}|null>(null);
  const scene=useMemo(()=>{
    let extreme=20, low=Infinity, high=-Infinity;
    for(const p of points)extreme=Math.max(extreme,Math.abs(p.north),Math.abs(p.east),Math.abs(p.altitude));
    for(const s of samples){extreme=Math.max(extreme,Math.abs(s.north),Math.abs(s.east),Math.abs(s.altitude));low=Math.min(low,s[field]);high=Math.max(high,s[field]);}
    return {extreme,low,high,span:Math.max(0.01,high-low)};
  },[points,samples,field]);
  useEffect(()=>{
    const canvas=ref.current;if(!canvas)return;
    const observer=new ResizeObserver(entries=>{
      const el=entries[0].contentRect;setSize({width:Math.max(10,el.width),height:Math.max(10,el.height)});
    });
    observer.observe(canvas);return ()=>observer.disconnect();
  },[]);
  useEffect(()=>{
    const canvas=ref.current;if(!canvas)return;
    const pixelRatio=Math.min(window.devicePixelRatio||1,2);
    canvas.width=Math.round(size.width*pixelRatio);canvas.height=Math.round(size.height*pixelRatio);
    const ctx=canvas.getContext('2d');if(!ctx)return;
    ctx.scale(pixelRatio,pixelRatio);
    const W=size.width,H=size.height;
    ctx.fillStyle='#0c1923';ctx.fillRect(0,0,W,H);
    const {extreme,low,span}=scene;
    const scale=Math.max(1,Math.min(W,H)*.65/extreme)*camera.zoom;
    const get=(n:number,e:number,h:number)=>project(n,e,h,camera.az,camera.el,scale,W/2,H*.69);
    const drawLine=(a:{x:number;y:number},b:{x:number;y:number},color:string,width=1)=>{
      ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();
    };
    const gridSize=10* Math.ceil(extreme/20);
    for(let v=-gridSize*2;v<=gridSize*2;v+=gridSize/2){
      drawLine(get(-gridSize*2,v,0),get(gridSize*2,v,0),'#223b4b');
      drawLine(get(v,-gridSize*2,0),get(v,gridSize*2,0),'#223b4b');
    }
    drawLine(get(0,0,0),get(gridSize,0,0),'#d36f73',2);
    drawLine(get(0,0,0),get(0,gridSize,0),'#56c18f',2);
    drawLine(get(0,0,0),get(0,0,gridSize),'#7ea6ee',2);
    ctx.font='12px ui-monospace, monospace';ctx.fillStyle='#b1c5d1';
    for(const [name,p] of [['N',get(gridSize,0,0)],['E',get(0,gridSize,0)],['UP',get(0,0,gridSize)]] as const)
      ctx.fillText(name,p.x+5,p.y-5);
    for(let i=1;i<points.length;i++)
      drawLine(get(points[i-1].north,points[i-1].east,points[i-1].altitude),
        get(points[i].north,points[i].east,points[i].altitude),'#71899a',1.5);
    if(samples.length){
      const stride=Math.max(1,Math.floor(samples.length/1400));
      for(let i=stride;i<samples.length;i+=stride){
        const a=samples[i-stride],b=samples[i];
        drawLine(get(a.north,a.east,a.altitude),get(b.north,b.east,b.altitude),col((b[field]-low)/span),2.8);
      }
      const last=samples[samples.length-1];
      if(last&&samples.length>1){
        const a=samples[Math.max(0,samples.length-1-stride)];
        drawLine(get(a.north,a.east,a.altitude),get(last.north,last.east,last.altitude),col((last[field]-low)/span),2.8);
      }
      const s=interpolate(samples,cursor);
      if(s){
        const p=get(s.north,s.east,s.altitude);
        ctx.fillStyle='#eff9fc';ctx.strokeStyle='#10b9e5';ctx.lineWidth=3;
        ctx.beginPath();ctx.arc(p.x,p.y,6,0,Math.PI*2);ctx.fill();ctx.stroke();
        const heading={x:p.x+Math.sin(s.yaw)*20,y:p.y-Math.cos(s.yaw)*20};
        drawLine(p,heading,'#fff',2);
      }
    }
    ctx.fillStyle='#b7ccd8';ctx.font='12px ui-monospace, monospace';
    ctx.fillText('NED frame · UP = −Down · equal axis scaling',16,H-16);
  },[size,camera,points,samples,cursor,field,scene]);
  return <div className="viewer">
    <div className="viewer-tools"><span>3D ORBIT / REFERENCE FIELD</span>
      <button type="button" title="Reset 3D view" onClick={()=>setCamera({az:-.65,el:.57,zoom:1})}>Reset</button></div>
    <canvas ref={ref} aria-label="3D trajectory orbit visualization; drag to rotate and scroll to zoom"
      onPointerDown={e=>{drag.current={x:e.clientX,y:e.clientY};e.currentTarget.setPointerCapture(e.pointerId);}}
      onPointerMove={e=>{if(!drag.current)return;const dx=e.clientX-drag.current.x,dy=e.clientY-drag.current.y;drag.current={x:e.clientX,y:e.clientY};
        setCamera(c=>({...c,az:c.az+dx*.009,el:Math.max(.05,Math.min(1.5,c.el+dy*.008))}));}}
      onPointerUp={()=>{drag.current=null;}} onPointerCancel={()=>{drag.current=null;}}
      onWheel={e=>{e.preventDefault();setCamera(c=>({...c,zoom:Math.max(.35,Math.min(6,c.zoom*(e.deltaY>0?.9:1.1)))}));}}/>
    <div className="field-legend"><span>{field}</span><span>{Number.isFinite(scene.low)?scene.low.toFixed(2):'—'}</span><div className="gradient-bar"/><span>{Number.isFinite(scene.high)?scene.high.toFixed(2):'—'}</span></div>
  </div>;
}
