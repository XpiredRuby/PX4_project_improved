export type Vec=[number,number,number];
export type Frame={time:number; measured:Vec; reference:Vec|null; phase:string};
export type Flight={frames:Frame[]; axes:string; kind:string; segment:string; segments:string[]; omitted:number; maxGap:number};

export function readFlight(text:string, selectedSegment?:string):Flight {
  const [header,...lines]=text.trim().split(/\r?\n/);
  const names=header.replace(/^\uFEFF/,'').split(',').map(n=>n.trim());
  const nano=names.includes('sp_ctrltarget_x')&&names.includes('px');
  const estimated=names.includes('estimate_x');
  const timeIndex=names.indexOf(nano?'t':'elapsed_s');
  const measure=(nano?['px','py','pz']:estimated?['estimate_x','estimate_y','estimate_z']:['truth_x','truth_y','truth_z']).map(n=>names.indexOf(n));
  if(timeIndex<0||measure.includes(-1))throw Error('Expected NanoBench flight CSV or elapsed_s plus truth_x/y/z or estimate_x/y/z in local NED.');
  const desired=(nano?['sp_ctrltarget_x','sp_ctrltarget_y','sp_ctrltarget_z']:['desired_x','desired_y','desired_z']).map(n=>names.indexOf(n));
  const phaseIndex=names.indexOf('phase');
  const segmentIndex=names.indexOf('segment');
  const groups=new Map<string,{frames:Frame[];omitted:number}>();
  let last=-Infinity,origin=0;

  for(const [index,line] of lines.entries()){
    const cells=line.split(',');
    if(cells.length!==names.length)throw Error('Invalid column count at row '+(index+2));
    const time=Number(cells[timeIndex]);
    if(!cells[timeIndex].trim()||!Number.isFinite(time)||time<=last)throw Error('Invalid or non-increasing timestamp at row '+(index+2));
    if(index===0)origin=time;
    last=time;
    const vec=(indices:number[]):Vec|null=>{
      if(indices.some(i=>i<0||!cells[i].trim()||!Number.isFinite(Number(cells[i]))))return null;
      return indices.map(i=>Number(cells[i])) as Vec;
    };
    const measured=vec(measure);
    const id=segmentIndex<0?'0':cells[segmentIndex];
    const group=groups.get(id)??{frames:[],omitted:0};
    groups.set(id,group);
    if(!measured){group.omitted++;continue;}
    const relative=time-origin;
    group.frames.push({time:relative,measured,reference:vec(desired),phase:cells[phaseIndex]??''});
  }
  const segments=[...groups.keys()].filter(key=>groups.get(key)!.frames.length>=2);
  const segment=selectedSegment??segments[0];
  if(!segments.includes(segment))throw Error('Recording needs at least two valid position samples in a continuous frame.');
  const {frames,omitted}=groups.get(segment)!;
  let maxGap=0;
  for(let i=1;i<frames.length;i++)maxGap=Math.max(maxGap,frames[i].time-frames[i-1].time);
  return {frames,omitted,maxGap,segments,segment,axes:nano?'Vicon X / Y / Z (m)':'North / East / Down (m)',kind:nano?'Hardware motion capture':estimated?'PX4 position estimate':'Recorded truth channel'};
}

export function tracking(f:Flight){
  const errors=f.frames.flatMap(row=>row.reference?[Math.hypot(...row.measured.map((v,i)=>v-row.reference![i]))]:[]);
  errors.sort((a,b)=>a-b);
  return {paired:errors.length,rows:f.frames.length,omitted:f.omitted,maxGap:f.maxGap,
    rms:errors.length?Math.sqrt(errors.reduce((v,e)=>v+e*e,0)/errors.length):null,
    p95:errors.length?errors[Math.ceil(.95*errors.length)-1]:null,max:errors.at(-1)??null};
}
