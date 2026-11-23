import { useEffect, useRef, useState } from 'react';
import { ChevronLeft, ChevronRight, Plus, Trash2 } from 'lucide-react';
import type { Point } from './types';

const headerHeight = 26;
const rowHeight = 29;

export default function WaypointDock({points, selected, select, change, add, remove}: {
  points: Point[];
  selected: number;
  select: (index: number) => void;
  change: (index: number, point: Point) => void;
  add: () => void;
  remove: () => void;
}) {
  const table = useRef<HTMLDivElement>(null);
  const [page, setPage] = useState(0);
  const [rows, setRows] = useState(4);
  useEffect(() => {
    if (!table.current) return;
    const observer = new ResizeObserver(([entry]) => {
      setRows(Math.max(1, Math.floor((entry.contentRect.height - headerHeight) / rowHeight)));
    });
    observer.observe(table.current);
    return () => observer.disconnect();
  }, []);
  useEffect(() => {
    if (selected >= 0) setPage(Math.floor(selected / rows));
  }, [selected, rows]);

  const lastPage=Math.max(0, Math.ceil(points.length / rows) - 1);
  const activePage=Math.min(page, lastPage);
  const start=activePage * rows;
  const end=Math.min(start + rows, points.length);

  function showPage(nextPage: number) {
    select(-1);
    setPage(nextPage);
  }

  return <section className="waypoint-dock" aria-labelledby="waypoint-title">
    <div className="dock-bar">
      <h2 id="waypoint-title">Waypoints</h2>
      <button onClick={add}><Plus size={14}/> Add row</button>
      <button disabled={selected<0||points.length<=2} onClick={remove} aria-label="Remove selected">
        <Trash2 size={14}/> <span className="remove-text">Remove</span>
      </button>
      <div className="pager" aria-label="Waypoint pages">
        <button aria-label="Previous waypoint page" disabled={activePage===0} onClick={()=>showPage(activePage-1)}><ChevronLeft size={17}/></button>
        <span aria-live="polite">{points.length ? `${start+1}–${end} of ${points.length}` : '0 points'}</span>
        <button aria-label="Next waypoint page" disabled={activePage>=lastPage} onClick={()=>showPage(activePage+1)}><ChevronRight size={17}/></button>
      </div>
    </div>
    <div className="waypoint-table-frame" ref={table}>
      <table className="point-table waypoint-table">
        <thead><tr><th>Point</th><th>North (m)</th><th>East (m)</th><th>Height (m)</th><th>Stop</th><th>Exact</th></tr></thead>
        <tbody>{points.slice(start,end).map((point,offset)=>{
          const i=start+offset;
          const endpoint=i===0||i===points.length-1;
          return <tr key={point.id+'-'+i} className={selected===i?'row-selected':''} onClick={()=>select(i)} onFocus={()=>select(i)}>
            <td title={point.id}>{point.id}</td>
            {(['north','east','altitude'] as const).map(key=><td key={key}>
              <input aria-label={point.id+' '+key} type="number" step=".1" value={point[key]}
                onChange={e=>{if(e.target.value!=='') change(i,{...point,[key]:Number(e.target.value)});}}/>
            </td>)}
            <td><input aria-label={point.id+' hard stop'} type="checkbox" checked={point.hard||endpoint} disabled={endpoint}
              onChange={e=>change(i,{...point,hard:e.target.checked,exact:e.target.checked?false:point.exact})}/></td>
            <td><input aria-label={point.id+' exact point'} type="checkbox" checked={!endpoint&&!point.hard&&point.exact} disabled={endpoint}
              onChange={e=>change(i,{...point,exact:e.target.checked,hard:e.target.checked?false:point.hard})}/></td>
          </tr>;
        })}</tbody>
      </table>
      {!points.length&&<p className="empty-table">Add a point on the map or use Add row.</p>}
    </div>
  </section>;
}
