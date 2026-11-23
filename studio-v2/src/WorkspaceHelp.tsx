import type { RefObject } from 'react';
import { BookOpen, CircleHelp, X } from 'lucide-react';

export type HelpTopic = 'start' | 'map' | 'settings' | 'results';

const guide = {
  start: {
    title: 'Get started',
    entries: [
      ['Choose where to start', 'Try a practice flight opens a step-by-step tutorial with an example route. With a blank route, click the map to add points in order.'],
      ['Check and watch', 'Choose Check route and review any changes. Watch flight preview starts playback in 3D.'],
      ['Find more tools', 'Review results includes checks, charts, and the simulator. More tools opens recordings and flight studies. On a narrow screen, Project and Settings open their own panels.'],
    ],
  },
  map: {
    title: 'Read the route map',
    entries: [
      ['North and east', 'The launch origin is (0, 0). North is up and east is right. Coordinates, the map ruler, and route dimensions are in metres.'],
      ['Height and distance', 'Height is measured above launch. The route length shown while editing joins the drawn points in 3D. The verified reference can have a different length.'],
      ['Stops and exact points', 'A stop calls for the aircraft to come to a rest. An exact point asks the planner to pass through that position. Start and end points always require a stop.'],
    ],
  },
  settings: {
    title: 'Set flight dimensions and limits',
    entries: [
      ['Aircraft model', 'Use the reference X500 to explore planning. For a connected simulation, use the registered aircraft that matches the simulator.'],
      ['Height above launch', 'Changing the reference altitude sets the height of all points. Edit the waypoint table when individual points need different heights.'],
      ['Preferred cruise and speed limit', 'Cruise is your target speed. The planner also checks the speed ceiling and aircraft limits. It may slow the reference for turns and stops.'],
      ['Route tolerance and heading', 'Tolerance is the allowed distance between your drawing and the planned path. A fixed heading of 0° faces north; 90° faces east.'],
    ],
  },
  results: {
    title: 'Read the results',
    entries: [
      ['Plan verified', 'The planner checked the mathematical reference against its model. Compare the accepted route with the draft, especially if the plan was adjusted.'],
      ['Simulator result', 'A completed Gazebo rehearsal includes recorded tracking and landing checks. Expand View simulator checks to see which checks passed.'],
      ['Save the evidence', 'Save the editable project separately. Route CSV saves the verified path, and Report JSON saves the settings and verification evidence.'],
    ],
  },
} satisfies Record<HelpTopic, {title: string; entries: string[][]}>;

export default function WorkspaceHelp({dialog, topic, onTopic, onPractice}: {
  dialog: RefObject<HTMLDialogElement | null>;
  topic: HelpTopic;
  onTopic: (topic: HelpTopic) => void;
  onPractice: () => void;
}) {
  return <dialog ref={dialog} className="help-dialog" aria-labelledby="help-title">
    <div className="help-header">
      <div className="help-mark"><CircleHelp size={23} aria-hidden="true"/></div>
      <h2 id="help-title">Studio help</h2>
      <form method="dialog"><button autoFocus className="ghost help-close" aria-label="Close help"><X size={19}/></button></form>
    </div>
    <nav className="help-topics" aria-label="Help topics">
      {(Object.keys(guide) as HelpTopic[]).map(key => <button key={key} type="button"
        aria-pressed={key === topic} onClick={() => onTopic(key)}>{guide[key].title}</button>)}
    </nav>
    <dl className="help-entries">
      {guide[topic].entries.map(([term, text]) => <div key={term}><dt>{term}</dt><dd>{text}</dd></div>)}
    </dl>
    <div className="help-footer"><button onClick={onPractice}><BookOpen size={16} aria-hidden="true"/> Start practice tutorial</button>
      <a href="https://github.com/XpiredRuby/PX4_project_improved/blob/feature/offline-research-studio/docs/trajectory_studio_handbook.md"
      target="_blank" rel="noreferrer"><BookOpen size={16} aria-hidden="true"/> Open the full handbook</a></div>
  </dialog>;
}
