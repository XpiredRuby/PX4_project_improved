import { ArrowLeft, ArrowRight, BookOpen, Check } from 'lucide-react';

export const practiceLessons = [
  {
    id: 'map', view: 'geometry', title: 'Meet the route map',
    text: 'The line connects your route points. Choose Add point and click the map to extend it, or use Select / move to drag a point. Fit route brings the whole path back into view.',
  },
  {
    id: 'waypoints', view: 'geometry', title: 'Edit a route point',
    text: 'Each row is a point the drone visits in order. Edit its position or height in metres. Tick Stop if the drone needs to pause there. Undo reverses an edit.',
  },
  {
    id: 'settings', view: 'geometry', title: 'Choose height and speed',
    text: 'Height is measured above takeoff. Preferred cruise is your target speed, and Never exceed sets a limit. The practice values are ready to use.',
  },
  {
    id: 'check', view: 'geometry', title: 'Check the route',
    text: 'The planner calculates how the drone could fly the path and checks it against the aircraft limits. Your drawing stays available to edit.',
  },
  {
    id: 'results', view: 'analysis', title: 'Read the route checks',
    text: 'Verified means the plan passed the model checks. Look for Changes to review, then watch the preview to see the planned motion.',
  },
  {
    id: 'camera', view: 'trajectory', title: 'Look around in 3D',
    text: 'Drag the view to look around. Top view looks straight down, and Orbit resets the 3D angle. Use the colour menu to see speed, height, or heading.',
  },
  {
    id: 'playback', view: 'trajectory', title: 'Control the preview',
    text: 'Use Play or Pause, or drag the time slider to jump ahead. Full mission includes takeoff and landing when available. Route shows only the path between points.',
  },
  {
    id: 'tools', view: 'analysis', title: 'Explore results or save your work',
    text: 'Use Charts for speed and height, More tools for recordings and studies, or Save to reopen the route later. Simulator runs are separate from the preview.',
  },
] as const;

export type PracticeLesson = typeof practiceLessons[number]['id'];

export default function PracticeGuide({lesson, action, disabled, onNext, onBack, onSkip}: {
  lesson: PracticeLesson;
  action: string;
  disabled: boolean;
  onNext: () => void;
  onBack: () => void;
  onSkip: () => void;
}) {
  const index = practiceLessons.findIndex(item => item.id === lesson);
  const {title, text} = practiceLessons[index];
  const done = action === 'Finish tutorial';

  return <section className="practice-guide" aria-label="Practice flight tutorial">
    <div className="practice-copy" aria-live="polite" aria-atomic="true">
      <span className="practice-progress"><BookOpen size={14} aria-hidden="true"/> Practice tutorial · {index+1} of {practiceLessons.length}</span>
      <h2>{title}</h2>
      <p>{text}</p>
    </div>
    <div className="practice-actions">
      <button className="skip-guide" onClick={onSkip}>Skip tutorial</button>
      <button onClick={onBack} disabled={index===0}><ArrowLeft size={14} aria-hidden="true"/> Back</button>
      <button className="primary" onClick={onNext} disabled={disabled}>
        {action}{done?<Check size={15} aria-hidden="true"/>:<ArrowRight size={15} aria-hidden="true"/>}
      </button>
    </div>
  </section>;
}
