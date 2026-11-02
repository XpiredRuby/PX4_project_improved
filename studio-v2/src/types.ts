export type Point = {
  id: string; north: number; east: number; altitude: number;
  hard: boolean; exact: boolean;
};
export type Tool = 'select'|'waypoint'|'draw'|'pan';
export type EnvMode = 'manual'|'live';
export type Environment = {
  mode: EnvMode; latitude: number; longitude: number;
  timestamp: string; windSpeed: number; windDirection: number;
  gust: number; temperature: number; pressure: number;
  source: string; fetchedAt?: string;
};
export type Sample = {
  time: number; north: number; east: number; altitude: number;
  vx: number; vy: number; vz: number; ax: number; ay: number; az: number;
  jx: number; jy: number; jz: number; yaw: number;
  speed: number; acceleration: number; jerk: number;
};
export type Preview = {
  accepted: boolean; error?: string; trajectory_csv?: string;
  full_mission_csv?: string; summary?: Record<string,unknown>;
  verification?: Record<string,unknown>;
  adjustments?: unknown; interpretation?: unknown;
  planning_adjustments?: { adjusted?:boolean; adjustments?:Array<{code:string;message:string}>;
    original_rounding_met?:boolean; curve_error_bound_from_requested_m?:number;
    requested_rounding_m?:number; verification_scope?:string; };
  requested_document?: unknown; intent?: unknown;
  release_class?: string; campaign_ran?: boolean;
};
export type Project = {
  schema: 1; title: string; points: Point[]; cruise: number;
  speedCap: number; altitude: number; tolerance: number;
  yaw: number; environment: Environment;
};
