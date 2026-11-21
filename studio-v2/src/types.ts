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
  humidity?: number; forecastValidAt?: string; source: string; fetchedAt?: string;
};
export type Sample = {
  time: number; north: number; east: number; altitude: number;
  vx: number; vy: number; vz: number; ax: number; ay: number; az: number;
  jx: number; jy: number; jz: number; yaw: number;
  yawRate: number; yawAcceleration: number; yawJerk: number;
  speed: number; acceleration: number; jerk: number;
};
export type Preview = {
  vehicle_model?: {mass_kg:number; maximum_thrust_n:number; maximum_tilt_deg:number};
  accepted: boolean; error?: string; trajectory_csv?: string;
  full_mission_csv?: string; mission?: {trajectory_csv?:string; phases?:unknown[]}; summary?: Record<string,unknown>;
  verification?: Record<string,unknown>;
  adjustments?: unknown; interpretation?: unknown;
  planning_adjustments?: { method?:string; adjusted?:boolean; adjustments?:Array<{code:string;message:string}>;
    original_rounding_met?:boolean; curve_error_bound_from_requested_m?:number;
    requested_rounding_m?:number; verification_scope?:string; };
  requested_document?: unknown; intent?: unknown;
  release_class?: string; campaign_ran?: boolean;
};
export type AircraftSetup = {
  preset: string; sha256: string;
  profile: {profile_id: string; name: string; maximum_speed_m_s: number; [key: string]: unknown};
};
export type Project = {
  aircraft?: AircraftSetup;
  schema: 1; title: string; points: Point[]; cruise: number;
  speedCap: number; altitude: number; tolerance: number;
  yaw: number; environment: Environment;
};
