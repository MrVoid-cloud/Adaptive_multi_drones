export interface DroneTelemetry {
  drone_id: string;
  ip_endpoint: string;
  latitude: number;
  longitude: number;
  altitude_m: number;
  speed_mps: number;
  battery_pct: number;
  rtl_energy_threshold_pct: number;
  status: 'IDLE' | 'READY' | 'STANDBY' | 'SEARCHING' | 'MAPPING' | 'RTL' | 'DISCONNECTED' | 'SORTIE_FLYING' | 'RETURNING_FOR_SWAP' | 'BATTERY_SWAPPING' | 'RELAUNCHING_SORTIE' | 'MISSION_COMPLETED' | string;
  current_cell_id?: string;
  lasers_active: boolean;
}

export interface TelemetryPacket {
  timestamp: string;
  fleet: DroneTelemetry[];
  terrain_cells_scanned: number;
}
