/**
/**
 * Deterministic Per-Drone Color Coding System
 * Phase 03: Dual-Mode Google Maps Visualization & Fleet Color Identity
 * Author: Principal Software Architect
 */

export interface DroneColorDefinition {
  id: string;
  index: number;
  hex: string;
  stroke: string;
  fill: string;
  glow: string;
  badgeBg: string;
  badgeBorder: string;
  badgeText: string;
}

// High-contrast, visually distinct cyber/military HUD colors for multi-drone fleets
const MASTER_FLEET_PALETTE: Omit<DroneColorDefinition, 'id' | 'index'>[] = [
  {
    // Drone 1: Electric Cyan
    hex: '#00f0ff',
    stroke: '#06b6d4',
    fill: 'rgba(6, 182, 212, 0.22)',
    glow: 'rgba(6, 182, 212, 0.45)',
    badgeBg: 'bg-cyan-950/70',
    badgeBorder: 'border-cyan-500/50',
    badgeText: 'text-cyan-300'
  },
  {
    // Drone 2: Vivid Amber / Orange
    hex: '#ff9900',
    stroke: '#f59e0b',
    fill: 'rgba(245, 158, 11, 0.22)',
    glow: 'rgba(245, 158, 11, 0.45)',
    badgeBg: 'bg-amber-950/70',
    badgeBorder: 'border-amber-500/50',
    badgeText: 'text-amber-300'
  },
  {
    // Drone 3: Cyber Emerald
    hex: '#00ff88',
    stroke: '#10b981',
    fill: 'rgba(16, 185, 129, 0.22)',
    glow: 'rgba(16, 185, 129, 0.45)',
    badgeBg: 'bg-emerald-950/70',
    badgeBorder: 'border-emerald-500/50',
    badgeText: 'text-emerald-300'
  },
  {
    // Drone 4: Neon Purple / Violet
    hex: '#bf00ff',
    stroke: '#a855f7',
    fill: 'rgba(168, 85, 247, 0.22)',
    glow: 'rgba(168, 85, 247, 0.45)',
    badgeBg: 'bg-purple-950/70',
    badgeBorder: 'border-purple-500/50',
    badgeText: 'text-purple-300'
  },
  {
    // Drone 5: Radiant Crimson / Rose
    hex: '#ff0055',
    stroke: '#f43f5e',
    fill: 'rgba(244, 63, 94, 0.22)',
    glow: 'rgba(244, 63, 94, 0.45)',
    badgeBg: 'bg-rose-950/70',
    badgeBorder: 'border-rose-500/50',
    badgeText: 'text-rose-300'
  },
  {
    // Drone 6: Golden Yellow
    hex: '#ffd700',
    stroke: '#eab308',
    fill: 'rgba(234, 179, 8, 0.22)',
    glow: 'rgba(234, 179, 8, 0.45)',
    badgeBg: 'bg-yellow-950/70',
    badgeBorder: 'border-yellow-500/50',
    badgeText: 'text-yellow-300'
  },
  {
    // Drone 7: Electric Sky Blue
    hex: '#0088ff',
    stroke: '#3b82f6',
    fill: 'rgba(59, 130, 246, 0.22)',
    glow: 'rgba(59, 130, 246, 0.45)',
    badgeBg: 'bg-blue-950/70',
    badgeBorder: 'border-blue-500/50',
    badgeText: 'text-blue-300'
  },
  {
    // Drone 8: Lime Green
    hex: '#76ff03',
    stroke: '#84cc16',
    fill: 'rgba(132, 204, 22, 0.22)',
    glow: 'rgba(132, 204, 22, 0.45)',
    badgeBg: 'bg-lime-950/70',
    badgeBorder: 'border-lime-500/50',
    badgeText: 'text-lime-300'
  },
  {
    // Drone 9: Magenta
    hex: '#ff00aa',
    stroke: '#d946ef',
    fill: 'rgba(217, 70, 239, 0.22)',
    glow: 'rgba(217, 70, 239, 0.45)',
    badgeBg: 'bg-fuchsia-950/70',
    badgeBorder: 'border-fuchsia-500/50',
    badgeText: 'text-fuchsia-300'
  },
  {
    // Drone 10: Deep Teal
    hex: '#00e5ff',
    stroke: '#14b8a6',
    fill: 'rgba(20, 184, 166, 0.22)',
    glow: 'rgba(20, 184, 166, 0.45)',
    badgeBg: 'bg-teal-950/70',
    badgeBorder: 'border-teal-500/50',
    badgeText: 'text-teal-300'
  }
];

/**
 * Returns a deterministic, consistent, high-contrast color scheme for a given drone.
 * Guaranteed to match between Google Maps Polylines, HUD badges, and waypoints tables.
 */
export function getDroneColor(droneIndex: number, droneId?: string): DroneColorDefinition {
  // If droneId has integer suffix like UAV-03, use 2 (0-indexed)
  let idx = droneIndex;
  if (droneId) {
    const match = droneId.match(/\d+/);
    if (match) {
      const parsed = parseInt(match[0], 10);
      if (!isNaN(parsed) && parsed > 0) {
        idx = parsed - 1;
      }
    }
  }

  const paletteIndex = Math.abs(idx) % MASTER_FLEET_PALETTE.length;
  const base = MASTER_FLEET_PALETTE[paletteIndex];
  const assignedId = droneId || `UAV-${(idx + 1).toString().padStart(2, '0')}`;

  return {
    ...base,
    id: assignedId,
    index: idx
  };
}

/**
 * Generates an indexed list of drone colors for an active fleet size.
 */
export function getFleetColors(fleetSize: number): DroneColorDefinition[] {
  return Array.from({ length: fleetSize }, (_, i) => getDroneColor(i));
}
