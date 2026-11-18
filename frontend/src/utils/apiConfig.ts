/**
 * API & WebSocket Host Dynamic Configuration
 * Ensures seamless operation whether on localhost, 127.0.0.1, or local LAN IP.
 */

export const getApiBaseUrl = (): string => {
  if (typeof window !== 'undefined') {
    const host = window.location.hostname || 'localhost';
    const protocol = window.location.protocol === 'https:' ? 'https:' : 'http:';
    return `${protocol}//${host}:8000`;
  }
  return 'http://localhost:8000';
};

export const getWsBaseUrl = (): string => {
  if (typeof window !== 'undefined') {
    const host = window.location.hostname || 'localhost';
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    return `${protocol}//${host}:8000/ws/telemetry`;
  }
  return 'ws://localhost:8000/ws/telemetry';
};
