import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import { VitePWA } from 'vite-plugin-pwa';

export default defineConfig({
  plugins: [
    react(),
    VitePWA({
      registerType: 'autoUpdate',
      injectRegister: 'auto',
      includeAssets: ['favicon.ico', 'robots.txt', 'icon.svg'],
      manifest: {
        name: 'Adaptive Multi-Drone Mission Command',
        short_name: 'DroneMission',
        description: 'Autonomous Search Space Discretization, LiDAR Mapping & Hybrid GA-PSO Optimizer',
        theme_color: '#0f172a',
        background_color: '#020617',
        display: 'standalone',
        orientation: 'landscape-primary',
        scope: '/',
        start_url: '/',
        icons: [
          {
            src: '/icon.svg',
            sizes: '192x192 512x512',
            type: 'image/svg+xml',
            purpose: 'any maskable'
          }
        ]
      },
      workbox: {
        globPatterns: ['**/*.{js,css,html,ico,png,svg,woff2}'],
        runtimeCaching: [
          // 1. Google Maps & Geodesic Tile Caching (Cache-First with TTL)
          {
            urlPattern: /^https:\/\/maps\.googleapis\.com\/.*/i,
            handler: 'CacheFirst',
            options: {
              cacheName: 'google-maps-cache',
              expiration: {
                maxEntries: 100,
                maxAgeSeconds: 60 * 60 * 24 * 7 // 7 days
              },
              cacheableResponse: {
                statuses: [0, 200]
              }
            }
          },
          {
            urlPattern: /^https:\/\/maps\.gstatic\.com\/.*/i,
            handler: 'CacheFirst',
            options: {
              cacheName: 'google-maps-static-cache',
              expiration: {
                maxEntries: 100,
                maxAgeSeconds: 60 * 60 * 24 * 7
              },
              cacheableResponse: {
                statuses: [0, 200]
              }
            }
          },
          // 2. Static Assets & Fonts (Cache-First)
          {
            urlPattern: /\.(?:png|jpg|jpeg|svg|gif|webp|woff2?|eot|ttf|otf)$/i,
            handler: 'CacheFirst',
            options: {
              cacheName: 'static-assets-cache',
              expiration: {
                maxEntries: 60,
                maxAgeSeconds: 60 * 60 * 24 * 30 // 30 days
              }
            }
          },
          // 3. Application Scripts & Styles (Stale-While-Revalidate)
          {
            urlPattern: /\.(?:js|css)$/i,
            handler: 'StaleWhileRevalidate',
            options: {
              cacheName: 'app-code-cache',
              expiration: {
                maxEntries: 50,
                maxAgeSeconds: 60 * 60 * 24 // 1 day
              }
            }
          },
          // 4. Mission Telemetry, WebSocket & Simulation Controls (Network-Only: NEVER CACHED)
          {
            urlPattern: /\/api\/v1\/(?:sim|fleet|adaptive|ws).*/i,
            handler: 'NetworkOnly'
          },
          {
            urlPattern: /\/ws\/.*/i,
            handler: 'NetworkOnly'
          }
        ]
      }
    })
  ],
  server: {
    port: 5173,
    host: '0.0.0.0'
  }
});
