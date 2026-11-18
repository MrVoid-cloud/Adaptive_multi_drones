/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        hud: {
          bg: "#050b14",
          panel: "#0b1526",
          border: "#1e293b",
          pso: "#38bdf8",     // Cyan for PSO
          ga: "#a855f7",      // Purple for GA
          hybrid: "#10b981",  // Emerald for Hybrid GA-PSO
          alert: "#ef4444"
        }
      }
    },
  },
  plugins: [],
}
