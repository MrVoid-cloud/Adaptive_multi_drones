import React, { useState, useEffect } from 'react';
import { Download, CheckCircle2, Smartphone, Monitor } from 'lucide-react';

interface BeforeInstallPromptEvent extends Event {
  readonly platforms: string[];
  readonly userChoice: Promise<{
    outcome: 'accepted' | 'dismissed';
    platform: string;
  }>;
  prompt(): Promise<void>;
}

export const InstallAppBanner: React.FC = () => {
  const [deferredPrompt, setDeferredPrompt] = useState<BeforeInstallPromptEvent | null>(null);
  const [isStandalone, setIsStandalone] = useState<boolean>(false);
  const [isDismissed, setIsDismissed] = useState<boolean>(false);
  const [isInstalled, setIsInstalled] = useState<boolean>(false);

  useEffect(() => {
    // Check if running in standalone display mode
    const checkStandalone = () => {
      const isStandaloneMode = 
        window.matchMedia('(display-mode: standalone)').matches ||
        (window.navigator as unknown as { standalone?: boolean }).standalone === true;
      setIsStandalone(isStandaloneMode);
    };

    checkStandalone();

    // Listen for beforeinstallprompt event
    const handleBeforeInstallPrompt = (e: Event) => {
      e.preventDefault();
      setDeferredPrompt(e as BeforeInstallPromptEvent);
    };

    // Listen for successful installation
    const handleAppInstalled = () => {
      setIsInstalled(true);
      setDeferredPrompt(null);
    };

    window.addEventListener('beforeinstallprompt', handleBeforeInstallPrompt);
    window.addEventListener('appinstalled', handleAppInstalled);

    return () => {
      window.removeEventListener('beforeinstallprompt', handleBeforeInstallPrompt);
      window.removeEventListener('appinstalled', handleAppInstalled);
    };
  }, []);

  const handleInstallClick = async () => {
    if (!deferredPrompt) return;

    await deferredPrompt.prompt();
    const { outcome } = await deferredPrompt.userChoice;
    
    if (outcome === 'accepted') {
      setIsInstalled(true);
    }
    setDeferredPrompt(null);
  };

  // If already standalone, display small indicator badge
  if (isStandalone || isInstalled) {
    return (
      <div 
        className="flex items-center space-x-1.5 px-2.5 py-1 rounded bg-emerald-500/10 border border-emerald-500/30 text-[10px] text-emerald-400 font-mono"
        title="Running in Standalone Progressive Web App Mode"
      >
        <CheckCircle2 className="w-3 h-3 text-emerald-400" />
        <span className="hidden sm:inline">STANDALONE PWA</span>
      </div>
    );
  }

  // If dismissed or prompt not ready, return null
  if (!deferredPrompt || isDismissed) {
    return null;
  }

  return (
    <div className="flex items-center space-x-2 animate-pulse">
      <button
        onClick={handleInstallClick}
        className="flex items-center space-x-1.5 px-3 py-1.5 rounded-lg bg-gradient-to-r from-emerald-600 to-teal-600 hover:from-emerald-500 hover:to-teal-500 text-slate-950 font-bold text-xs shadow-lg shadow-emerald-950/30 transition-all border border-emerald-400/40"
        title="Install Mission Control as Standalone PWA"
      >
        <Download className="w-3.5 h-3.5 fill-current" />
        <span>Install App</span>
      </button>

      <button
        onClick={() => setIsDismissed(true)}
        className="text-slate-500 hover:text-slate-300 text-xs px-1"
        title="Dismiss prompt"
      >
        &times;
      </button>
    </div>
  );
};

export default InstallAppBanner;
