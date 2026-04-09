export const registerServiceWorker = async () => {
  if (!import.meta.env.PROD || !('serviceWorker' in navigator)) {
    return;
  }

  try {
    await navigator.serviceWorker.register('/sw.js');
  } catch (error) {
    console.error('Service worker registration failed:', error);
  }
};
