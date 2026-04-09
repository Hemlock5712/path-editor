import { registerSW } from 'virtual:pwa-register';

export const registerServiceWorker = () => {
  if (!import.meta.env.PROD) {
    return;
  }

  registerSW({
    immediate: true,
    onRegisterError(error) {
      console.error('Service worker registration failed:', error);
    },
  });
};
