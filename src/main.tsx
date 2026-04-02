import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import { BrowserRouter, Routes, Route } from 'react-router-dom';
import App from './App';
import { SettingsPage } from './components/settings/SettingsPage';
import { DownloadsPage } from './components/downloads/DownloadsPage';
import './app.css';

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<App />} />
        <Route path="/settings" element={<SettingsPage />} />
        <Route path="/downloads" element={<DownloadsPage />} />
      </Routes>
    </BrowserRouter>
  </StrictMode>,
);
