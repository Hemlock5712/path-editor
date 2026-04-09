import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import { BrowserRouter, Routes, Route } from 'react-router-dom';
import App from './App';
import { SettingsPage } from './components/settings/SettingsPage';
import { DownloadsPage } from './components/downloads/DownloadsPage';
import { GettingStartedPage } from './components/docs/GettingStartedPage';
import { RobotIntegrationPage } from './components/docs/RobotIntegrationPage';
import { EditorGuidePage } from './components/docs/EditorGuidePage';
import './app.css';
import { registerServiceWorker } from './registerServiceWorker';

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<App />} />
        <Route path="/settings" element={<SettingsPage />} />
        <Route path="/downloads" element={<DownloadsPage />} />
        <Route path="/docs/getting-started" element={<GettingStartedPage />} />
        <Route
          path="/docs/robot-integration"
          element={<RobotIntegrationPage />}
        />
        <Route path="/docs/editor-guide" element={<EditorGuidePage />} />
      </Routes>
    </BrowserRouter>
  </StrictMode>
);

registerServiceWorker();
