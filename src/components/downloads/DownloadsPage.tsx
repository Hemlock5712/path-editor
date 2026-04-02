import { useState } from 'react';
import { Download, FileCode, Package, Puzzle, Loader2 } from 'lucide-react';
import JSZip from 'jszip';
import { saveAs } from 'file-saver';
import { Titlebar } from '../layout/Titlebar';
import { JAVA_FILES, type JavaFileInfo } from '../../utils/javaFiles';

export function DownloadsPage() {
  const [downloading, setDownloading] = useState(false);

  const coreFiles = JAVA_FILES.filter((f) => f.category === 'core');
  const commandFiles = JAVA_FILES.filter((f) => f.category === 'command');
  const depFiles = JAVA_FILES.filter((f) => f.category === 'dependency');

  const downloadFile = async (file: JavaFileInfo) => {
    const res = await fetch(`/java/${file.relativePath}`);
    const text = await res.text();
    const blob = new Blob([text], { type: 'text/x-java-source' });
    saveAs(blob, file.filename);
  };

  const downloadZip = async () => {
    setDownloading(true);
    try {
      const zip = new JSZip();
      await Promise.all(
        JAVA_FILES.map(async (file) => {
          const res = await fetch(`/java/${file.relativePath}`);
          const text = await res.text();
          zip.file(`${file.packagePath}${file.filename}`, text);
        }),
      );
      const blob = await zip.generateAsync({ type: 'blob' });
      saveAs(blob, 'frc-path-following.zip');
    } finally {
      setDownloading(false);
    }
  };

  return (
    <div className="h-screen flex flex-col bg-surface-950">
      <Titlebar />
      <div className="flex-1 overflow-y-auto">
        <div className="max-w-2xl mx-auto px-6 py-8 space-y-5 animate-fadeIn">
          <div className="mb-6">
            <h2 className="text-sm font-light tracking-[0.15em] uppercase text-accent-green/60">
              Java Path Following Files
            </h2>
            <p className="text-xs text-zinc-500 mt-1">
              Download the Java source files needed to integrate the distance-based
              path following system into your FRC robot project.
            </p>
          </div>

          {/* Download All */}
          <button
            onClick={downloadZip}
            disabled={downloading}
            className="btn-primary flex items-center gap-2 text-sm w-full justify-center py-2.5"
          >
            {downloading ? (
              <Loader2 size={14} className="animate-spin" />
            ) : (
              <Download size={14} />
            )}
            {downloading ? 'Generating ZIP...' : 'Download All as ZIP'}
          </button>

          {/* Core Path Utilities */}
          <FileSection
            icon={<Package size={13} />}
            title="Core Path Utilities"
            description="frc.robot.utils.path"
            files={coreFiles}
            onDownload={downloadFile}
          />

          {/* Path Following Command */}
          <FileSection
            icon={<FileCode size={13} />}
            title="Path Following Command"
            description="frc.robot.commands"
            files={commandFiles}
            onDownload={downloadFile}
          />

          {/* Dependencies */}
          <div className="space-y-0">
            <FileSection
              icon={<Puzzle size={13} />}
              title="Dependencies"
              description="Required by the path system. You may already have equivalents in your project."
              files={depFiles}
              onDownload={downloadFile}
            />
          </div>

          {/* Usage instructions */}
          <div className="neon-panel p-4">
            <h3 className="flex items-center gap-2 text-[11px] font-light tracking-wide text-accent-green/40 mb-3">
              Quick Start
            </h3>
            <div className="text-xs text-zinc-400 space-y-2 font-mono">
              <p className="text-zinc-500">1. Copy files to your robot project maintaining the package structure</p>
              <p className="text-zinc-500">2. Place path JSON files in <span className="text-accent-green">src/main/deploy/paths/</span></p>
              <p className="text-zinc-500">3. Use in autonomous:</p>
              <pre className="bg-surface-900 rounded p-3 text-[11px] overflow-x-auto text-zinc-300">
{`var path = PathJsonLoader.fromFile("myPath.json");
new FollowPath(drivetrain, path)
    .withLookahead(0.15, 0.15, 1.0)
    .withCrossTrackGains(3.0, 0.5);`}
              </pre>
            </div>
          </div>

          <div className="pb-8" />
        </div>
      </div>
    </div>
  );
}

function FileSection({
  icon,
  title,
  description,
  files,
  onDownload,
}: {
  icon: React.ReactNode;
  title: string;
  description: string;
  files: JavaFileInfo[];
  onDownload: (file: JavaFileInfo) => void;
}) {
  return (
    <div className="neon-panel p-4">
      <h3 className="flex items-center gap-2 text-[11px] font-light tracking-wide text-accent-green/40 mb-1">
        {icon}
        {title}
      </h3>
      <p className="text-[10px] text-zinc-600 mb-3">{description}</p>
      <div className="space-y-1.5">
        {files.map((file) => (
          <FileCard key={file.filename} file={file} onDownload={() => onDownload(file)} />
        ))}
      </div>
    </div>
  );
}

function FileCard({
  file,
  onDownload,
}: {
  file: JavaFileInfo;
  onDownload: () => void;
}) {
  return (
    <div className="flex items-center gap-3 py-1.5 px-2 rounded hover:bg-white/[0.02] transition-colors group">
      <FileCode size={14} className="text-zinc-600 shrink-0" />
      <div className="flex-1 min-w-0">
        <div className="text-xs font-mono text-accent-green truncate">{file.filename}</div>
        <div className="text-[10px] text-zinc-600 truncate">{file.description}</div>
      </div>
      <span className="text-[10px] text-zinc-600 font-mono shrink-0">{file.lineCount}L</span>
      <button
        onClick={onDownload}
        className="btn-ghost p-1 opacity-0 group-hover:opacity-100 transition-opacity"
        title={`Download ${file.filename}`}
      >
        <Download size={12} />
      </button>
    </div>
  );
}
