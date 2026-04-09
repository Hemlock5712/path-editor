import { useState } from 'react';
import {
  Download,
  FileCode,
  Package,
  Puzzle,
  Loader2,
  Lock,
} from 'lucide-react';
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
        JAVA_FILES.filter((f) => !f.private).map(async (file) => {
          const res = await fetch(`/java/${file.relativePath}`);
          const text = await res.text();
          zip.file(`${file.packagePath}${file.filename}`, text);
        })
      );
      const blob = await zip.generateAsync({ type: 'blob' });
      saveAs(blob, 'frc-path-following.zip');
    } finally {
      setDownloading(false);
    }
  };

  return (
    <div className="bg-surface-950 flex h-screen flex-col">
      <Titlebar />
      <div className="flex-1 overflow-y-auto">
        <div className="animate-fadeIn mx-auto max-w-2xl space-y-5 px-6 py-8">
          <div className="mb-6">
            <h2 className="text-accent-green/60 text-sm font-light tracking-[0.15em] uppercase">
              Java Path Following Files
            </h2>
            <p className="mt-1 text-xs text-zinc-500">
              Download the Java source files needed to integrate the
              distance-based path following system into your FRC robot project.
            </p>
          </div>

          {/* Download All */}
          <button
            onClick={downloadZip}
            disabled={downloading}
            className="btn-primary flex w-full items-center justify-center gap-2 py-2.5 text-sm"
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
              description="Required by the path system. Private files require permission to access."
              files={depFiles}
              onDownload={downloadFile}
            />
          </div>

          {/* Usage instructions */}
          <div className="neon-panel p-4">
            <h3 className="text-accent-green/40 mb-3 flex items-center gap-2 text-[11px] font-light tracking-wide">
              Quick Start
            </h3>
            <div className="space-y-2 font-mono text-xs text-zinc-400">
              <p className="text-zinc-500">
                1. Copy files to your robot project maintaining the package
                structure
              </p>
              <p className="text-zinc-500">
                2. Save <span className="text-accent-green">Paths.java</span>{' '}
                from the editor and add it to your project
              </p>
              <p className="text-zinc-500">3. Use in autonomous:</p>
              <pre className="bg-surface-900 overflow-x-auto rounded p-3 text-[11px] text-zinc-300">
                {`// Auto-mirrors path when on red alliance
var pathData = Paths.forAlliance(Paths.MY_PATH);
return autoCommands.followPathWithActions(
    pathData,
    List.of(
        new AutoCommands.PathAction("intake", 0.5, intakeCommand),
        new AutoCommands.PathAction("shoot", 0.5, shootCommand)));`}
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
      <h3 className="text-accent-green/40 mb-1 flex items-center gap-2 text-[11px] font-light tracking-wide">
        {icon}
        {title}
      </h3>
      <p className="mb-3 text-[10px] text-zinc-600">{description}</p>
      <div className="space-y-1.5">
        {files.map((file) => (
          <FileCard
            key={file.filename}
            file={file}
            onDownload={() => onDownload(file)}
          />
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
    <div className="group flex items-center gap-3 rounded px-2 py-1.5 transition-colors hover:bg-white/[0.02]">
      <FileCode size={14} className="shrink-0 text-zinc-600" />
      <div className="min-w-0 flex-1">
        <div className="text-accent-green truncate font-mono text-xs">
          {file.filename}
        </div>
        <div className="truncate text-[10px] text-zinc-600">
          {file.description}
        </div>
      </div>
      <span className="shrink-0 font-mono text-[10px] text-zinc-600">
        {file.lineCount}L
      </span>
      {file.private ? (
        <span className="flex items-center gap-1 text-[10px] text-amber-500/70">
          <Lock size={10} />
          Private
        </span>
      ) : (
        <button
          onClick={onDownload}
          className="btn-ghost p-1 opacity-0 transition-opacity group-hover:opacity-100"
          title={`Download ${file.filename}`}
        >
          <Download size={12} />
        </button>
      )}
    </div>
  );
}
