import { useRef, useEffect, useCallback } from 'react';
import { useEditorStore } from '../stores/editorStore';
import { SplinePath } from '../math/SplinePath';
import { VelocityProfile } from '../math/VelocityProfile';

export function usePlayback(
  splinePath: SplinePath | null,
  velocityProfile: VelocityProfile | null,
) {
  const playbackState = useEditorStore((s) => s.playbackState);
  const playbackSpeed = useEditorStore((s) => s.playbackSpeed);
  const setPlaybackState = useEditorStore((s) => s.setPlaybackState);
  const setScrubberDistance = useEditorStore((s) => s.setScrubberDistance);

  const playbackSRef = useRef(0);
  const lastFrameTimeRef = useRef(0);
  const rafRef = useRef(0);
  const prevTotalLengthRef = useRef(0);

  // Rescale scrubber proportionally when path length changes
  useEffect(() => {
    const newLen = splinePath?.totalLength ?? 0;
    const prevLen = prevTotalLengthRef.current;
    if (prevLen > 0 && newLen > 0 && newLen !== prevLen && playbackSRef.current > 0) {
      const ratio = playbackSRef.current / prevLen;
      const newDistance = Math.max(0, Math.min(ratio * newLen, newLen));
      playbackSRef.current = newDistance;
      setScrubberDistance(newDistance);
    }
    prevTotalLengthRef.current = newLen;
  }, [splinePath, setScrubberDistance]);

  // Playback animation loop
  useEffect(() => {
    if (playbackState !== 'playing' || !splinePath || !velocityProfile) {
      cancelAnimationFrame(rafRef.current);
      return;
    }

    const totalLen = splinePath.totalLength;

    const animate = (timestamp: number) => {
      if (lastFrameTimeRef.current === 0) {
        lastFrameTimeRef.current = timestamp;
      }
      const dt = Math.min((timestamp - lastFrameTimeRef.current) / 1000, 0.05);
      lastFrameTimeRef.current = timestamp;

      const s = playbackSRef.current;
      const v = velocityProfile.getVelocity(s);
      playbackSRef.current = s + v * dt * playbackSpeed;

      if (playbackSRef.current >= totalLen) {
        playbackSRef.current = totalLen;
        setScrubberDistance(totalLen);
        setPlaybackState('stopped');
        return;
      }

      setScrubberDistance(playbackSRef.current);

      rafRef.current = requestAnimationFrame(animate);
    };

    rafRef.current = requestAnimationFrame(animate);
    return () => cancelAnimationFrame(rafRef.current);
  }, [
    playbackState,
    splinePath,
    velocityProfile,
    playbackSpeed,
    setScrubberDistance,
    setPlaybackState,
  ]);

  const play = useCallback(() => {
    if (!splinePath || !velocityProfile) return;
    playbackSRef.current = 0;
    lastFrameTimeRef.current = 0;
    setScrubberDistance(0);
    setPlaybackState('playing');
  }, [splinePath, velocityProfile, setPlaybackState, setScrubberDistance]);

  const pause = useCallback(() => {
    setPlaybackState('paused');
  }, [setPlaybackState]);

  const resume = useCallback(() => {
    lastFrameTimeRef.current = 0;
    setPlaybackState('playing');
  }, [setPlaybackState]);

  const stop = useCallback(() => {
    setPlaybackState('stopped');
    setScrubberDistance(0);
  }, [setPlaybackState, setScrubberDistance]);

  const stepForward = useCallback(() => {
    if (!splinePath || !velocityProfile) return;
    const newS = Math.min(playbackSRef.current + 0.05, splinePath.totalLength);
    playbackSRef.current = newS;
    setScrubberDistance(newS);
    if (playbackState === 'stopped') setPlaybackState('paused');
  }, [
    splinePath,
    velocityProfile,
    playbackState,
    setScrubberDistance,
    setPlaybackState,
  ]);

  const stepBackward = useCallback(() => {
    if (!splinePath) return;
    const newS = Math.max(playbackSRef.current - 0.05, 0);
    playbackSRef.current = newS;
    setScrubberDistance(newS);
    if (playbackState === 'stopped') setPlaybackState('paused');
  }, [
    splinePath,
    playbackState,
    setScrubberDistance,
    setPlaybackState,
  ]);

  const scrubTo = useCallback(
    (s: number) => {
      if (!splinePath) return;
      const clampedS = Math.max(0, Math.min(s, splinePath.totalLength));
      playbackSRef.current = clampedS;
      setScrubberDistance(clampedS);
      if (playbackState === 'stopped') setPlaybackState('paused');
    },
    [
      splinePath,
      playbackState,
      setScrubberDistance,
      setPlaybackState,
    ],
  );

  return { play, pause, resume, stop, stepForward, stepBackward, scrubTo, playbackState };
}
