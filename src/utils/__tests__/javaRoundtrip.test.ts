import { describe, it, expect } from 'vitest';
import { generatePathsJava } from '../javaExport';
import { parsePathsJava } from '../javaParser';
import { NamedPath, DEFAULT_CONSTRAINTS } from '../../types';

describe('Java export/parse roundtrip', () => {
  const testPaths: NamedPath[] = [
    {
      name: 'Score Left',
      controlPoints: [
        { x: 1.5, y: 2.0 },
        { x: 3.0, y: 4.5 },
        { x: 6.0, y: 2.0 },
        { x: 8.5, y: 5.5 },
      ],
      controlPointRefs: [null, null, null, null],
      headingWaypoints: [
        { waypointIndex: 0, degrees: 45 },
        { waypointIndex: 3, degrees: 135 },
      ],
      constraints: {
        ...DEFAULT_CONSTRAINTS,
        maxVelocity: 4.0,
        maxAcceleration: 8.5,
        startVelocity: 1.0,
        endVelocity: 0.5,
      },
      constraintZones: [
        {
          id: 'z1',
          startWaypointIndex: 1,
          endWaypointIndex: 2,
          maxVelocity: 2.0,
          maxAcceleration: 5.0,
        },
      ],
      rotationZones: [
        {
          id: 'rz1',
          startWaypointIndex: 0,
          endWaypointIndex: 3,
          targetPoint: { x: 4.0, y: 3.0 },
        },
      ],
      waypointFlags: [
        { id: 'wf1', waypointIndex: 1, label: 'intake' },
        { id: 'wf2', waypointIndex: 1, label: 'settle' },
        { id: 'wf3', waypointIndex: 3, label: 'shoot' },
      ],
    },
  ];

  it('roundtrips control points', () => {
    const java = generatePathsJava(testPaths);
    const parsed = parsePathsJava(java);

    expect(parsed).toHaveLength(1);
    expect(parsed[0].controlPoints).toHaveLength(
      testPaths[0].controlPoints.length
    );

    for (let i = 0; i < testPaths[0].controlPoints.length; i++) {
      expect(parsed[0].controlPoints[i].x).toBeCloseTo(
        testPaths[0].controlPoints[i].x
      );
      expect(parsed[0].controlPoints[i].y).toBeCloseTo(
        testPaths[0].controlPoints[i].y
      );
    }
  });

  it('roundtrips heading waypoints', () => {
    const java = generatePathsJava(testPaths);
    const parsed = parsePathsJava(java);

    expect(parsed[0].headingWaypoints).toHaveLength(
      testPaths[0].headingWaypoints.length
    );
    for (let i = 0; i < testPaths[0].headingWaypoints.length; i++) {
      expect(parsed[0].headingWaypoints[i].waypointIndex).toBe(
        testPaths[0].headingWaypoints[i].waypointIndex
      );
      expect(parsed[0].headingWaypoints[i].degrees).toBeCloseTo(
        testPaths[0].headingWaypoints[i].degrees
      );
    }
  });

  it('roundtrips velocity constraints', () => {
    const java = generatePathsJava(testPaths);
    const parsed = parsePathsJava(java);

    expect(parsed[0].constraints.maxVelocity).toBe(4.0);
    expect(parsed[0].constraints.maxAcceleration).toBe(8.5);
    expect(parsed[0].constraints.startVelocity).toBe(1.0);
    expect(parsed[0].constraints.endVelocity).toBe(0.5);
  });

  it('roundtrips constraint zones', () => {
    const java = generatePathsJava(testPaths);
    const parsed = parsePathsJava(java);

    expect(parsed[0].constraintZones).toHaveLength(1);
    expect(parsed[0].constraintZones[0].startWaypointIndex).toBe(1);
    expect(parsed[0].constraintZones[0].endWaypointIndex).toBe(2);
    expect(parsed[0].constraintZones[0].maxVelocity).toBe(2.0);
    expect(parsed[0].constraintZones[0].maxAcceleration).toBe(5.0);
  });

  it('roundtrips rotation zones', () => {
    const java = generatePathsJava(testPaths);
    const parsed = parsePathsJava(java);

    expect(parsed[0].rotationZones).toHaveLength(1);
    expect(parsed[0].rotationZones[0].startWaypointIndex).toBe(0);
    expect(parsed[0].rotationZones[0].endWaypointIndex).toBe(3);
    expect(parsed[0].rotationZones[0].targetPoint.x).toBeCloseTo(4.0);
    expect(parsed[0].rotationZones[0].targetPoint.y).toBeCloseTo(3.0);
  });

  it('roundtrips waypoint flags', () => {
    const java = generatePathsJava(testPaths);
    const parsed = parsePathsJava(java);

    expect(parsed[0].waypointFlags).toEqual(testPaths[0].waypointFlags);
  });

  it('roundtrips path name as UPPER_SNAKE_CASE', () => {
    const java = generatePathsJava(testPaths);
    const parsed = parsePathsJava(java);

    expect(parsed[0].name).toBe('SCORE_LEFT');
  });

  it('roundtrips multiple paths', () => {
    const twoPaths: NamedPath[] = [
      {
        ...testPaths[0],
        name: 'Path A',
      },
      {
        name: 'Path B',
        controlPoints: [
          { x: 0, y: 0 },
          { x: 10, y: 5 },
        ],
        controlPointRefs: [null, null],
        headingWaypoints: [],
        constraints: DEFAULT_CONSTRAINTS,
        constraintZones: [],
        rotationZones: [],
        waypointFlags: [],
      },
    ];

    const java = generatePathsJava(twoPaths);
    const parsed = parsePathsJava(java);

    expect(parsed).toHaveLength(2);
    expect(parsed[0].name).toBe('PATH_A');
    expect(parsed[1].name).toBe('PATH_B');
  });

  it('handles empty heading/zone lists', () => {
    const simplePath: NamedPath[] = [
      {
        name: 'Simple',
        controlPoints: [
          { x: 0, y: 0 },
          { x: 5, y: 5 },
        ],
        controlPointRefs: [null, null],
        headingWaypoints: [],
        constraints: DEFAULT_CONSTRAINTS,
        constraintZones: [],
        rotationZones: [],
        waypointFlags: [],
      },
    ];

    const java = generatePathsJava(simplePath);
    const parsed = parsePathsJava(java);

    expect(parsed[0].headingWaypoints).toHaveLength(0);
    expect(parsed[0].constraintZones).toHaveLength(0);
    expect(parsed[0].rotationZones).toHaveLength(0);
    expect(parsed[0].waypointFlags).toHaveLength(0);
  });
});
