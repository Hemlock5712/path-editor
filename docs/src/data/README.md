# docs/src/data

`assists.json` holds the driver-assist animations (the rail, the intake assist, a goal that follows a tag). It is
written by the Java library itself, because those pieces have no TypeScript port for the site to run when it
builds. Regenerate it after changing `RailLaw`, `RailFollower`, `PieceAssist` or the moving goal:

```
lib/gradlew -p lib jar
java -cp lib/build/libs/pathcore.jar frc.team5712.path.tools.DocScenes docs/src/data/assists.json
```

The other animations are worked out at build time by `../lib/scenes.ts` from `editor/core`.
