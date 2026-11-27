# docs: the Myelin documentation site

The website that explains the vendordep to students: how to install it, how to use it from robot code, how each
piece works, and a reference page for every public class. Built with Astro Starlight; the pages are Markdown in
`src/content/docs/`.

## Running it

```
cd docs
npm install
npm run dev        # http://localhost:4321/docs/
npm run build      # static site in docs/dist, with the search index
npm run preview    # serves docs/dist
npm run check:links  # after a build: every internal link and anchor
```

It needs Node 22.12 or newer.

## Where it is served

At https://paths.frc5712.com/docs/, from the same Vercel deployment as the editor. The root `vercel.json` builds
the editor, builds this site with the base path `/docs`, and `tools/merge-docs.mjs` copies `docs/dist` into
`editor/app/dist/docs`. The editor's catch-all rewrite leaves `/docs` alone, and its service worker never caches
anything under it. To try the whole thing on one machine:

```
npm --prefix editor/app run build && npm --prefix docs run build && node tools/merge-docs.mjs
npm --prefix editor/app run preview      # http://localhost:5713 and http://localhost:5713/docs/
```

Pages link to each other without the base (`/guides/events/`); `astro.config.mjs` adds it. To serve the site
somewhere else, set `DOCS_BASE` (for example `/`) and `DOCS_SITE` when building.

## Where things go

| Folder (in `src/content/docs/`) | What | Reader |
| --- | --- | --- |
| `start/` | Install it, the files, the first path | A student on day one |
| `guides/` | One robot-code task per page | A student writing robot code |
| `concepts/` | How each piece works, in order: plan, follow, on the fly | A student who wants to know why |
| `files/` | Every key of every file the library reads | Anyone editing a file by hand |
| `reference/<package>/` | One page per public class | Anyone looking up a method |

The sidebar is generated from these folders. A page's place in its group is `sidebar.order` in its frontmatter.

The look (colours, type, spacing) is `src/styles/theme.css`. It borrows the workshop site's tokens
(frc5712.com): dark navy, warm paper in light mode, one orange accent, Newsreader for reading, Instrument Sans for
the interface, JetBrains Mono for code and labels.

## Writing rules

The reader is 11 to 18 years old and is probably standing next to a robot. These are the workshop site's rules,
and every page follows them.

**Truth**

- Never invent an API. Every class, method, parameter, default and number comes from the source in `lib/src`
  (or `lib/README.md`, which the code backs). If the source and the README disagree, the source wins.
- Say what has not been checked. The library has not run on a real robot; numbers come from the simulator and the
  bench. Do not write that something "works on the robot".
- No gains to copy. Show the library's defaults (they are in the source); never present a tuned value as the
  answer for a team's robot.

**Voice**

- Flat and procedural, like the CTRE Phoenix 6 docs. Second person. No opinions, no jokes, no hype.
- Plain words first. Name the technical term once, in bold, the first time, with what it means.
- No em dashes, and no parentheses standing in for them. End the sentence or use a comma.
- No emoji. No exclamation marks.
- Vary sentence length. A page of uniformly short sentences reads as a machine.
- American spelling: meters, center, color, behavior. The code says `positionToleranceMeters`.
- Units always: meters, seconds, radians or degrees, m/s, m/s². Say which.

**Shape**

- Frontmatter: `title` (a noun phrase, five words at most; a reference page's title is the class name),
  `description` (one sentence, 18 words at most), `sidebar: { order: N }`.
- Do not repeat the title as an `#` heading. Start with one or two paragraphs that say what the thing is and when
  a student needs it. Sections are `##`, sub-sections `###`.
- Code blocks are fenced with a language (`java`, `json`, `text`, `bash`). Java examples are Commands v3 on
  WPILib 2027: `Mechanism`, `Command`, `ChassisVelocities`, `Pose2d`. Never Commands v2, never `RobotContainer`.
  Lambdas are written `() -> drive.getPose()`, not `drive::getPose`. The drivetrain variable is `drive`, the
  library is `paths`, the on-the-fly factory is `otf`.
- Asides are `:::note`, `:::tip`, `:::caution` and `:::danger`. Two per page, three at most. An aside is for
  something that will bite; an explanation belongs in the paragraph.
- Links to other pages are root-absolute with a trailing slash: `/guides/events/`,
  `/reference/wpilib/pathlibrary/`. Slugs are the lower-case file names.
- Tables for anything with more than three parallel items: settings, keys, methods, telemetry entries.
- Plain `.md` unless a page needs a component (then `.mdx`).

**A reference page** has, in this order: what the class is for (one paragraph), where a student meets it, a short
example, then every public constructor, method, field and constant in tables or short sections, with units,
defaults and what it throws. Package-private members are left out.

**A concept page** answers: what problem this piece solves, what it does step by step, what its numbers are and
where they come from, and what it does not do. It links to the reference pages of the classes it describes.
