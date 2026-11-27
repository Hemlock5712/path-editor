import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';
import { unified } from '@astrojs/markdown-remark';

// The docs are served from the editor's site, under /docs (see ../vercel.json).
const site = process.env.DOCS_SITE ?? 'https://paths.frc5712.com';
const base = process.env.DOCS_BASE ?? '/docs';

// Pages link to each other with root-absolute paths ("/guides/events/"). This puts the base in front of
// them, in Markdown links and in components' href attributes, so the pages never have to know it.
function withBase() {
  const prefix = base.replace(/\/$/, '');
  const fix = (url) => (typeof url === 'string' && url.startsWith('/') && !url.startsWith('//') ? prefix + url : url);
  const visit = (node) => {
    if (node.type === 'link') node.url = fix(node.url);
    if (node.type === 'mdxJsxFlowElement' || node.type === 'mdxJsxTextElement') {
      for (const attr of node.attributes ?? []) {
        if (attr.type === 'mdxJsxAttribute' && attr.name === 'href') attr.value = fix(attr.value);
      }
    }
    for (const child of node.children ?? []) visit(child);
  };
  return (tree) => visit(tree);
}

const group = (label, directory, collapsed = false) => ({
  label,
  collapsed,
  items: [{ autogenerate: { directory } }],
});

export default defineConfig({
  site,
  base,
  trailingSlash: 'always',
  markdown: { processor: unified({ remarkPlugins: [withBase] }) },
  integrations: [
    starlight({
      title: 'Myelin',
      description: "Team 5712's path library for WPILib 2027: draw a path, follow it, and plan one on the field.",
      logo: { src: './src/assets/mark.svg', alt: '' },
      favicon: '/favicon.svg',
      social: [
        { icon: 'github', label: 'Source on GitHub', href: 'https://github.com/Hemlock5712/path-editor' },
        { icon: 'external', label: 'Open the path editor', href: 'https://paths.frc5712.com/' },
      ],
      customCss: [
        '@fontsource-variable/newsreader/opsz.css',
        '@fontsource-variable/newsreader/opsz-italic.css',
        '@fontsource-variable/instrument-sans/wdth.css',
        '@fontsource-variable/jetbrains-mono/wght.css',
        './src/styles/theme.css',
      ],
      components: {
        Footer: './src/components/Footer.astro',
        Hero: './src/components/Hero.astro',
      },
      lastUpdated: false,
      pagination: true,
      tableOfContents: { minHeadingLevel: 2, maxHeadingLevel: 3 },
      expressiveCode: {
        // Code stays dark in both themes, as on the workshop site: a code block is a terminal.
        themes: ['github-dark-default'],
        useStarlightDarkModeSwitch: false,
        useStarlightUiThemeColors: false,
        styleOverrides: {
          borderRadius: '3px',
          borderColor: 'var(--rule)',
          codeBackground: 'var(--code-bg)',
          codeFontFamily: 'var(--sl-font-mono)',
          codeFontSize: '0.84rem',
          codeLineHeight: '1.7',
          uiFontFamily: 'var(--sl-font-mono)',
          uiFontSize: '0.7rem',
          frames: {
            shadowColor: 'transparent',
            editorBackground: 'var(--code-bg)',
            terminalBackground: 'var(--code-bg)',
            editorTabBarBackground: 'var(--code-bg)',
            editorTabBarBorderBottomColor: 'var(--code-rule)',
            editorActiveTabBackground: 'var(--code-bg)',
            editorActiveTabForeground: 'var(--code-tx2)',
            editorActiveTabBorderColor: 'transparent',
            editorActiveTabIndicatorTopColor: 'var(--code-accent)',
            editorActiveTabIndicatorBottomColor: 'transparent',
            terminalTitlebarBackground: 'var(--code-bg)',
            terminalTitlebarForeground: 'var(--code-tx2)',
            terminalTitlebarBorderBottomColor: 'var(--code-rule)',
            terminalTitlebarDotsOpacity: '0',
            frameBoxShadowCssValue: 'none',
          },
        },
      },
      // Grouped by what the reader is doing, not by kind of page: a student writing an autonomous never has
      // to scroll past the teleop pages, and the other way round. The pages' addresses do not follow the
      // groups (guides/ and concepts/ stay as they are), so regrouping never breaks a link.
      sidebar: [
        group('Start here', 'start'),
        {
          label: 'Autonomous paths',
          items: ['guides/follow-a-path', 'guides/events', 'guides/chaining-paths', 'guides/alliances'],
        },
        {
          label: 'Driving to a spot',
          items: ['guides/drive-to-pose', 'guides/pathfinding', 'guides/moving-goals', 'guides/tag-alignment'],
        },
        {
          label: 'Teleop',
          items: ['guides/teleop-limiter', 'guides/rails', 'guides/intake-assist'],
        },
        {
          label: 'On the robot',
          items: ['guides/telemetry', 'guides/tuning', 'guides/wheel-forces', 'guides/troubleshooting'],
        },
        {
          label: 'How it works',
          items: [
            'concepts/overview',
            {
              label: 'Planning a path',
              collapsed: true,
              items: ['concepts/the-curve', 'concepts/heading', 'concepts/speed-plan', 'concepts/traction', 'concepts/field-frame'],
            },
            {
              label: 'Following it',
              collapsed: true,
              items: [
                'concepts/the-follower',
                'concepts/stopping-law',
                'concepts/output-limiter',
                'concepts/feed-forward',
                'concepts/path-events',
              ],
            },
            {
              label: 'Planning on the field',
              collapsed: true,
              items: ['concepts/pathfinder', 'concepts/on-the-fly', 'concepts/footprints', 'concepts/moving-goals', 'concepts/tag-filter'],
            },
            {
              label: 'Driver assists',
              collapsed: true,
              items: ['concepts/rail-law', 'concepts/piece-assist'],
            },
          ],
        },
        group('Files', 'files'),
        {
          label: 'Reference',
          items: [
            group('wpilib: robot code', 'reference/wpilib', true),
            group('follow: the follower', 'reference/follow', true),
            group('plan: on the fly', 'reference/plan', true),
            group('core: the path maths', 'reference/core', true),
            group('sim: a test drivetrain', 'reference/sim', true),
          ],
        },
        { label: 'Glossary', link: '/glossary/' },
      ],
    }),
  ],
});
