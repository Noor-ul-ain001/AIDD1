import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'Bridging the Digital Brain and Physical Body',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-github-username', // Usually your GitHub org/user name.
  projectName: 'hack-book', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-github-username/hack-book/tree/main/frontend/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-github-username/hack-book/tree/main/frontend/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar', // This will be updated to a more general sidebar later
          position: 'left',
          label: 'Textbook',
        },
        //{to: '/blog', label: 'Blog', position: 'left'},
        {to: '/docs/chatbot', label: 'Chatbot', position: 'left'}, // Add link to chatbot page
        {
          href: 'https://github.com/your-github-username/hack-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Course Modules',
          items: [
            {
              label: 'Module 1: Robotic Nervous System',
              to: '/docs/module1',
            },
            {
              label: 'Module 2: Digital Twin',
              to: '/docs/module2',
            },
            {
              label: 'Module 3: AI-Robot Brain',
              to: '/docs/module3',
            },
            {
              label: 'Module 4: Vision-Language-Action',
              to: '/docs/module4',
            },
          ],
        },
        {
          title: 'Learning Resources',
          items: [
            {
              label: 'Interactive Chatbot',
              to: '/docs/chatbot',
            },
            {
              label: 'Code Examples',
              href: 'https://github.com/your-github-username/hack-book',
            },
            {
              label: 'Practice Exercises',
              to: '/docs/exercises',
            },
            {
              label: 'Glossary',
              to: '/docs/glossary',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Discord',
              href: 'https://discord.gg/robotics-ai',
            },
            {
              label: 'GitHub Discussions',
              href: 'https://github.com/your-github-username/hack-book/discussions',
            },
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/robotics-ai',
            },
            {
              label: 'Twitter/X',
              href: 'https://x.com/RoboticsAI',
            },
          ],
        },
        {
          title: 'About',
          items: [
            {
              label: 'Course Syllabus',
              to: '/docs/syllabus',
            },
            {
              label: 'Instructors',
              to: '/docs/instructors',
            },
            {
              label: 'License',
              to: '/docs/license',
            },
            {
              label: 'Contact',
              to: '/docs/contact',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;