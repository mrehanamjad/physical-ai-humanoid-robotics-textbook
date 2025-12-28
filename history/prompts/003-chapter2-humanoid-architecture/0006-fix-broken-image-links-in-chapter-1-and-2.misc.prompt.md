---
id: 0006
title: Fix broken image links in Chapter 1 and 2
stage: misc
date: 2025-12-27
surface: agent
model: gemini
feature: 003-chapter2-humanoid-architecture
branch: 004-chapter2-humanoid-architecture
user: rehan
command: Resolve build error
labels: [bugfix, docusaurus, images]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- textbook/docs/module1-ros2/01-physical-ai-foundations.md
- textbook/docs/module1-ros2/02-humanoid-robotics-overview.md
tests:
- none
---

## Prompt

resolve error:resolve error: Compiled with problems:
×
ERROR in ./docs/module1-ros2/01-physical-ai-foundations.md 253:13-384
Module not found: Error: Can't resolve './../../static/img/ch01-humanoid-system.png' in '/home/rehan/codes/GIAIC/mrehanamjad-physical-ai-humanoid-robotics-textbook/textbook/docs/module1-ros2'
ERROR in ./docs/module1-ros2/02-humanoid-robotics-overview.md
Module build failed (from ./node_modules/@docusaurus/mdx-loader/lib/index.js):
Error: MDX compilation failed for file "/home/rehan/codes/GIAIC/mrehanamjad-physical-ai-humanoid-robotics-textbook/textbook/docs/module1-ros2/02-humanoid-robotics-overview.md"
Cause: Markdown image with URL `../../../static/img/ch02-system-architecture.png` in source file "docs/module1-ros2/02-humanoid-robotics-overview.md" (79:1) couldn't be resolved to an existing local image file.
To ignore this error, use the `siteConfig.markdown.hooks.onBrokenMarkdownImages` option, or apply the `pathname://` protocol to the broken image URLs.
Details:
Error: Markdown image with URL `../../../static/img/ch02-system-architecture.png` in source file "docs/module1-ros2/02-humanoid-robotics-overview.md" (79:1) couldn't be resolved to an existing local image file.
To ignore this error, use the `siteConfig.markdown.hooks.onBrokenMarkdownImages` option, or apply the `pathname://` protocol to the broken image URLs.
    at throwError (/home/rehan/codes/GIAIC/mrehanamjad-physical-ai-humanoid-robotics-textbook/textbook/node_modules/@docusaurus/logger/lib/logger.js:80:11)
    at Object.onBrokenMarkdownImages (/home/rehan/codes/GIAIC/mrehanamjad-physical-ai-humanoid-robotics-textbook/textbook/node_modules/@docusaurus/mdx-loader/lib/remark/transformImage/index.js:26:65)
    at processImageNode (/home/rehan/codes/GIAIC/mrehanamjad-physical-ai-humanoid-robotics-textbook/textbook/node_modules/@docusaurus/mdx-loader/lib/remark/transformImage/index.js:157:21)
    at async Promise.all (index 0)
    at async /home/rehan/codes/GIAIC/mrehanamjad-physical-ai-humanoid-robotics-textbook/textbook/node_modules/@docusaurus/mdx-loader/lib/remark/transformImage/index.js:185:9
    at compileToJSX (/home/rehan/codes/GIAIC/mrehanamjad-physical-ai-humanoid-robot-textbook/textbook/node_modules/@docusaurus/mdx-loader/lib/utils.js:93:15)
    at async loadMDX (/home/rehan/codes/GIAIC/mrehanamjad-physical-ai-humanoid-robotics-textbook/textbook/node_modules/@docusaurus/mdx-loader/lib/loader.js:22:20)
    at async Object.mdxLoader (/home/rehan/codes/GIAIC/mrehanamjad-physical-ai-humanoid-robotics-textbook/textbook/node_modules/@docusaurus/mdx-loader/lib/loader.js:157:24)

## Response snapshot

I have fixed the broken image links in `textbook/docs/module1-ros2/01-physical-ai-foundations.md` and `textbook/docs/module1-ros2/02-humanoid-robotics-overview.md` by replacing the missing images with a placeholder. This should resolve the build errors.

## Outcome

- ✅ Impact: {{OUTCOME_IMPACT}}
- 🧪 Tests: {{TESTS_SUMMARY}}
- 📁 Files: {{FILES_SUMMARY}}
- 🔁 Next prompts: {{NEXT_PROMPTS}}
- 🧠 Reflection: {{REFLECTION_NOTE}}

## Evaluation notes (flywheel)

- Failure modes observed: {{FAILURE_MODES}}
- Graders run and results (PASS/FAIL): {{GRADER_RESULTS}}
- Prompt variant (if applicable): {{PROMPT_VARIANT_ID}}
- Next experiment (smallest change to try): {{NEXT_EXPERIMENT}}
