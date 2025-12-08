---
id: 0001
title: User initial greeting
stage: general
date: 2025-12-08
surface: agent
model: gemini-1.5-pro
feature: none
branch: master
user: user
command: hi
labels: ["greeting"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- history/prompts/general/0001-user-initial-greeting.general.prompt.md
tests:
---

## Prompt

hi

## Response snapshot

Okay, I'm ready for your first command.

## Outcome

- ✅ Impact: User greeted successfully, context established.
- 🧪 Tests: None required.
- 📁 Files: One PHR file created.
- 🔁 Next prompts: Awaiting user's first command.
- 🧠 Reflection: Initial interaction, setting up PHR.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
