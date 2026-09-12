# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Repository reality

This repository is not a software codebase yet. It currently contains a single source artifact:

- `分组探究.docx` — the assignment brief for a computer architecture group exploration project

There is no existing build system, package manifest, test suite, linter configuration, README, Cursor rules, or Copilot instructions in the repository at this time.

## Common commands

Because this repo is currently document-centric, there are no project-specific build, lint, or test commands.

### Inspect repository contents
```bash
ls -la
find . -maxdepth 2 -mindepth 1 -printf '%P\n' | sort
```

### Inspect the Word document as an OOXML archive
```bash
python - <<'PY'
import zipfile
p = r'D:/[AUTHOR]/Documents/updating/大二下/ComputerArchitectureGroupDiscussion/分组探究.docx'
with zipfile.ZipFile(p) as z:
    for name in z.namelist():
        print(name)
PY
```

### Extract paragraph text from the assignment brief
```bash
PYTHONIOENCODING=utf-8 python - <<'PY'
import zipfile
import xml.etree.ElementTree as ET
p = r'D:/[AUTHOR]/Documents/updating/大二下/ComputerArchitectureGroupDiscussion/分组探究.docx'
ns = {'w':'http://schemas.openxmlformats.org/wordprocessingml/2006/main'}
with zipfile.ZipFile(p) as z:
    xml = z.read('word/document.xml')
root = ET.fromstring(xml)
for para in root.findall('.//w:body/w:p', ns):
    text = ''.join((t.text or '') for t in para.findall('.//w:t', ns)).strip()
    if text:
        print(text)
PY
```

## High-level structure

The current project is centered on the contents of `分组探究.docx`, not on application code. The document is an assignment specification titled `分组探究说明` and is organized into three functional sections:

1. **Task requirements**
   - Team-based work
   - Maximum group size of 5
   - Choose one topic and investigate it through one or more methods such as literature review, surveys, or experiments
   - Produce a written report
   - Stronger submissions also prepare a presentation deck and either a recorded video or live presentation

2. **Allowed topic areas**
   The brief offers chapter-aligned topic directions rather than a fixed implementation target:
   - High-performance computing (HPC)
   - The development of storage
   - The development of instruction set systems
   - Chinese chip/processor topics
   - Industry standards in computing
   - Novel and diverse input/output devices

3. **Evaluation rubric**
   The deliverable is graded in three buckets:
   - Exploration content (40)
   - Exploration method (30)
   - Final report / presentation quality (30)

## Working assumptions for future Codex instances

- Treat this repository as an **assignment workspace**, not an existing software system.
- If asked to “implement” or “analyze the codebase,” first verify whether new source files have been added after this `AGENTS.md` was written.
- If the user starts adding code, data, slides, or report drafts, update this file to reflect the new project structure and actual development commands.
- For now, the most relevant artifacts to maintain are likely to be:
  - the assignment brief (`分组探究.docx`)
  - future report drafts
  - future presentation materials
  - any research notes or experimental scripts added later

## What is currently missing

The following do **not** exist yet and should not be assumed:

- build pipeline
- test runner
- lint configuration
- package manager metadata
- application entrypoint
- source directory conventions

If any of those appear later, re-analyze the repository and extend this file with the real commands and architecture.