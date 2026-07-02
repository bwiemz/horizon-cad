# Phase 41: Multi-Document Architecture & Part Files Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Support multiple document types for the parts → assemblies → drawings workflow: `.hzpart` part documents, `.hzasm` assembly documents, a `DocumentManager` that owns open documents and cross-document references, tessellation caching for lightweight assembly loading, and multi-tab UI integration.

**Architecture:** `hz::doc` gains a `DocumentType` tag on `Document`, a new `AssemblyDocument` (component instances = part path + transform + cached mesh), and a `DocumentManager` that deduplicates open documents by canonical path, polls file mtimes for external change notifications, and resolves component references in **lightweight** (cached tessellation only) or **resolved** (full feature tree) mode. Because the dependency direction is `fileio → document`, `DocumentManager` receives loader functions by injection instead of calling `hz::io` directly. `hz::io::NativeFormat` gains `.hzpart` (full part serialization + optional tessellation cache) and `.hzasm` (component references + transforms) support, and the existing feature-tree serialization defects are fixed (real sketch IDs, extrude direction, revolve axis). The UI replaces its single implicit document with DocumentManager-backed tabs over one shared viewport.

**Tech Stack:** C++20, Qt6, Google Test

**Spec Reference:** docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md — Section 5.1

---

## Spec Compliance Check

| # | Spec Requirement | Plan Task | Status |
|---|-----------------|-----------|--------|
| 1 | Document types: `.hzpart` (single solid + feature tree) | Task 1, Task 3 | ✅ |
| 2 | Document types: `.hzasm` (hierarchical part references + mates) — mates deferred to Phase 42 by roadmap order | Task 2, Task 3 | ✅ |
| 3 | Document types: `.hzdwg` deferred to Era 3 | — (explicitly deferred) | ✅ |
| 4 | `DocumentManager`: manages open documents | Task 4 | ✅ |
| 5 | `DocumentManager`: tracks cross-document references | Task 4 | ✅ |
| 6 | `DocumentManager`: handles external file-change notifications | Task 4 | ✅ |
| 7 | In-memory caching: referenced parts cache tessellated mesh | Task 3 (tessellation cache), Task 4 (lightweight resolve) | ✅ |
| 8 | Lightweight vs. resolved mode | Task 2 (`ComponentInstance` state), Task 4 (resolve modes) | ✅ |
| 9 | Feature tree only loaded on edit | Task 4 (`resolveComponent(..., Resolved)` on demand) | ✅ |

### Defects fixed along the way (pre-existing, discovered during Phase 41 reconnaissance)

| Defect | Fix Task |
|--------|----------|
| Feature-tree serialization writes the feature's own index as `sketchIndex`, drops extrude direction and revolve axis/angle | Task 3 |
| `MainWindow::onOpenFile` copies only layers/entities/blocks/constraints — silently drops sketches, feature tree, design variables | Task 5 |
| `MainWindow::m_featureTree` duplicates `Document::featureTree()` and the two never sync; extrude/revolve UI bypasses both | Task 5 |
| `Sketch` IDs from a process-global counter collide across multiple loaded documents | Task 1 |

---

## File Map

| Action | File | Responsibility |
|--------|------|----------------|
| Modify | src/document/include/horizon/document/Document.h | `DocumentType` enum, `type()`, built-solid cache (`rebuildModel()`, `solid()`) |
| Modify | src/document/src/Document.cpp | Implementation of the above |
| Modify | src/document/include/horizon/document/Sketch.h | `claimId()` static to bump the ID counter past loaded IDs |
| Modify | src/document/src/Sketch.cpp | Counter bump on `setId` |
| Create | src/document/include/horizon/document/AssemblyDocument.h | `ComponentInstance`, `AssemblyDocument` |
| Create | src/document/src/AssemblyDocument.cpp | Implementation |
| Create | src/document/include/horizon/document/DocumentManager.h | `DocumentManager` with loader injection, path dedup, mtime polling, resolve modes |
| Create | src/document/src/DocumentManager.cpp | Implementation |
| Modify | src/document/CMakeLists.txt | Add new sources |
| Modify | src/fileio/include/horizon/fileio/NativeFormat.h | `.hzpart`/`.hzasm` entry points, `loadPartMesh` |
| Modify | src/fileio/src/NativeFormat.cpp | v16, document type tag, tessellation cache, feature-tree serialization fixes, assembly serialization |
| Modify | src/ui/include/horizon/ui/MainWindow.h | DocumentManager member, document tabs, remove duplicate feature tree |
| Modify | src/ui/src/MainWindow.cpp | Tabbed documents, full-document open, unified feature tree, part/assembly actions |
| Create | tests/document/test_AssemblyDocument.cpp | Unit tests |
| Create | tests/document/test_DocumentManager.cpp | Unit tests |
| Modify | tests/document/CMakeLists.txt | Add new tests |
| Create | tests/fileio/test_PartFormat.cpp | `.hzpart` round-trip, tessellation cache, lightweight mesh load |
| Create | tests/fileio/test_AssemblyFormat.cpp | `.hzasm` round-trip, relative path handling |
| Create | tests/fileio/CMakeLists.txt | New test target `hz_fileio_tests` |
| Modify | tests/CMakeLists.txt | Add `fileio` subdirectory |
| Create | tests/integration/test_MultiDocument.cpp | Part → assembly → lightweight/resolved end-to-end |
| Modify | tests/integration/CMakeLists.txt | Add new test file |

---

## Task 1: Document type tag and built-solid cache

**Files:**
- Modify: src/document/include/horizon/document/Document.h
- Modify: src/document/src/Document.cpp
- Modify: src/document/include/horizon/document/Sketch.h
- Modify: src/document/src/Sketch.cpp

- [x] **Step 1: Write tests** — extend tests/document (new assertions inside test_DocumentManager.cpp cover type; solid-cache tests live in test_FeatureTree.cpp additions).
- [x] **Step 2: Implement** — `enum class DocumentType { Drawing, Part, Assembly };` `Document::type()/setType()`. `Document::rebuildModel()` runs `featureTree().buildWithDiagnostics()`, stores the resulting solid + diagnostics on the document; `solid()` accessor. `Sketch::setId` bumps the global counter past the loaded ID so future sketches never collide.
- [x] **Step 3: Build and run tests.**
- [x] **Step 4: Commit** — `feat(document): document type tag, built-solid cache, sketch ID collision fix`

---

## Task 2: AssemblyDocument and ComponentInstance

**Files:**
- Create: src/document/include/horizon/document/AssemblyDocument.h
- Create: src/document/src/AssemblyDocument.cpp
- Modify: src/document/CMakeLists.txt
- Create: tests/document/test_AssemblyDocument.cpp
- Modify: tests/document/CMakeLists.txt

- [x] **Step 1: Write tests** — add/remove/find components, transform storage, suppression, lightweight/resolved state transitions, dirty tracking.
- [x] **Step 2: Implement** — `ComponentInstance { id, name, partPath, transform (Mat4), suppressed, cachedMesh (shared_ptr<geo::MeshData>), resolvedPart (shared_ptr<Document>), state }`. `AssemblyDocument { components, addComponent, removeComponent, component(id), filePath, dirty }`.
- [x] **Step 3: Build and run tests.**
- [x] **Step 4: Commit** — `feat(document): AssemblyDocument with component instances`

---

## Task 3: .hzpart / .hzasm serialization + feature-tree serialization fixes

**Files:**
- Modify: src/fileio/include/horizon/fileio/NativeFormat.h
- Modify: src/fileio/src/NativeFormat.cpp
- Create: tests/fileio/test_PartFormat.cpp
- Create: tests/fileio/test_AssemblyFormat.cpp
- Create: tests/fileio/CMakeLists.txt
- Modify: tests/CMakeLists.txt

- [x] **Step 1: Write tests** — part round-trip preserving feature tree (real sketch reference, direction, distance, axis, angle), tessellation-cache round-trip, `loadPartMesh` without constructing a Document, assembly round-trip (components, transforms, suppressed), version compat (v15 file still loads).
- [x] **Step 2: Implement** — bump to v16. `type` field: `"hcad" | "hzpart" | "hzasm"` from `Document::type()`. Feature tree entries serialize `type`, `sketchId` (real), `direction`, `distance` / `axisPoint`, `axisDir`, `angle`; loader resolves sketch by ID with legacy `sketchIndex` fallback. Optional `tessellationCache` section (positions/normals/indices) written when the document has a built solid. `saveAssembly/loadAssembly` (components with relative-to-assembly paths). `loadPartMesh(path)` reads only the cache section.
- [x] **Step 3: Build and run tests.**
- [x] **Step 4: Commit** — `feat(fileio): .hzpart/.hzasm formats, tessellation cache, feature-tree serialization fixes`

---

## Task 4: DocumentManager

**Files:**
- Create: src/document/include/horizon/document/DocumentManager.h
- Create: src/document/src/DocumentManager.cpp
- Modify: src/document/CMakeLists.txt
- Create: tests/document/test_DocumentManager.cpp
- Modify: tests/document/CMakeLists.txt
- Create: tests/integration/test_MultiDocument.cpp
- Modify: tests/integration/CMakeLists.txt

- [x] **Step 1: Write tests** — new/open/close lifecycle, canonical-path deduplication (opening the same part twice returns the same instance), loader injection, external-change detection via mtime polling, `resolveComponent` lightweight (mesh only, no feature tree) vs resolved (full document, cached thereafter), unresolved-path error handling.
- [x] **Step 2: Implement** — loader/mesh-loader function injection; `newPart/newDrawing/newAssembly/openPart/openAssembly/close`; open-document registries keyed by canonical path; `pollExternalChanges()` comparing stored mtimes and invoking a callback; `resolveComponent(instance, mode)` filling `cachedMesh` (Lightweight) or `resolvedPart` (Resolved, feature tree loaded on demand — "loaded on edit").
- [x] **Step 3: Build and run tests.**
- [x] **Step 4: Commit** — `feat(document): DocumentManager with lightweight/resolved component resolution`

---

## Task 5: UI integration — tabbed documents

**Files:**
- Modify: src/ui/include/horizon/ui/MainWindow.h
- Modify: src/ui/src/MainWindow.cpp

- [x] **Step 1: Implement** — `DocumentManager` member; `QTabBar` above the shared viewport; per-tab document entries (part/drawing documents and assembly documents); tab switch saves/restores camera, repoints the viewport, rebuilds the scene from the document's feature tree, refreshes panels. New Part / New Assembly / Insert Component actions. `onOpenFile` loads the complete document through the manager (fixes the lossy copy). `MainWindow::m_featureTree` removed — the panel and extrude/revolve slots now use `activeDocument()->featureTree()` so features append to the parametric history.
- [x] **Step 2: Build, run full suite.**
- [x] **Step 3: Commit** — `feat(ui): tabbed multi-document UI with DocumentManager`

---

## Task 6: Final Phase Commit + Push

- [x] **Step 1: Run complete test suite.** Report exact test count.
- [x] **Step 2: Commit and push**

```bash
git add -A
git commit -m "Phase 41: Multi-document architecture with part and assembly files

- DocumentType tag + built-solid cache on Document
- AssemblyDocument with component instances (path + transform + cached mesh)
- DocumentManager: open-document registry, loader injection, mtime change polling,
  lightweight/resolved component resolution
- .hzpart/.hzasm JSON formats (v16) with tessellation cache
- Feature-tree serialization fixes (real sketch IDs, direction, axis)
- Tabbed multi-document UI; feature tree unified into Document"
git push -u origin claude/production-readiness-roadmap-29aacn
```
