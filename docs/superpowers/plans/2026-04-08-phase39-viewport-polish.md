# Phase 39: 3D Viewport & Navigation Polish Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Professional-grade 3D interaction.

**Tech Stack:** C++20, Qt6, OpenGL 3.3

**Spec Reference:** `docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md` — Section 4.9

---

## Spec Compliance Check

| # | Spec Requirement | Plan Task | Status |
|---|---|---|---|
| 1 | Middle-mouse orbit (SolidWorks-style) | Task 1 | ✅ |
| 2 | Scroll zoom-to-cursor | Task 1 | ✅ |
| 3 | Shift+middle pan | Task 1 | ✅ |
| 4 | Edge rendering: silhouette (black), sharp (dark gray), sketch (blue) | Task 2 | ✅ |
| 5 | Transparency mode | Task 2 | ✅ |
| 6 | Section plane tool | Task 3 | ✅ |
| 7 | GPU color-picking for 3D selection | Task 3 | ✅ |
| 8 | PBR-lite: metallic-roughness, matte gray default | Task 2 | ✅ |

---

## Task 1: SolidWorks-Style Navigation
- Middle-mouse drag = orbit (already partially implemented via right-mouse)
- Shift+middle = pan
- Scroll wheel = zoom-to-cursor (zoom centered on cursor position, not viewport center)
- Modify `ViewportInputHandler` to implement these mappings

## Task 2: Visual Quality
- Edge rendering with 3 categories (silhouette/sharp/sketch) using different colors and line widths
- Transparency mode: alpha blending for see-through solid editing
- PBR-lite: upgrade Phong shader to metallic-roughness model (ambient occlusion, roughness parameter)

## Task 3: Selection + Section Plane
- GPU color-picking: render entity IDs to offscreen FBO, read pixel under cursor for selection
- Section plane tool: clip the solid at a user-defined plane for interior visualization

## Task 4: Final commit + push

---

Full implementation details in each task follow the same pattern as previous phases.
File modifications: ViewportInputHandler.cpp, ViewportRenderer.cpp, GLRenderer.cpp/.h, ShaderProgram (new shaders).
