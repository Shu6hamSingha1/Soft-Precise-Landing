---
name: excalidraw-diagram
description: Author and embed Excalidraw diagrams in the IEEE TAES manuscript. Documents the round-trip workflow from .excalidraw source files to vector-format figures (SVG/PDF) used by \input or \includegraphics in tex, plus the storage conventions and the criteria for choosing Excalidraw vs the existing TikZ figures.
---

# /excalidraw-diagram

Author conceptual or schematic diagrams in [Excalidraw](https://excalidraw.com), export them as vector graphics, and embed them in the manuscript alongside the existing TikZ figures.

## When to use Excalidraw vs TikZ

The project currently has two TikZ figures: `block_diagram.tex` (control-pipeline cascade) and `frames_planes.tex` (frames + image planes schematic). Both are precise, code-driven, and version-controlled by `git diff`.

Pick **TikZ** when:
- The diagram needs precise coordinates, mathematical alignment, or symbolic equation labels (e.g., signal-flow block diagrams).
- The diagram changes will be small and local — easy to express as a code edit.
- The journal expects vector graphics with exact font matching to the body text.

Pick **Excalidraw** when:
- The diagram is conceptual / illustrative and benefits from a hand-drawn aesthetic (e.g., "high-level intuition" cartoons, intro-section conceptual sketches, supplementary illustrations).
- Iteration is faster in a visual editor than in TikZ syntax.
- The figure is exploratory and may be discarded.

For IEEE TAES submissions, prefer TikZ for body figures unless there is a specific aesthetic reason to use Excalidraw. Excalidraw figures sit naturally in supplements, talks, or rebuttal letters.

## Storage convention

```
Soft_Precise_Landing/
  Figures/
    excalidraw/                       # source .excalidraw JSON files (committed)
      <descriptive-name>.excalidraw
    generated/
      <descriptive-name>.svg          # exported vector graphic (committed)
      <descriptive-name>.pdf          # optional; some workflows convert SVG → PDF
```

- The `.excalidraw` JSON file is the source of truth and goes in `Figures/excalidraw/`.
- Exports go in `Figures/generated/` next to the existing `plasmc_*.pdf` files.
- Both source and export are committed to git, mirroring how `make_*.py` scripts and their PDFs are paired.

## Workflow

### Authoring

1. Open [excalidraw.com](https://excalidraw.com) (or the desktop app, or VS Code Excalidraw extension).
2. Optionally start from an existing `.excalidraw` file: drag-drop or "Open" → `Soft_Precise_Landing/Figures/excalidraw/<name>.excalidraw`.
3. Sketch the diagram. Use the paper's notation conventions (memory: `feedback_image_plane_axis_vs_component`, `feedback_funnel_symbol_convention`, `feedback_no_channel_word`, `feedback_ic_notation`).
4. **Save** the source: File → "Save to..." → `Soft_Precise_Landing/Figures/excalidraw/<name>.excalidraw`.

### Exporting to SVG (preferred for tex)

1. In Excalidraw: File → Export image → SVG.
2. Recommended settings:
   - **Background**: transparent (off).
   - **Embed scene**: ON (so the SVG round-trips back into Excalidraw later).
   - **Dark mode**: OFF.
   - **Scale**: 2× or higher (avoids pixelation when included via `\includegraphics`).
3. Save to `Soft_Precise_Landing/Figures/generated/<name>.svg`.

### Embedding in tex

Use `\includegraphics` inside a `figure` (or `figure*`) environment, just like the existing `plasmc_*.pdf` figures:

```latex
\begin{figure}[!t]
\centering
    \includegraphics[width=\columnwidth]{Figures/generated/<name>.svg}
    \caption{Concise caption following project conventions (no leaks of MATLAB filenames, no banned words like "channel" or "realistic", IC indices in `IC$_N$` form).}
    \label{fig: <descriptive-label>}
\end{figure}
```

Notes:
- IEEE TAES `IEEEtaes.cls` accepts SVG via `\includegraphics` only if the LaTeX engine has SVG support (XeLaTeX or LuaLaTeX with `svg` package, or pdfLaTeX with a pre-converted PDF). If pdfLaTeX is the engine, also export a PDF (Excalidraw → File → Export image → PDF) and reference the `.pdf`.
- SVG is preferred when the engine supports it, since it preserves vector quality at any zoom.

### Updating an Excalidraw figure

1. Open the `.excalidraw` source from `Figures/excalidraw/`.
2. Edit visually.
3. Re-export to `Figures/generated/<name>.svg` (overwrite).
4. Commit both the updated `.excalidraw` and the new `.svg` together so the git history pairs source + render.

## Conventions to respect (paper-wide)

When authoring diagram labels in Excalidraw, follow the locked manuscript conventions:

- **Frames**: $\mathcal{I}$ inertial, $\mathcal{B}$ body, $\mathcal{C}$ camera, $\mathcal{V}$ virtual.
- **Image-plane axes**: capital with camera-frame subscript — $\hat{X}_\text{c}, \hat{Y}_\text{c}$.
- **Feature-point components**: lowercase with camera-frame superscript — $\,^\mathcal{C}\hat{x}, \,^\mathcal{C}\hat{y}$. NEVER $(u, v)$.
- **Funnels**: $\boldsymbol{p}_1(t)$ = target image funnel (pixels); $\boldsymbol{p}_2(t)$ = optic-flow funnel (rad/s).
- **Indexed ICs**: `IC$_N$` form (e.g., IC₂ = `IC$_2$`).
- **Cone cap**: $\theta_\text{cap} = 60°$ (NEVER 35° — that's a stale value).
- **No "channel"** in any label — use "axis", "loop", "control loop", or "component".

Excalidraw supports basic LaTeX math via the "Text" tool with `$...$` delimiters (depending on plugin/extension). For complex equations, render the equation as a separate image (e.g., via TikZ or matplotlib) and import it into the Excalidraw scene.

## Limitations / when to bail

- Excalidraw fonts won't match the manuscript body text exactly. If perfect typography is required, use TikZ instead.
- Excalidraw exports do not auto-update on tex rebuild. Treat each export as a manual step; commit the SVG after every iteration.
- Diff-friendliness: `.excalidraw` JSON does diff in git, but the diff is noisy because of internal IDs. Code review should focus on the rendered SVG, not the JSON.

## Catalog of Excalidraw figures (current)

None yet. Existing schematic figures (`block_diagram.tex`, `frames_planes.tex`) are TikZ and remain TikZ. This skill enables future Excalidraw additions (e.g., a high-level intuition cartoon for the introduction, a control-loop sketch for a slide deck or rebuttal letter).

When a new Excalidraw figure is added, append a one-line entry here:

| File (source) | Export | Used in | Caption summary |
|---|---|---|---|
| `<name>.excalidraw` | `<name>.svg` | (e.g. `manuscript.tex` Fig. X) | (one line) |
