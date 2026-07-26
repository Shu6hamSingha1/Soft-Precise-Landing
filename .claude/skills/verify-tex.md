# /verify-tex

Run a static consistency check on the manuscript LaTeX sources in `Soft_Precise_Landing/`.

## What it checks

- Every `\ref{}`/`\eqref{}`/`\autoref{}`/`\cref{}` resolves to a `\label{}` defined somewhere in the four `.tex` files (`manuscript.tex`, `control_formulation.tex`, `results.tex`, `supplemental.tex`). Labels are pooled across all four files, so supplement-internal refs and supplement-to-main-paper refs (when present) both resolve.
- Every `\cite{}` key resolves to an entry in `bibliography.bib`.
- The brace count `{` vs `}` matches for each of the four `.tex` files (catches dangling environments and unbalanced groups).

This is a fast pre-flight before asking the user to recompile the manuscript — it catches the broken-reference and missing-citation classes of errors that typically show up only at compile time.

## When to use

- After any non-trivial edit to `manuscript.tex`, `control_formulation.tex`, `results.tex`, or `supplemental.tex` (especially renaming labels, removing/adding figures or tables, or rewriting bibliography keys).
- Before declaring a multi-step manuscript edit task complete.
- Whenever the user reports a `?` reference or undefined citation in their compiled PDF.

## How to invoke

```bash
cd "L:/Claude/Soft Landing"
PYTHONIOENCODING=utf-8 python scripts/_verify.py
```

The script lives at `scripts/_verify.py` and resolves the manuscript directory relative to its own path (`../Soft_Precise_Landing`). It is intentionally pure-Python (`re` + `os` only) and does not require LaTeX to be installed.

## Output

```
labels   : <count>
refs     : <count> (missing: <count>)
  ! <missing-label-1>           # listed only if non-zero
bib keys : <count>
cites    : <count> (missing: <count>)
  ! <missing-cite-1>            # listed only if non-zero
manuscript.tex                  { } balance: <open> / <close>  OK|MISMATCH
control_formulation.tex         { } balance: <open> / <close>  OK|MISMATCH
results.tex                     { } balance: <open> / <close>  OK|MISMATCH
supplemental.tex                { } balance: <open> / <close>  OK|MISMATCH
```

A clean run reports zero missing refs, zero missing cites, and `OK` on every brace line. Any other line is a real defect to investigate before handing the file back to the user.

## Notes

- The script does **not** detect logically wrong cross-references (e.g., `\ref{tab:foo}` resolving to a figure label) — it only checks existence.
- `bibliography.bib` is read from `Soft_Precise_Landing/bibliography.bib` by default; if missing, citation checking is skipped silently.
- The supplement uses textual cross-refs to main-paper sections ("Section III-A2 of the main paper") per `feedback_supplement_cross_refs.md` rather than `\ref{}` — so cross-document refs are NOT checked. Only `\ref{}`-form refs in supplemental.tex resolve against the pooled label set.
- The brace check is character-level and will mis-flag a comment containing a stray `{` or `}` — investigate before assuming a real error.

## Manual cross-checks the script does NOT cover

- **Case-number ↔ trajectory-name consistency.** Section IV-A defines Case 1=Static, Case 2=Linear, Case 3=Sinusoidal, Case 4=Lissajous, Case 5=Circular. Every other table header, figure caption, and worst-case bookkeeping line in `results.tex`/`supplemental.tex` MUST follow this mapping. After Table IV edits in particular, eyeball that Col 4 says "Lissajous" and Col 5 says "Circular".
- **`.mat` data freshness.** Compare `Common/Constants.m`, `Multi_init_cond/visualControl_IBVS_adaptive.m`, `Comparison/InitGains_Comparison.m` mtimes against the `*_multi_init.mat` and `*_comparison.mat` mtimes. A `.mat` older than the source is potentially stale — check the diff to see if the source change is cosmetic (e.g. variable rename) or behavior-affecting before recommending a re-run.
- **Termination convention — above-target gap, not absolute altitude.** `visualControl_comparison.m:383,812` fires on `alt_above = abs(I_p_c(3) - x_t(3,idx)); if alt_above <= zf`. The manuscript locks `z_f` as the UAV altitude ABOVE THE TARGET (see `results.tex:4,21`). On Cases 2 (Linear) and 5 (Circular) the target z heaves by $\pm 0.2$ m, so Table IV's `z_f` column is NOT the absolute altitude — it is the above-target gap. Any new prose saying "altitude falls below" or "absolute altitude at touchdown" without the above-target qualifier is wrong; any `z_f`-valued cell > 0.20 m is a failure-to-reach-landing reading, measured above the target.
