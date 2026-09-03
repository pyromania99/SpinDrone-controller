#!/usr/bin/env python3
"""
interactive_dashboard.py - Plotly-based Interactive Visualizations for Genetic Controller Search

Produces four HTML dashboard pages as requested:
1. Publication-quality Pareto plot
2. Interactive HTML scatter
3. Generation convergence animation
4. Parallel coordinates plot

── Grid-archive CSV format ────────────────────────────────────────────────────
Any grid sweep (eigen ab-sweep, PD grid, MPC grid, ...) can be loaded with full
parameter labels — exactly like the genetic archives — by writing a
"grid_archive.csv" file per (controller, motor_moment) directory in this shape:

    gen, <param_1>, <param_2>, ..., motor_moment, score, effort
    -1,  2.000,     0.000,     ...,  4.00e-07,     0.842, 12.4
    -1,  2.000,     2.000,     ...,  4.00e-07,     0.910, 15.1
    ...

Rules:
  * "gen" is always -1 for grid sweeps (reserves the field so grid and GA rows
    can be concatenated and told apart; GA rows use their real generation
    index, >= 0).
  * <param_*> columns should use the SAME names as the matching genetic
    sweep's gene names where one exists (e.g. "wn_mdyn", "b_mdyn", "Kp_att")
    so grid points and GA points overlay on the same parallel-coordinates
    axis instead of appearing as separate unlabeled dimensions.
  * "motor_moment", "score", "effort" are required. Any other columns
    (rise_time, settling_time, ...) are optional and will just flow through
    to hover data / parallel coordinates if present.

Any controller's grid_archive.csv, wherever it lives, is picked up via
load_grid_archive() the same way genetic_archive.csv is via
load_genetic_archive() — no per-controller special-casing needed in this file.

Usage:
  python examples/controller_tests/visualization/interactive_dashboard.py \
      --motor-moment 4.0e-07
"""

import argparse
import pandas as pd
import numpy as np
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from pathlib import Path

import csv

def load_genetic_archive(path, name):
    if not path.exists(): return pd.DataFrame()
    df = pd.read_csv(path)
    df['controller'] = name
    return df


def load_grid_archive(path, name):
    """Generic loader for any grid sweep's grid_archive.csv (see module
    docstring for the required format). Structurally identical to
    load_genetic_archive — the only difference is grid rows carry gen=-1.
    Works for eigen, PD, MPC, or any future grid sweep, as long as the sweep
    script writes a grid_archive.csv with named parameter columns."""
    if not path.exists(): return pd.DataFrame()
    df = pd.read_csv(path)
    if 'gen' not in df.columns:
        df['gen'] = -1
    df['controller'] = name
    return df


def load_grid_sweep(s_path, e_path, name):
    """Legacy loader for the old unlabeled wn\\b matrix scores.csv/effort.csv
    format (no parameter columns — every value cell becomes one point with
    no a/b/param labels). Kept for sweeps that haven't been migrated to
    grid_archive.csv yet. Prefer load_grid_archive for anything new."""
    if not s_path.exists() or not e_path.exists(): return pd.DataFrame()
    scores, efforts = [], []
    with open(s_path) as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            for v in row[1:]:
                v = v.strip()
                scores.append(np.inf if v == "inf" else float(v))
    with open(e_path) as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            for v in row[1:]:
                v = v.strip()
                efforts.append(np.inf if v == "inf" else float(v))
    df = pd.DataFrame({'score': scores, 'effort': efforts})
    df['controller'] = name
    df['gen'] = -1
    return df


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--motor-moment", type=str, default="4.0e-07", help="Motor moment directory (e.g. 4.0e-07 or 0.0e+00)")
    p.add_argument("--results-dir", type=str, default="examples/controller_tests/results")
    return p.parse_args()

def _pareto_frontier(efforts, scores):
    """Return boolean mask of pareto optimal points (min effort, min score)"""
    mask = np.ones(len(efforts), dtype=bool)
    for i in range(len(efforts)):
        for j in range(len(efforts)):
            if i == j: continue
            if scores[j] <= scores[i] and efforts[j] <= efforts[i] and \
               (scores[j] < scores[i] or efforts[j] < efforts[i]):
                mask[i] = False
                break
    return mask


def _validate_comparability(df, requested_mm):
    """Print loud warnings for conditions that make the cross-controller
    Pareto comparison misleading, rather than letting them pass silently:

      1. A source's stored motor_moment does not match the directory it was
         loaded from (requested_mm) — the runs being compared were simulated
         under different rotor drag, so they are not the same operating point.
      2. A grid source arrived WITHOUT named parameter columns (loaded via the
         legacy scores.csv/effort.csv path) — its points carry no tuning, so
         they cannot appear in parallel coordinates and their "optimal" point
         cannot be reported as an actual gain set.
      3. A source is missing the step-response validation columns — it will be
         absent from the validation Pareto front, so that plot is not a
         like-for-like comparison across all controllers.
    """
    try:
        req = float(requested_mm)
    except (TypeError, ValueError):
        req = None

    label_cols = {  # controller substring -> expected parameter columns
        "Eigen": ["wn_mdyn", "b_mdyn"],
        "PD":    ["Kp_att", "Kd_att"],
    }

    print("── Data-comparability check " + "─" * 40)
    for ctrl in df['controller'].unique():
        d = df[df['controller'] == ctrl]
        issues = []

        # 1. motor_moment consistency
        if req is not None and 'motor_moment' in d.columns:
            stored = pd.to_numeric(d['motor_moment'], errors='coerce').dropna().unique()
            bad = [v for v in stored if abs(v - req) > 1e-12]
            if bad:
                issues.append(
                    f"motor_moment MISMATCH: stored {sorted(set(bad))} but loaded as "
                    f"{req:.2e} — comparing different rotor-drag operating points")

        # 2. missing parameter labels
        expected = next((cols for key, cols in label_cols.items() if key in ctrl), None)
        if expected:
            missing = [c for c in expected if c not in d.columns or not d[c].notna().any()]
            if missing:
                issues.append(
                    f"missing tuning columns {missing} (loaded unlabeled) — "
                    f"points have no gains attached")

        # 3. missing validation columns
        if 'step_score' not in d.columns or not np.isfinite(
                pd.to_numeric(d.get('step_score', pd.Series(dtype=float)),
                              errors='coerce')).any():
            issues.append("no step-response validation data — absent from validation Pareto front")

        if issues:
            print(f"  ⚠ {ctrl}:")
            for it in issues:
                print(f"      - {it}")
        else:
            print(f"  ✓ {ctrl}: labelled, mm-consistent, validated")
    print("─" * 67)


def _controller_basin(dfc):
    """Rows forming the 'stable basin' for a controller: prefer the explicit
    'stable' flag; fall back to the better-scoring half when it's absent."""
    if 'stable' in dfc.columns and dfc['stable'].notna().any():
        b = dfc[dfc['stable'] == 1]
        if len(b) >= 5:
            return b
    thr = dfc['score'].quantile(0.5)
    return dfc[dfc['score'] <= thr]


def _gene_space_figure(dfc, gene_cols, ctrl):
    """Interactive gene-space view for one controller.

    Each consecutive pair of swept genes becomes a scatter panel colored by
    log10(score). This is the projection the effort-vs-score scatter can't
    show: it puts the actual tuning on the axes so you can see the stability
    basin in parameter space.

    For controllers with >2 genes (e.g. Eigen's wn_mdyn/b_mdyn/Kp_att/Kd_att)
    each pair is shown TWICE so the two gene groups can be *decoupled*:
      * all points   — the raw projection, still confounded by the other genes
                       (two points at the same wn,b can differ only in Kp,Kd);
      * decoupled    — the OTHER genes are held inside the stable-basin
                       interquartile range and the axes are clipped to the
                       basin extent, isolating this pair's marginal effect.
    Returns a plotly Figure, or None if the controller has <2 usable genes.
    """
    # Only genes that actually vary in this controller's data. A controlled
    # slice (e.g. the Kp/Kd sweep at fixed wn,b) writes the pinned genes as
    # constant columns; dropping them here leaves just the swept plane.
    genes = [g for g in gene_cols
             if g in dfc.columns and dfc[g].notna().any() and dfc[g].nunique() > 1]
    pairs = [(genes[i], genes[i + 1]) for i in range(0, len(genes) - 1, 2)]
    if not pairs:
        return None

    basin = _controller_basin(dfc)
    iqr = {g: (basin[g].quantile(0.25), basin[g].quantile(0.75)) for g in genes}
    ext = {g: (basin[g].min(), basin[g].max()) for g in genes}

    decouple = len(genes) > 2            # only meaningful with other genes to hold
    ncols = len(pairs) * (2 if decouple else 1)
    cmin, cmax = float(np.log10(0.03)), float(np.log10(40.0))

    titles = []
    for gx, gy in pairs:
        titles.append(f"{gx} × {gy} — all points")
        if decouple:
            titles.append(f"{gx} × {gy} — others held in basin IQR")
    fig = make_subplots(rows=1, cols=ncols, subplot_titles=titles,
                        horizontal_spacing=0.06)

    def _scatter(d, showbar):
        lc = np.log10(np.clip(d['score'].values, 1e-2, None))
        if 'stable' in d.columns and d['stable'].notna().any():
            sym = np.where(d['stable'].values == 1, 'circle', 'x')
        else:
            sym = 'circle'
        return go.Scatter(
            x=d[gx], y=d[gy], mode='markers',
            marker=dict(color=lc, colorscale='Viridis', cmin=cmin, cmax=cmax,
                        size=7, symbol=sym, line=dict(width=0.4, color='#0c0e12'),
                        showscale=showbar,
                        colorbar=dict(title='log₁₀ score<br>(lower=better)', x=1.02)
                        if showbar else None),
            text=[f"score={s:.3f} effort={e:.1f}"
                  for s, e in zip(d['score'], d['effort'])],
            hovertemplate=f"{gx}=%{{x:.2f}}<br>{gy}=%{{y:.2f}}<br>%{{text}}<extra></extra>",
            showlegend=False)

    col = 1
    for gx, gy in pairs:
        others = [g for g in genes if g not in (gx, gy)]

        fig.add_trace(_scatter(dfc, showbar=(col == ncols)), row=1, col=col)
        fig.update_xaxes(title_text=gx, row=1, col=col)
        fig.update_yaxes(title_text=gy, row=1, col=col)
        col += 1

        if decouple:
            mask = np.ones(len(dfc), bool)
            for g in others:
                mask &= (dfc[g] >= iqr[g][0]) & (dfc[g] <= iqr[g][1])
            fig.add_trace(_scatter(dfc[mask], showbar=(col == ncols)), row=1, col=col)
            fig.update_xaxes(title_text=gx, range=list(ext[gx]), row=1, col=col)
            fig.update_yaxes(title_text=gy, range=list(ext[gy]), row=1, col=col)
            col += 1

    fig.update_layout(
        title=f"Gene-space response — {ctrl}  "
              f"(color = log₁₀ stability score; × = crashed/OOB)",
        template="plotly_dark", height=520, width=560 * ncols,
        margin=dict(l=60, r=100, t=90, b=60))
    return fig


def main():
    args = parse_args()
    mm_dir = f"mm_{args.motor_moment}"
    results_dir = Path(__file__).resolve().parent.parent / "results"
    if not results_dir.exists():
        results_dir = Path(args.results_dir)

    dfs = []
    dfs.append(load_genetic_archive(
        results_dir / "genetic_eigen_sweep" / mm_dir / "genetic_archive.csv",
        "Eigen (GA)"
    ))
    dfs.append(load_genetic_archive(
        results_dir / "genetic_pd_sweep" / mm_dir / "genetic_archive.csv",
        "Cascaded PD (GA)"
    ))

    # ----------------------------------------------------------------------
    # Normal grid sweeps (single grid_archive.csv)
    # ----------------------------------------------------------------------
    # run_all_sweeps.sh writes the ab-sweep under a --run-tag suffix so a
    # coarse and a refined pass can coexist (ab_sweep_wide, ab_sweep_fine)
    # instead of the second overwriting the first. Try the tagged variants
    # before falling back to the untagged legacy name.
    grid_sweeps = []
    for tag, label in [("_wide", "Eigen (Grid, wide b)"),
                        ("_fine", "Eigen (Grid, fine b)"),
                        ("", "Eigen (Grid)")]:
        cand = results_dir / "eigen_sweep" / f"ab_sweep{tag}" / mm_dir
        if (cand / "grid_archive.csv").exists():
            grid_sweeps.append((cand, label))
    grid_sweeps += [
        (results_dir / "eigen_sweep" / "kpkd_sweep" / mm_dir, "Eigen Kp/Kd @fixed wn,b"),
        (results_dir / "mpc_sweep" / mm_dir,                  "LPV-MPC (Grid)"),
        (results_dir / "pd_sweep" / mm_dir,                   "Cascaded PD (Grid)"),
    ]

    for sweep_dir, name in grid_sweeps:
        archive_path = sweep_dir / "grid_archive.csv"
        if archive_path.exists():
            dfs.append(load_grid_archive(archive_path, name))
        else:
            dfs.append(load_grid_sweep(
                sweep_dir / "scores.csv",
                sweep_dir / "effort.csv",
                name
            ))

    # ----------------------------------------------------------------------
    # Eigen wn/ζ sweep (one grid_archive.csv per zeta directory)
    # ----------------------------------------------------------------------
    wn_eta_root = results_dir / "eigen_sweep" / "wn_eta_sweep" / mm_dir

    if wn_eta_root.exists():

        for zeta_dir in sorted(wn_eta_root.glob("zeta_*")):
            archive_path = zeta_dir / "grid_archive.csv"
            if archive_path.exists():
                dfs.append(
                    load_grid_archive(
                        archive_path,
                        f"Eigen ({zeta_dir.name})"
                    )
                )

    dfs = [d for d in dfs if not d.empty]
    if not dfs:
        print("Error: No data found for any controllers.")
        return
        
    df = pd.concat(dfs, ignore_index=True)

    # Filter out infinities or extreme scores for sane visualization
    df = df[np.isfinite(df['score']) & np.isfinite(df['effort'])].copy()

    # ── Data-integrity checks (surface silently-misleading data) ─────────────
    _validate_comparability(df, args.motor_moment)

    # ── Pareto frontiers are computed on the FULL finite data, per controller,
    # BEFORE any percentile clipping. Computing them on the p95-filtered subset
    # (the previous behaviour) could silently drop a genuine frontier endpoint —
    # e.g. a low-score / high-effort trade-off point — and thereby MISREPRESENT
    # the optimal tuning. The clip below is display-only and never removes a
    # frontier point.
    df['is_ctrl_pareto'] = False
    for ctrl in df['controller'].unique():
        mask = df['controller'] == ctrl
        ce = df.loc[mask, 'effort'].values
        cs = df.loc[mask, 'score'].values
        df.loc[mask, 'is_ctrl_pareto'] = _pareto_frontier(ce, cs)

    # ── Display clip: hide extreme outliers so the scatter is readable, but the
    # clip is computed PER CONTROLLER (not pooled) so a controller with a wide
    # spread — typically a coarse grid — isn't censored more aggressively than a
    # tightly-clustered one just because they share one global percentile. Any
    # point on a controller's Pareto frontier is always kept regardless of clip.
    keep = df['is_ctrl_pareto'].copy()
    for ctrl in df['controller'].unique():
        mask = df['controller'] == ctrl
        cs = df.loc[mask, 'score']
        ce = df.loc[mask, 'effort']
        p95_score = np.percentile(cs, 95) if len(cs) else np.inf
        p95_effort = np.percentile(ce, 95) if len(ce) else np.inf
        keep |= mask & (df['score'] <= p95_score * 1.5) & (df['effort'] <= p95_effort * 1.5)
    df_clean = df[keep].copy()

    if len(df_clean) == 0:
        print("No valid data after filtering.")
        return

    out_dir = results_dir / "plotly_dashboards" / mm_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Generating aggregate visualizations from {len(df_clean)} points ({mm_dir})...")
    
    colors = px.colors.qualitative.Plotly
    ctrl_colors = {ctrl: colors[i % len(colors)] for i, ctrl in enumerate(df_clean['controller'].unique())}
    
    # ── 1. Publication-quality Pareto Plot ─────────────────────────────────────
    fig1 = go.Figure()
    
    # Add scatter for all points
    for ctrl in df_clean['controller'].unique():
        df_c = df_clean[df_clean['controller'] == ctrl]
        fig1.add_trace(go.Scatter(
            x=df_c['effort'], y=df_c['score'], mode='markers',
            marker=dict(color=ctrl_colors[ctrl], size=5, opacity=0.3),
            name=f"{ctrl} (All)", hoverinfo='skip', showlegend=False
        ))
        
        # Highlight Pareto front for this controller
        df_pf = df_c[df_c['is_ctrl_pareto']].sort_values('effort')
        if not df_pf.empty:
            fig1.add_trace(go.Scatter(
                x=df_pf['effort'], y=df_pf['score'], mode='lines+markers',
                line=dict(color=ctrl_colors[ctrl], width=3),
                marker=dict(symbol='star', size=12, color=ctrl_colors[ctrl], line=dict(color='white', width=1)),
                name=f"{ctrl} Frontier"
            ))
    
    fig1.update_layout(
        title="Publication-Quality Pareto Frontiers (All Controllers)",
        xaxis_title="Control Effort",
        yaxis_title="Stability Score (Lower is better)",
        template="plotly_white",
        width=1000, height=600,
        margin=dict(l=60, r=40, t=60, b=60)
    )
    fig1.write_html(str(out_dir / "1_publication_pareto.html"))
    
    # ── 2. Interactive Scatter (Full Hover Data) ───────────────────────────────
    hover_cols = [c for c in df_clean.columns if c not in ['score', 'effort', 'is_ctrl_pareto', 'controller']]
    fig2 = px.scatter(
        df_clean, x="effort", y="score", color="controller", 
        color_discrete_map=ctrl_colors,
        hover_data=hover_cols,
        title="Interactive Exploration (All Controllers)",
        template="plotly_dark"
    )
    # Overlay the pareto frontiers
    for ctrl in df_clean['controller'].unique():
        df_pf = df_clean[(df_clean['controller'] == ctrl) & df_clean['is_ctrl_pareto']].sort_values('effort')
        if not df_pf.empty:
            fig2.add_trace(go.Scatter(
                x=df_pf['effort'], y=df_pf['score'], mode='lines',
                line=dict(color=ctrl_colors[ctrl], width=2, dash='dash'),
                name=f'{ctrl} Front', hoverinfo='skip', showlegend=False
            ))
    fig2.update_traces(marker=dict(size=8, line=dict(width=0.5, color='black')), selector=dict(mode='markers'))
    fig2.write_html(str(out_dir / "2_interactive_scatter.html"))
    
    # ── 3. Generation Animation (Only GA points animate, Grid is static) ───────
    frames_df = []
    max_gen = int(df_clean['gen'].max()) if 'gen' in df_clean and not pd.isna(df_clean['gen'].max()) else 0
    grid_df = df_clean[df_clean['gen'] < 0].copy()
    
    for g in range(max_gen + 1):
        df_g = df_clean[(df_clean['gen'] >= 0) & (df_clean['gen'] <= g)].copy()
        df_frame = pd.concat([df_g, grid_df])
        df_frame['anim_frame'] = g
        frames_df.append(df_frame)
        
    if frames_df:
        anim_df = pd.concat(frames_df)
        fig3 = px.scatter(
            anim_df, x="effort", y="score", 
            animation_frame="anim_frame", 
            color="controller",
            color_discrete_map=ctrl_colors,
            range_x=[df_clean['effort'].min()*0.9, df_clean['effort'].max()*1.1],
            range_y=[df_clean['score'].min()*0.9, df_clean['score'].max()*1.1],
            title="GA Convergence vs Static Grids (Cumulative)",
            template="plotly_dark",
            labels={"anim_frame": "Generation up to"}
        )
        fig3.update_traces(marker=dict(size=8, line=dict(width=0.5, color='black')))
        fig3.write_html(str(out_dir / "3_convergence_animation.html"))
    
    # ── 4. Parallel Coordinates (GA + Grid: any controller with param cols) ────
    # Previously restricted to gen >= 0 (GA only) "since grids lack genes".
    # Now that grid sweeps can supply named parameter columns via
    # grid_archive.csv, include every row that has at least one populated
    # parameter column, not just GA rows. Controllers using different
    # parameter names will show NaN gaps on axes that don't apply to them —
    # that's expected when mixing heterogeneous controllers on one plot.
    exclude_cols = ['gen', 'score', 'effort', 'is_ctrl_pareto', 'controller', 'motor_moment',
                   'rise_time', 'settling_time', 'max_overshoot', 'rms_error',
                   'peak_effort', 'total_energy', 'stable', 'oscillation',
                   'step_score', 'step_effort']
    gene_cols = [c for c in df_clean.columns if c not in exclude_cols and df_clean[c].notna().any()]

    df_pc = df_clean[df_clean[gene_cols].notna().any(axis=1)].copy() if gene_cols else df_clean.iloc[0:0].copy()
    if not df_pc.empty and gene_cols:
        # Convert controller string to a numeric category for parallel coordinates
        ctrl_map = {c: i for i, c in enumerate(df_pc['controller'].unique())}
        df_pc['controller_id'] = df_pc['controller'].map(ctrl_map)

        pc_cols = ['controller_id'] + gene_cols + ['effort', 'score']

        fig4 = px.parallel_coordinates(
            df_pc, color="score",
            dimensions=pc_cols,
            color_continuous_scale=px.colors.diverging.Tealrose,
            color_continuous_midpoint=df_pc['score'].median(),
            title="Parallel Coordinates: Parameter mapping to Performance (GA + Grid)",
            labels={c: c.replace("_mdyn", "").replace("_att", "") for c in pc_cols}
        )
        fig4.write_html(str(out_dir / "4_parallel_coords.html"))

    # ── 5. Validation Pareto Front (step-response, per controller) ─────────────
    # Every sweep script (genetic_sweep.py, eigen_stability_sweep.py's
    # ab-sweep, pd_sweep.py) now runs a step-response validation pass on
    # whatever it considers its (score, effort) Pareto front, logging the
    # result as step_score/step_effort. This is a SEPARATE Pareto front
    # computed only from points that were actually validated — i.e. proven
    # to handle a real position-control task, not just hover — so grid and
    # GA results for the same controller (already distinguished by the
    # "(GA)"/"(Grid)" suffix in the controller name) are directly comparable
    # on equal footing instead of comparing a hover-only grid best against a
    # partially-validated GA best.
    if 'step_score' in df_clean.columns and 'step_effort' in df_clean.columns:
        df_val = df_clean[np.isfinite(df_clean['step_score']) & np.isfinite(df_clean['step_effort'])].copy()
    else:
        df_val = df_clean.iloc[0:0].copy()

    if df_val.empty:
        print("No step-response validation data found — skipping validation Pareto front "
              "(re-run the sweeps with the validation loop enabled to populate step_score/step_effort).")
    else:
        df_val['is_ctrl_pareto_validated'] = False
        for ctrl in df_val['controller'].unique():
            mask = df_val['controller'] == ctrl
            ce = df_val.loc[mask, 'step_effort'].values
            cs = df_val.loc[mask, 'step_score'].values
            df_val.loc[mask, 'is_ctrl_pareto_validated'] = _pareto_frontier(ce, cs)

        fig5 = go.Figure()
        for ctrl in df_val['controller'].unique():
            df_c = df_val[df_val['controller'] == ctrl]
            color = ctrl_colors.get(ctrl, "#888888")
            fig5.add_trace(go.Scatter(
                x=df_c['step_effort'], y=df_c['step_score'], mode='markers',
                marker=dict(color=color, size=6, opacity=0.35),
                name=f"{ctrl} (validated)", hoverinfo='skip', showlegend=False
            ))

            df_pf = df_c[df_c['is_ctrl_pareto_validated']].sort_values('step_effort')
            if not df_pf.empty:
                fig5.add_trace(go.Scatter(
                    x=df_pf['step_effort'], y=df_pf['step_score'], mode='lines+markers',
                    line=dict(color=color, width=3),
                    marker=dict(symbol='star', size=12, color=color, line=dict(color='white', width=1)),
                    name=f"{ctrl} Validated Frontier"
                ))

        fig5.update_layout(
            title="Validation Pareto Front — Step-Response Performance (Pareto-front candidates only, GA vs Grid)",
            xaxis_title="Step-Response Control Effort",
            yaxis_title="Step-Response Stability Score (Lower is better)",
            template="plotly_white",
            width=1000, height=600,
            margin=dict(l=60, r=40, t=60, b=60)
        )
        fig5.write_html(str(out_dir / "5_validation_pareto.html"))
        print(f"   Validation Pareto front: {len(df_val)} validated points across "
              f"{df_val['controller'].nunique()} controllers")

    # ── 6. Gene-space response (per controller; wn,b vs Kp,Kd decoupling) ───────
    # The effort-vs-score scatter collapses every gene onto one (score, effort)
    # point, so the effect of individual tunings is invisible there. These panels
    # put the genes back on the axes. For multi-gene controllers each pair is also
    # shown with the *other* genes held in the stable-basin IQR, so e.g. the pure
    # Kp_att/Kd_att effect can be read without wn_mdyn/b_mdyn confounding it.
    for ctrl in df_clean['controller'].unique():
        dfc = df_clean[df_clean['controller'] == ctrl]
        figg = _gene_space_figure(dfc, gene_cols, ctrl)
        if figg is not None:
            safe = (ctrl.replace(' ', '_').replace('(', '').replace(')', '')
                        .replace('/', ''))
            figg.write_html(str(out_dir / f"6_gene_space_{safe}.html"))
            print(f"   Gene-space panel: {ctrl} → 6_gene_space_{safe}.html")

    print(f"\n✅ Created Aggregate Plotly HTML dashboards in:\n   {out_dir.resolve()}")

if __name__ == "__main__":
    main()