"""
Rendering for sim3d.py.

  render_png  - static two-view figure (matplotlib, already a dependency). This
                is what gets committed to docs/img/.
  write_html  - interactive viewer: orbit/zoom, and a slider that replays the
                sweep with the coverage growing. Needs plotly (optional):
                    pip install plotly

Conventions: x along the hull (bow on the left when you look at the port side),
y outboard, vertical axis = height relative to the waterline (so the keel is at
the bottom and depth z becomes -z).
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

CLEAN, DIRTY = "#9ccbf4", "#c98b66"     # cleaned / not yet cleaned skin
PATH, ROBOT, STBD = "#14407a", "#eda100", "#d9d8d2"
LIFT = 0.02                              # m, draw the path just off the skin so it isn't hidden


def _path(sim, every: int = 1) -> np.ndarray:
    L = sim.log
    return np.column_stack([np.asarray(L["x"]), np.asarray(L["y"]) + LIFT, -np.asarray(L["z"])])[::every]


def _caption(sim) -> str:
    p = sim.params
    return (f"{sim.ctrl.lane_idx + 1} lanes · {sim.t / 60:.1f} min · "
            f"coverage {sim.grid.coverage(p.max_depth):.1%} of the hull above {p.max_depth:.2f} m")


# ---------------------------------------------------------------------------
def render_png(sim, out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import to_rgba
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    m = sim.mesh
    X, Y, Z = m.v[..., 0], m.v[..., 1], -m.v[..., 2]
    fc = np.empty((m.nu + 1, m.nx + 1, 4))      # plot_surface reads one color per cell from the vertex-shaped array
    fc[:] = to_rgba(DIRTY)
    fc[:-1, :-1][sim.grid.cleaned] = to_rgba(CLEAN)
    path = _path(sim, every=2)

    fig = plt.figure(figsize=(12, 9.2))
    fig.patch.set_facecolor("white")
    views = [(24, 100, "From outboard and above (port side)"), (-32, 112, "From below and forward: the bilge and keel")]
    for k, (elev, azim, title) in enumerate(views):
        ax = fig.add_subplot(2, 1, k + 1, projection="3d")
        mirror = ax.plot_surface(X, -Y, Z, color=STBD, alpha=0.22, linewidth=0, antialiased=False, rstride=3, cstride=3)
        skin = ax.plot_surface(X, Y, Z, facecolors=fc, shade=False, linewidth=0, antialiased=False, rstride=1, cstride=1)
        (trail,) = ax.plot(path[:, 0], path[:, 1], path[:, 2], color=PATH, lw=1.0)
        mirror.set_zorder(1); skin.set_zorder(2); trail.set_zorder(10)   # mplot3d sorts per artist; force the path on top
        ax.set_box_aspect((sim.hull.plan.length, 2 * sim.hull.half_beam, sim.hull.plan.draft * 1.6), zoom=1.45)
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title, loc="left", fontsize=11, color="#21201c")
        ax.set_xlabel("along hull x (m)  [bow = left]")
        ax.set_zlabel("height vs waterline (m)")
        ax.set_yticks([])
        for a in (ax.xaxis, ax.yaxis, ax.zaxis):
            a.pane.set_facecolor((1, 1, 1, 0))
            a.pane.set_edgecolor("#e4e3dd")
        ax.tick_params(colors="#6b6a63", labelsize=8)
    fig.suptitle("Glaucus I sweep on a 3D hull  ·  " + _caption(sim), x=0.02, y=0.985, ha="left", fontsize=12, color="#21201c")
    fig.legend(handles=[Patch(facecolor=CLEAN, label="Cleaned"), Patch(facecolor=DIRTY, label="Not cleaned"),
                        Line2D([0], [0], color=PATH, lw=1.5, label="Robot path"),
                        Patch(facecolor=STBD, alpha=0.5, label="Starboard side (mirror, not simulated)")],
               loc="lower center", ncol=4, frameon=False, fontsize=9)
    fig.subplots_adjust(left=0.0, right=1.0, top=0.93, bottom=0.04, hspace=-0.12)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------------------
def write_html(sim, out: Path, max_frames: int = 45) -> None:
    try:
        import plotly.graph_objects as go
    except ImportError as e:  # pragma: no cover
        raise SystemExit("The interactive viewer needs plotly:  pip install plotly") from e

    m = sim.mesh
    verts = m.v.reshape(-1, 3)
    vx, vy, vz = verts[:, 0], verts[:, 1], -verts[:, 2]
    nx, nu = m.nx, m.nu
    jj, ii = np.meshgrid(np.arange(nu), np.arange(nx), indexing="ij")
    a, b = jj * (nx + 1) + ii, jj * (nx + 1) + ii + 1
    c, d = (jj + 1) * (nx + 1) + ii + 1, (jj + 1) * (nx + 1) + ii
    tri_i = np.concatenate([a.ravel(), a.ravel()])
    tri_j = np.concatenate([b.ravel(), c.ravel()])
    tri_k = np.concatenate([c.ravel(), d.ravel()])

    def skin(cleaned: np.ndarray):
        return np.concatenate([cleaned.ravel(), cleaned.ravel()]).astype(np.int8)

    path = np.round(_path(sim, every=6), 3)
    snaps = sim.snapshots
    step = max(1, len(snaps) // max_frames)
    snaps = snaps[::step] + ([snaps[-1]] if (len(snaps) - 1) % step else [])
    fin_cleaned = sim.grid.cleaned

    port = go.Mesh3d(x=vx, y=vy, z=vz, i=tri_i, j=tri_j, k=tri_k, intensity=skin(fin_cleaned),
                     intensitymode="cell", colorscale=[[0, DIRTY], [1, CLEAN]], cmin=0, cmax=1,
                     showscale=False, flatshading=True, name="Port hull",
                     lighting=dict(ambient=0.75, diffuse=0.5, specular=0.05), hoverinfo="skip")
    stbd = go.Mesh3d(x=vx, y=-vy, z=vz, i=tri_i, j=tri_j, k=tri_k, color=STBD, opacity=0.22,
                     flatshading=True, name="Starboard (mirror)", hoverinfo="skip")
    wl = m.v[0]
    waterline = go.Scatter3d(x=np.r_[wl[:, 0], wl[::-1, 0]], y=np.r_[wl[:, 1], -wl[::-1, 1]],
                             z=np.zeros(2 * len(wl)), mode="lines", line=dict(color="#4a90d9", width=3),
                             name="Waterline", hoverinfo="skip")
    trail = go.Scatter3d(x=path[:, 0], y=path[:, 1], z=path[:, 2], mode="lines",
                         line=dict(color=PATH, width=3), name="Robot path", hoverinfo="skip")
    p_end = _path(sim)[-1]
    robot = go.Scatter3d(x=[p_end[0]], y=[p_end[1]], z=[p_end[2]], mode="markers",
                         marker=dict(size=6, color=ROBOT, symbol="diamond", line=dict(color="#21201c", width=1)),
                         name="Glaucus I", hoverinfo="skip")

    frames, ticks = [], np.asarray([t for t, _ in snaps])
    full = np.round(_path(sim), 3)
    for (tick, cleaned) in snaps:
        n = max(1, tick // 6)
        pt = full[min(tick, len(full)) - 1]
        frames.append(go.Frame(
            name=f"{tick * sim.dt / 60:.1f} min", traces=[0, 3, 4],
            data=[go.Mesh3d(intensity=skin(cleaned)),
                  go.Scatter3d(x=path[:n, 0], y=path[:n, 1], z=path[:n, 2]),
                  go.Scatter3d(x=[pt[0]], y=[pt[1]], z=[pt[2]])]))

    fig = go.Figure(data=[port, stbd, waterline, trail, robot], frames=frames)
    fig.update_layout(
        title=dict(text="Glaucus I · 3D sweep<br><sup>" + _caption(sim) + "</sup>", x=0.02),
        paper_bgcolor="white", margin=dict(l=0, r=0, t=70, b=0), legend=dict(orientation="v", x=0.99, xanchor="right", y=0.98, yanchor="top"),
        scene=dict(aspectmode="data", camera=dict(eye=dict(x=-0.3, y=2.2, z=0.7)),
                   xaxis=dict(title="along hull x (m)  [bow = +x]"), yaxis=dict(title="y (m)"),
                   zaxis=dict(title="height vs waterline (m)")),
        updatemenus=[dict(type="buttons", direction="left", x=0.02, y=0.0, xanchor="left", yanchor="top",
                          buttons=[dict(label="Play", method="animate",
                                        args=[None, dict(frame=dict(duration=80, redraw=True), fromcurrent=True)]),
                                   dict(label="Pause", method="animate",
                                        args=[[None], dict(frame=dict(duration=0), mode="immediate")])])],
        sliders=[dict(active=len(frames) - 1, x=0.14, len=0.84, y=0.0, yanchor="top",
                      currentvalue=dict(prefix="t = "),
                      steps=[dict(label=f.name, method="animate",
                                  args=[[f.name], dict(mode="immediate", frame=dict(duration=0, redraw=True))])
                             for f in frames])])
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(out), include_plotlyjs=True, full_html=True, auto_play=False)
