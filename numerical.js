/* Independent Cartesian constraint solver. Runs in a browser or Node. */
(function (root, factory) {
  if (typeof module === "object" && module.exports) module.exports = factory();
  else root.N = factory();
})(typeof globalThis === "object" ? globalThis : this, function () {
  "use strict";

  const moving = ["JX", "J4", "EE", "J5", "J6", "J7", "J8"];
  const rowLabels = [
    "1X x",
    "1X y",
    "34 distance",
    "4X distance",
    "XE parallel",
    "XE transverse",
    "25 x",
    "25 y",
    "X6 distance",
    "56 distance",
    "X7 parallel",
    "X7 transverse",
    "E8 distance",
    "78 distance",
  ];
  const hypot = Math.hypot;
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1]];
  const dist = (a, b) => hypot(...sub(a, b));
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1];
  const cross = (a, b) => a[0] * b[1] - a[1] * b[0];
  const vec = (l, t) => [l * Math.cos(t), l * Math.sin(t)];
  const finitePoint = (a) => Array.isArray(a) && a.length === 2 && a.every(Number.isFinite);

  function residuals(p, q1, q2, pts) {
    const { JX: x, J4: v, EE: z, J5: j, J6: u, J7: w, J8: k } = pts;
    const ax = sub(x, v),
      xe = sub(z, x),
      xu = sub(u, x),
      xw = sub(w, x);
    const drivenX = vec(p.r1, q1);
    const driven5 = p.mode === "rotary" ? vec(p.r2, q2) : vec(q2, p.axis);
    return [
      x[0] - p.m1x - drivenX[0],
      x[1] - p.m1y - drivenX[1],
      dist(v, [p.j3x, p.j3y]) - p.a,
      hypot(...ax) - p.b,
      dot(ax, xe) / p.b - p.c * Math.cos(p.delta),
      cross(ax, xe) / p.b - p.c * Math.sin(p.delta),
      j[0] - p.m2x - driven5[0],
      j[1] - p.m2y - driven5[1],
      hypot(...xu) - p.d,
      dist(u, j) - p.f,
      dot(xu, xw) / p.d - p.e * Math.cos(p.beta),
      cross(xu, xw) / p.d - p.e * Math.sin(p.beta),
      dist(k, z) - p.h,
      dist(k, w) - p.g,
    ];
  }

  function unpack(v, p) {
    const pts = { M1: [p.m1x, p.m1y], M2: [p.m2x, p.m2y], J3: [p.j3x, p.j3y] };
    moving.forEach((name, i) => {
      pts[name] = [v[2 * i], v[2 * i + 1]];
    });
    return pts;
  }

  // Row-pivoted elimination on the damped normal system (14 unknowns).
  function linear(A, b) {
    const n = b.length,
      m = A.map((r, i) => [...r, b[i]]);
    for (let j = 0; j < n; j++) {
      let pivot = j;
      for (let i = j + 1; i < n; i++) if (Math.abs(m[i][j]) > Math.abs(m[pivot][j])) pivot = i;
      if (!Number.isFinite(m[pivot][j]) || Math.abs(m[pivot][j]) < 1e-16) return null;
      [m[j], m[pivot]] = [m[pivot], m[j]];
      for (let i = j + 1; i < n; i++) {
        const f = m[i][j] / m[j][j];
        for (let k = j + 1; k <= n; k++) m[i][k] -= f * m[j][k];
      }
    }
    const x = new Array(n);
    for (let i = n - 1; i >= 0; i--) {
      let v = m[i][n];
      for (let j = i + 1; j < n; j++) v -= m[i][j] * x[j];
      x[i] = v / m[i][i];
    }
    return x;
  }

  function numericalJacobian(v, fun) {
    const n = v.length;
    const J = Array.from({ length: n }, () => Array(n));
    for (let j = 0; j < n; j++) {
      const h = 3e-6 * Math.max(1, Math.abs(v[j]));
      const plus = v.slice(),
        minus = v.slice();
      plus[j] += h;
      minus[j] -= h;
      const fp = fun(plus),
        fm = fun(minus);
      for (let i = 0; i < n; i++) J[i][j] = (fp[i] - fm[i]) / (2 * h);
    }
    return J;
  }

  function lmStep(J, r, damping) {
    const n = r.length,
      H = Array.from({ length: n }, () => Array(n).fill(0)),
      b = Array(n).fill(0);
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < n; j++) {
        b[j] -= J[i][j] * r[i];
        for (let k = 0; k <= j; k++) H[j][k] += J[i][j] * J[i][k];
      }
    }
    for (let j = 0; j < n; j++) {
      H[j][j] += damping * (1 + H[j][j]);
      for (let k = 0; k < j; k++) H[k][j] = H[j][k];
    }
    return linear(H, b);
  }

  function solve(p, q1, q2, seed, options = {}) {
    const fail = (reason) => ({ ok: false, reason });
    if (!p || !seed || !moving.every((name) => finitePoint(seed[name])))
      return fail("A finite seed is required for all seven moving points.");
    const lengths = ["r1", "a", "b", "c", "d", "e", "f", "g", "h"];
    if (p.mode === "rotary") lengths.push("r2");
    if (
      !["slider", "rotary"].includes(p.mode) ||
      lengths.some((k) => !Number.isFinite(p[k]) || p[k] <= 0) ||
      ![
        p.m1x,
        p.m1y,
        p.m2x,
        p.m2y,
        p.j3x,
        p.j3y,
        p.delta,
        p.beta,
        p.tool,
        q1,
        q2,
        p.mode === "slider" ? p.axis : 0,
      ].every(Number.isFinite) ||
      [p.q1min, p.q1max, p.q2min, p.q2max].some((value) => value != null && !Number.isFinite(value))
    )
      return fail("Invalid geometry or input.");
    if (
      (p.q1min != null && q1 < p.q1min) ||
      (p.q1max != null && q1 > p.q1max) ||
      (p.q2min != null && q2 < p.q2min) ||
      (p.q2max != null && q2 > p.q2max)
    )
      return fail("Inputs exceed the selected operating bounds.");
    const scale = Math.max(...lengths.map((k) => p[k]));
    // Use M1 as the origin and scale out the working length before differencing.
    const loc = {
      ...p,
      m1x: 0,
      m1y: 0,
      m2x: (p.m2x - p.m1x) / scale,
      m2y: (p.m2y - p.m1y) / scale,
      j3x: (p.j3x - p.m1x) / scale,
      j3y: (p.j3y - p.m1y) / scale,
    };
    lengths.forEach((k) => {
      loc[k] = p[k] / scale;
    });
    const u2 = p.mode === "slider" ? q2 / scale : q2;
    const perturb = options.perturb === undefined ? 0.006 * scale : options.perturb;
    const tolerance = options.tolerance === undefined ? 2e-12 * scale : options.tolerance;
    const maxIterations = options.maxIterations === undefined ? 90 : options.maxIterations;
    if (
      !Number.isFinite(perturb) ||
      perturb < 0 ||
      !Number.isFinite(tolerance) ||
      tolerance <= 0 ||
      !Number.isInteger(maxIterations) ||
      maxIterations < 1
    )
      return fail("Invalid solver options.");
    let v = moving
      .flatMap((name) => sub(seed[name], [p.m1x, p.m1y]))
      .map((x, i) => (x + perturb * Math.sin((i + 1) * 2.33)) / scale);
    const fun = (v) => residuals(loc, q1, u2, unpack(v, loc));
    const maxAbs = (r) => Math.max(...r.map(Math.abs));
    const cost = (r) => r.reduce((s, x) => s + x * x, 0);
    let r = fun(v),
      lambda = 1e-5,
      iterations = 0;
    const history = [maxAbs(r) * scale];
    while (maxAbs(r) * scale > tolerance && iterations < maxIterations) {
      const J = numericalJacobian(v, fun);
      let accepted = false;
      for (let attempt = 0; attempt < 14; attempt++) {
        const step = lmStep(J, r, lambda);
        if (step) {
          const trial = v.map((x, i) => x + step[i]),
            rt = fun(trial);
          if (cost(rt) < cost(r)) {
            v = trial;
            r = rt;
            lambda = Math.max(lambda * 0.18, 1e-14);
            accepted = true;
            break;
          }
        }
        lambda *= 8;
      }
      iterations++;
      history.push(maxAbs(r) * scale);
      if (!accepted) break;
    }
    const points = unpack(
      v.map((x, i) => x * scale + (i % 2 ? p.m1y : p.m1x)),
      p,
    );
    const { J3, J4, J5, J6, J7, J8, JX, EE } = points;
    const branch = [
      cross(sub(JX, J3), sub(J4, J3)),
      cross(sub(J5, JX), sub(J6, JX)),
      cross(sub(J7, EE), sub(J8, EE)),
    ].map(Math.sign);
    const phi0 = Math.atan2(J8[1] - EE[1], J8[0] - EE[0]) + p.tool;
    const maxResidual = maxAbs(residuals(p, q1, q2, points));
    const ok = maxResidual <= tolerance;
    return {
      ok,
      reason: ok ? "Converged" : "No convergence within the requested residual tolerance.",
      points,
      iterations,
      maxResidual,
      tolerance,
      history,
      branch,
      phi: Math.atan2(Math.sin(phi0), Math.cos(phi0)),
      difference: Math.max(...moving.map((k) => dist(points[k], seed[k]))),
    };
  }
  return { solve, residuals, rowLabels, moving };
});
