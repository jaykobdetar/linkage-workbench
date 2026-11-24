/* Planar pin-jointed linkage. All input angles are radians; lengths share a unit.
 * No dependencies. Works as window.K under file:// and as a CommonJS module. */
(function (root, build) {
  const api = build();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.K = api;
})(typeof globalThis === "object" ? globalThis : this, function () {
  "use strict";

  const TAU = 2 * Math.PI;
  const EPS = Number.EPSILON;
  const add = (a, b) => [a[0] + b[0], a[1] + b[1]];
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1]];
  const mul = (v, k) => [v[0] * k, v[1] * k];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1];
  const cross = (a, b) => a[0] * b[1] - a[1] * b[0];
  const perp = (v) => [-v[1], v[0]];
  const mag = (v) => Math.hypot(v[0], v[1]);
  const distance = (a, b) => mag(sub(a, b));
  const unit = (a) => [Math.cos(a), Math.sin(a)];
  const arg = (v) => Math.atan2(v[1], v[0]);
  const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
  function turn(v, angle, scale = 1) {
    const c = Math.cos(angle),
      s = Math.sin(angle);
    return [scale * (v[0] * c - v[1] * s), scale * (v[0] * s + v[1] * c)];
  }
  const dimensions = ["r1", "a", "b", "c", "d", "e", "f", "g", "h", "r2"];
  const defaults = Object.freeze({
    m1x: 856,
    m1y: 0,
    m2x: 504,
    m2y: 0,
    j3x: 0,
    j3y: 0,
    r1: 1250,
    a: 1968.1544996806547,
    b: 599.0926472591698,
    c: 1400,
    d: 431.8703509156423,
    e: 468.95628794163747,
    f: 869.4297304743365,
    g: 1495.798248955367,
    h: 560.357029044876,
    r2: 620.6319360136087,
    delta: 0.38550682796146457,
    beta: -1.7968431572737187,
    tool: -2.462392432117185,
    axis: 1.5256657017401476,
    mode: "rotary",
    q1min: null,
    q1max: null,
    q2min: null,
    q2max: null,
  });
  const home = Object.freeze({
    q1: 0.8069955026375185,
    q2: 1.5256657017401476,
    branch: Object.freeze([1, -1, -1]),
  });
  const branches = [];
  for (const s4 of [-1, 1])
    for (const s6 of [-1, 1]) for (const s8 of [-1, 1]) branches.push(Object.freeze([s4, s6, s8]));
  Object.freeze(branches);

  function validParams(p) {
    if (!p || !["rotary", "slider"].includes(p.mode)) return "Select a rotary or slider M2.";
    for (const k of dimensions)
      if (!Number.isFinite(p[k]) || p[k] <= 0) return k + " must be a positive finite length.";
    for (const k of ["m1x", "m1y", "m2x", "m2y", "j3x", "j3y", "delta", "beta", "tool", "axis"])
      if (!Number.isFinite(p[k])) return k + " must be finite.";
    for (const name of ["q1", "q2"]) {
      const lo = p[name + "min"],
        hi = p[name + "max"];
      if ((lo != null && !Number.isFinite(lo)) || (hi != null && !Number.isFinite(hi)))
        return name + " limits must be finite or blank.";
      if (lo != null && hi != null && lo > hi) return name + " minimum exceeds its maximum.";
    }
    return null;
  }
  function tolerance(p) {
    const size = Math.max(
      ...dimensions.filter((k) => p.mode !== "slider" || k !== "r2").map((k) => p[k]),
    );
    const origin = Math.max(...["m1x", "m1y", "m2x", "m2y", "j3x", "j3y"].map((k) => Math.abs(p[k])));
    return 2e-9 * size + 64 * EPS * origin;
  }
  // Roundoff band local to the predicate; also handles large translated coordinates.
  function roundoff(A, B, ...lengths) {
    return 48 * EPS * Math.max(...A.map(Math.abs), ...B.map(Math.abs), ...lengths.map(Math.abs));
  }
  function circles(A, r, B, s) {
    const v = sub(B, A),
      d = mag(v),
      epsilon = roundoff(A, B, r, s);
    if (d <= epsilon)
      return Math.abs(r - s) <= epsilon
        ? r <= epsilon
          ? { kind: "tangent", plus: A.slice(), minus: A.slice() }
          : { kind: "continuum" }
        : { kind: "none" };
    if (d > r + s + epsilon || d < Math.abs(r - s) - epsilon) return { kind: "none" };
    const axis = mul(v, 1 / d);
    // Normalize to avoid overflow/underflow in squares and the Heron product.
    const scale = Math.max(d, r, s),
      D = d / scale,
      R = r / scale,
      S = s / scale;
    const x = (scale * (D * D + (R - S) * (R + S))) / (2 * D);
    const center = add(A, mul(axis, x));
    if (Math.min(Math.abs(d - r - s), Math.abs(d - Math.abs(r - s))) <= epsilon)
      return { kind: "tangent", plus: center, minus: center };
    const y =
      (scale * Math.sqrt(Math.max(0, (R + S + D) * (R + S - D) * (D + R - S) * (D - R + S)))) / (2 * D);
    const offset = mul(perp(axis), y);
    return { kind: "pair", plus: add(center, offset), minus: sub(center, offset) };
  }
  function discrete(c) {
    if (c.kind === "pair") return [c.plus, c.minus];
    return c.kind === "tangent" ? [c.plus] : [];
  }
  const failure = (code, stage, reason) => ({ ok: false, code, stage, reason });
  function pick(A, r, B, s, sign, stage, angles) {
    const c = circles(A, r, B, s);
    if (c.kind === "none") return failure("unreachable", stage, stage + " closure circles do not meet.");
    if (c.kind === "continuum") {
      if (!Number.isFinite(angles[stage]))
        return failure(
          "continuum",
          stage,
          stage + " has coincident closure circles. A free angle is required.",
        );
      return { ok: true, point: add(A, mul(unit(angles[stage]), r)), sign: 0, free: true };
    }
    return { ok: true, point: sign > 0 ? c.plus : c.minus, sign: c.kind === "tangent" ? 0 : sign };
  }
  function inBounds(q, lo, hi, tol) {
    return Number.isFinite(q) && (lo == null || q >= lo - tol) && (hi == null || q <= hi + tol);
  }
  function rotaryBounds(p, base, angle) {
    const lo = p[base + "min"],
      hi = p[base + "max"];
    const angularTol = 2e-12;
    const kmin = lo == null ? -Infinity : Math.ceil((lo - angle - angularTol) / TAU);
    const kmax = hi == null ? Infinity : Math.floor((hi - angle + angularTol) / TAU);
    if (kmin > kmax) return null;
    const k = Math.max(kmin, Math.min(kmax, 0));
    return {
      value: angle + TAU * k,
      principal: angle,
      period: TAU,
      minTurn: Number.isFinite(kmin) ? kmin : null,
      maxTurn: Number.isFinite(kmax) ? kmax : null,
    };
  }
  function basePoints(p) {
    return { M1: [p.m1x, p.m1y], M2: [p.m2x, p.m2y], J3: [p.j3x, p.j3y] };
  }
  function atM2(p, q2) {
    const v = p.mode === "rotary" ? mul(unit(q2), p.r2) : mul(unit(p.axis), q2);
    return add([p.m2x, p.m2y], v);
  }
  function residuals(p, P, q1, q2) {
    const { M1, J3, J4, J5, J6, J7, J8, JX, EE } = P;
    const err = (A, B, l) => distance(A, B) - l;
    return {
      M1X: err(M1, JX, p.r1),
      L34: err(J3, J4, p.a),
      L4X: err(J4, JX, p.b),
      LXE: err(JX, EE, p.c),
      LX6: err(JX, J6, p.d),
      LX7: err(JX, J7, p.e),
      L56: err(J5, J6, p.f),
      L78: err(J7, J8, p.g),
      LE8: err(EE, J8, p.h),
      M1drive: distance(JX, add(M1, mul(unit(q1), p.r1))),
      M2drive: distance(J5, atM2(p, q2)),
      blackOffset: mag(sub(sub(EE, JX), turn(sub(JX, J4), p.delta, p.c / p.b))),
      blueOffset: mag(sub(sub(J7, JX), turn(sub(J6, JX), p.beta, p.e / p.d))),
    };
  }
  function black(p, q1, s4 = home.branch[0], freeAngles = {}) {
    const invalid = validParams(p);
    if (invalid) return failure("invalid", "geometry", invalid);
    if (s4 !== 1 && s4 !== -1)
      return failure("invalid", "branch", "Select a black branch sign (+1 or −1).");
    if (!inBounds(q1, p.q1min, p.q1max, 2e-12))
      return failure("limit", "actuator", "M1 value is nonfinite or outside its configured limits.");
    const P = basePoints(p);
    P.JX = add(P.M1, mul(unit(q1), p.r1));
    const joint = pick(P.J3, p.a, P.JX, p.b, s4, "J4", freeAngles);
    if (!joint.ok) return joint;
    P.J4 = joint.point;
    P.EE = add(P.JX, turn(sub(P.JX, P.J4), p.delta, p.c / p.b));
    return {
      ok: true,
      points: P,
      q1,
      JX: P.JX,
      J4: P.J4,
      EE: P.EE,
      s4: joint.sign,
      sign: joint.sign,
      blackAngle: arg(sub(P.J4, P.J3)),
      free: !!joint.free,
    };
  }
  function forward(p, q1, q2, branch = home.branch, freeAngles = {}) {
    if (!Array.isArray(branch) || branch.length !== 3 || !branch.every((v) => v === -1 || v === 1))
      return failure("invalid", "branch", "A branch must contain three signs (+1 or −1).");
    const blackPart = black(p, q1, branch[0], freeAngles);
    if (!blackPart.ok) return blackPart;
    const tol = tolerance(p);
    if (!inBounds(q2, p.q2min, p.q2max, p.mode === "rotary" ? 2e-12 : tol))
      return failure("limit", "actuator", "M2 value is nonfinite or outside its configured limits.");
    const P = blackPart.points;
    P.J5 = atM2(p, q2);
    const blue = pick(P.JX, p.d, P.J5, p.f, branch[1], "J6", freeAngles);
    if (!blue.ok) return blue;
    P.J6 = blue.point;
    P.J7 = add(P.JX, turn(sub(P.J6, P.JX), p.beta, p.e / p.d));
    const terminal = pick(P.EE, p.h, P.J7, p.g, branch[2], "J8", freeAngles);
    if (!terminal.ok) return terminal;
    P.J8 = terminal.point;
    const errors = residuals(p, P, q1, q2);
    const maxResidual = Math.max(...Object.values(errors).map(Math.abs));
    if (!(maxResidual <= tol))
      return failure(
        "precision",
        "residual",
        "Closure residual exceeds the configured geometry tolerance.",
      );
    return {
      ok: true,
      points: P,
      q1,
      q2,
      branch: branch.slice(),
      signs: [blackPart.sign, blue.sign, terminal.sign],
      phi: wrap(arg(sub(P.J8, P.EE)) + p.tool),
      residuals: errors,
      maxResidual,
      freeAngles: { ...freeAngles },
      continuum: !!(blackPart.free || blue.free || terminal.free),
    };
  }
  function forwardAll(p, q1, q2, freeAngles = {}) {
    const out = [],
      seen = new Set();
    for (const branch of branches) {
      const s = forward(p, q1, q2, branch, freeAngles);
      if (!s.ok) continue;
      const key = s.signs.join("/");
      if (!seen.has(key)) {
        out.push(s);
        seen.add(key);
      }
    }
    return out;
  }
  function lineCircle(origin, axis, B, r) {
    const dir = unit(axis),
      rel = sub(B, origin);
    const parallel = dot(dir, rel),
      across = Math.abs(cross(dir, rel));
    const eps = roundoff(origin, B, r);
    if (across > r + eps) return [];
    if (Math.abs(across - r) <= eps) return [parallel];
    const half = Math.sqrt(Math.max(0, (r - across) * (r + across)));
    return [parallel - half, parallel + half];
  }
  // q2 for which J5 is the given radial distance from center.
  function m2AtRadius(p, center, r) {
    const M2 = [p.m2x, p.m2y];
    if (p.mode === "slider")
      return {
        values: lineCircle(M2, p.axis, center, r).filter((q) =>
          inBounds(q, p.q2min, p.q2max, tolerance(p)),
        ),
        kind: "discrete",
      };
    const c = circles(M2, p.r2, center, r);
    const values = discrete(c)
      .map((pt) => rotaryBounds(p, "q2", arg(sub(pt, M2))))
      .filter(Boolean)
      .map((q) => q.value);
    return { values, kind: c.kind === "continuum" ? "continuum" : "discrete" };
  }
  function turnSign(A, B, Q, length) {
    const ray = sub(B, A),
      arm = sub(Q, A);
    const c = cross(ray, arm);
    const epsilon = roundoff(A, B, length, mag(ray)) * Math.max(length, mag(ray));
    return Math.abs(c) <= epsilon ? 0 : Math.sign(c);
  }
  function blackInverse(p, E, diagnostics, exceptional) {
    const { J3, M1 } = basePoints(p),
      tol = tolerance(p);
    const black = [];
    function register(X) {
      const J4 = sub(X, turn(sub(E, X), -p.delta, p.b / p.c));
      if (Math.abs(distance(J3, J4) - p.a) > tol) return;
      const lift = rotaryBounds(p, "q1", arg(sub(X, M1)));
      if (!lift) return;
      if (black.some((v) => distance(v.JX, X) <= roundoff(v.JX, X, p.r1))) return;
      black.push({
        q1: lift.value,
        q1Turns: lift,
        J4,
        JX: X,
        s4: turnSign(J3, X, J4, p.a),
        blackAngle: arg(sub(J4, J3)),
      });
    }
    const direct = circles(M1, p.r1, E, p.c);
    if (direct.kind !== "continuum") {
      discrete(direct).forEach(register);
      return black;
    }
    // J4→EE = R(theta4X) * [b + c cos(delta), c sin(delta)].
    const diagonal = [p.b + p.c * Math.cos(p.delta), p.c * Math.sin(p.delta)];
    const len = mag(diagonal);
    if (len <= roundoff(E, J3, p.b, p.c)) {
      if (Math.abs(distance(E, J3) - p.a) <= tol) {
        diagnostics.push(
          "Black inverse has a free M1 angle; E=J4 and both radial constraints coincide.",
        );
        exceptional.push({
          kind: "black-angle",
          coordinate: "q1",
          target: E.slice(),
          condition:
            "Use forward(q1,q2,branch) and retain the specified EE position and optional orientation.",
        });
      }
      return black;
    }
    const viaJ4 = circles(J3, p.a, E, len);
    if (viaJ4.kind === "continuum") {
      diagnostics.push(
        "Both black inverse circle pairs coincide; M1 angle is free subject to the blue closures.",
      );
      exceptional.push({
        kind: "black-angle",
        coordinate: "q1",
        target: E.slice(),
        condition:
          "Use forward(q1,q2,branch) and retain the specified EE position and optional orientation.",
      });
    } else {
      for (const J4 of discrete(viaJ4)) register(add(J4, turn(sub(E, J4), -arg(diagonal), p.b / len)));
    }
    return black;
  }

  // Partition the M2 coordinate by every event at which a circle-pair count can change.
  // The boundaries only depend on the reconstructed black pose and fixed dimensions.
  function familyDomain(p, B, E) {
    // Evaluate the black coordinates using the same actuator representative used to
    // replay events. An epsilon of reconstruction error can split a double root.
    const canonical = black(p, B.q1, B.s4 || 1, { J4: B.blackAngle });
    if (!canonical.ok || distance(canonical.EE, E) > 2 * tolerance(p))
      return { intervals: [], events: [], exceptional: [] };
    B = canonical;
    const X = B.JX;
    E = B.EE;
    let lo, hi;
    if (p.mode === "slider") {
      const d = unit(p.axis),
        projection = dot(d, sub(X, [p.m2x, p.m2y]));
      lo = Math.max(p.q2min ?? -Infinity, projection - p.d - p.f);
      hi = Math.min(p.q2max ?? Infinity, projection + p.d + p.f);
    } else {
      lo = p.q2min ?? (p.q2max == null ? -Math.PI : p.q2max - TAU);
      hi = Math.min(lo + TAU, p.q2max ?? Infinity);
    }
    if (hi < lo) return { intervals: [], events: [], exceptional: [] };
    const cutTol =
      p.mode === "slider"
        ? 128 * EPS * Math.max(p.d, p.f, Math.abs(lo), Math.abs(hi))
        : 128 * EPS * Math.max(1, Math.abs(lo), Math.abs(hi));
    let cuts = [lo, hi];
    function cut(q) {
      if (p.mode === "rotary") q += TAU * Math.ceil((lo - q - cutTol) / TAU);
      if (q >= lo - cutTol && q <= hi + cutTol) cuts.push(Math.max(lo, Math.min(hi, q)));
    }
    for (const radius of [p.d + p.f, Math.abs(p.d - p.f)]) m2AtRadius(p, X, radius).values.forEach(cut);
    for (const radius of [p.h + p.g, Math.abs(p.h - p.g)]) {
      const intersections = circles(X, p.e, E, radius);
      for (const J7 of discrete(intersections)) {
        const J6 = add(X, turn(sub(J7, X), -p.beta, p.d / p.e));
        m2AtRadius(p, J6, p.f).values.forEach(cut);
      }
    }
    cuts.sort((a, b) => a - b);
    cuts = cuts.filter((q, i, a) => i === 0 || q - a[i - 1] > cutTol);
    const intervals = [],
      events = [],
      exceptional = [];
    function freeBlue() {
      // At J5=X,d=f choose theta=arg(X→J6). Only the distal triangle remains.
      let angles = [-Math.PI, Math.PI];
      for (const radius of [Math.abs(p.g - p.h), p.g + p.h])
        for (const J7 of discrete(circles(X, p.e, E, radius)))
          angles.push(wrap(arg(sub(J7, X)) - p.beta));
      angles.sort((u, v) => u - v);
      angles = angles.filter((v, i) => i === 0 || v - angles[i - 1] > 128 * EPS);
      const accepted = (theta) =>
        circles(E, p.h, add(X, mul(unit(theta + p.beta), p.e)), p.g).kind !== "none";
      const events = angles.filter(accepted),
        intervals = [];
      for (let i = 1; i < angles.length; i++) {
        if (accepted((angles[i] + angles[i - 1]) / 2)) intervals.push([angles[i - 1], angles[i]]);
      }
      return { coordinate: "angle(JX→J6)", intervals, events };
    }
    function inspect(q2, event, lower = q2, upper = q2) {
      const choices = [];
      const sixes = circles(X, p.d, atM2(p, q2), p.f);
      if (sixes.kind === "continuum") {
        const angles = freeBlue();
        if (angles.intervals.length || angles.events.length)
          exceptional.push({
            q2,
            lo: lower,
            hi: upper,
            stage: "J6",
            kind: event ? "event" : "interval",
            angles,
            message: "J6 coincides with a free closure circle; use a feasible free angle.",
          });
        return choices;
      }
      for (const [i, J6] of discrete(sixes).entries()) {
        const s6 = sixes.kind === "tangent" ? 0 : i === 0 ? 1 : -1;
        const J7 = add(X, turn(sub(J6, X), p.beta, p.e / p.d));
        const eights = circles(E, p.h, J7, p.g);
        if (eights.kind === "continuum") {
          exceptional.push({
            q2,
            lo: lower,
            hi: upper,
            stage: "J8",
            kind: event ? "event" : "interval",
            s6,
            message: "Any angle of EE→J8 satisfies the final circle pair.",
          });
        } else if (eights.kind === "tangent") {
          choices.push([B.s4, s6, 0]);
        } else if (eights.kind === "pair") {
          choices.push([B.s4, s6, 1], [B.s4, s6, -1]);
        }
      }
      return choices;
    }
    for (let i = 0; i < cuts.length; i++) {
      const q = cuts[i],
        choices = inspect(q, true);
      if (choices.length) events.push({ q2: q, branches: choices });
      if (i + 1 < cuts.length) {
        const upper = cuts[i + 1],
          active = inspect((q + upper) / 2, false, q, upper);
        if (active.length) intervals.push({ lo: q, hi: upper, branches: active });
      }
    }
    return { intervals, events, exceptional };
  }

  function inverse(p, target, knownBlack) {
    const result = { solutions: [], families: [], diagnostics: [], exceptional: [] };
    const { diagnostics, exceptional } = result;
    const invalid = validParams(p);
    if (invalid) {
      diagnostics.push(invalid);
      return result;
    }
    if (
      !target ||
      !Number.isFinite(target.x) ||
      !Number.isFinite(target.y) ||
      (target.phi != null && !Number.isFinite(target.phi)) ||
      [target.q1, target.q2, target.blueAngle, target.terminalAngle].some(
        (v) => v != null && !Number.isFinite(v),
      )
    ) {
      diagnostics.push("Target coordinates and any specified angle or actuator value must be finite.");
      return result;
    }
    const E = [target.x, target.y],
      tol = tolerance(p);
    let candidates = [];
    if (knownBlack) {
      const verified = black(p, knownBlack.q1, knownBlack.s4 || 1, { J4: knownBlack.blackAngle });
      if (verified.ok && distance(verified.EE, E) <= 2 * tol) candidates = [verified];
    } else {
      candidates = blackInverse(p, E, diagnostics, exceptional);
      // An explicit q1 samples a genuinely continuous black inverse family.
      const q1Sample = target.q1 ?? (p.q1min != null && p.q1min === p.q1max ? p.q1min : null);
      if (exceptional.some((f) => f.kind === "black-angle") && Number.isFinite(q1Sample)) {
        const M1 = [p.m1x, p.m1y],
          J3 = [p.j3x, p.j3y];
        const X = add(M1, mul(unit(q1Sample), p.r1));
        const J4 = sub(X, turn(sub(E, X), -p.delta, p.b / p.c));
        const B = black(p, q1Sample, turnSign(J3, X, J4, p.a) || 1, { J4: arg(sub(J4, J3)) });
        if (B.ok && distance(B.EE, E) <= 2 * tol) candidates.push(B);
      }
    }
    function record(s) {
      if (!s.ok || distance(E, s.points.EE) > 2 * tol) return;
      if (target.phi != null && Math.abs(wrap(s.phi - target.phi)) > (2 * tol) / p.h + 2e-12) return;
      const duplicate = result.solutions.some(
        (a) =>
          Math.abs(wrap(a.q1 - s.q1)) < 1e-12 &&
          (p.mode === "rotary" ? Math.abs(wrap(a.q2 - s.q2)) < 1e-12 : Math.abs(a.q2 - s.q2) < tol) &&
          ["J4", "J6", "J8"].every((j) => distance(a.points[j], s.points[j]) <= tol),
      );
      if (!duplicate) {
        s.turns = {
          q1: rotaryBounds(p, "q1", wrap(s.q1)),
          q2: p.mode === "rotary" ? rotaryBounds(p, "q2", wrap(s.q2)) : null,
        };
        result.solutions.push(s);
      }
    }
    for (const B of candidates) {
      if (target.phi == null) {
        const domain = familyDomain(p, B, E);
        if (domain.intervals.length || domain.events.length || domain.exceptional.length)
          result.families.push({ ...B, ...domain });
        if (target.q2 != null)
          for (const s of forwardAll(p, B.q1, target.q2, {
            J4: B.blackAngle,
            J6: target.blueAngle == null ? undefined : target.blueAngle - p.beta,
            J8: target.terminalAngle,
          }))
            if (distance(s.points.J4, B.J4) <= 2 * tol) record(s);
        continue;
      }
      const X = B.JX,
        J8 = add(E, mul(unit(target.phi - p.tool), p.h));
      const sevens = circles(X, p.e, J8, p.g);
      const possibleSevens = discrete(sevens);
      if (sevens.kind === "continuum") {
        // With X=J8 and e=g all J7 are compatible with the prescribed tool.
        // Domain of q2 is exactly the ordinary position domain for this black pose.
        const domain = familyDomain(p, B, E);
        if (!domain.intervals.length && !domain.events.length && !domain.exceptional.length) continue;
        const continuous = domain.intervals.length || domain.exceptional.length;
        if (continuous) {
          diagnostics.push("J7 may take any angle about JX compatible with the M2 closure.");
          exceptional.push({
            ...B,
            kind: "blue-angle",
            coordinate: "angle(JX→J7)",
            J8,
            q2Domain: domain,
            condition: "J6=JX+(d/e)R(−beta)(J7−JX). Find M2 values with distance(J5,J6)=f.",
          });
        }
        if (target.blueAngle != null) possibleSevens.push(add(X, mul(unit(target.blueAngle), p.e)));
        const samples = target.q2 == null ? [] : [target.q2];
        if (!continuous) samples.push(...domain.events.map((v) => v.q2));
        for (const coordinate of samples) {
          const J5 = atM2(p, coordinate);
          for (const J6 of discrete(circles(X, p.d, J5, p.f)))
            possibleSevens.push(add(X, turn(sub(J6, X), p.beta, p.e / p.d)));
        }
      }
      for (const J7 of possibleSevens) {
        const J6 = add(X, turn(sub(J7, X), -p.beta, p.d / p.e));
        const inputs = m2AtRadius(p, J6, p.f);
        if (inputs.kind === "continuum") {
          const fixed = p.q2min != null && p.q2min === p.q2max;
          if (!fixed) {
            diagnostics.push(
              "J6 coincides with M2 and f=r2; the rotary M2 angle is free within its limits.",
            );
            exceptional.push({ ...B, kind: "m2-angle", coordinate: "q2", J6, J7, J8 });
          }
          const coordinate = fixed ? p.q2min : target.q2;
          if (coordinate != null && inBounds(coordinate, p.q2min, p.q2max, 2e-12))
            inputs.values.push(coordinate);
        }
        for (const q2 of inputs.values) {
          const free = { J4: B.blackAngle, J6: arg(sub(J6, X)), J8: arg(sub(J8, E)) };
          for (const s of forwardAll(p, B.q1, q2, free))
            if (distance(s.points.J4, B.J4) <= 2 * tol && distance(s.points.J6, J6) <= 2 * tol)
              record(s);
        }
      }
    }
    if (!result.solutions.length && !result.families.length && !exceptional.length)
      diagnostics.push(
        candidates.length
          ? "No compatible blue closure or actuator value."
          : "Target misses the black coupler locus or the configured M1 range.",
      );
    return result;
  }
  // Hold the two drives and the other elbow's bend fixed. Either EE or J7
  // moves around JX; the J8 closure exists exactly when the distance between
  // these two points lies in [|h-g|,h+g]. Partition the circle at both bounds.
  function bendDomain(p, q1, q2, branch, freeAngles, coordinate) {
    const empty = { intervals: [], events: [] };
    if (!Array.isArray(branch) || branch.length !== 3 || !branch.every((s) => s === -1 || s === 1))
      return { ...empty, ...failure("invalid", "branch", "A branch must contain three signs (+1 or −1).") };
    const B = black(p, q1, branch[0], freeAngles);
    if (!B.ok) return { ...empty, ...B };
    if (!inBounds(q2, p.q2min, p.q2max, p.mode === "rotary" ? 2e-12 : tolerance(p)))
      return { ...empty, ...failure("limit", "actuator", "M2 is outside its configured limits.") };
    const X = B.JX, J5 = atM2(p, q2);
    const blue = pick(X, p.d, J5, p.f, branch[1], "J6", freeAngles);
    if (!blue.ok) return { ...empty, ...blue, JX: X, J4: B.J4, EE: B.EE, J5 };
    const blueArm = sub(blue.point, X);
    const J7 = add(X, turn(blueArm, p.beta, p.e / p.d));
    const sweepBlue = coordinate === "beta";
    const arm = sweepBlue ? blueArm : sub(X, B.J4);
    const radius = sweepBlue ? p.e : p.c;
    const ratio = sweepBlue ? p.e / p.d : p.c / p.b;
    const fixed = sweepBlue ? B.EE : J7;
    const baseAngle = arg(arm);
    const cuts = [-Math.PI, Math.PI];
    for (const reach of [Math.abs(p.h - p.g), p.h + p.g]) {
      for (const point of discrete(circles(X, radius, fixed, reach)))
        cuts.push(wrap(arg(sub(point, X)) - baseAngle));
    }
    cuts.sort((a, b) => a - b);
    const angles = cuts.filter((v, i) => i === 0 || v - cuts[i - 1] > 128 * EPS);
    // Keep the fundamental domain's end even when an event is at the seam.
    angles[angles.length - 1] = Math.PI;
    function kindAt(angle) {
      const tip = add(X, turn(arm, angle, ratio));
      // Match forward's use of EE as the first circle center at boundary events.
      return sweepBlue ? circles(fixed, p.h, tip, p.g).kind : circles(tip, p.h, fixed, p.g).kind;
    }
    const events = [], intervals = [];
    for (let i = 0; i < angles.length; i++) {
      const angle = angles[i], kind = kindAt(angle);
      if (kind !== "none") events.push({ [coordinate]: angle, kind });
      if (i === 0) continue;
      const lo = angles[i - 1], hi = angle;
      if (kindAt((lo + hi) / 2) !== "none") intervals.push({ lo, hi });
    }
    const geometry = { JX: X, J4: B.J4, J5, J6: blue.point, J7, EE: B.EE };
    if (!intervals.length && !events.length)
      return { ...empty, ...geometry, ...failure("unreachable", "J8", "No bend angle closes the terminal pin.") };
    return { ok: true, intervals, events, ...geometry };
  }

  function hingeDomain(p, q1, q2, branch = home.branch, freeAngles = {}) {
    return bendDomain(p, q1, q2, branch, freeAngles, "delta");
  }

  function blueHingeDomain(p, q1, q2, branch = home.branch, freeAngles = {}) {
    return bendDomain(p, q1, q2, branch, freeAngles, "beta");
  }

  // Resolve the black subsystem alone, without imposing an angular law at
  // the blue bend or requiring a compatible distal transmission.
  function blackTargetCandidates(p, target, freeBlack = false) {
    const out = { candidates: [], diagnostics: [], exceptional: [] };
    const invalid = validParams(p);
    if (invalid) {
      out.diagnostics.push(invalid);
      return out;
    }
    if (!target || !Number.isFinite(target.x) || !Number.isFinite(target.y) ||
        [target.q1, target.q2, target.phi, target.blackAngle, target.blueAngle, target.terminalAngle]
          .some((v) => v != null && !Number.isFinite(v))) {
      out.diagnostics.push("Target coordinates and supplied angles or actuator samples must be finite.");
      return out;
    }
    const E = [target.x, target.y];
    const { M1, J3 } = basePoints(p);
    if (!freeBlack) {
      const { candidates, diagnostics, exceptional } = out;
      const raw = blackInverse(p, E, diagnostics, exceptional);
      const q1 = target.q1 ?? (p.q1min != null && p.q1min === p.q1max ? p.q1min : null);
      if (exceptional.some((f) => f.kind === "black-angle") && Number.isFinite(q1)) {
        const X = add(M1, mul(unit(q1), p.r1));
        const J4 = sub(X, turn(sub(E, X), -p.delta, p.b / p.c));
        raw.push({ q1, s4: turnSign(J3, X, J4, p.a), blackAngle: arg(sub(J4, J3)) });
      }
      for (const shape of raw) {
        const B = black(p, shape.q1, shape.s4 || 1, { J4: shape.blackAngle });
        if (B.ok && distance(B.EE, E) <= 2 * tolerance(p))
          candidates.push({ ...B, delta: p.delta });
      }
      if (!candidates.length && !exceptional.length)
        diagnostics.push("Target misses the fixed black coupler or its configured M1 limits.");
      return out;
    }
    const centers = circles(M1, p.r1, E, p.c);
    const motor1 = [];
    if (centers.kind === "continuum") {
      const q1 = target.q1 ?? (p.q1min != null && p.q1min === p.q1max ? p.q1min : null);
      if (p.q1min == null || p.q1min !== p.q1max)
        out.exceptional.push({ kind: "hinge-q1", coordinate: "q1", target: E, sampledAt: q1,
          condition: "EE coincides with M1 and c equals r1. Supply q1 to evaluate a member subject to the remaining closures." });
      if (inBounds(q1, p.q1min, p.q1max, 2e-12)) {
        motor1.push(q1);
      }
    } else {
      for (const X of discrete(centers)) {
        const coordinate = rotaryBounds(p, "q1", arg(sub(X, M1)));
        if (coordinate) motor1.push(coordinate.value);
      }
    }
    for (const q1 of motor1) {
      // Rebuild the center from its selected winding exactly as in forward().
      const X = add(M1, mul(unit(q1), p.r1));
      const fours = circles(J3, p.a, X, p.b);
      const positions = discrete(fours);
      if (fours.kind === "continuum") {
        out.exceptional.push({ kind: "hinge-j4", coordinate: "angle(J3→J4)", q1, JX: X,
          sampledAt: target.blackAngle ?? null,
          condition: "J3 and JX coincide with equal radii. Supply blackAngle to sample J4 and recover its bend." });
        if (target.blackAngle != null) positions.push(add(J3, mul(unit(target.blackAngle), p.a)));
      }
      for (const J4 of positions) {
        const delta = wrap(arg(sub(E, X)) - arg(sub(X, J4)));
        const fixed = { ...p, delta };
        const B = black(fixed, q1, turnSign(J3, X, J4, p.a) || 1, { J4: arg(sub(J4, J3)) });
        if (B.ok && distance(B.EE, E) <= 2 * tolerance(p))
          out.candidates.push({ ...B, delta });
      }
    }
    if (!out.candidates.length && !out.exceptional.length)
      out.diagnostics.push("No free black elbow satisfies the target and M1 limits.");
    return out;
  }

  // With each recovered black bend fixed, use the original blue assembly
  // inverse on that member of the moving-black-elbow family.
  function hingeInverse(p, target) {
    const blackResult = blackTargetCandidates(p, target, true);
    const out = { solutions: [], families: [], diagnostics: blackResult.diagnostics, exceptional: blackResult.exceptional };
    for (const B of blackResult.candidates) {
      const result = inverse({ ...p, delta: B.delta }, target, B);
      for (const key of ["solutions", "families", "exceptional"])
        for (const entry of result[key]) out[key].push({ ...entry, delta: B.delta });
    }
    if (!out.solutions.length && !out.families.length && !out.exceptional.length && !out.diagnostics.length)
      out.diagnostics.push("No free-hinge assembly satisfies the target, pin closures and actuator limits.");
    for (const item of out.exceptional)
      if (item.condition && !out.diagnostics.includes(item.condition)) out.diagnostics.push(item.condition);
    return out;
  }
  return Object.freeze({
    defaults,
    home,
    branches,
    validParams,
    tolerance,
    forward,
    forwardAll,
    inverse,
    residuals,
    circles,
    lineCircle,
    wrap,
    rotaryBounds,
    black,
    familyDomain,
    hingeDomain,
    blueHingeDomain,
    blackTargetCandidates,
    hingeInverse,
  });
});
