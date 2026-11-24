/* Differential kinematics for the planar, rigid effective-span reconstruction.
 * Classic browser script: D.evaluate(parameters, K.forward(...), options).
 * CommonJS: require('./differential.js'). No runtime dependencies.
 */
(function (root, factory) {
  if (typeof module === "object" && module.exports) module.exports = factory();else root.D = factory();
})(typeof globalThis === "undefined" ? this : globalThis, function () {
  "use strict";

  const EPS_RANK = 1e-10;
  const members = {
    L1X: ["M1", "JX"],
    L25: ["M2", "J5"],
    L34: ["J3", "J4"],
    L4X: ["J4", "JX"],
    LXE: ["JX", "EE"],
    L56: ["J5", "J6"],
    L6X: ["J6", "JX"],
    LX7: ["JX", "J7"],
    L78: ["J7", "J8"],
    LE8: ["EE", "J8"]
  };
  const parameters = ["m1x", "m1y", "m2x", "m2y", "j3x", "j3y", "r1", "a", "b", "c", "d", "e", "f", "g", "h", "delta", "beta", "tool", "axis", "r2"];
  const rows = [["J3", "J4", "a"], ["J5", "J6", "f"], ["J7", "J8", "g"]];
  const zero = () => [0, 0];
  const add = (a, b) => [a[0] + b[0], a[1] + b[1]];
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1]];
  const scale = (a, k) => [a[0] * k, a[1] * k];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1];
  const cross = (a, b) => a[0] * b[1] - a[1] * b[0];
  const rot90 = a => [-a[1], a[0]];
  const unit = t => [Math.cos(t), Math.sin(t)];
  const norm = v => Math.hypot(v[0], v[1]);
  const transpose = columns => columns[0].map((_, i) => columns.map(col => col[i]));
  const mult = (m, v) => m.map(row => row.reduce((s, x, i) => s + x * v[i], 0));
  const asDirection = (v, r) => scale(v, 1 / r);
  const vFrom = (pts, a, b) => sub(pts[b], pts[a]);
  function frame(origin, angle) {
    const c = Math.cos(angle),
      s = Math.sin(angle),
      [x, y] = origin;
    return {
      forward: [[c, -s, x], [s, c, y], [0, 0, 1]],
      inverse: [[c, s, -c * x - s * y], [-s, c, s * x - c * y], [0, 0, 1]]
    };
  }
  function matrixRank(matrix) {
    const a = matrix.map(row => row.slice());
    const threshold = EPS_RANK * Math.max(1, ...a.flat().map(Math.abs));
    let rank = 0;
    for (let j = 0; j < a[0].length && rank < a.length; j++) {
      let pivot = rank;
      for (let i = rank + 1; i < a.length; i++) {
        if (Math.abs(a[i][j]) > Math.abs(a[pivot][j])) pivot = i;
      }
      if (Math.abs(a[pivot][j]) <= threshold) continue;
      [a[pivot], a[rank]] = [a[rank], a[pivot]];
      for (let i = rank + 1; i < a.length; i++) {
        const factor = a[i][j] / a[rank][j];
        for (let k = j; k < a[i].length; k++) a[i][k] -= factor * a[rank][k];
      }
      rank++;
    }
    return rank;
  }

  // H has zero entries above its diagonal, and H[1,0]=0 by construction.
  function solveH(H, rhs) {
    const a = rhs[0] / H[0][0],
      b = rhs[1] / H[1][1];
    return [a, b, (rhs[2] - H[2][0] * a - H[2][1] * b) / H[2][2]];
  }
  function gains2(m) {
    const [[a, b], [c, d]] = m;
    const sum = Math.hypot(a + d, c - b),
      difference = Math.hypot(a - d, c + b);
    const largest = (sum + difference) / 2;
    const smallest = largest ? Math.abs(a * d - b * c) / largest : 0;
    return [largest, smallest];
  }
  // Only called after the dimensionless matrix has been checked for full rank.
  function inverse3(matrix) {
    const rows = matrix.map((row, i) => [...row, ...[0, 1, 2].map(j => Number(i === j))]);
    for (let j = 0; j < 3; j++) {
      let pivot = j;
      for (let i = j + 1; i < 3; i++) {
        if (Math.abs(rows[i][j]) > Math.abs(rows[pivot][j])) pivot = i;
      }
      [rows[j], rows[pivot]] = [rows[pivot], rows[j]];
      const divisor = rows[j][j];
      rows[j] = rows[j].map(value => value / divisor);
      for (let i = 0; i < 3; i++) {
        if (i === j) continue;
        const multiple = rows[i][j];
        rows[i] = rows[i].map((value, column) => value - multiple * rows[j][column]);
      }
    }
    return rows.map(row => row.slice(3));
  }
  // One-sided Jacobi SVD applied to the three columns of the transposed
  // normalized pose Jacobian. This avoids squaring the condition number.
  function poseSpectrum(matrix) {
    const columns = matrix.map(row => row.slice());
    const left = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    function rotatePair(vectors, p, q, cosine, sine) {
      const a = vectors[p], b = vectors[q];
      vectors[p] = a.map((x, i) => cosine * x - sine * b[i]);
      vectors[q] = b.map((y, i) => sine * a[i] + cosine * y);
    }
    for (let sweep = 0; sweep < 20; sweep++) {
      let changed = false;
      for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) {
        const a = columns[p], b = columns[q];
        const ab = a.reduce((sum, x, i) => sum + x * b[i], 0);
        const na = Math.hypot(...a), nb = Math.hypot(...b);
        if (Math.abs(ab) <= 8 * Number.EPSILON * na * nb) continue;
        const tau = ((nb - na) * (nb + na)) / (2 * ab);
        const t = (tau < 0 ? -1 : 1) / (Math.abs(tau) + Math.hypot(1, tau));
        const cosine = 1 / Math.hypot(1, t), sine = t * cosine;
        rotatePair(columns, p, q, cosine, sine);
        rotatePair(left, p, q, cosine, sine);
        changed = true;
      }
      if (!changed) break;
    }
    const singular = columns.map(col => Math.hypot(...col));
    const threshold = EPS_RANK * Math.max(1, ...singular);
    const retained = singular.map(x => x > threshold);
    const pseudo = columns[0].map((_, i) => left[0].map((_, j) =>
      singular.reduce((sum, s, k) => retained[k] ? sum + (columns[k][i] / s) * (left[k][j] / s) : sum, 0)));
    return {rank: retained.filter(Boolean).length, pseudo, gains: singular.sort((a, b) => b - a)};
  }
  function pair(value, name, size) {
    if (value === undefined) return Array(size).fill(0);
    if (!Array.isArray(value) || value.length !== size || !value.every(Number.isFinite)) {
      throw new TypeError(name + " must contain " + size + " finite numbers");
    }
    return value;
  }
  function evaluate(p, state, options = {}) {
    return evaluateCoordinates(p, state, options, false, false);
  }
  function freeHinge(p, state, options = {}) {
    return evaluateCoordinates(p, state, options, true, false);
  }
  function elbows(p, state, options = {}) {
    return evaluateCoordinates(p, state, options, Boolean(options.blackFree), Boolean(options.blueFree));
  }
  function evaluateCoordinates(p, state, options, blackFree, blueFree) {
    if (!state || !state.ok) return {
      regular: false,
      reason: "A valid closed assembly is required."
    };
    const coordinates = ["q1", "q2"];
    if (blackFree) coordinates.push("delta");
    if (blueFree) coordinates.push("beta");
    const hasFreeElbow = coordinates.length > 2;
    const rates = pair(options.rates, "rates", coordinates.length);
    const inputAccel = pair(options.accelerations, "accelerations", coordinates.length);
    const wrench = pair(options.wrench, "wrench", 3);
    const pt = state.points,
      rotary = p.mode === "rotary";
    const r1 = vFrom(pt, "M1", "JX"),
      r2 = vFrom(pt, "M2", "J5");
    const b = vFrom(pt, "JX", "J4"),
      c = vFrom(pt, "JX", "EE");
    const d = vFrom(pt, "JX", "J6"),
      e = vFrom(pt, "JX", "J7"),
      h = vFrom(pt, "EE", "J8");
    const spans = rows.map(([from, to]) => vFrom(pt, from, to));
    const transforms = {};
    for (const [name, [from, to]] of Object.entries(members)) {
      const v = vFrom(pt, from, to);
      // When the prismatic span is collapsed its guide still defines a frame.
      const angle = norm(v) ? Math.atan2(v[1], v[0]) : name === "L25" && !rotary ? p.axis : null;
      transforms[name] = angle === null ? null : frame(pt[from], angle);
    }
    transforms.tool = frame(pt.EE, state.phi);

    // Independent variations in q, z and dimensional parameters, before closure.
    function variation(seed) {
      const k = name => seed[name] || 0;
      function radius(v, len, dlen, ang) {
        return add(scale(asDirection(v, len), dlen), scale(rot90(v), ang));
      }
      const M1 = [k("m1x"), k("m1y")],
        M2 = [k("m2x"), k("m2y")],
        J3 = [k("j3x"), k("j3y")];
      const JX = add(M1, radius(r1, p.r1, k("r1"), k("q1")));
      const J5 = rotary ? add(M2, radius(r2, p.r2, k("r2"), k("q2"))) : add(M2, add(scale(unit(p.axis), k("q2")), scale(rot90(r2), k("axis"))));
      const J4 = add(JX, radius(b, p.b, k("b"), k("alpha")));
      const EE = add(JX, radius(c, p.c, k("c"), k("alpha") + k("delta")));
      const J6 = add(JX, radius(d, p.d, k("d"), k("betaAngle")));
      const J7 = add(JX, radius(e, p.e, k("e"), k("betaAngle") + k("beta")));
      const J8 = add(EE, radius(h, p.h, k("h"), k("gamma")));
      const points = {
        M1,
        M2,
        J3,
        J4,
        J5,
        J6,
        J7,
        J8,
        JX,
        EE
      };
      const constraint = rows.map(([from, to, len], i) => dot(spans[i], vFrom(points, from, to)) - p[len] * k(len));
      return {
        points,
        constraint,
        pose: [...EE, k("gamma") + k("tool")]
      };
    }
    const vars = ["alpha", "betaAngle", "gamma", ...coordinates];
    const partials = vars.map(name => variation({
      [name]: 1
    }));
    const constraintJacobian = transpose(partials.map(x => x.constraint));
    const H = constraintJacobian.map(row => row.slice(0, 3));
    const B = constraintJacobian.map(row => row.slice(3));
    const characteristicLength = Math.max(p.a, p.b, p.c, p.d, p.e, p.f, p.g, p.h, p.r1, rotary ? p.r2 : 0);
    const qScale = coordinates.map(name => name === "q2" && !rotary ? 1 : characteristicLength);
    // Equilibrate units before comparing pivots. Multiply slider column by L
    // to express all generalized coordinates as angles or length divided by L.
    const denom = [p.a * p.b, p.f * p.d, p.g * p.h];
    const tangency = H.map((row, i) => row[i] / denom[i]);
    const scaled = constraintJacobian.map((row, i) => row.map((v, j) => v * (j === 4 && !rotary ? characteristicLength : 1) / denom[i]));
    const rankH = matrixRank(scaled.map(row => row.slice(0, 3)));
    const rankConstraint = matrixRank(scaled);
    const freeWithMotorsLocked = scaled.map(row => [...row.slice(0, 3), ...row.slice(5)]);
    // A tangent may be returned as a single labeled root by the geometric solver.
    // Use its measured angle and constraint rank, not its branch label, here.
    const regular = rankH === 3 && tangency.every(x => Math.abs(x) > EPS_RANK);
    const out = {
      regular,
      freeHinge: blackFree,
      blackFree,
      blueFree,
      coordinates,
      reason: regular ? "" : "Dependent constraint matrix is singular or numerically rank deficient.",
      H,
      B,
      constraintJacobian,
      constraintResiduals: spans.map((span, i) => norm(span) - p[rows[i][2]]),
      rankH,
      rankConstraint,
      tangency,
      characteristicLength,
      transforms,
      mobilityLinearized: vars.length - rankConstraint,
      lockedNullity: freeWithMotorsLocked[0].length - matrixRank(freeWithMotorsLocked),
      nearSingular: tangency.some(x => Math.abs(x) < 1e-4),
      poseJacobian: null,
      dependentJacobian: null,
      pointJacobians: null,
      memberJacobians: null,
      positionJacobian: null,
      positionPseudoInverse: null,
      positionRank: null,
      reducedJacobian: null,
      inverseReduced: null,
      poseLeftInverse: null,
      effortMap: null,
      inverseEffortMap: null,
      effort: null,
      balancingEffort: null,
      spanAxialForces: null,
      reducedSingularValues: null,
      normalizedForceGains: null,
      sensitivity: null,
      poseVelocity: null,
      poseAcceleration: null,
      jointVelocity: null,
      jointAcceleration: null,
      memberMotion: null,
      constraintRates: null,
      constraintAccelerations: null,
      poseRank: null,
      poseNullity: null,
      posePseudoInverse: null,
      poseInverse: null,
      tangent: null,
      dependentVelocity: null,
      dependentAcceleration: null,
      amplificationMatrix: null
    };
    if (!regular) return out;
    function closed(seed, direct) {
      const dz = solveH(H, direct.constraint.map(x => -x));
      const total = variation({
        ...seed,
        alpha: dz[0],
        betaAngle: dz[1],
        gamma: dz[2]
      });
      return {
        dz,
        total
      };
    }
    const input = coordinates.map((name, j) => closed({
      [name]: 1
    }, partials[j + 3]));
    const J = transpose(input.map(x => x.total.pose));
    const Z = transpose(input.map(x => x.dz));
    const pointJacobians = {};
    for (const name of Object.keys(pt)) {
      if (name in input[0].total.points) pointJacobians[name] = transpose(input.map(x => x.total.points[name]));
    }
    const sensitivity = {};
    for (const name of parameters) {
      if (coordinates.includes(name)) continue;
      const seed = {
        [name]: 1
      };
      sensitivity[name] = closed(seed, variation(seed)).total.pose;
    }
    const dz = mult(Z, rates);
    const motionSeed = values => Object.fromEntries(coordinates.map((name, j) => [name, values[j]]));
    const rateSeed = motionSeed(rates);
    const v = variation({
      ...rateSeed,
      alpha: dz[0],
      betaAngle: dz[1],
      gamma: dz[2]
    });
    // Quadratic (centripetal) terms at q''=z''=0. Geometric parameters are constant.
    const centripetal = (r, w) => scale(r, -w * w);
    const Xb = centripetal(r1, rates[0]);
    const Eb = add(Xb, centripetal(c, dz[0] + (rateSeed.delta || 0)));
    const bias = {
      M1: zero(),
      M2: zero(),
      J3: zero(),
      JX: Xb,
      EE: Eb,
      J4: add(Xb, centripetal(b, dz[0])),
      J5: rotary ? centripetal(r2, rates[1]) : zero(),
      J6: add(Xb, centripetal(d, dz[1])),
      J7: add(Xb, centripetal(e, dz[1] + (rateSeed.beta || 0))),
      J8: add(Eb, centripetal(h, dz[2]))
    };
    const curvature = rows.map(([from, to], i) => {
      const dv = vFrom(v.points, from, to);
      return dot(dv, dv) + dot(spans[i], vFrom(bias, from, to));
    });
    const Bqdd = mult(B, inputAccel);
    const ddz = solveH(H, curvature.map((x, i) => -x - Bqdd[i]));
    const linearAcc = variation({
      ...motionSeed(inputAccel),
      alpha: ddz[0],
      betaAngle: ddz[1],
      gamma: ddz[2]
    });
    const acc = {};
    for (const name of Object.keys(bias)) acc[name] = add(bias[name], linearAcc.points[name]);
    const memberMotion = {},
      memberJacobians = {};
    for (const [name, [from, to]] of Object.entries(members)) {
      const r = vFrom(pt, from, to),
        dv = vFrom(v.points, from, to),
        da = vFrom(acc, from, to);
      const square = dot(r, r);
      const omega = square ? cross(r, dv) / square : null;
      const angularAcceleration = square ? (cross(r, da) - 2 * dot(r, dv) * omega) / square : null;
      memberMotion[name] = {
        omega,
        angularAcceleration
      };
      memberJacobians[name] = square ? input.map(col => cross(r, vFrom(col.total.points, from, to)) / square) : null;
    }
    memberMotion.tool = {
      omega: dz[2],
      angularAcceleration: ddz[2]
    };
    memberJacobians.tool = J[2].slice();
    const effortMap = transpose(J);
    const effort = mult(effortMap, wrench);
    Object.assign(out, {
      poseJacobian: J,
      positionJacobian: J.slice(0, 2),
      dependentJacobian: Z,
      pointJacobians,
      memberJacobians,
      sensitivity,
      effortMap,
      effort,
      poseVelocity: v.pose,
      poseAcceleration: [...acc.EE, ddz[2]],
      jointVelocity: v.points,
      jointAcceleration: acc,
      dependentVelocity: dz,
      dependentAcceleration: ddz,
      memberMotion,
      constraintRates: v.constraint,
      constraintAccelerations: rows.map(([from, to], i) => {
        const dv = vFrom(v.points, from, to);
        return dot(dv, dv) + dot(spans[i], vFrom(acc, from, to));
      })
    });
    if (hasFreeElbow) {
      // Balance angular rates using the same characteristic length as the
      // fixed-elbow analysis. Passive generalized loads are moments about JX.
      const outputScale = [1, 1, characteristicLength];
      const normalized = J.map((row, i) => row.map((x, j) => x * outputScale[i] / qScale[j]));
      const spectrum = poseSpectrum(normalized);
      out.poseRank = spectrum.rank;
      out.poseNullity = coordinates.length - out.poseRank;
      out.positionRank = matrixRank(normalized.slice(0, 2));
      const nativeInverse = m => m.map((row, i) => row.map((x, j) => x * outputScale[j] / qScale[i]));
      out.poseInverse = coordinates.length === 3 && out.poseRank === 3 ? nativeInverse(inverse3(normalized)) : null;
      out.posePseudoInverse = nativeInverse(spectrum.pseudo);
      out.normalizedPoseGains = spectrum.gains;
      if (blueFree) {
        // Change M2 while cancelling the X→J6 bearing rate with the β rate.
        // X→J7, J8 and the tool are then stationary on a regular branch.
        out.blueSelfMotion = coordinates.map(name => name === "q2" ? 1 : name === "beta" ? -Z[1][1] : 0);
      }
      return out;
    }

    // The black transmission fixes EE xy at each q1. A speed magnitude sets the
    // first reduced coordinate even at a stationary point; no tangent is defined there.
    const velocity1 = [J[0][0], J[1][0]],
      speed = norm(velocity1);
    const tangent = speed > EPS_RANK * characteristicLength ? scale(velocity1, 1 / speed) : null;
    const reduced = [[speed, 0], J[2].slice()];
    const normalized = reduced.map((row, i) => row.map((x, j) => x * (i === 1 ? characteristicLength : 1) / qScale[j]));
    const singularValues = gains2(normalized);
    const poseRank = singularValues.filter(x => x > EPS_RANK * Math.max(1, singularValues[0])).length;
    let inv = null,
      left = null;
    if (tangent && poseRank === 2) {
      inv = [[1 / speed, 0], [-reduced[1][0] / (speed * reduced[1][1]), 1 / reduced[1][1]]];
      left = inv.map(([u, w]) => [u * tangent[0], u * tangent[1], w]);
    }
    // C=.5(|span|²-L²); -C_z^T lambda supplies the passive generalized
    // reaction. Positive lambda*L pulls the span ends toward one another.
    const passiveLoad = mult(partials.slice(0, 3).map(x => x.pose), wrench);
    const last = passiveLoad[2] / H[2][2];
    const spanAxialForces = {
      L34: p.a * (passiveLoad[0] - H[2][0] * last) / H[0][0],
      L56: p.f * (passiveLoad[1] - H[2][1] * last) / H[1][1],
      L78: p.g * last
    };
    Object.assign(out, {
      poseRank,
      poseNullity: 2 - poseRank,
      tangent,
      positionRank: tangent ? 1 : 0,
      positionPseudoInverse: tangent ? [scale(tangent, 1 / speed), [0, 0]] : [[0, 0], [0, 0]],
      reducedJacobian: reduced,
      inverseReduced: inv,
      poseLeftInverse: left,
      amplificationMatrix: normalized,
      reducedSingularValues: singularValues,
      normalizedForceGains: poseRank === 2 ? [1 / singularValues[1], 1 / singularValues[0]] : null,
      inverseEffortMap: inv ? transpose(inv) : null,
      balancingEffort: effort.map(x => -x),
      spanAxialForces
    });
    return out;
  }
  return {
    evaluate,
    freeHinge,
    elbows,
    frame,
    members,
    parameters,
    rankTolerance: EPS_RANK
  };
});
