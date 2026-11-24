/* Inverse geometry with JX→J6 and JX→J7 free to rotate relative to each other.
 * Both position and pose queries take a slice at a specified M2 input. */
(function (root, build) {
  if (typeof module === "object" && module.exports) module.exports = build(require("./kinematics.js"));
  else root.BIK = build(root.K);
})(globalThis, function (K) {
  "use strict";
  const angle = (a, b) => Math.atan2(b[1] - a[1], b[0] - a[0]);
  const distance = (a, b) => Math.hypot(b[0] - a[0], b[1] - a[1]);
  const radial = (center, radius, theta) => [
    center[0] + radius * Math.cos(theta), center[1] + radius * Math.sin(theta)
  ];

  function solve(p, target, { blackFree = false } = {}) {
    const result = { solutions: [], betaFamilies: [], exceptional: [], diagnostics: [] };
    const invalid = K.validParams(p);
    if (invalid || !target || ![target.x, target.y, target.q2].every(Number.isFinite) ||
        [target.q1, target.phi, target.blackAngle, target.sixAngle, target.terminalAngle]
          .some(v => v != null && !Number.isFinite(v))) {
      result.diagnostics.push(invalid || "Enter finite target coordinates and an M2 value.");
      return result;
    }
    const tol = K.tolerance(p), q2 = target.q2;
    const limitTol = p.mode === "rotary" ? 2e-12 : tol;
    if (p.q2min != null && q2 < p.q2min - limitTol || p.q2max != null && q2 > p.q2max + limitTol) {
      result.diagnostics.push("The selected M2 lies outside its operating limits.");
      return result;
    }
    const black = K.blackTargetCandidates(p, target, blackFree);
    result.exceptional.push(...black.exceptional);
    result.diagnostics.push(...black.diagnostics);
    const E = [target.x, target.y];
    const J8 = target.phi == null ? null : radial(E, p.h, target.phi - p.tool);
    const eightAngle = J8 ? target.phi - p.tool : target.terminalAngle ?? 0;
    for (const B of black.candidates) {
      const geom = { ...p, delta: B.delta };
      const s4 = B.s4 || 1;
      const free = { J4: B.blackAngle, J8: eightAngle };
      const sixPoints = [];
      for (const s6 of [-1, 1]) {
        const branch = [s4, s6, 1];
        // At coincident J6 circles an arbitrary J6 witness is valid. For any
        // other J6 direction, subtract that change from β to keep J7 fixed.
        let domain = K.blueHingeDomain(geom, B.q1, q2, branch, free);
        if (domain.code === "continuum" && domain.stage === "J6") {
          free.J6 = target.sixAngle ?? 0;
          domain = K.blueHingeDomain(geom, B.q1, q2, branch, free);
          if (domain.ok) result.exceptional.push({ kind: "coincident J6 circles", coordinate: "angle(JX→J6)",
            condition: "The displayed blue-angle results use one J6 direction. Rotating J6 by any angle and subtracting it from β preserves J7 and the tool pose." });
        }
        if (!domain.ok || sixPoints.some(point => distance(point, domain.J6) <= tol)) continue;
        sixPoints.push(domain.J6);
        const sixAngle = angle(B.JX, domain.J6);

        function record(beta) {
          const sampled = { ...geom, beta };
          const signs = new Set();
          for (const s8 of [-1, 1]) {
            const s = K.forward(sampled, B.q1, q2, [s4, s6, s8], free);
            if (!s.ok || distance(s.points.EE, E) > 2 * tol ||
                (J8 && distance(s.points.J8, J8) > 2 * tol)) continue;
            signs.add(s.signs[2]);
            const duplicate = result.solutions.some(other =>
              ["J4", "J6", "J7", "J8", "JX"].every(j => distance(s.points[j], other.points[j]) <= tol));
            if (!duplicate) result.solutions.push({ ...s, beta, delta: B.delta });
          }
          return signs.size;
        }

        if (J8) {
          // A heading fixes J8. Its circles about JX and J8 enumerate J7,
          // hence β relative to this J6. Final forward closure selects s8.
          const sevens = K.circles(B.JX, p.e, J8, p.g);
          if (sevens.kind === "continuum") {
            result.exceptional.push({ kind: "coincident J7 circles", coordinate: "β",
              condition: "The target places J8 at JX with equal J7 radii. Every β on this J6 slice can realize the specified heading; the displayed solution samples the current β." });
            record(K.wrap(p.beta));
          } else if (sevens.kind === "pair" || sevens.kind === "tangent") {
            record(K.wrap(angle(B.JX, sevens.plus) - sixAngle));
            if (sevens.kind === "pair") record(K.wrap(angle(B.JX, sevens.minus) - sixAngle));
          }
          continue;
        }

        const { intervals, events } = domain;
        const sixKind = K.circles(B.JX, p.d, domain.J5, p.f).kind;
        result.betaFamilies.push({ q1: B.q1, delta: B.delta, s4: B.s4, s6: sixKind === "pair" ? s6 : 0,
          sixAngle, intervals, events });
        // Retain the current sample when possible and show an interior
        // witness for each other interval, plus every isolated event.
        const current = K.wrap(p.beta);
        const currentSigns = record(current);
        for (const { lo, hi } of intervals) {
          if (currentSigns < 2 || current <= lo || current >= hi) record((lo + hi) / 2);
        }
        for (const { beta, kind } of events) {
          if (kind === "continuum") result.exceptional.push({ kind: "coincident J8 circles",
            coordinate: "angle(EE→J8)", q1: B.q1, delta: B.delta, beta,
            condition: "At β = " + (beta * 180 / Math.PI).toFixed(4) + "° on this branch, any tool angle satisfies the final pin closure." });
          if (!intervals.some(({ lo, hi }) => beta >= lo - 1e-14 && beta <= hi + 1e-14))
            record(beta);
        }
      }
    }
    if (!result.solutions.length && !result.exceptional.length && !result.diagnostics.length)
      result.diagnostics.push("The target has no closed assembly on the selected M2 slice.");
    return result;
  }
  return Object.freeze({ solve });
});
