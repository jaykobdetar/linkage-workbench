/* UI units are millimetres and degrees. The engine receives millimetres and radians. */
(() => {
  "use strict";

  const $ = id => document.getElementById(id);
  const rad = Math.PI / 180,
    deg = x => x / rad,
    tau = 2 * Math.PI;
  const fmt = (x, n = 3) => !Number.isFinite(x) ? "—" : (Math.abs(x) < 0.5 * 10 ** -n ? 0 : x).toFixed(n);
  const precise = x => !Number.isFinite(x) ? "—" : x === 0 ? "0" : Math.abs(x) < 0.001 || Math.abs(x) >= 1e6 ? x.toExponential(3) : fmt(x, 5);
  const esc = s => String(s).replace(/[&<>"']/g, k => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    '"': "&quot;",
    "'": "&#39;"
  })[k]);
  const pointNames = ["M1", "M2", "J3", "J4", "J5", "J6", "JX", "J7", "J8", "EE"];
  const spanDefs = [["L1X", "M1", "JX", "base"], ["L34", "J3", "J4", "base"], ["L4X", "J4", "JX", "base"], ["LXE", "JX", "EE", "base"], ["L25", "M2", "J5", "blue"], ["L56", "J5", "J6", "blue"], ["L6X", "J6", "JX", "blue"], ["LX7", "JX", "J7", "blue"], ["L78", "J7", "J8", "blue"], ["LE8", "EE", "J8", "base"]];
  const blockedPairs = new Set();
  const labels = [["r1", "L1X · M1 to JX"], ["a", "L34 · J3 to J4"], ["b", "L4X · J4 to JX"], ["c", "LXE · JX to EE"], ["d", "L6X · JX to J6"], ["e", "LX7 · JX to J7"], ["f", "L56 · J5 to J6"], ["g", "L78 · J7 to J8"], ["h", "LE8 · EE to J8"], ["r2", "L25 · rotary radius"]];
  const angleKeys = ["delta", "beta", "tool", "axis", "q1min", "q1max"];
  const vec = (a, b) => [b[0] - a[0], b[1] - a[1]];
  const norm = a => Math.hypot(...a),
    dist = (a, b) => norm(vec(a, b));
  const direction = (a, b) => Math.atan2(b[1] - a[1], b[0] - a[0]);
  const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
  const cross = (a, b) => a[0] * b[1] - a[1] * b[0];
  const canvas = $("plot"),
    ctx = canvas.getContext("2d");
  const transformCells = [makeTransformCells("toolForward"), makeTransformCells("toolReverse")];
  let p,
    q1,
    q2,
    branch,
    state,
    diff,
    tab = "joints",
    controlMode = "forward";
  let trail = [],
    sampleKey = "",
    samples = [],
    camera = null,
    map = null,
    target = null,
    drag = null,
    ikList = [],
    raf = 0;
  let playback = null,
    freeAngles = {};
  let comparison = null,
    crossingInfo = [];
  const rateKeys = ["dq1", "dq2", "ddq1", "ddq2", "fx", "fy", "moment"];
  const defaultParams = () => ({
    ...K.defaults
  });
  function table(head, rows) {
    return '<div class="tablewrap"><table><thead><tr>' + head.map(h => "<th>" + esc(h) + "</th>").join("") + "</tr></thead><tbody>" + rows.map(r => "<tr>" + r.map(x => "<td>" + esc(x) + "</td>").join("") + "</tr>").join("") + "</tbody></table></div>";
  }
  function matrix(name, mat) {
    const value = Array.isArray(mat) && mat.length ? mat.map(row => (Array.isArray(row) ? row : [row]).map(x => precise(x).padStart(12)).join(" ")).join("\n") : "Undefined in this configuration";
    return '<div class="matrix"><strong>' + esc(name) + "</strong><pre>" + esc(value) + "</pre></div>";
  }
  function makeTransformCells(id) {
    const body = $(id).createTBody();
    const cells = [];
    for (let r = 0; r < 3; r++) {
      const row = body.insertRow();
      for (let c = 0; c < 3; c++) {
        const cell = row.insertCell();
        cell.dataset.coefficient = String(3 * r + c);
        cells.push(cell);
      }
    }
    return cells;
  }
  function updateToolTransforms(tool) {
    const values = [tool?.forward, tool?.inverse];
    transformCells.forEach((cells, direction) => {
      const coefficients = values[direction]?.flat();
      cells.forEach((cell, i) => {
        const value = coefficients?.[i];
        let display = "—";
        if (Number.isFinite(value)) {
          const digits = i % 3 === 2 ? 3 : 6;
          display = Math.abs(value) >= 1e6 ? value.toExponential(3) : String(Number(fmt(value, digits)));
        }
        if (cell.textContent !== display) cell.textContent = display;
      });
    });
    const status = $("transformStatus");
    status.hidden = Boolean(tool);
    if (!tool) status.textContent = "The current inputs have no assembled tool frame.";
  }
  function say(s = "", warn = false) {
    $("notice").textContent = s;
    $("notice").classList.toggle("warn", warn);
  }
  function home() {
    freeAngles = {};
    for (const k of [4, 6, 8]) $("free" + k).value = "";
    p = defaultParams();
    q1 = K.home.q1;
    q2 = K.home.q2;
    branch = [...K.home.branch];
    trail = [];
    sampleKey = "";
    target = null;
    ikList = [];
    comparison = null;
    camera = null;
    drag = null;
  }
  function changeTab(id) {
    tab = id;
    for (const button of document.querySelectorAll("[data-tab]")) {
      const active = button.dataset.tab === id;
      button.setAttribute("aria-pressed", String(active));
      $(button.dataset.tab).hidden = !active;
    }
    drawInspection();
  }
  function rangeAngles(q, min, max) {
    // Show a full revolution around the current turn when explicit bounds are absent.
    const center = 360 * Math.floor((deg(q) + 180) / 360);
    const lo = min == null ? center - 180 : deg(min),
      hi = max == null ? center + 180 : deg(max);
    return [Math.min(lo, deg(q)), Math.max(hi, deg(q))];
  }
  function updateInputs() {
    const rotary = p.mode === "rotary",
      extent = Math.max(p.r2, p.f, p.d, p.r1, 1);
    const ar = rangeAngles(q1, p.q1min, p.q1max);
    const br = rotary ? rangeAngles(q2, p.q2min, p.q2max) : [Math.min(p.q2min ?? -extent, q2), Math.max(p.q2max ?? extent, q2)];
    for (const [key, range, val, unit] of [["q1", ar, deg(q1), "°"], ["q2", br, rotary ? deg(q2) : q2, rotary ? "°" : " mm"]]) {
      $(key).min = range[0];
      $(key).max = range[1];
      $(key).step = "any";
      $(key).value = val;
      $(key + "n").value = +fmt(val, 7);
      $(key + "lo").textContent = fmt(range[0], 1) + unit;
      $(key + "hi").textContent = fmt(range[1], 1) + unit;
    }
    $("q2unit").textContent = rotary ? "°" : "mm";
    $("q2label").textContent = rotary ? "crank angle" : "slider travel";
    $("inverseMotors").value = state.ok ? "M1 " + fmt(deg(q1)) + "° · M2 " +
      fmt(rotary ? deg(q2) : q2) + (rotary ? "°" : " mm") : "No assembled pose";
    [4, 6, 8].forEach((id, i) => $("s" + id).value = String(branch[i]));
  }
  function allowedForward(params, a, b, choice) {
    const st = K.forward(params, a, b, choice, freeAngles);
    if (st.ok && blockedPairs.size) {
      const hits = findCrossings(st.points).filter(c => blockedPairs.has(c.pair));
      if (hits.length) return {
        ok: false,
        code: "crossing",
        reason: "Excluded projected crossing: " + hits.map(c => c.pair).join(", ") + "."
      };
    }
    return st;
  }
  function refresh(options = {}) {
    state = allowedForward(p, q1, q2, branch);
    comparison = null;
    diff = null;
    crossingInfo = state.ok ? findCrossings(state.points) : [];
    if (state.ok && $("trace").checked && options.record !== false) {
      const entry = state.points.EE;
      if (!trail.length || trail.at(-1) === null || dist(entry, trail.at(-1)) > K.tolerance(p)) {
        trail.push(entry);
        if (trail.length > 1000) trail.shift();
      }
    } else if (!state.ok && trail.length && trail.at(-1) !== null) trail.push(null);
    if (!options.keepMessage) say(state.ok ? "Drag EE to move along this assembly’s path." : state.reason, !state.ok);
    if (!state.ok && options.keepMessage) say(state.reason, true);
    if (state.ok && $("numericOverlay").checked) comparison = N.solve(p, q1, q2, state.points);
    updateInputs();
    updateStatus();
    drawInspection();
    queueDraw();
    if (controlMode === "inverse" && !target && state.ok) fillTarget();
  }
  function updateStatus() {
    const valid = state.ok;
    const d = valid ? getDiff() : null;
    updateToolTransforms(valid ? (d?.transforms.tool ?? D.frame(state.points.EE, state.phi)) : null);
    let status = valid ? "Assembled" : "Unreachable";
    if (state.code === "continuum") {
      status = "Free angle needed";
    } else if (valid && state.continuum) {
      status = "Sampled continuum";
    } else if (valid && ((d && !d.regular) || state.signs?.includes(0))) {
      status = "Singular assembly";
    } else if (d?.nearSingular) {
      status = "Near singularity";
    } else if (d && d.poseRank < 2) {
      status = "Pose rank loss";
    }
    $("validity").textContent = status;
    $("validity").className = "status" + (status === "Unreachable" ? " bad" : status === "Assembled" ? "" : " warn");
    $("freeControls").hidden = state.code !== "continuum" && !state.continuum && !Object.keys(freeAngles).length;
    if (state.code === "continuum") $("freeControls").open = true;
    $("rx").textContent = valid ? fmt(state.points.EE[0]) : "—";
    $("ry").textContent = valid ? fmt(state.points.EE[1]) : "—";
    $("rphi").textContent = valid ? fmt(deg(state.phi), 2) : "—";
    $("rr").textContent = valid ? maxResidual().toExponential(2) : "—";
  }
  function maxResidual() {
    return Math.max(0, ...Object.values(state.residuals || {}).filter(Number.isFinite).map(Math.abs));
  }
  function setMode(id) {
    if (id === controlMode) return;
    if (playback) stopPlay();
    clearTarget();
    controlMode = id;
    $("forwardPanel").hidden = id !== "forward";
    $("inversePanel").hidden = id !== "inverse";
    $("forwardMode").setAttribute("aria-pressed", id === "forward");
    $("inverseMode").setAttribute("aria-pressed", id === "inverse");
    if (state.ok && id === "inverse") {
      fillTarget();
      say("Target starts at the displayed EE. Drag to pick a new position.");
    } else {
      say(state.ok ? "Adjust M1 or M2 to move the linkage." : state.reason, !state.ok);
    }
    queueDraw();
  }
  function fillTarget(preserveAngle = false) {
    const point = state.points.EE;
    $("tx").value = point[0].toPrecision(16);
    $("ty").value = point[1].toPrecision(16);
    if (!preserveAngle) $("tphi").value = deg(state.phi).toPrecision(16);
  }
  function clearTarget() {
    target = null;
    ikList = [];
    $("ikResults").replaceChildren();
  }
  function traceBreak() {
    if (trail.length && trail.at(-1) !== null) trail.push(null);
  }
  function setDrive(i, value) {
    if (!Number.isFinite(value)) return;
    stopPlay();
    clearTarget();
    if (i === 1) q1 = value * rad;else q2 = p.mode === "rotary" ? value * rad : value;
    refresh();
  }
  function adopt(solution) {
    stopPlay();
    if (solution.continuum) for (const n of [4, 6, 8]) {
      const v = solution.freeAngles?.["J" + n];
      if (Number.isFinite(v)) {
        freeAngles["J" + n] = v;
        $("free" + n).value = deg(v);
      }
    }
    q1 = solution.q1;
    q2 = solution.q2;
    branch = [...solution.branch];
    traceBreak();
    refresh({
      keepMessage: true
    });
    if (state.ok && !$("orient").checked) $("tphi").value = deg(state.phi).toPrecision(16);
  }
  function solveTarget(x, y, phi) {
    if (!Number.isFinite(x) || !Number.isFinite(y) || phi !== undefined && !Number.isFinite(phi)) {
      clearTarget();
      say("Enter finite numbers for each target coordinate.", true);
      queueDraw();
      return;
    }
    const args = {
      x,
      y,
      q1,
      q2,
      blueAngle: freeAngles.J6 == null ? undefined : freeAngles.J6 + p.beta,
      terminalAngle: freeAngles.J8
    };
    if (phi !== undefined) args.phi = phi;
    const result = K.inverse(p, args);
    ikList = (result.solutions || []).filter(s => !blockedPairs.size || !findCrossings(s.points).some(c => blockedPairs.has(c.pair)));
    const families = result.families || [];
    target = [x, y];
    let html = "";
    if (ikList.length) {
      const branchChanges = s => s.branch.filter((sign, i) => sign !== branch[i]).length;
      const motorDistance = s => Math.hypot(wrap(s.q1 - q1),
        p.mode === "rotary" ? wrap(s.q2 - q2) : (s.q2 - q2) / p.r1);
      ikList.sort((a, b) => branchChanges(a) - branchChanges(b) || motorDistance(a) - motorDistance(b));
      const msg = ikList.length + (ikList.length === 1 ? " solution" : " solutions") +
        (phi === undefined ? " at the current M2." : " for this position and tool angle.");
      html = "<p>" + esc(msg) + "</p>" + ikList.map((s, i) => '<button class="chip" data-sol="' + i + '" title="Select this solution">#' + (i + 1) + " " + (s.signs || s.branch).map(signLabel).join("") + " · M1 " + fmt(deg(s.q1), 3) + "° · M2 " + fmt(p.mode === "rotary" ? deg(s.q2) : s.q2, 3) + (p.mode === "rotary" ? "°" : " mm") + "</button>").join("");
      const alreadyAtTarget = state.ok && branchChanges(ikList[0]) === 0 && motorDistance(ikList[0]) <= 1e-12;
      say(alreadyAtTarget ? "The displayed pose already reaches this target." : "Solved. The linkage shows the selected motor settings.");
      adopt(ikList[0]);
    } else {
      const reason = result.exceptional?.length ? "A continuous inverse family exists. See its free coordinate below." : families.length ? "No solution at the current M2. Inspect the position families below for other M2 settings." : "No assembly reaches this exact target. Drag with snapping on to pick a reachable position.";
      const msg = reason + (state.ok ? " The linkage holds its previous pose." : "");
      say(msg, true);
      html = '<p class="warning">' + esc(msg) + "</p>";
      queueDraw();
    }
    if (families.length) {
      html += "<details><summary>Position families (" + families.length + ")</summary>" + families.map(f => "<p>M1 " + fmt(deg(f.q1), 5) + "°, J4 sign " + esc(f.s4) + "<br>" + familySummary(f) + "</p>").join("") + "</details>";
    }
    if (result.exceptional?.length) {
      html += "<details open><summary>Exceptional families (" + result.exceptional.length + ")</summary>" + result.exceptional.map(f => "<p>" + esc(f.kind) + " · free coordinate " + esc(f.coordinate || "free coordinate") + "</p>").join("") + "</details>";
    }
    if (result.diagnostics?.length) html += "<details><summary>Inverse diagnostics</summary>" + result.diagnostics.map(t => "<p>" + esc(typeof t === "string" ? t : JSON.stringify(t)) + "</p>").join("") + "</details>";
    $("ikResults").innerHTML = html;
    queueDraw();
  }
  function signLabel(v) {
    return v > 0 ? "+" : v < 0 ? "−" : "0";
  }
  function familySummary(f) {
    const unit = p.mode === "rotary" ? "°" : " mm",
      val = v => fmt(p.mode === "rotary" ? deg(v) : v, 4) + unit;
    const signs = options => options.map(b => b.map(signLabel).join("")).join(", ");
    const intervals = (f.intervals || []).map(i => val(i.lo) + " … " + val(i.hi) + " · " + signs(i.branches));
    const events = (f.events || []).map(i => val(i.q2) + " · " + signs(i.branches));
    const exceptional = (f.exceptional || []).map(i => "Free-angle closure at M2 " + val(i.q2) + " · " + esc(i.stage));
    const rows = [...intervals, ...events, ...exceptional];
    return rows.length ? '<span class="family-domain">' + rows.join("<br>") + "</span>" : "No feasible M2 interval or isolated event.";
  }
  function findCrossings(points) {
    const hits = [];
    for (let i = 0; i < spanDefs.length; i++) for (let j = i + 1; j < spanDefs.length; j++) {
      const a = spanDefs[i],
        b = spanDefs[j];
      if ([a[1], a[2]].some(k => k === b[1] || k === b[2])) continue;
      const u = vec(points[a[1]], points[a[2]]),
        v = vec(points[b[1]], points[b[2]]),
        w = vec(points[a[1]], points[b[1]]),
        det = cross(u, v);
      if (Math.abs(det) <= 1e-12 * norm(u) * norm(v)) continue;
      const t = cross(w, v) / det,
        s = cross(w, u) / det;
      if (t > 1e-8 && t < 1 - 1e-8 && s > 1e-8 && s < 1 - 1e-8) hits.push({
        pair: a[0] + " × " + b[0],
        xy: [points[a[1]][0] + t * u[0], points[a[1]][1] + t * u[1]]
      });
    }
    return hits;
  }
  function queueDraw() {
    if (!raf) raf = requestAnimationFrame(() => {
      raf = 0;
      draw();
    });
  }
  function makeSamples() {
    // Plot one actuator slice. All accepted samples satisfy all three closures.
    const key = JSON.stringify([p, q2, branch, freeAngles, [...blockedPairs]]);
    if (key === sampleKey) return;
    samples = [];
    sampleKey = key;
    const lo = p.q1min ?? (p.q1max == null ? -Math.PI : p.q1max - tau),
      hi = Math.min(p.q1max ?? lo + tau, lo + tau);
    const count = 420;
    for (let i = 0; i <= count; i++) {
      const q = lo + (hi - lo) * i / count,
        st = allowedForward(p, q, q2, branch);
      samples.push({
        q,
        pt: st.ok ? st.points.EE : null,
        near: false
      });
    }
    // Refine each detected valid/invalid transition to the boundary. A boundary may
    // also be due to actuator limits, so check the link-circle triangle slacks.
    for (let i = 1; i < samples.length; i++) {
      let a = samples[i - 1],
        b = samples[i];
      if (Boolean(a.pt) === Boolean(b.pt)) continue;
      let inside = a.pt ? a.q : b.q,
        outside = a.pt ? b.q : a.q,
        st;
      for (let k = 0; k < 30; k++) {
        const mid = (inside + outside) / 2,
          r = allowedForward(p, mid, q2, branch);
        if (r.ok) inside = mid;else outside = mid;
      }
      st = allowedForward(p, inside, q2, branch);
      if (!st.ok) continue;
      const z = st.points;
      const margins = [[dist(z.J3, z.JX), p.a, p.b], [dist(z.J5, z.JX), p.f, p.d], [dist(z.J7, z.EE), p.g, p.h]];
      const near = margins.some(([d, r, s]) => Math.min(Math.abs(d - r - s), Math.abs(d - Math.abs(r - s))) <= 1e-5 * Math.max(1, d, r, s));
      (a.pt ? a : b).edge = {
        pt: st.points.EE,
        near
      };
    }
  }
  function fit(all = false) {
    const base = [[p.m1x, p.m1y], [p.j3x, p.j3y], [p.m2x, p.m2y]];
    if (state.ok) base.push(...Object.values(state.points));
    if (all && $("workspace").checked) {
      makeSamples();
      base.push(...samples.filter(s => s.pt).map(s => s.pt));
    }
    const xs = base.map(a => a[0]),
      ys = base.map(a => a[1]);
    let left = Math.min(...xs),
      right = Math.max(...xs),
      low = Math.min(...ys),
      high = Math.max(...ys);
    const L = Math.max(1e-9, p.r1, p.r2, p.a, p.b, p.c, p.f, p.g);
    const dx = Math.max(right - left, L * 0.24),
      dy = Math.max(high - low, L * 0.24);
    camera = {
      cx: (left + right) / 2,
      cy: (low + high) / 2,
      w: dx * 1.16,
      h: dy * 1.32
    };
    queueDraw();
  }
  function draw() {
    const width = canvas.clientWidth,
      height = canvas.clientHeight;
    if (!width || !height) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 3);
    if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
      canvas.width = Math.round(width * dpr);
      canvas.height = Math.round(height * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    if (!camera) fit();
    const scale = Math.max(1e-12, Math.min((width - 30) / camera.w, (height - 54) / camera.h));
    map = {
      x: v => width / 2 + (v - camera.cx) * scale,
      y: v => height / 2 - (v - camera.cy) * scale,
      world: (x, y) => [camera.cx + (x - width / 2) / scale, camera.cy - (y - height / 2) / scale],
      scale
    };
    const xy = a => [map.x(a[0]), map.y(a[1])];
    const baseStep = 65 / scale,
      mag = 10 ** Math.floor(Math.log10(baseStep)),
      major = [1, 2, 5, 10].find(v => v * mag >= baseStep) * mag;
    const low = map.world(0, height),
      hi = map.world(width, 0);
    function line(A, B, color, stroke = 1, dash = []) {
      ctx.lineWidth = stroke;
      ctx.strokeStyle = color;
      ctx.setLineDash(dash);
      ctx.beginPath();
      ctx.moveTo(...A);
      ctx.lineTo(...B);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    function circle(P, r, fill, stroke, w = 1.5) {
      ctx.beginPath();
      ctx.arc(...P, r, 0, tau);
      if (fill) {
        ctx.fillStyle = fill;
        ctx.fill();
      }
      if (stroke) {
        ctx.strokeStyle = stroke;
        ctx.lineWidth = w;
        ctx.stroke();
      }
    }
    function grid(step, color) {
      for (let i = Math.ceil(low[0] / step), k = 0; i <= Math.floor(hi[0] / step) && k < 300; i++, k++) line([map.x(i * step), 0], [map.x(i * step), height], color);
      for (let j = Math.ceil(low[1] / step), k = 0; j <= Math.floor(hi[1] / step) && k < 300; j++, k++) line([0, map.y(j * step)], [width, map.y(j * step)], color);
    }
    grid(major / 5, "#ebefea");
    grid(major, "#dbe4df");
    if ($("axes").checked) {
      line([map.x(0), 0], [map.x(0), height], "#acbcb7", 1, [4, 4]);
      line([0, map.y(0)], [width, map.y(0)], "#acbcb7", 1, [4, 4]);
      ctx.fillStyle = "#60736b";
      ctx.font = "10px monospace";
      ctx.textAlign = "center";
      for (let i = Math.ceil(low[0] / major), n = 0; i <= Math.floor(hi[0] / major) && n < 300; i++, n++) {
        const x = map.x(i * major);
        if (x > 28 && x < width - 22) ctx.fillText(Number((i * major).toPrecision(4)), x, height - 6);
      }
      ctx.textAlign = "left";
      for (let i = Math.ceil(low[1] / major), n = 0; i <= Math.floor(hi[1] / major) && n < 300; i++, n++) {
        const y = map.y(i * major);
        if (y > 16 && y < height - 23) ctx.fillText(Number((i * major).toPrecision(4)), 4, y - 3);
      }
    }
    if ($("workspace").checked) {
      makeSamples();
      ctx.strokeStyle = "#d28b44";
      ctx.lineWidth = 2.3;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      let prev = false;
      for (let i = 0; i < samples.length; i++) {
        const s = samples[i];
        if (s.pt) {
          const pos = xy(s.pt);
          if (!prev && s.edge) {
            ctx.moveTo(...xy(s.edge.pt));
            ctx.lineTo(...pos);
          } else if (prev) ctx.lineTo(...pos);else ctx.moveTo(...pos);
          if (s.edge && i + 1 < samples.length && !samples[i + 1].pt) ctx.lineTo(...xy(s.edge.pt));
          prev = true;
        } else prev = false;
      }
      ctx.stroke();
      ctx.setLineDash([]);
      for (const s of samples) {
        if (s.edge?.near) {
          const a = xy(s.edge.pt);
          ctx.fillStyle = "#bd702b";
          ctx.beginPath();
          ctx.moveTo(a[0], a[1] - 4.5);
          ctx.lineTo(a[0] + 4.5, a[1]);
          ctx.lineTo(a[0], a[1] + 4.5);
          ctx.lineTo(a[0] - 4.5, a[1]);
          ctx.closePath();
          ctx.fill();
        }
      }
    }
    if ($("trace").checked) {
      let prev = null;
      for (const s of trail) {
        if (s && prev) line(xy(prev), xy(s), "#8265a0", 2);
        prev = s;
      }
    }
    if (state.ok) {
      const z = state.points;
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      function arm(A, B, color, inner, w) {
        line(xy(z[A]), xy(z[B]), color, w);
        line(xy(z[A]), xy(z[B]), inner, Math.max(1, w - 3));
      }
      function zig(A, B, color) {
        const a = xy(z[A]),
          b = xy(z[B]),
          v = vec(a, b),
          len = norm(v);
        if (len < 2) return;
        const ux = v[0] / len,
          uy = v[1] / len,
          amp = Math.min(3.4, len * 0.023),
          cycles = Math.max(4, Math.min(24, Math.round(len / 9)));
        line(a, b, color, 1);
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.25;
        ctx.beginPath();
        ctx.moveTo(...a);
        for (let i = 1; i < 2 * cycles; i++) {
          const f = i / (2 * cycles),
            off = (i % 2 ? 1 : -1) * amp;
          ctx.lineTo(a[0] + v[0] * f - uy * off, a[1] + v[1] * f + ux * off);
        }
        ctx.lineTo(...b);
        ctx.stroke();
      }
      arm("J3", "J4", "#233c44", "#415963", 5);
      arm("J4", "JX", "#213940", "#3e5861", 7);
      arm("JX", "EE", "#213940", "#3e5861", 7);
      arm("EE", "J8", "#213940", "#3e5861", 6);
      zig("M1", "JX", "#476068");
      zig("M2", "J5", "#19779b");
      zig("J5", "J6", "#19779b");
      zig("J7", "J8", "#19779b");
      arm("JX", "J6", "#155d88", "#3887ad", 5.5);
      arm("JX", "J7", "#155d88", "#3887ad", 5.5);
      const end = xy(z.EE),
        length = 28;
      line(end, [end[0] + length * Math.cos(state.phi), end[1] - length * Math.sin(state.phi)], "#a85d23", 2);
      for (const k of ["J3", "M1", "M2"]) {
        const [x, y] = xy(z[k]);
        line([x - 11, y + 7], [x + 11, y + 7], "#50676a", 2);
        for (const t of [-8, -2, 4, 10]) line([x + t, y + 8], [x + t - 3, y + 12], "#718789", 1);
      }
      const font = width < 485 ? 11 : 13;
      const offsets = {
        J3: [-6, 26],
        M2: [-2, 26],
        M1: [0, 26],
        J4: [-6, -12],
        J5: [-10, -10],
        J6: [-11, -10],
        JX: [0, 22],
        J7: [7, -10],
        J8: [7, -10],
        EE: [6, 24]
      };
      for (const k of pointNames) {
        const P = xy(z[k]),
          isBlue = ["M2", "J5", "J6", "J7", "J8"].includes(k),
          col = isBlue ? "#176a97" : "#29434a";
        if (k === "EE") {
          circle(P, 12, "#f7d4a98a", "#ae702e", 1);
          circle(P, 7, "#fffaec", "#985b27", 2.3);
          circle(P, 2.8, "#935322");
        } else {
          circle(P, k === "JX" ? 7.2 : 4.8, "#f9fbf8", col, 2);
          if (["J3", "M1", "M2", "JX"].includes(k)) circle(P, 2.2, col);
        }
        const off = offsets[k];
        ctx.textAlign = "center";
        ctx.font = "bold " + font + "px monospace";
        ctx.strokeStyle = "#f9fbf7";
        ctx.lineWidth = 3.6;
        ctx.strokeText(k, P[0] + off[0], P[1] + off[1]);
        ctx.fillStyle = k === "EE" ? "#87521c" : col;
        ctx.fillText(k, P[0] + off[0], P[1] + off[1]);
      }
      if ($("numericOverlay").checked && comparison?.ok && matchesNumeric(comparison)) {
        for (const k of pointNames) {
          const [x, y] = xy(comparison.points[k]);
          ctx.strokeStyle = "#b42c80";
          ctx.lineWidth = 1;
          ctx.strokeRect(x - 4, y - 4, 8, 8);
        }
      }
      if ($("linkLabels").checked) {
        for (const [name, a, b] of spanDefs) {
          const A = xy(z[a]),
            B = xy(z[b]),
            m = [(A[0] + B[0]) / 2, (A[1] + B[1]) / 2 - 7];
          ctx.font = "10px monospace";
          ctx.textAlign = "center";
          ctx.lineWidth = 3;
          ctx.strokeStyle = "#f9fbf7";
          ctx.strokeText(name, ...m);
          ctx.fillStyle = "#405963";
          ctx.fillText(name, ...m);
        }
      }
      if ($("crossings").checked) for (const hit of crossingInfo) {
        const [x, y] = xy(hit.xy);
        circle([x, y], 6, null, "#b64e46", 1.4);
        line([x - 2, y - 2], [x + 2, y + 2], "#b64e46");
        line([x - 2, y + 2], [x + 2, y - 2], "#b64e46");
      }
    } else {
      ctx.fillStyle = "#805039";
      ctx.font = "600 14px system-ui";
      ctx.textAlign = "center";
      ctx.fillText(state.code === "continuum" ? "A free closure angle is required" : state.code === "crossing" ? "Projected crossing excluded" : "No closed assembly at these inputs", width / 2, height / 2);
    }
    if (target) {
      const a = xy(target);
      if (state.ok && dist(state.points.EE, target) * scale > 3) line(a, xy(state.points.EE), "#ad5d38", 1, [3, 3]);
      line([a[0] - 5, a[1] - 5], [a[0] + 5, a[1] + 5], "#a3472a", 1.8);
      line([a[0] - 5, a[1] + 5], [a[0] + 5, a[1] - 5], "#a3472a", 1.8);
    }
    $("plotInfo").textContent = ($("workspace").checked ? "Dashed amber · sampled EE locus at current M2 and branch. Diamonds · detected closure tangencies. " : "") + ($("crossings").checked ? crossingInfo.length + " projected centerline crossing(s). " : "") + "Drag EE or click a target.";
  }
  function jointsPanel() {
    if (!state.ok) {
      $("joints").innerHTML = '<p class="warning">' + esc(state.reason) + "</p>";
      return;
    }
    const z = state.points,
      jointRows = pointNames.map(k => [k, fmt(z[k][0]), fmt(z[k][1])]);
    const memberRows = spanDefs.map(([k, a, b]) => [k, a + " → " + b, fmt(dist(z[a], z[b])), dist(z[a], z[b]) ? fmt(deg(direction(z[a], z[b])), 2) : "undefined"]);
    const joints = [["J4", "J3", "J4", "JX"], ["J5", "M2", "J5", "J6"], ["J6", "J5", "J6", "JX"], ["J7", "JX", "J7", "J8"], ["J8", "J7", "J8", "EE"], ["EE", "JX", "EE", "J8"], ["JX · black", "J4", "JX", "EE"], ["JX · blue", "J6", "JX", "J7"], ["JX · input / black", "M1", "JX", "J4"], ["JX · black / blue", "J4", "JX", "J6"]];
    const turns = joints.map(([k, a, b, c]) => [k, a + " → " + b + " → " + c, dist(z[a], z[b]) && dist(z[b], z[c]) ? fmt(deg(wrap(direction(z[b], z[c]) - direction(z[a], z[b]))), 3) : "undefined"]);
    const expanded = $("joints").querySelector("details")?.open;
    $("joints").innerHTML = '<div class="data-grid"><div><h3>Joint coordinates</h3>' + table(["Joint", "X (mm)", "Y (mm)"], jointRows) + "</div><div><h3>Directed members</h3>" + table(["Link", "Direction", "mm", "Angle (°)"], memberRows) + "</div></div><details><summary>Joint angles and actuator inputs</summary>" + table(["Joint", "Incoming → outgoing", "Signed turn (°)"], turns) + table(["Drive", "Quantity", "Value"], [["M1", "Global angle (°)", fmt(deg(q1), 5)], ["M2", p.mode === "rotary" ? "Global angle (°)" : "Signed travel (mm)", fmt(p.mode === "rotary" ? deg(q2) : q2, 5)], ["EE", "Tool angle (°)", fmt(deg(state.phi), 5)]]) + "</details>";
    $("joints").querySelector("details").open = Boolean(expanded);
  }

  // The inspector accepts derivatives in the engine's native angular units.
  function rateFields() {
    const units = p.mode === "rotary" ? ["rad/s", "rad/s²"] : ["mm/s", "mm/s²"];
    const names = ["M1 rate · rad/s", "M2 rate · " + units[0], "M1 accel. · rad/s²", "M2 accel. · " + units[1], "Force X · N", "Force Y · N", "Moment · N·mm"];
    const old = rateKeys.map(k => $(k)?.value || 0);
    $("rateFields").innerHTML = '<div class="intro"><h2>Differential kinematics</h2><p>Set instantaneous input derivatives and an external wrench at EE. Angular quantities in this panel use radians.</p></div><div class="rategrid">' + rateKeys.map((k, i) => "<label>" + names[i] + '<input id="' + k + '" type="number" step="any" value="' + esc(old[i]) + '"></label>').join("") + "</div>";
    rateKeys.forEach(k => $(k).addEventListener("input", () => {
      diff = null;
      motionPanel();
    }));
  }
  function getDiff() {
    if (!diff) {
      const a = rateKeys.map(k => $(k).valueAsNumber);
      if (!a.every(Number.isFinite)) return null;
      diff = D.evaluate(p, state, {
        rates: a.slice(0, 2),
        accelerations: a.slice(2, 4),
        wrench: a.slice(4)
      });
    }
    return diff;
  }
  function motionPanel() {
    if (!state.ok) {
      $("motionData").innerHTML = '<p class="notice warn">' + esc(state.reason) + "</p>";
      return;
    }
    const d = getDiff();
    if (!d) {
      $("motionData").innerHTML = '<p class="notice warn">Enter finite rates and wrench components.</p>';
      return;
    }
    const block = (title, body) => '<div class="block"><h3>' + esc(title) + "</h3>" + body + "</div>";
    const matList = entries => '<div class="matrix-grid">' + entries.map(([k, v]) => matrix(k, v)).join("") + "</div>";
    const details = (title, body) => "<details><summary>" + esc(title) + "</summary>" + body + "</details>";
    let html = block("Constraint diagnostics", table(["Dependent rank / 3", "Full rank / 3", "Linearized mobility", "Locked nullity", "Pose rank"], [[d.rankH, d.rankConstraint, d.mobilityLinearized, d.lockedNullity, d.poseRank ?? "—"]]) + '<p class="subtle">' + (d.nearSingular ? "Near a closure singularity. " : "") + esc(d.reason || "") + "</p>");
    if (d.regular) {
      html += block("Velocity and inverse mappings", matList([["Pose Jacobian · [x, y, φ] from q̇", d.poseJacobian], ["Pose left inverse · compatible twists", d.poseLeftInverse], ["Reduced Jacobian · [tangent speed, φ̇]", d.reducedJacobian], ["Reduced inverse", d.inverseReduced], ["Dependent angular Jacobian", d.dependentJacobian], ["XY pseudoinverse · minimum norm", d.positionPseudoInverse]]));
      html += block("Instantaneous output", table(["Quantity", "X", "Y", "φ"], [["Velocity · mm/s, rad/s", ...d.poseVelocity.map(precise)], ["Acceleration · mm/s², rad/s²", ...d.poseAcceleration.map(precise)]]));
      html += details("All joint velocities and accelerations", table(["Joint", "Vx", "Vy", "Ax", "Ay"], pointNames.map(k => [k, ...d.jointVelocity[k].map(precise), ...d.jointAcceleration[k].map(precise)])));
      html += details("Angular motion of each member", table(["Member", "ω (rad/s)", "α (rad/s²)"], Object.entries(d.memberMotion).map(([k, m]) => [k, precise(m.omega), precise(m.angularAcceleration)])));
      html += details("Joint and member velocity Jacobians", matList(Object.entries(d.pointJacobians)) + matList(Object.entries(d.memberJacobians).map(([k, v]) => [k, v ? [v] : null])));
      html += details("Forces and velocity amplification", table(["Quantity", "M1", "M2"], [["External generalized effort", ...d.effort.map(precise)], ["Balancing actuator effort", ...d.balancingEffort.map(precise)]]) + '<p class="subtle">Rotary effort in N·mm, slider effort in N. Span force is positive in tension. Gains use model length ' + fmt(d.characteristicLength, 2) + " mm for scaling.</p>" + table(["Spanning link", "Axial force (N)"], Object.entries(d.spanAxialForces).map(([k, v]) => [k, precise(v)])) + matList([["Wrench → generalized effort", d.effortMap], ["Inverse reduced effort map", d.inverseEffortMap], ["Normalized velocity singular values", [d.reducedSingularValues]], ["Normalized force gains", d.normalizedForceGains ? [d.normalizedForceGains] : null]]));
      html += details("Dimensional sensitivity at fixed drives", table(["Parameter", "∂x / ∂p", "∂y / ∂p", "∂φ / ∂p"], Object.entries(d.sensitivity).map(([k, v]) => [k, ...v.map(precise)])));
    }
    html += details("Constraint Jacobians and derivatives", matList([["Dependent block H", d.H], ["Input block B", d.B], ["Full Jacobian [H B]", d.constraintJacobian], ["Normalized tangency indicators", [d.tangency]], ["Constraint rates", d.constraintRates ? [d.constraintRates] : null], ["Constraint accelerations", d.constraintAccelerations ? [d.constraintAccelerations] : null]]));
    const currentFrame = $("frameSelect")?.value || "tool";
    html += block("Homogeneous frame transforms", '<label>Local frame <select class="text-select" style="width:auto" id="frameSelect">' + Object.keys(d.transforms).map(k => "<option " + (k === currentFrame ? "selected" : "") + ">" + esc(k) + "</option>").join("") + '</select></label><div id="frameData" class="block"></div>');
    const expanded = new Set([...$("motionData").querySelectorAll("details[open] > summary")].map(item => item.textContent));
    $("motionData").innerHTML = html;
    for (const detail of $("motionData").querySelectorAll("details")) {
      detail.open = expanded.has(detail.querySelector("summary").textContent);
    }
    const frames = () => {
      const a = d.transforms[$("frameSelect").value];
      $("frameData").innerHTML = matList([["Local → world", a?.forward], ["World → local", a?.inverse]]);
    };
    $("frameSelect").onchange = frames;
    frames();
  }
  function renderGeo() {
    const form = $("geometryForm");
    const modeRot = p.mode === "rotary";
    const angles = key => angleKeys.includes(key) || modeRot && ["q2min", "q2max"].includes(key);
    function field([key, label]) {
      const v = p[key],
        value = v == null ? "" : angles(key) ? deg(v).toPrecision(16) : v;
      return "<label>" + esc(label) + '<input type="number" step="any" name="' + key + '" value="' + esc(value) + '" ' + (key.endsWith("min") || key.endsWith("max") ? 'placeholder="unbounded"' : "required") + "></label>";
    }
    const section = (title, arr) => "<fieldset><legend>" + title + '</legend><div class="geogrid">' + arr.map(field).join("") + "</div></fieldset>";
    form.innerHTML = '<label>M2 drive model<select name="mode" style="width:auto;max-width:300px"><option value="rotary" ' + (modeRot ? "selected" : "") + '>Rotary crank</option><option value="slider" ' + (!modeRot ? "selected" : "") + ">Prismatic guide</option></select></label>" + section("Ground points · mm", [["j3x", "J3 · x"], ["j3y", "J3 · y"], ["m1x", "M1 · x"], ["m1y", "M1 · y"], ["m2x", "M2 · x"], ["m2y", "M2 · y"]]) + section("Member dimensions · mm", labels) + section("Fixed angular offsets · degrees", [["delta", "J4→X to X→EE"], ["beta", "X→J6 to X→J7"], ["tool", "E→J8 to tool axis"], ["axis", "M2 slider guide"]]) + section("Optional operating bounds", [["q1min", "M1 lower · °"], ["q1max", "M1 upper · °"], ["q2min", "M2 lower"], ["q2max", "M2 upper"]]) + '<p class="subtle">M2 bounds use degrees for a rotary crank and mm for a slider. Blank bounds allow unrestricted input. Slider controls show an illustrative range; type any admissible travel.</p><button class="primary" type="submit">Apply geometry</button><span class="formmsg" id="geomsg" role="status"></span>';
  }
  function applyGeo(e) {
    e.preventDefault();
    stopPlay();
    const form = $("geometryForm"),
      data = new FormData(form),
      np = {
        ...p
      };
    for (const [key, val] of data) {
      if (key === "mode") {
        np.mode = val;
        continue;
      }
      const raw = val.trim();
      np[key] = raw === "" ? null : Number(raw);
      if (np[key] != null && (angleKeys.includes(key) || data.get("mode") === "rotary" && ["q2min", "q2max"].includes(key))) np[key] *= rad;
    }
    const reason = K.validParams(np);
    if (reason) {
      $("geomsg").textContent = reason;
      return;
    }
    if (p.mode !== np.mode) {
      q2 = np.mode === "rotary" ? p.axis : np.r2 * Math.cos(q2 - np.axis);
      for (const k of ["dq2", "ddq2"]) $(k).value = 0;
    }
    p = np;
    traceBreak();
    clearTarget();
    sampleKey = "";
    camera = null;
    rateFields();
    renderGeo();
    refresh();
    $("geomsg").textContent = state.ok ? "Geometry applied." : "Geometry applied. " + state.reason;
  }
  // Overlay only a numerically converged realization of the displayed assembly.
  function matchesNumeric(r) {
    return r.ok && r.branch.every((s, i) => state.signs[i] === 0 || s === state.signs[i]) && r.difference <= 10 * K.tolerance(p);
  }
  function crossingFilters() {
    let items = "";
    for (let i = 0; i < spanDefs.length; i++) for (let j = i + 1; j < spanDefs.length; j++) {
      const a = spanDefs[i],
        b = spanDefs[j];
      if ([a[1], a[2]].some(n => n === b[1] || n === b[2])) continue;
      const name = a[0] + " × " + b[0];
      items += '<label class="check"><input type="checkbox" data-block="' + esc(name) + '" ' + (blockedPairs.has(name) ? "checked" : "") + ">" + esc(name) + "</label>";
    }
    return '<details id="exclusions"><summary>Optional centerline crossing exclusions</summary><p class="subtle">Select pairs that must not cross in projection. This workbench filter applies only to strict interior intersections. Inverse-family ranges use the ideal unrestricted layer ordering.</p><div class="exclusion-grid">' + items + "</div></details>";
  }
  function checksPanel() {
    $("compare").disabled = !state.ok;
    let out = "";
    if (comparison) {
      const r = comparison;
      out = table(["Result", "Iterations", "Max Cartesian residual · mm", "Max point difference · mm", "Branch"], [[r.reason, r.iterations ?? "—", precise(r.maxResidual), precise(r.difference), r.branch?.join(" / ") || "—"]]);
      if (r.ok && !matchesNumeric(r)) out += '<p class="warning">Converged to a distinct assembly or point discrepancy exceeds overlay tolerance. Overlay withheld.</p>';
    }
    $("compareData").innerHTML = out;
    const notes = '<p class="subtle">The primary model treats the zigzag paths as effective fixed spans, with separate black and blue bodies rotating at JX. The right-hand dogleg has no enforced ground joint. EE xy lies on a curve; a reachable pose can also control tool orientation. Projected crossings identify possible interference because the drawing supplies no thickness or layer ordering.</p>';
    let html = '<div class="block"><h3>Model assumptions</h3>' + notes + "</div>" + crossingFilters();
    if (state.ok) html += '<div class="block"><h3>Analytical residuals</h3><p class="subtle">Acceptance tolerance ' + precise(K.tolerance(p)) + " mm.</p>" + table(["Constraint", "Residual (mm)"], Object.entries(state.residuals).map(([k, v]) => [k, precise(v)])) + '</div><div class="block"><h3>Projected centerline crossings</h3>' + (crossingInfo.length ? table(["Members", "X", "Y"], crossingInfo.map(c => [c.pair, ...c.xy.map(n => fmt(n))])) : '<p class="subtle">No strict interior crossings detected.</p>') + "</div>";else html += '<p class="notice warn">' + esc(state.reason) + "</p>";
    const wasOpen = $("exclusions")?.open;
    $("checkData").innerHTML = html;
    $("exclusions").open = Boolean(wasOpen);
    for (const input of document.querySelectorAll("[data-block]")) input.onchange = () => {
      stopPlay();
      if (input.checked) blockedPairs.add(input.dataset.block);else blockedPairs.delete(input.dataset.block);
      traceBreak();
      clearTarget();
      refresh();
    };
  }
  function drawInspection() {
    if (!state) return;
    if (tab === "joints") jointsPanel();
    if (tab === "motion") motionPanel();
    if (tab === "checks") checksPanel();
  }
  function screenPoint(e) {
    const rect = canvas.getBoundingClientRect();
    return map.world(e.clientX - rect.left, e.clientY - rect.top);
  }
  function projectionDomain() {
    // Find the connected feasible interval surrounding the displayed input.
    const dq = tau / 360,
      start = q1,
      rows = [{
        q: start,
        pt: state.points.EE
      }];
    for (const sign of [-1, 1]) {
      let last = start;
      for (let i = 1; i <= 180; i++) {
        const trial = start + sign * dq * i,
          s = allowedForward(p, trial, q2, branch);
        if (s.ok) {
          rows.push({
            q: trial,
            pt: s.points.EE
          });
          last = trial;
          continue;
        }
        let inside = last,
          outside = trial;
        for (let k = 0; k < 32; k++) {
          const mid = (inside + outside) / 2;
          if (allowedForward(p, mid, q2, branch).ok) inside = mid;else outside = mid;
        }
        const boundary = allowedForward(p, inside, q2, branch);
        if (boundary.ok) rows.push({
          q: inside,
          pt: boundary.points.EE
        });
        break;
      }
    }
    return rows.sort((a, b) => a.q - b.q);
  }
  function projectTarget(point) {
    const domain = drag.domain;
    if (!domain.length) return;
    let best = 0,
      score = Infinity;
    for (let i = 0; i < domain.length; i++) {
      const v = dist(domain[i].pt, point);
      if (v < score) {
        score = v;
        best = i;
      }
    }
    let lo = domain[Math.max(0, best - 1)].q,
      hi = domain[Math.min(domain.length - 1, best + 1)].q;
    let qbest = domain[best].q;
    function objective(q) {
      const s = allowedForward(p, q, q2, branch);
      if (!s.ok) return Infinity;
      const d = dist(s.points.EE, point);
      if (d < score) {
        score = d;
        qbest = q;
      }
      return d;
    }
    for (let i = 0; i < 30; i++) {
      const a = (2 * lo + hi) / 3,
        b = (lo + 2 * hi) / 3;
      if (objective(a) < objective(b)) hi = b;else lo = a;
    }
    q1 = qbest;
    target = point;
    say("Snapped to the reachable path at fixed M2. Pointer offset " + fmt(score, 3) + " mm.");
    refresh({
      keepMessage: true
    });
    if (controlMode === "inverse" && state.ok) fillTarget($("orient").checked);
  }
  canvas.addEventListener("pointerdown", e => {
    if (!map || e.button !== 0) return;
    stopPlay();
    e.preventDefault();
    const point = screenPoint(e);
    const project = state.ok && $("snap").checked;
    drag = {
      id: e.pointerId,
      domain: project ? projectionDomain() : null
    };
    canvas.setPointerCapture(e.pointerId);
    if (project) {
      clearTarget();
      projectTarget(point);
    } else {
      setMode("inverse");
      clearTarget();
      target = point;
      $("tx").value = point[0].toPrecision(16);
      $("ty").value = point[1].toPrecision(16);
      say("Release to solve for this exact target.");
      queueDraw();
    }
  });
  canvas.addEventListener("pointermove", e => {
    if (!drag || e.pointerId !== drag.id) return;
    const pos = screenPoint(e);
    if (drag.domain) {
      projectTarget(pos);
    } else {
      target = pos;
      $("tx").value = pos[0].toPrecision(16);
      $("ty").value = pos[1].toPrecision(16);
      queueDraw();
    }
  });
  function endDrag(e) {
    if (!drag || e.pointerId !== drag.id) return;
    const exact = !drag.domain;
    drag = null;
    if (exact && target) solveTarget(...target, $("orient").checked ? $("tphi").valueAsNumber * rad : undefined);
  }
  canvas.addEventListener("pointerup", endDrag);
  canvas.addEventListener("pointercancel", () => {
    drag = null;
  });
  $("forwardMode").onclick = () => setMode("forward");
  $("inverseMode").onclick = () => setMode("inverse");
  function targetEdited() {
    clearTarget();
    say("Choose Solve exact target to apply these coordinates.");
    queueDraw();
  }
  for (const id of ["tx", "ty", "tphi"]) $(id).oninput = targetEdited;
  $("orient").onchange = () => {
    $("angleLabel").hidden = !$("orient").checked;
    targetEdited();
  };
  $("solve").onclick = () => solveTarget($("tx").valueAsNumber, $("ty").valueAsNumber, $("orient").checked ? $("tphi").valueAsNumber * rad : undefined);
  $("ikResults").addEventListener("click", e => {
    if (e.target.dataset.sol != null) {
      const i = Number(e.target.dataset.sol);
      say("Selected inverse realization " + (i + 1) + ".");
      adopt(ikList[i]);
    }
  });
  for (const i of [1, 2]) {
    $("q" + i).oninput = e => setDrive(i, e.target.valueAsNumber);
    $("q" + i + "n").onchange = e => setDrive(i, e.target.valueAsNumber);
  }
  [4, 6, 8].forEach((k, i) => $("s" + k).onchange = e => {
    stopPlay();
    branch[i] = Number(e.target.value);
    traceBreak();
    clearTarget();
    refresh();
  });
  $("fit").onclick = () => fit(true);
  for (const k of ["axes", "workspace", "crossings", "linkLabels", "trace"]) $(k).onchange = () => {
    if (k === "trace") trail = state.ok && $("trace").checked ? [state.points.EE] : [];
    queueDraw();
  };
  document.querySelectorAll("[data-tab]").forEach(b => b.onclick = () => changeTab(b.dataset.tab));
  $("geometryForm").onsubmit = applyGeo;
  $("compare").onclick = () => {
    if (state.ok) {
      comparison = N.solve(p, q1, q2, state.points);
      checksPanel();
      queueDraw();
    }
  };
  $("numericOverlay").onchange = () => {
    if ($("numericOverlay").checked && !comparison && state.ok) comparison = N.solve(p, q1, q2, state.points);
    checksPanel();
    queueDraw();
  };
  $("reset").onclick = () => {
    stopPlay();
    blockedPairs.clear();
    home();
    clearTarget();
    rateKeys.forEach(k => $(k).value = 0);
    rateFields();
    renderGeo();
    refresh();
    setMode("forward");
    say("Restored the illustrative model and assembly.");
  };
  function stopPlay(text) {
    if (!playback) return;
    cancelAnimationFrame(playback.frame);
    playback = null;
    $("play").setAttribute("aria-pressed", "false");
    $("play").textContent = "▶ Play motor sweep";
    for (const k of rateKeys.slice(0, 4)) {
      $(k).value = 0;
      $(k).readOnly = false;
    }
    diff = null;
    if (tab === "motion") motionPanel();
    if (text) say(text);
  }
  // q(t) = q(0) + A(1−cos(ωt)). Both derivatives use this simulation clock.
  function startPlay() {
    if (controlMode !== "forward") return;
    if (!state.ok) {
      say("Choose a closed assembly before playing motion.", true);
      return;
    }
    clearTarget();
    const amplitudes = [3 * rad, p.mode === "rotary" ? -3 * rad : -0.018 * p.r2],
      omegas = [tau / 8, tau / 6];
    playback = {
      initial: [q1, q2],
      last: performance.now(),
      t: 0,
      frame: 0
    };
    for (const k of rateKeys.slice(0, 4)) $(k).readOnly = true;
    $("play").setAttribute("aria-pressed", "true");
    $("play").textContent = "Ⅱ Pause motor sweep";
    say("Playing a cosine sweep from this pose. Both input derivatives are shown in Motion matrices.");
    function frame(now) {
      const run = playback;
      if (!run) return;
      const elapsed = (now - run.last) / 1000;
      if (elapsed >= 1 / 30) {
        run.t += Math.min(0.05, elapsed);
        run.last = now;
        const values = run.initial.map((start, i) => start + amplitudes[i] * (1 - Math.cos(omegas[i] * run.t)));
        const candidate = allowedForward(p, values[0], values[1], branch);
        if (!candidate.ok) {
          stopPlay();
          say("Playback stopped at the last valid pose. Rejected M1 " + fmt(deg(values[0]), 3) + "°, M2 " + fmt(p.mode === "rotary" ? deg(values[1]) : values[1], 3) + (p.mode === "rotary" ? "°. " : " mm. ") + candidate.reason, true);
          return;
        }
        q1 = values[0];
        q2 = values[1];
        for (let i = 0; i < 2; i++) {
          $(rateKeys[i]).value = amplitudes[i] * omegas[i] * Math.sin(omegas[i] * run.t);
          $(rateKeys[i + 2]).value = amplitudes[i] * omegas[i] ** 2 * Math.cos(omegas[i] * run.t);
        }
        refresh({
          keepMessage: true
        });
      }
      if (playback) playback.frame = requestAnimationFrame(frame);
    }
    playback.frame = requestAnimationFrame(frame);
  }
  $("play").onclick = () => playback ? stopPlay("Playback paused.") : startPlay();
  document.addEventListener("visibilitychange", () => {
    if (document.hidden && playback) stopPlay("Playback paused while this window was hidden.");
  });
  for (const n of [4, 6, 8]) $("free" + n).onchange = e => {
    stopPlay();
    const v = e.target.valueAsNumber;
    if (Number.isFinite(v)) freeAngles["J" + n] = v * rad;else delete freeAngles["J" + n];
    sampleKey = "";
    traceBreak();
    clearTarget();
    refresh();
  };
  new ResizeObserver(() => queueDraw()).observe(canvas);
  home();
  rateFields();
  renderGeo();
  refresh({
    record: false
  });
  setMode("forward");
})();
