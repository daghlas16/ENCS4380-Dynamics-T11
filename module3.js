/*
  Dynamics Lab — Module 3: Second-order time response
  ---------------------------------------------------------
  Mass–spring–damper system:
  m*x'' + b*x' + k*x = F(t)

  Transfer function:
  G(s) = (1/k) / ((m/k)s² + (b/k)s + 1)

  Damping ratio:       ζ = b / (2√(km))
  Natural frequency:   ωn = √(k/m)
  Damped frequency:    ωd = ωn√(1 - ζ²), for ζ < 1

  Includes step response, system metrics, poles, phase plane,
  state-space representation, prediction questions and validation.

  Pure JavaScript and SVG, no server or chart library needed.
*/

(function () {
  'use strict';

  // --- 1. Mathematical formulas and system calculations ---
  const EPS = 1e-9;          // tolerance for "ζ is exactly 1" inside the formulas
  const CRIT_TOL = 0.005;    // |ζ − 1| below this is *labelled* "critically damped"
  const PI = Math.PI;

  // Slider ranges. State is kept as exact numbers (presets are not rounded to the slider step).
  const RANGE = {
    m: { min: 0.5, max: 4,  step: 0.1,  def: 1, unit: 'kg',    dec: 1 },
    b: { min: 0.2, max: 20, step: 0.01, def: 2, unit: 'N·s/m', dec: 2 },
    k: { min: 1,   max: 20, step: 0.1,  def: 4, unit: 'N/m',   dec: 1 }
  };

  // m x'' + b x' + k x = F(t)   →   G(s) = Kωn² / (s² + 2ζωn s + ωn²),  K = 1/k
  function derive(m, b, k) {
    return { m: m, b: b, k: k, wn: Math.sqrt(k / m), zeta: b / (2 * Math.sqrt(k * m)), K: 1 / k };
  }

  function classify(zeta) {
    if (Math.abs(zeta - 1) <= CRIT_TOL) return 'crit';
    return zeta < 1 ? 'under' : 'over';
  }

  function dampedFreq(zeta, wn) { return zeta < 1 ? wn * Math.sqrt(1 - zeta * zeta) : 0; }

  // Poles of G(s): roots of s² + 2ζωn s + ωn²
  function poles(zeta, wn) {
    if (zeta < 1 - EPS) {
      const wd = dampedFreq(zeta, wn);
      return [{ re: -zeta * wn, im: wd }, { re: -zeta * wn, im: -wd }];
    }
    if (zeta <= 1 + EPS) return [{ re: -wn, im: 0 }, { re: -wn, im: 0 }];
    const r = Math.sqrt(zeta * zeta - 1);
    return [{ re: -wn / (zeta + r), im: 0 }, { re: -wn * (zeta + r), im: 0 }]; // slow pole, fast pole
  }

  // Decay rate of the slowest mode (1/time-constant of the envelope)
  function slowestRate(zeta, wn) {
    if (zeta < 1 - EPS) return zeta * wn;
    if (zeta <= 1 + EPS) return wn;
    return wn / (zeta + Math.sqrt(zeta * zeta - 1));
  }

  // Unit-step response of the NORMALISED system (final value 1). Closed forms from the lecture.
  function stepNorm(zeta, wn, t) {
    if (zeta < 1 - EPS) {                                   // underdamped
      const s = zeta * wn, wd = dampedFreq(zeta, wn);
      return 1 - Math.exp(-s * t) * (Math.cos(wd * t) + (s / wd) * Math.sin(wd * t));
    }
    if (zeta <= 1 + EPS) return 1 - (1 + wn * t) * Math.exp(-wn * t);   // critically damped
    const p = poles(zeta, wn), p1 = p[0].re, p2 = p[1].re;               // overdamped
    return 1 + (p2 * Math.exp(p1 * t) - p1 * Math.exp(p2 * t)) / (p1 - p2);
  }

  // Derivative of the normalised step response (used for the phase plane: x2 = velocity)
  function velNorm(zeta, wn, t) {
    if (zeta < 1 - EPS) {
      const wd = dampedFreq(zeta, wn);
      return (wn * wn / wd) * Math.exp(-zeta * wn * t) * Math.sin(wd * t);
    }
    if (zeta <= 1 + EPS) return wn * wn * t * Math.exp(-wn * t);
    const p = poles(zeta, wn), p1 = p[0].re, p2 = p[1].re;
    return (p1 * p2 * (Math.exp(p1 * t) - Math.exp(p2 * t))) / (p1 - p2);
  }

  // Sample the response long enough to be fully settled (14 slow time-constants), finely enough to read the peak.
  function simulate(p) {
    const tEnd = 14 / slowestRate(p.zeta, p.wn);
    const wd = dampedFreq(p.zeta, p.wn);
    let dt = tEnd / 4000;
    if (wd > 0) dt = Math.min(dt, (2 * PI / wd) / 400);
    const n = Math.min(20000, Math.ceil(tEnd / dt)) + 1;
    const t = new Float64Array(n), y = new Float64Array(n);
    for (let i = 0; i < n; i++) { t[i] = (tEnd * i) / (n - 1); y[i] = stepNorm(p.zeta, p.wn, t[i]); }
    return { t: t, y: y };
  }

  // Response over a chosen display window (also gives velocity for the phase plane)
  function curve(p, T, n) {
    const t = new Float64Array(n), y = new Float64Array(n), v = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      t[i] = (T * i) / (n - 1);
      y[i] = stepNorm(p.zeta, p.wn, t[i]);
      v[i] = velNorm(p.zeta, p.wn, t[i]);
    }
    return { t: t, y: y, v: v, T: T };
  }

  // Read the metrics OFF THE SAMPLED CURVE (y normalised so the final value is 1)
  function analyze(t, y) {
    const n = t.length;
    let im = 0;
    for (let i = 1; i < n; i++) if (y[i] > y[im]) im = i;
    let peakT = t[im], peakY = y[im];
    if (im > 0 && im < n - 1) {                       // parabolic refinement around the top sample
      const y0 = y[im - 1], y1 = y[im], y2 = y[im + 1], den = y0 - 2 * y1 + y2;
      if (den < 0) {
        const d = (0.5 * (y0 - y2)) / den;
        peakT = t[im] + d * (t[1] - t[0]);
        peakY = y1 - 0.25 * (y0 - y2) * d;
      }
    }
    const overshoot = (peakY - 1) * 100;
    const hasPeak = overshoot > 1e-3 && im < n - 1;

    const cross = function (level) {                   // first time the curve reaches `level`
      for (let i = 1; i < n; i++) {
        if (y[i] >= level) return t[i - 1] + ((level - y[i - 1]) / (y[i] - y[i - 1])) * (t[i] - t[i - 1]);
      }
      return null;
    };
    const t10 = cross(0.1), t90 = cross(0.9);

    // 2 % settling time = last time the curve is outside the ±2 % band
    const band = 0.02;
    let last = -1;
    for (let i = n - 1; i >= 0; i--) if (Math.abs(y[i] - 1) > band) { last = i; break; }
    let settleT = null;
    if (last < n - 1) {
      if (last < 0) settleT = 0;
      else {
        const e0 = Math.abs(y[last] - 1) - band, e1 = Math.abs(y[last + 1] - 1) - band;
        settleT = t[last] + ((t[last + 1] - t[last]) * e0) / (e0 - e1);
      }
    }
    return {
      hasPeak: hasPeak,
      overshoot: hasPeak ? overshoot : 0,
      peakT: hasPeak ? peakT : null,
      peakY: hasPeak ? peakY : 1,
      t10: t10, t90: t90,
      riseT: t10 != null && t90 != null ? t90 - t10 : null,
      settleT: settleT
    };
  }

  // Textbook overshoot, shown beside the number read off the curve
  function theoryOvershoot(zeta) {
    return zeta < 1 ? 100 * Math.exp((-PI * zeta) / Math.sqrt(1 - zeta * zeta)) : 0;
  }

  // Eigenvalues of a 2×2 real matrix [[a, b], [c, d]] (trace / determinant formula)
  function eig2(a, b, c, d) {
    const tr = a + d, det = a * d - b * c;
    let disc = (tr * tr) / 4 - det;
    if (Math.abs(disc) <= 1e-12 * ((tr * tr) / 4 + Math.abs(det))) disc = 0;   // kill rounding noise at ζ = 1
    if (disc < 0) { const w = Math.sqrt(-disc); return [{ re: tr / 2, im: w }, { re: tr / 2, im: -w }]; }
    const r = Math.sqrt(disc);
    return [{ re: tr / 2 + r, im: 0 }, { re: tr / 2 - r, im: 0 }];
  }

  // State space: x1 = x, x2 = x' ;  x' = A x + B F ;  y = C x
  function stateSpace(p) {
    const A = [[0, 1], [-p.k / p.m, -p.b / p.m]];
    const B = [0, 1 / p.m];
    return { A: A, B: B, eig: eig2(A[0][0], A[0][1], A[1][0], A[1][1]) };
  }

  function samePoles(e, q, tol) {
    const key = function (list) {
      return list.slice().sort(function (u, v) { return u.re - v.re || u.im - v.im; });
    };
    const a = key(e), b = key(q);
    return a.every(function (z, i) { return Math.abs(z.re - b[i].re) < tol && Math.abs(z.im - b[i].im) < tol; });
  }

  // Overshoot at the very top of the visible window (T) – window length for plotting
  function windowFor(p, a) {
    let T = 1.35 * (a.settleT != null ? a.settleT : 14 / slowestRate(p.zeta, p.wn));
    if (a.hasPeak) T = Math.max(T, 1.4 * a.peakT);
    return Math.max(T, 0.2);
  }

  function plotPoints(p, T) {
    const wd = dampedFreq(p.zeta, p.wn);
    const n = wd > 0 ? Math.min(4000, Math.max(500, Math.ceil(((T * wd) / (2 * PI)) * 24))) : 600;
    return curve(p, T, n);
  }

  // The three homework validation checks that belong to Module 3 (tests 4, 5, 6)
  function runValidation() {
    const rows = [];
    const MINUS = '−';
    const fmt = function (v, d) { const s = v.toFixed(d); return (Number(s) === 0 ? (0).toFixed(d) : s).replace('-', MINUS); };

    let p = derive(1, 2, 4);                                  // ζ = 0.5, ωn = 2 rad/s
    let s = simulate(p), a = analyze(s.t, s.y);
    rows.push({
      test: 'ζ = 0.5 → overshoot', expected: '16.3 %', calculated: fmt(a.overshoot, 2) + ' %',
      pass: Math.abs(a.overshoot - 16.3) < 0.05
    });

    p = derive(1, 4, 4);                                      // ζ = 1
    s = simulate(p); a = analyze(s.t, s.y);
    rows.push({
      test: 'ζ = 1 → overshoot', expected: 'none (0 %)', calculated: fmt(a.overshoot, 2) + ' %' + (a.hasPeak ? '' : ' (no peak)'),
      pass: !a.hasPeak && a.overshoot === 0
    });

    p = derive(1, 2, 4);                                      // m = 1, b = 2, k = 4
    const ss = stateSpace(p), pl = poles(p.zeta, p.wn);
    const txt = function (l) { return fmt(l[0].re, 3) + ' ± j' + fmt(Math.abs(l[0].im), 3); };
    rows.push({
      test: 'm = 1, b = 2, k = 4 → eig(A) = poles', expected: MINUS + '1 ± j1.732',
      calculated: 'eig(A): ' + txt(ss.eig) + '<br>poles: ' + txt(pl),
      pass: samePoles(ss.eig, pl, 1e-9) && Math.abs(ss.eig[0].re + 1) < 1e-9 && Math.abs(Math.abs(ss.eig[0].im) - 1.7320508) < 1e-6
    });
    return rows;
  }

  const engine = {
    derive: derive, classify: classify, poles: poles, stepNorm: stepNorm, velNorm: velNorm,
    simulate: simulate, curve: curve, analyze: analyze, theoryOvershoot: theoryOvershoot,
    eig2: eig2, stateSpace: stateSpace, samePoles: samePoles, runValidation: runValidation,
    windowFor: windowFor, plotPoints: plotPoints, RANGE: RANGE
  };

  // Lets Node load just the maths for testing; in the browser this line is skipped.
  if (typeof document === 'undefined') { if (typeof module !== 'undefined') module.exports = engine; return; }

  // --- 2. Formatting helpers and SVG drawing ---
  const MINUS = '−';
  const C = { purple: '#6d62e8', deep: '#5146c3', teal: '#16a6a0', peach: '#e78d54', grid: '#e8eaf5', axis: '#cdd3e4', text: '#8590a5', gray: '#a8b1c4' };

  const clamp = function (v, lo, hi) { return Math.min(hi, Math.max(lo, v)); };
  const r1 = function (v) { return Math.round(v * 10) / 10; };
  const num = function (v, d) {
    if (d == null) d = 2;
    const s = Number(v).toFixed(d);
    return (Number(s) === 0 ? (0).toFixed(d) : s).replace('-', MINUS);
  };
  const fmtT = function (v) { return v == null ? '—' : v >= 100 ? v.toFixed(0) : v >= 10 ? v.toFixed(1) : v.toFixed(2); };
  const decimalsFor = function (step) {
    for (let d = 0; d < 7; d++) { const s = step * Math.pow(10, d); if (Math.abs(s - Math.round(s)) < 1e-6) return d; }
    return 6;
  };
  function niceStep(span, target) {
    const raw = span / Math.max(1, target), mag = Math.pow(10, Math.floor(Math.log10(raw))), n = raw / mag;
    return (n < 1.5 ? 1 : n < 3.5 ? 2 : n < 7.5 ? 5 : 10) * mag;
  }
  function ticksIn(lo, hi, step) {
    const out = [];
    for (let i = Math.ceil(lo / step - 1e-9); i * step <= hi + step * 1e-9; i++) out.push(i * step);
    return out;
  }
  const line = function (x1, y1, x2, y2, stroke, extra) {
    return '<line x1="' + r1(x1) + '" y1="' + r1(y1) + '" x2="' + r1(x2) + '" y2="' + r1(y2) + '" stroke="' + stroke + '" ' + (extra || '') + '/>';
  };
  const text = function (x, y, str, o) {
    o = o || {};
    return '<text x="' + r1(x) + '" y="' + r1(y) + '" font-size="' + (o.size || 11) + '" fill="' + (o.fill || C.text) +
      '" text-anchor="' + (o.anchor || 'start') + '" font-weight="' + (o.weight || 500) + '"' +
      (o.halo ? ' paint-order="stroke" stroke="#fff" stroke-width="3" stroke-linejoin="round"' : '') +
      (o.rotate ? ' transform="rotate(' + o.rotate + ' ' + r1(x) + ' ' + r1(y) + ')"' : '') + '>' + str + '</text>';
  };
  const pathOf = function (pts) {
    let d = '';
    for (let i = 0; i < pts.length; i++) d += (i ? 'L' : 'M') + pts[i][0].toFixed(1) + ' ' + pts[i][1].toFixed(1);
    return d;
  };
  const cross = function (x, y, size, color) {
    return '<path d="M' + r1(x - size) + ' ' + r1(y - size) + 'L' + r1(x + size) + ' ' + r1(y + size) + 'M' + r1(x - size) + ' ' + r1(y + size) +
      'L' + r1(x + size) + ' ' + r1(y - size) + '" stroke="' + color + '" stroke-width="3" stroke-linecap="round"/>';
  };

  // --- 3. Module 3 page and interface ---
  const root = document.getElementById('page-module3');
  if (!root) return;

  const ui = {};                       // element references
  const state = { m: RANGE.m.def, b: RANGE.b.def, k: RANGE.k.def };
  let model = null;                    // last computed result
  let raf = 0;

  // Run after module1.js has finished its own start-up.
  function start() {
    if (root.dataset.m3Ready) return;
    root.dataset.m3Ready = '1';
    build();
  }
  if (document.readyState === 'complete') start(); else window.addEventListener('load', start);

  function build() {
    injectStyle();
    root.innerHTML = template();
    const ids = [
      'mass', 'damping', 'spring', 'mass-out', 'damping-out', 'spring-out', 'reset', 'back',
      'case', 'footnote', 'step-box', 'step-svg', 's-box', 's-svg', 'p-box', 'p-svg',
      's-strip', 'p-strip', 'p-strip-label',
      'v-zeta', 'v-wn', 'v-os', 'v-tp', 'v-tr', 'v-ts', 'c-os', 'c-tp',
      'A-num', 'B-num', 'eig-txt', 'poles-txt', 'eig-status',
      'val-body', 'val-count',
      'pred-count', 'pred-q', 'pred-choices', 'pred-btn', 'pred-result', 'pred-feedback', 'pred-before', 'pred-after', 'pred-hint'
    ];
    ids.forEach(function (id) { ui[id] = document.getElementById('m3-' + id); });
    ui.stepBox = ui['step-box']; ui.stepSvg = ui['step-svg'];
    ui.sBox = ui['s-box'];       ui.sSvg = ui['s-svg'];
    ui.pBox = ui['p-box'];       ui.pSvg = ui['p-svg'];

    wireControls();
    wireHover();
    renderValidation();
    renderQuestion();
    markHomeTile();
    syncSliders();
    update();

    // Re-draw crisply when the page becomes visible or the width changes (phone rotation, resize)
    const widths = { a: 0, b: 0, c: 0 };
    const onSize = function () {
      const w = [ui.stepBox.clientWidth, ui.sBox.clientWidth, ui.pBox.clientWidth];
      if (w[0] !== widths.a || w[1] !== widths.b || w[2] !== widths.c) { widths.a = w[0]; widths.b = w[1]; widths.c = w[2]; schedule(); }
    };
    if (typeof ResizeObserver !== 'undefined') {
      const ro = new ResizeObserver(onSize);
      [ui.stepBox, ui.sBox, ui.pBox].forEach(function (el) { ro.observe(el); });
    } else window.addEventListener('resize', onSize);
  }

  // Make the home-page tile say LIVE, like Module 1's tile
  function markHomeTile() {
    const tile = document.querySelector('.module-tile[data-open="module3"]');
    if (!tile) return;
    tile.classList.add('ready');
    const badge = tile.querySelector('.tile-badge');
    if (badge) { badge.className = 'tile-badge live'; badge.textContent = '● LIVE'; }
    const link = tile.querySelector('.tile-link');
    if (link && link.firstChild && link.firstChild.nodeType === 3) link.firstChild.nodeValue = 'Open module ';
  }

  /* ---------- styles that style.css does not have (scoped to this page) ---------- */
  function injectStyle() {
    if (document.getElementById('module3-style')) return;
    const css = [
      '#page-module3 .m3-tags{display:flex;align-items:center;gap:8px;flex-wrap:wrap;justify-content:flex-end}',
      '#page-module3 .m3-case{transition:background .2s,color .2s}',
      '#page-module3 .m3-case.under{background:#f2f0ff;color:#6860d4}',
      '#page-module3 .m3-case.crit{background:#e9fbf4;color:#14916e}',
      '#page-module3 .m3-case.over{background:#fff1e9;color:#c9783f}',
      '#page-module3 .m3-seg{background:#f2f4fa;display:grid;grid-template-columns:repeat(3,1fr);padding:4px;border-radius:11px;gap:2px}',
      '#page-module3 .m3-seg button{border:0;padding:9px 6px;border-radius:8px;background:transparent;color:#8a95a8;font-weight:700;font-size:12px}',
      '#page-module3 .m3-seg button.active{color:#5e53d1;background:#fff;box-shadow:0 2px 8px #1c265018}',
      '#page-module3 #m3-step-svg{touch-action:pan-y;cursor:crosshair}',
      '#page-module3 .m3-eq{font-size:13px;color:#28344b;font-weight:720;background:#f8f9fc;border-radius:11px;padding:12px 14px;text-align:center;margin:12px 0 4px}',
      '#page-module3 .m3-mats{display:flex;flex-wrap:wrap;align-items:center;gap:12px 14px;margin:16px 0 18px;font-variant-numeric:tabular-nums}',
      '#page-module3 .m3-row{display:flex;align-items:center;gap:10px}',
      '#page-module3 .m3-lbl{font-weight:800;color:#6558d3;font-size:15px}',
      '#page-module3 .m3-op{color:#9aa3b5;font-weight:700}',
      '#page-module3 .m3-mat{display:inline-flex;align-items:stretch}',
      '#page-module3 .m3-mat i{width:7px;border:2px solid #7d73e8;border-right:0;border-radius:3px 0 0 3px}',
      '#page-module3 .m3-mat i.r{border:2px solid #7d73e8;border-left:0;border-radius:0 3px 3px 0}',
      '#page-module3 .m3-mat b{display:grid;gap:5px 14px;padding:4px 8px;font-weight:700;font-size:13px;color:#28344b;text-align:center;align-items:center}',
      '#page-module3 .m3-eigs{display:grid;grid-template-columns:1fr auto 1fr;gap:10px;align-items:center}',
      '#page-module3 .m3-eig{border:1px solid #eeebfb;background:#f9f8ff;border-radius:10px;padding:12px;min-width:0}',
      '#page-module3 .m3-eig .metric-label{display:block}',
      '#page-module3 .m3-eig strong{display:block;color:#6558d3;font-size:14px;margin-top:3px;font-variant-numeric:tabular-nums}',
      '#page-module3 .m3-eq-mid{text-align:center}',
      '#page-module3 .m3-one{grid-template-columns:1fr}',
      '@media (max-width:520px){#page-module3 .m3-eigs{grid-template-columns:1fr}#page-module3 .m3-eq-mid{display:none}}'
    ].join('\n');
    const el = document.createElement('style');
    el.id = 'module3-style';
    el.textContent = css;
    document.head.appendChild(el);
  }

  /* ---------- page markup (uses the classes already defined in style.css) ---------- */
  function sliderBlock(key, id, label, symbol, help) {
    const r = RANGE[key];
    return '<div class="control-block"><div class="control-top"><label for="m3-' + id + '">' + label + ' <span class="unit">' + symbol + '</span></label>' +
      '<output id="m3-' + id + '-out" for="m3-' + id + '">' + r.def.toFixed(r.dec) + ' ' + r.unit + '</output></div>' +
      '<input id="m3-' + id + '" type="range" min="' + r.min + '" max="' + r.max + '" step="' + r.step + '" value="' + r.def + '" />' +
      '<div class="range-labels"><span>' + r.min.toFixed(r.dec) + ' ' + r.unit + '</span><span>' + r.max.toFixed(r.dec) + ' ' + r.unit + '</span></div>' +
      '<p class="help-text">' + help + '</p></div>';
  }
  function metricCard(label, id, capId, caption) {
    return '<article class="metric-card"><span class="metric-label">' + label + '</span><div class="metric-value" id="m3-v-' + id + '">—</div><p' + (capId ? ' id="m3-c-' + capId + '"' : '') + '>' + caption + '</p></article>';
  }

  function template() {
    return '' +
      '<div class="module-hero">' +
        '<button type="button" id="m3-back" class="breadcrumb-link"><span aria-hidden="true">←</span> Back to home</button>' +
        '<div class="module-title-row">' +
          '<div><p class="section-kicker">MODULE 03 <span class="kicker-divider">/</span> SECOND ORDER</p><h1>Second-order response<span class="title-period">.</span></h1><p>Change the mass, damping and spring and watch the oscillation appear and fade.</p></div>' +
          '<span class="formula-badge">G(s) = Kω<sub>n</sub>² / (s² + 2ζω<sub>n</sub>s + ω<sub>n</sub>²)</span>' +
        '</div>' +
      '</div>' +

      '<div class="workspace-grid">' +
        '<aside class="controls-stack" aria-label="Simulation controls">' +
          '<section class="panel controls-panel">' +
            '<div class="panel-heading"><div class="panel-heading-icon purple" aria-hidden="true">⚙</div><div><h3>Simulation controls</h3><p>Tune your mass–spring–damper</p></div></div>' +
            sliderBlock('m', 'mass', 'Mass', 'm', 'A heavier mass reacts more slowly (lower ω<sub>n</sub>).') +
            sliderBlock('b', 'damping', 'Damping', 'b', 'More damping removes oscillation, and too much makes it sluggish.') +
            sliderBlock('k', 'spring', 'Spring', 'k', 'A stiffer spring raises ω<sub>n</sub> and lowers the final value 1/k.') +
            '<div class="control-block"><span class="control-top"><span class="control-label">Damping preset</span></span>' +
              '<div class="m3-seg" role="group" aria-label="Damping preset"><button type="button" data-zeta="0.5">Under</button><button type="button" data-zeta="1">Critical</button><button type="button" data-zeta="1.5">Over</button></div>' +
              '<p class="help-text">Sets b so that ζ = 0.5, 1 or 1.5 (m and k stay). Input: unit step force F₀ = 1 N at t = 0.</p></div>' +
            '<button id="m3-reset" class="reset-button" type="button"><span aria-hidden="true">↺</span> Reset parameters</button>' +
          '</section>' +
          '<section class="panel prediction-panel" aria-labelledby="m3-pred-title">' +
            '<div class="prediction-heading"><span class="sparkle" aria-hidden="true">✦</span><h3 id="m3-pred-title">Predict &amp; check</h3><span class="mini-label" id="m3-pred-count">QUESTION 1 OF 2</span></div>' +
            '<p class="prediction-question" id="m3-pred-q"></p>' +
            '<div class="prediction-choices" id="m3-pred-choices" role="radiogroup" aria-label="Select your prediction"></div>' +
            '<button id="m3-pred-btn" type="button" class="primary-button">Check my prediction <span aria-hidden="true">→</span></button>' +
            '<p class="prediction-instruction">Choose an answer and click the button. We’ll change the system automatically and show the result.</p>' +
            '<div id="m3-pred-result" class="prediction-result" hidden><p id="m3-pred-feedback" class="prediction-feedback" role="status" aria-live="polite"></p><div class="prediction-comparison"><span id="m3-pred-before"></span><span class="comparison-arrow" aria-hidden="true">→</span><span id="m3-pred-after"></span></div></div>' +
            '<p id="m3-pred-hint" class="prediction-hint" role="status" aria-live="polite"></p>' +
          '</section>' +
        '</aside>' +

        '<div class="results-stack">' +
          '<section class="panel chart-panel" aria-labelledby="m3-resp-title">' +
            '<div class="chart-heading"><div><p class="chart-eyebrow">LIVE VISUALIZATION</p><h3 id="m3-resp-title">Step response</h3></div>' +
              '<div class="m3-tags"><span class="small-tag m3-case under" id="m3-case">Underdamped</span><span class="live-chip"><span class="live-indicator" aria-hidden="true"></span> Live</span></div></div>' +
            '<div class="chart-legend" aria-hidden="true"><span><i class="line-key output"></i> Displacement x(t)</span><span><i class="line-key target"></i> Final value F₀/k</span><span><i class="line-key second"></i> ±2% settling band</span></div>' +
            '<div class="chart-box" id="m3-step-box"><svg id="m3-step-svg" viewBox="0 0 900 390" role="img" aria-label="Second-order step response"></svg></div>' +
            '<p id="m3-footnote" class="chart-footnote"></p>' +
          '</section>' +
          '<div class="metrics-grid">' +
            metricCard('Damping ratio ζ', 'zeta', '', 'b / (2√(km))') +
            metricCard('Natural frequency ω<sub>n</sub>', 'wn', '', '√(k/m)') +
            metricCard('Overshoot', 'os', 'os', 'read off the curve') +
            metricCard('Peak time', 'tp', 'tp', 'time of the first peak') +
            metricCard('Rise time', 'tr', '', '10% → 90% of final value') +
            metricCard('Settling time', 'ts', '', 'stays within ±2%') +
          '</div>' +
          '<section class="panel log-panel" aria-labelledby="m3-s-title"><div class="chart-heading"><div><p class="chart-eyebrow">WHERE THE SYSTEM LIVES</p><h3 id="m3-s-title">Poles in the s-plane</h3></div><span class="small-tag">Re(s) vs Im(s)</span></div>' +
            '<p class="subtle-intro">Poles are the roots of s² + 2ζω<sub>n</sub>s + ω<sub>n</sub>². Further left means faster decay; further from the real axis means more oscillation.</p>' +
            '<div class="chart-box" id="m3-s-box"><svg id="m3-s-svg" viewBox="0 0 560 380" role="img" aria-label="Poles of the system in the s-plane"></svg></div>' +
            '<div class="slope-strip"><span>Poles</span><strong id="m3-s-strip">—</strong></div></section>' +
        '</div>' +
      '</div>' +

      '<div class="lower-grid">' +
        '<section class="panel validation-panel" aria-labelledby="m3-p-title"><div class="chart-heading"><div><p class="chart-eyebrow">STATE-SPACE VIEW</p><h3 id="m3-p-title">Phase plane</h3></div><span class="small-tag">x₁ = x, x₂ = ẋ</span></div>' +
          '<p class="subtle-intro">The path of (x₁, x₂) during the step response, from rest at the origin to the equilibrium (F₀/k, 0).</p>' +
          '<div class="chart-box" id="m3-p-box"><svg id="m3-p-svg" viewBox="0 0 560 380" role="img" aria-label="Phase plane trajectory of displacement against velocity"></svg></div>' +
          '<div class="slope-strip"><span id="m3-p-strip-label">Trajectory</span><strong id="m3-p-strip">—</strong></div></section>' +
        '<section class="panel log-panel" aria-labelledby="m3-ss-title"><div class="chart-heading"><div><p class="chart-eyebrow">SAME MODEL, MATRIX FORM</p><h3 id="m3-ss-title">State-space panel</h3></div><span class="small-tag">x = [x₁, x₂]ᵀ</span></div>' +
          '<p class="subtle-intro">Move m, b or k and A and B update at once. The eigenvalues of A are the same numbers as the poles on the s-plane plot.</p>' +
          '<div class="m3-eq">ẋ = A x + B F(t) &nbsp;&nbsp;·&nbsp;&nbsp; y = C x</div>' +
          '<div class="m3-mats">' +
            '<div class="m3-row"><span class="m3-lbl">A =</span>' + matrix([['0', '1'], ['−k/m', '−b/m']], 2) + '<span class="m3-op">=</span><span id="m3-A-num"></span></div>' +
            '<div class="m3-row"><span class="m3-lbl">B =</span>' + matrix([['0'], ['1/m']], 1) + '<span class="m3-op">=</span><span id="m3-B-num"></span></div>' +
            '<div class="m3-row"><span class="m3-lbl">C =</span>' + matrix([['1', '0']], 2) + '</div>' +
          '</div>' +
          '<div class="m3-eigs"><div class="m3-eig"><span class="metric-label">Eigenvalues of A</span><strong id="m3-eig-txt">—</strong></div>' +
            '<div class="m3-eq-mid"><span class="pass-chip" id="m3-eig-status">SAME</span></div>' +
            '<div class="m3-eig"><span class="metric-label">Poles of G(s)</span><strong id="m3-poles-txt">—</strong></div></div>' +
        '</section>' +
      '</div>' +

      '<div class="lower-grid m3-one">' +
        '<section class="panel validation-panel" aria-labelledby="m3-val-title"><div class="chart-heading"><div><p class="chart-eyebrow">ENGINEERING CHECK</p><h3 id="m3-val-title">Validation tests</h3></div><span id="m3-val-count" class="validation-summary">—</span></div>' +
          '<p class="subtle-intro">Second-order results from the lecture slides, recomputed by this page’s own code.</p>' +
          '<div class="table-scroll"><table><thead><tr><th>Test</th><th>Expected</th><th>Calculated</th><th>Status</th></tr></thead><tbody id="m3-val-body"></tbody></table></div>' +
          '<div class="note"><span class="note-icon" aria-hidden="true">ⓘ</span><span>Overshoot depends only on ζ: <strong>M<sub>p</sub> = e<sup>−πζ/√(1−ζ²)</sup></strong>. At ζ = 0.5 that is 16.3%, and at ζ ≥ 1 there is no overshoot.</span></div>' +
        '</section>' +
      '</div>';
  }

  function matrix(rows, cols) {
    let cells = '';
    rows.forEach(function (r) { r.forEach(function (c) { cells += '<span>' + c + '</span>'; }); });
    return '<span class="m3-mat"><i></i><b style="grid-template-columns:repeat(' + cols + ',auto)">' + cells + '</b><i class="r"></i></span>';
  }

  /* ---------- controls ---------- */
  function wireControls() {
    [['m', 'mass'], ['b', 'damping'], ['k', 'spring']].forEach(function (pair) {
      ui[pair[1]].addEventListener('input', function () {
        state[pair[0]] = parseFloat(ui[pair[1]].value);
        syncOutputs();
        schedule();
      });
    });
    ui.reset.addEventListener('click', function () { setState({ m: RANGE.m.def, b: RANGE.b.def, k: RANGE.k.def }); });
    root.querySelectorAll('.m3-seg button').forEach(function (btn) {
      btn.addEventListener('click', function () {
        const zt = parseFloat(btn.getAttribute('data-zeta'));
        setState({ b: 2 * zt * Math.sqrt(state.k * state.m) });
      });
    });
    ui.back.addEventListener('click', function () {
      const tab = document.getElementById('tab-home');
      if (tab) tab.click();
      window.scrollTo(0, 0);
    });
    ui['pred-btn'].addEventListener('click', onPredictionButton);
  }

  // Write values into the state (clamped to the slider range), then redraw
  function setState(patch, now) {
    ['m', 'b', 'k'].forEach(function (key) {
      if (patch[key] != null) state[key] = clamp(patch[key], RANGE[key].min, RANGE[key].max);
    });
    syncSliders();
    if (now) update(); else schedule();
  }
  function syncSliders() {
    ui.mass.value = state.m; ui.damping.value = state.b; ui.spring.value = state.k;
    syncOutputs();
  }
  function syncOutputs() {
    ui['mass-out'].textContent = state.m.toFixed(RANGE.m.dec) + ' ' + RANGE.m.unit;
    ui['damping-out'].textContent = state.b.toFixed(RANGE.b.dec) + ' ' + RANGE.b.unit;
    ui['spring-out'].textContent = state.k.toFixed(RANGE.k.dec) + ' ' + RANGE.k.unit;
  }
  function schedule() {
    if (raf) return;
    raf = requestAnimationFrame(function () { raf = 0; update(); });
  }

  /* ---------- compute + render everything ---------- */
  function compute(s) {
    const p = derive(s.m, s.b, s.k);
    const sim = simulate(p);
    const a = analyze(sim.t, sim.y);
    const T = windowFor(p, a);
    const plot = plotPoints(p, T);
    return { p: p, a: a, plot: plot, yss: p.K, kind: classify(p.zeta), poles: poles(p.zeta, p.wn), ss: stateSpace(p) };
  }

  function update() {
    model = compute(state);
    renderStep(model);
    renderSPlane(model);
    renderPhase(model);
    renderMetrics(model);
    renderStateSpace(model);
    root.querySelectorAll('.m3-seg button').forEach(function (btn) {
      const z = parseFloat(btn.getAttribute('data-zeta'));
      btn.classList.toggle('active', (z < 1 && model.kind === 'under') || (z === 1 && model.kind === 'crit') || (z > 1 && model.kind === 'over'));
    });
  }

  // Sets the svg viewBox to the real pixel width of its box so text stays crisp on phones
  function fit(box, svg, ratio, minH, maxH) {
    const w = box.clientWidth;
    const W = w > 0 ? Math.round(w) : 900;
    const H = Math.round(clamp(W * ratio, minH, maxH));
    svg.setAttribute('viewBox', '0 0 ' + W + ' ' + H);
    return { W: W, H: H };
  }

  const KIND_NAME = { under: 'Underdamped', crit: 'Critically damped', over: 'Overdamped' };

  function polesText(list) {
    const a = list[0], b = list[1];
    if (Math.abs(a.im) > 1e-9) return num(a.re, 3) + ' ± j' + num(Math.abs(a.im), 3);
    if (Math.abs(a.re - b.re) < 1e-6) return num(a.re, 3) + ' (double)';
    return num(a.re, 3) + ', ' + num(b.re, 3);
  }

  /* ----- step response chart ----- */
  function renderStep(M) {
    const p = M.p, a = M.a, plot = M.plot, yss = M.yss;
    const size = fit(ui.stepBox, ui.stepSvg, 0.43, 250, 420), W = size.W, H = size.H;
    const narrow = W < 520;
    const mg = { l: narrow ? 50 : 64, r: 20, t: 22, b: 52 };
    const pw = W - mg.l - mg.r, ph = H - mg.t - mg.b, T = plot.T;
    const ymax = Math.max(1, a.hasPeak ? a.peakY : 1) * yss * 1.22;
    const X = function (t) { return mg.l + (t / T) * pw; };
    const Y = function (v) { return mg.t + ph - (v / ymax) * ph; };
    const base = Y(0);
    let s = '<defs><linearGradient id="m3-fill" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#8a7ff8" stop-opacity=".22"/><stop offset="1" stop-color="#8a7ff8" stop-opacity="0"/></linearGradient></defs>';

    // grid and tick labels
    const ys = niceStep(ymax, ph / 58), yd = decimalsFor(ys);
    ticksIn(0, ymax, ys).forEach(function (v) {
      s += line(mg.l, Y(v), mg.l + pw, Y(v), C.grid, 'stroke-dasharray="4 6"');
      s += text(mg.l - 8, Y(v) + 4, num(v, yd), { anchor: 'end' });
    });
    const xs = niceStep(T, pw / 90), xd = decimalsFor(xs);
    ticksIn(0, T, xs).forEach(function (v) {
      s += line(X(v), mg.t, X(v), base, C.grid, 'stroke-dasharray="4 6"');
      s += text(X(v), base + 18, num(v, xd), { anchor: 'middle' });
    });
    s += line(mg.l, mg.t, mg.l, base, C.axis, 'stroke-width="1.5"') + line(mg.l, base, mg.l + pw, base, C.axis, 'stroke-width="1.5"');
    s += text(mg.l + pw / 2, H - 8, 'Time t (s)', { anchor: 'middle', size: 11, weight: 600 });
    s += text(13, mg.t + ph / 2, 'Displacement x (m)', { anchor: 'middle', size: 11, weight: 600, rotate: -90 });

    // ±2 % band and final value
    s += '<rect x="' + mg.l + '" y="' + r1(Y(1.02 * yss)) + '" width="' + pw + '" height="' + r1(Y(0.98 * yss) - Y(1.02 * yss)) + '" fill="' + C.teal + '" opacity=".09"/>';
    s += line(mg.l, Y(1.02 * yss), mg.l + pw, Y(1.02 * yss), C.teal, 'stroke-dasharray="5 5" opacity=".5"');
    s += line(mg.l, Y(0.98 * yss), mg.l + pw, Y(0.98 * yss), C.teal, 'stroke-dasharray="5 5" opacity=".5"');
    s += line(mg.l, Y(yss), mg.l + pw, Y(yss), C.gray, 'stroke-width="2" stroke-dasharray="7 6"');
    s += text(mg.l + pw - 4, Y(yss) - 8, narrow ? 'F₀/k' : 'final value F₀/k = ' + num(yss, 3) + ' m', { anchor: 'end', size: 11, fill: '#7d88a0', weight: 650, halo: true });

    // rise-time construction (10 % to 90 %)
    let riseLabel = null;
    if (a.t10 != null && a.t90 != null && a.t90 <= T) {
      const x10 = X(a.t10), x90 = X(a.t90), y10 = Y(0.1 * yss), y90 = Y(0.9 * yss);
      s += line(x10, y10, x10, base, C.teal, 'stroke-dasharray="2 4" opacity=".8"') + line(x90, y90, x90, base, C.teal, 'stroke-dasharray="2 4" opacity=".8"');
      s += line(x10, base, x90, base, C.teal, 'stroke-width="5" stroke-linecap="round"');
      const rtxt = narrow ? 'Rise' : 'Rise ' + fmtT(a.riseT) + ' s', half = rtxt.length * 3.6 + 4;
      // keep the label inside the plot even when the rise is very fast (label would spill over the axis)
      riseLabel = { x: clamp((x10 + x90) / 2, mg.l + half, mg.l + pw - half), txt: rtxt };
    }

    // settling time marker
    if (a.settleT != null && a.settleT > 0 && a.settleT <= T) {
      s += line(X(a.settleT), base, X(a.settleT), mg.t + 4, C.peach, 'stroke-width="2" stroke-dasharray="6 5"');
    }

    // curve + area
    const pts = [];
    for (let i = 0; i < plot.t.length; i++) pts.push([X(plot.t[i]), Y(plot.y[i] * yss)]);
    const d = pathOf(pts);
    s += '<path d="' + d + 'L' + r1(X(T)) + ' ' + r1(base) + 'L' + r1(X(0)) + ' ' + r1(base) + 'Z" fill="url(#m3-fill)"/>';
    s += '<path d="' + d + '" fill="none" stroke="' + C.purple + '" stroke-width="3.5" stroke-linecap="round" stroke-linejoin="round"/>';

    // markers drawn above the curve
    if (riseLabel) {
      s += '<circle cx="' + r1(X(a.t10)) + '" cy="' + r1(Y(0.1 * yss)) + '" r="4.5" fill="#fff" stroke="' + C.teal + '" stroke-width="2.5"/>';
      s += '<circle cx="' + r1(X(a.t90)) + '" cy="' + r1(Y(0.9 * yss)) + '" r="4.5" fill="#fff" stroke="' + C.teal + '" stroke-width="2.5"/>';
      s += text(riseLabel.x, base - 10, riseLabel.txt, { anchor: 'middle', size: 11, fill: '#139690', weight: 750, halo: true });
    }
    if (a.settleT != null && a.settleT > 0 && a.settleT <= T) {
      const sx = X(a.settleT), lbl = narrow ? 'Settle' : 'Settle ' + fmtT(a.settleT) + ' s';
      const left = sx > W * 0.72;
      const ly = riseLabel && Math.abs(sx - riseLabel.x) < 95 ? base - 28 : base - 10;
      s += text(left ? sx - 6 : sx + 6, ly, lbl, { anchor: left ? 'end' : 'start', size: 11, fill: '#c9783f', weight: 750, halo: true });
    }
    if (a.hasPeak && a.peakT <= T) {
      const cx = X(a.peakT), cy = Y(a.peakY * yss), right = cx > W * 0.55;
      s += '<circle cx="' + r1(cx) + '" cy="' + r1(cy) + '" r="5.5" fill="#fff" stroke="' + C.purple + '" stroke-width="3.5"/>';
      s += text(right ? cx - 10 : cx + 10, cy - 10, (narrow ? '' : 'Peak ') + '+' + num(a.overshoot, 1) + '%', { anchor: right ? 'end' : 'start', size: 12, fill: C.deep, weight: 800, halo: true });
    }

    // hover read-out (hidden until the pointer is over the chart)
    s += '<g id="m3-hover" style="display:none" pointer-events="none"><line id="m3-h-line" y1="' + mg.t + '" y2="' + r1(base) + '" stroke="#9188ac" stroke-dasharray="3 4"/>' +
      '<circle id="m3-h-dot" r="5" fill="#fff" stroke="' + C.purple + '" stroke-width="3"/>' +
      '<rect id="m3-h-box" x="' + (mg.l + 8) + '" y="' + (mg.t + 4) + '" height="24" rx="7" fill="#fff" stroke="#e7e5fc"/>' +
      '<text id="m3-h-txt" x="' + (mg.l + 16) + '" y="' + (mg.t + 20) + '" font-size="12" font-weight="700" fill="' + C.deep + '"></text></g>';

    ui.stepSvg.innerHTML = s;
    ui.stepSvg.setAttribute('aria-label', 'Step response, ' + KIND_NAME[M.kind].toLowerCase() + ', zeta ' + p.zeta.toFixed(2) +
      (a.hasPeak ? ', overshoot ' + a.overshoot.toFixed(1) + ' percent' : ', no overshoot') + ', settling time ' + fmtT(a.settleT) + ' seconds');
    ui.geo = { W: W, l: mg.l, pw: pw, T: T, t: mg.t, ph: ph, ymax: ymax, yss: yss, zeta: p.zeta, wn: p.wn, base: base };
  }

  function wireHover() {
    const svg = ui.stepSvg;
    const hide = function () { const g = svg.querySelector('#m3-hover'); if (g) g.style.display = 'none'; };
    const move = function (e) {
      const g = ui.geo; if (!g) return;
      const rect = svg.getBoundingClientRect();
      const px = (e.clientX - rect.left) * (g.W / rect.width);
      if (px < g.l || px > g.l + g.pw) return hide();
      const t = ((px - g.l) / g.pw) * g.T;
      const yv = stepNorm(g.zeta, g.wn, t) * g.yss;
      const cy = g.t + g.ph - (yv / g.ymax) * g.ph;
      const grp = svg.querySelector('#m3-hover');
      if (!grp) return;
      grp.style.display = '';
      const ln = grp.querySelector('#m3-h-line'), dot = grp.querySelector('#m3-h-dot'), box = grp.querySelector('#m3-h-box'), tx = grp.querySelector('#m3-h-txt');
      ln.setAttribute('x1', px); ln.setAttribute('x2', px);
      dot.setAttribute('cx', px); dot.setAttribute('cy', cy);
      const label = 't = ' + t.toFixed(2) + ' s  ·  x = ' + yv.toFixed(3) + ' m  ·  ' + ((yv / g.yss) * 100).toFixed(1) + '%';
      tx.textContent = label;
      box.setAttribute('width', Math.round(label.length * 6.9 + 18));
    };
    svg.addEventListener('pointermove', move);
    svg.addEventListener('pointerdown', move);
    svg.addEventListener('pointerleave', hide);
    svg.addEventListener('pointercancel', hide);
  }

  /* ----- s-plane ----- */
  function renderSPlane(M) {
    const p = M.p, pl = M.poles;
    const size = fit(ui.sBox, ui.sSvg, 0.66, 290, 340), W = size.W, H = size.H;
    const mg = { l: 46, r: 16, t: 18, b: 40 };
    const pw = W - mg.l - mg.r, ph = H - mg.t - mg.b;
    const under = Math.abs(pl[0].im) > 0;
    let maxRe = 1e-6;
    pl.forEach(function (q) { maxRe = Math.max(maxRe, Math.abs(q.re)); });
    const needL = 1.25 * (under ? p.wn : maxRe);
    const needV = under ? 1.25 * p.wn : 0.2 * needL;
    const scale = Math.min((0.66 * pw) / needL, (ph / 2) / needV);
    const ox = mg.l + 0.66 * pw, oy = mg.t + ph / 2;
    const SX = function (re) { return ox + re * scale; }, SY = function (im) { return oy - im * scale; };
    let s = '<defs><clipPath id="m3-clip-s"><rect x="' + mg.l + '" y="' + mg.t + '" width="' + pw + '" height="' + ph + '"/></clipPath></defs>';

    // stable / unstable halves
    s += '<rect x="' + r1(ox) + '" y="' + mg.t + '" width="' + r1(mg.l + pw - ox) + '" height="' + ph + '" fill="#fff4f0" opacity=".8"/>';

    // grid + tick labels
    const reLo = (mg.l - ox) / scale, reHi = (mg.l + pw - ox) / scale, imHi = (ph / 2) / scale;
    const sr = niceStep(reHi - reLo, pw / 80), dr = decimalsFor(sr);
    ticksIn(reLo, reHi, sr).forEach(function (v) {
      s += line(SX(v), mg.t, SX(v), mg.t + ph, C.grid, 'stroke-dasharray="4 6"');
      s += text(SX(v), mg.t + ph + 16, num(v, dr), { anchor: 'middle' });
    });
    const si = niceStep(2 * imHi, ph / 62), di = decimalsFor(si);
    ticksIn(-imHi, imHi, si).forEach(function (v) {
      s += line(mg.l, SY(v), mg.l + pw, SY(v), C.grid, 'stroke-dasharray="4 6"');
      if (Math.abs(v) > 1e-9) s += text(mg.l - 8, SY(v) + 4, (v < 0 ? MINUS : '') + 'j' + num(Math.abs(v), di), { anchor: 'end' });
    });
    s += line(mg.l, oy, mg.l + pw, oy, C.axis, 'stroke-width="1.5"') + line(ox, mg.t, ox, mg.t + ph, '#b8bfd3', 'stroke-width="1.5"');
    s += text(mg.l + 8, mg.t + 14, 'stable', { size: 10, fill: '#9aa3b5', weight: 700 });
    s += text(mg.l + pw - 8, mg.t + 14, 'unstable', { anchor: 'end', size: 10, fill: '#d9a08f', weight: 700 });
    s += text(ox + 6, mg.t + 14, 'Im(s)', { anchor: 'start', size: 10, fill: C.text, weight: 700 });
    s += text(mg.l + pw - 4, oy - 8, 'Re(s)', { anchor: 'end', size: 10, fill: C.text, weight: 700 });

    // underdamped extras: circle |s| = ωn, angle θ with ζ = cos θ
    if (under) {
      const R = p.wn * scale, up = pl[0];
      s += '<circle cx="' + r1(ox) + '" cy="' + r1(oy) + '" r="' + r1(R) + '" fill="none" stroke="#c5bdff" stroke-width="1.5" stroke-dasharray="5 5" clip-path="url(#m3-clip-s)"/>';
      s += text(ox - R, oy + 16, '−ωn', { anchor: 'middle', size: 10, fill: '#8a7ff8', weight: 750, halo: true });
      s += line(ox, oy, SX(up.re), SY(up.im), C.purple, 'stroke-width="1.5" opacity=".45"');
      s += line(SX(up.re), SY(up.im), SX(up.re), oy, C.purple, 'stroke-width="1.2" stroke-dasharray="2 4" opacity=".45"');
      const th = Math.acos(p.zeta), ar = 30;
      s += '<path d="M' + r1(ox - ar) + ' ' + r1(oy) + 'A' + ar + ' ' + ar + ' 0 0 1 ' + r1(ox - ar * Math.cos(th)) + ' ' + r1(oy - ar * Math.sin(th)) + '" fill="none" stroke="' + C.purple + '" stroke-width="1.5"/>';
      if (th > 0.2) s += text(ox - 44 * Math.cos(th / 2), oy - 44 * Math.sin(th / 2) + 4, 'θ', { anchor: 'middle', size: 12, fill: C.deep, weight: 800, halo: true });
    }

    // pole markers + labels
    const lbl = function (x, y, str) {
      const wide = str.length * 6.6;
      const anchorEnd = x - 12 - wide > mg.l;
      return text(anchorEnd ? x - 12 : x + 12, y, str, { anchor: anchorEnd ? 'end' : 'start', size: 11, fill: C.deep, weight: 750, halo: true });
    };
    if (under) {
      const u = pl[0], l = pl[1];
      s += cross(SX(u.re), SY(u.im), 7, C.purple) + cross(SX(l.re), SY(l.im), 7, C.purple);
      s += lbl(SX(u.re), SY(u.im) - 8, num(u.re, 2) + ' + j' + num(u.im, 2));
      s += lbl(SX(l.re), SY(l.im) + 18, num(l.re, 2) + ' − j' + num(u.im, 2));
    } else if (pl[0].re === pl[1].re) {
      s += cross(SX(pl[0].re), oy, 7, C.purple);
      s += lbl(SX(pl[0].re), oy - 14, num(pl[0].re, 2) + ' (double pole)');
    } else {
      pl.forEach(function (q, i) {
        s += cross(SX(q.re), oy, 7, C.purple);
        const close = Math.abs(SX(pl[0].re) - SX(pl[1].re)) < 90;
        s += lbl(SX(q.re), oy + (close && i === 1 ? 26 : -14), num(q.re, 2));
      });
    }
    ui.sSvg.innerHTML = s;
    ui['s-strip'].textContent = polesText(pl) + (under ? '   (θ = ' + num((Math.acos(p.zeta) * 180) / PI, 1) + '°, ζ = cos θ)' : '');
  }

  /* ----- phase plane ----- */
  function renderPhase(M) {
    const plot = M.plot, yss = M.yss, p = M.p;
    const size = fit(ui.pBox, ui.pSvg, 0.66, 290, 400), W = size.W, H = size.H;
    const mg = { l: 58, r: 16, t: 18, b: 44 };
    const pw = W - mg.l - mg.r, ph = H - mg.t - mg.b, n = plot.t.length;
    let x0 = 0, x1 = 0, v0 = 0, v1 = 0;
    for (let i = 0; i < n; i++) {
      const x = plot.y[i] * yss, v = plot.v[i] * yss;
      if (x < x0) x0 = x; if (x > x1) x1 = x; if (v < v0) v0 = v; if (v > v1) v1 = v;
    }
    x1 = Math.max(x1, yss); v1 = Math.max(v1, 1e-9);
    const dx = x1 - x0, dv = v1 - v0;
    x0 -= 0.06 * dx; x1 += 0.08 * dx; v0 -= 0.12 * dv; v1 += 0.14 * dv;
    const X = function (x) { return mg.l + ((x - x0) / (x1 - x0)) * pw; };
    const Y = function (v) { return mg.t + ph - ((v - v0) / (v1 - v0)) * ph; };
    let s = '';

    const sx = niceStep(x1 - x0, pw / 80), dxd = decimalsFor(sx);
    ticksIn(x0, x1, sx).forEach(function (v) {
      s += line(X(v), mg.t, X(v), mg.t + ph, C.grid, 'stroke-dasharray="4 6"');
      s += text(X(v), mg.t + ph + 16, num(v, dxd), { anchor: 'middle' });
    });
    const sv = niceStep(v1 - v0, ph / 58), dvd = decimalsFor(sv);
    ticksIn(v0, v1, sv).forEach(function (v) {
      s += line(mg.l, Y(v), mg.l + pw, Y(v), C.grid, 'stroke-dasharray="4 6"');
      s += text(mg.l - 8, Y(v) + 4, num(v, dvd), { anchor: 'end' });
    });
    s += line(mg.l, mg.t, mg.l, mg.t + ph, C.axis, 'stroke-width="1.5"') + line(mg.l, mg.t + ph, mg.l + pw, mg.t + ph, C.axis, 'stroke-width="1.5"');
    if (x0 < 0 && x1 > 0) s += line(X(0), mg.t, X(0), mg.t + ph, '#b8bfd3', 'stroke-width="1.5"');
    if (v0 < 0 && v1 > 0) s += line(mg.l, Y(0), mg.l + pw, Y(0), '#b8bfd3', 'stroke-width="1.5"');
    s += text(mg.l + pw / 2, H - 6, 'x₁ = displacement x (m)', { anchor: 'middle', weight: 600 });
    s += text(13, mg.t + ph / 2, 'x₂ = velocity ẋ (m/s)', { anchor: 'middle', weight: 600, rotate: -90 });

    const pts = [];
    for (let i = 0; i < n; i++) pts.push([X(plot.y[i] * yss), Y(plot.v[i] * yss)]);
    s += '<path d="' + pathOf(pts) + '" fill="none" stroke="' + C.purple + '" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"/>';

    // direction arrows along the path
    [0.1, 0.28, 0.5].forEach(function (f) {
      const i = Math.floor(f * (n - 1)), j = Math.min(n - 1, i + 3);
      const ddx = pts[j][0] - pts[i][0], ddy = pts[j][1] - pts[i][1];
      if (Math.hypot(ddx, ddy) < 1.2) return;
      s += '<path d="M-6 -4.5L5 0L-6 4.5Z" fill="' + C.deep + '" transform="translate(' + r1(pts[i][0]) + ' ' + r1(pts[i][1]) + ') rotate(' + r1((Math.atan2(ddy, ddx) * 180) / PI) + ')"/>';
    });

    // start and equilibrium
    s += '<circle cx="' + r1(X(0)) + '" cy="' + r1(Y(0)) + '" r="5" fill="#fff" stroke="' + C.teal + '" stroke-width="3"/>';
    s += text(X(0) + 9, Y(0) - 9, 'start', { size: 11, fill: '#139690', weight: 750, halo: true });
    s += '<circle cx="' + r1(X(yss)) + '" cy="' + r1(Y(0)) + '" r="5.5" fill="' + C.purple + '" stroke="#fff" stroke-width="2"/>';
    const eqLeft = X(yss) > mg.l + pw * 0.6;
    s += text(eqLeft ? X(yss) - 9 : X(yss) + 9, Y(0) + 20, (W < 520 ? 'F₀/k' : 'equilibrium (F₀/k, 0)'), { anchor: eqLeft ? 'end' : 'start', size: 11, fill: C.deep, weight: 750, halo: true });

    ui.pSvg.innerHTML = s;
    ui['p-strip'].textContent = p.zeta < 1 - EPS ? 'Spiral → complex poles' : 'No spiral → real poles';
  }

  /* ----- metric cards, footnote, state-space panel ----- */
  function renderMetrics(M) {
    const p = M.p, a = M.a;
    const unit = function (v, u) { return v + (u ? '<span>' + u + '</span>' : ''); };
    ui['v-zeta'].innerHTML = num(p.zeta, 3);
    ui['v-wn'].innerHTML = unit(num(p.wn, 2), 'rad/s');
    ui['v-os'].innerHTML = unit(a.overshoot < 0.1 && a.overshoot > 0 ? a.overshoot.toFixed(2) : a.overshoot.toFixed(1), '%');
    ui['v-tp'].innerHTML = a.hasPeak ? unit(fmtT(a.peakT), 's') : '—';
    ui['v-tr'].innerHTML = a.riseT != null ? unit(fmtT(a.riseT), 's') : '—';
    ui['v-ts'].innerHTML = a.settleT != null ? unit(fmtT(a.settleT), 's') : '—';
    const th = theoryOvershoot(p.zeta);
    ui['c-os'].textContent = M.kind === 'under' && th > 1e-3 ? 'formula: ' + th.toFixed(th < 0.1 ? 2 : 1) + '%' : 'no overshoot when ζ ≥ 1';
    ui['c-tp'].textContent = a.hasPeak ? 'time of the first peak' : 'no overshoot, so no peak';

    ui.case.textContent = KIND_NAME[M.kind];
    ui.case.className = 'small-tag m3-case ' + M.kind;
    const os = a.hasPeak ? a.overshoot.toFixed(1) : '0';
    ui.footnote.textContent = M.kind === 'under'
      ? 'Underdamped (ζ < 1): the poles are complex, so the output rings around the final value. Overshoot ' + os + '% depends only on ζ.'
      : M.kind === 'crit'
        ? 'Critically damped (ζ = 1): a double real pole at −ωn — the fastest response that does not overshoot.'
        : 'Overdamped (ζ > 1): two real poles, no overshoot; the slower pole dominates, so the response is sluggish.';
  }

  function matCells(rows) {
    const cols = rows[0].length;
    let cells = '';
    rows.forEach(function (r) { r.forEach(function (c) { cells += '<span>' + num(c, 2) + '</span>'; }); });
    return '<span class="m3-mat"><i></i><b style="grid-template-columns:repeat(' + cols + ',auto)">' + cells + '</b><i class="r"></i></span>';
  }

  function renderStateSpace(M) {
    const ss = M.ss;
    ui['A-num'].innerHTML = matCells(ss.A);
    ui['B-num'].innerHTML = matCells([[ss.B[0]], [ss.B[1]]]);
    ui['eig-txt'].textContent = polesText(ss.eig);
    ui['poles-txt'].textContent = polesText(M.poles);
    const same = samePoles(ss.eig, M.poles, 1e-6);
    ui['eig-status'].textContent = same ? '= SAME' : '≠ DIFFERENT';
    ui['eig-status'].className = 'pass-chip' + (same ? '' : ' fail');
  }

  /* ----- validation table (fixed test cases, independent of the sliders) ----- */
  function renderValidation() {
    const rows = runValidation();
    let html = '', passed = 0;
    rows.forEach(function (r) {
      if (r.pass) passed++;
      html += '<tr><td>' + r.test + '</td><td>' + r.expected + '</td><td>' + r.calculated + '</td><td><span class="pass-chip' + (r.pass ? '' : ' fail') + '">' + (r.pass ? 'PASS' : 'FAIL') + '</span></td></tr>';
    });
    ui['val-body'].innerHTML = html;
    ui['val-count'].textContent = passed + ' / ' + rows.length + ' passed';
    ui['val-count'].style.background = passed === rows.length ? '' : '#fff0ed';
    ui['val-count'].style.color = passed === rows.length ? '' : '#b04b37';
  }

  // --- 4. Predict and check ---
  const QUESTIONS = [
    {
      prompt: 'What happens to the <strong>overshoot</strong> if we <strong>double the damping b</strong>?',
      choices: [['down', 'It gets smaller'], ['up', 'It gets larger'], ['same', 'It stays the same']],
      label: 'Overshoot', unit: '%', digits: 1,
      metric: function (a) { return a.overshoot; },
      ready: function (s) { return derive(s.m, s.b, s.k).zeta < 0.85 && s.b * 2 <= RANGE.b.max; },
      change: function (s) { return { b: s.b * 2 }; },
      what: 'overshoot',
      explain: 'Damping removes energy, so the first peak is lower. Overshoot depends only on ζ = b / (2√(km)).'
    },
    {
      prompt: 'What happens to the <strong>peak time</strong> if we make the spring <strong>4× stiffer</strong> (k × 4)?',
      choices: [['up', 'The first peak comes later'], ['down', 'The first peak comes earlier'], ['same', 'It stays the same']],
      label: 'Peak time', unit: ' s', digits: 2,
      metric: function (a) { return a.peakT; },
      ready: function (s) { return derive(s.m, s.b, s.k).zeta < 0.85 && s.k * 4 <= RANGE.k.max; },
      change: function (s) { return { k: s.k * 4 }; },
      what: 'peak time',
      explain: 'A stiffer spring raises ωn = √(k/m), so the system oscillates faster and reaches its first peak sooner (tₚ = π / ωd).'
    }
  ];
  const FALLBACK = { m: 1, b: 1.2, k: 4 };            // a clearly underdamped start (ζ = 0.3)
  let qi = 0, answered = false;

  function renderQuestion() {
    const q = QUESTIONS[qi];
    ui['pred-count'].textContent = 'QUESTION ' + (qi + 1) + ' OF ' + QUESTIONS.length;
    ui['pred-q'].innerHTML = q.prompt;
    let h = '';
    q.choices.forEach(function (c) {
      h += '<label class="choice"><input type="radio" name="m3-prediction" value="' + c[0] + '" /><span>' + c[1] + '</span></label>';
    });
    ui['pred-choices'].innerHTML = h;
    ui['pred-btn'].innerHTML = 'Check my prediction <span aria-hidden="true">→</span>';
    ui['pred-result'].hidden = true;
    ui['pred-hint'].textContent = '';
    answered = false;
  }

  function onPredictionButton() {
    if (answered) { qi = (qi + 1) % QUESTIONS.length; renderQuestion(); return; }
    const q = QUESTIONS[qi];
    const picked = ui['pred-choices'].querySelector('input:checked');
    if (!picked) { ui['pred-hint'].textContent = 'Pick an answer first — guessing before you look is the whole point.'; return; }
    ui['pred-hint'].textContent = '';

    let note = '';
    if (!q.ready(state)) {
      setState(FALLBACK, true);
      note = ' (Your sliders were already near critical damping or at a limit, so we started from m = 1, b = 1.2, k = 4.)';
    }
    const before = compute(state);
    setState(q.change(state), true);
    const after = compute(state);

    const bv = q.metric(before.a) || 0, av = q.metric(after.a) || 0;
    const rel = Math.abs(av - bv) / Math.max(Math.abs(bv), 1e-9);
    const outcome = rel < 0.01 ? 'same' : av < bv ? 'down' : 'up';
    const right = picked.value === outcome;
    const word = outcome === 'same' ? 'did not change' : (outcome === 'down' ? 'went down' : 'went up');

    ui['pred-feedback'].className = 'prediction-feedback ' + (right ? 'correct' : 'incorrect');
    ui['pred-feedback'].textContent = (right ? 'Correct! ' : 'Not quite — ') + 'The ' + q.what + ' ' + word + '. ' + q.explain + note;
    const show = function (v) { return q.label + ' ' + v.toFixed(q.digits) + q.unit; };
    ui['pred-before'].textContent = show(bv);
    ui['pred-after'].textContent = show(av);
    ui['pred-result'].hidden = false;
    ui['pred-choices'].querySelectorAll('input').forEach(function (i) { i.disabled = true; });
    ui['pred-btn'].innerHTML = 'Try another prediction <span aria-hidden="true">→</span>';
    answered = true;
  }
})();