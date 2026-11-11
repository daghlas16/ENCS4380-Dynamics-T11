/*
  Dynamics Lab — Module 4: Guess the sensor
  ---------------------------------------------------------
  A hidden sensor is drawn at random and its noisy response is plotted.
  The user estimates the parameters, then the true curve is revealed.

  First order, step:   y(t) = K (1 - exp(-t/τ))              -> estimate K, τ
  First order, sine:   y(t) = K/(1+(ωτ)²) [sin ωt - ωτ cos ωt + ωτ exp(-t/τ)]   -> estimate K, τ
  Second order, step:  G(s) = ωn² / (s² + 2ζωn s + ωn²)       (K = 1) -> estimate ζ, ωn

  Sine input: both K and τ are unknown. The log-error plot helps for the step case.
  Second order also covers critically damped (ζ = 1) and overdamped (ζ > 1) sensors.

  Wrapped in an IIFE so it never clashes with the globals of the other modules.
*/
(() => {
  'use strict';

  const $ = (id) => document.getElementById(id);
  const SAMPLES = 120;
  const FULL_MARKS_TOLERANCE = 0.10; // within 10% = 100 points
  const ZERO_MARKS_ERROR = 0.50;     // 50% error or more = 0 points

  const PARAM_INFO = {
    K:    { label: 'Gain K',          unit: '',       decimals: 2, slider: 'm4-gain' },
    tau:  { label: 'Time constant τ', unit: ' s',     decimals: 2, slider: 'm4-tau' },
    zeta: { label: 'Damping ratio ζ', unit: '',       decimals: 2, slider: 'm4-zeta' },
    wn:   { label: 'Natural freq. ωn', unit: ' rad/s', decimals: 2, slider: 'm4-wn' }
  };

  const HINTS = {
    'first-step': [
      'The curve settles at the final value K.',
      'At t = τ the output has reached 63.2% of K.',
      'ln(1 − y/K) against t is a straight line with slope −1/τ.'
    ],
    'first-sine': [
      'In steady state the output is a sine with amplitude K·M, for an input amplitude of 1.',
      'M = 1 / √(1 + (ωτ)²), so once you know M = amplitude / K, τ = √(1/M² − 1) / ω.',
      'The output lags the input by φ = tan⁻¹(ωτ); the start is a transient that fades like e^(−t/τ).'
    ],
    'second-over': [
      'No overshoot means ζ ≥ 1.',
      'ζ = 1 is the fastest response without overshoot; larger ζ gives a slower, lazier rise.',
      'Settling takes about 4/ωn near ζ = 1 and longer when overdamped: adjust until the dashed curve overlays the data.'
    ],
    'second': [
      'Overshoot depends only on ζ.',
      'ζ = −ln(Mp) / √(π² + ln²(Mp)), where Mp is the fractional overshoot.',
      'Peak time tp = π / (ωn√(1 − ζ²)); once ζ is known, solve for ωn.'
    ]
  };

  let sensor = null;
  let revealed = false;
  let hintCount = 0;
  let guess = null;
  const session = { total: 0, rounds: 0 };
  const hintKey = () => (sensor.kind === 'second' && sensor.params.zeta >= 1 - 1e-6 ? 'second-over' : sensor.kind);

  // --- 1. Random helpers ---
  const rand = (a, b) => a + Math.random() * (b - a);
  const roundTo = (v, step) => Math.round(v / step) * step;
  function gaussian() {
    let u = 0;
    while (u === 0) u = Math.random();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * Math.random());
  }

  // --- 2. Closed-form responses ---
  function firstStep(t, p) {
    return p.K * (1 - Math.exp(-t / p.tau));
  }

  function firstSine(t, p) {
    const wt = p.omega * p.tau;
    return p.K * (Math.sin(p.omega * t) - wt * Math.cos(p.omega * t) + wt * Math.exp(-t / p.tau)) / (1 + wt * wt);
  }

  function secondStep(t, p) {
    const { zeta, wn } = p;
    if (zeta < 1 - 1e-6) {
      const wd = wn * Math.sqrt(1 - zeta * zeta);
      return 1 - Math.exp(-zeta * wn * t) * (Math.cos(wd * t) + zeta / Math.sqrt(1 - zeta * zeta) * Math.sin(wd * t));
    }
    if (zeta <= 1 + 1e-6) return 1 - Math.exp(-wn * t) * (1 + wn * t);
    const r = wn * Math.sqrt(zeta * zeta - 1);
    const s1 = -zeta * wn + r, s2 = -zeta * wn - r;
    return 1 - (s2 * Math.exp(s1 * t) - s1 * Math.exp(s2 * t)) / (s2 - s1);
  }

  const CURVE = { 'first-step': firstStep, 'first-sine': firstSine, 'second': secondStep };

  // --- 3. Create a hidden sensor ---
  function newSensor() {
    const pick = Math.random();
    const kind = pick < 0.4 ? 'first-step' : pick < 0.6 ? 'first-sine' : 'second';
    let params, tMax, sigma;

    if (kind === 'first-step') {
      params = { K: roundTo(rand(0.5, 2.5), 0.1), tau: roundTo(rand(0.4, 4), 0.1) };
      tMax = Math.ceil(5 * params.tau);
      sigma = 0.025 * params.K;
    } else if (kind === 'first-sine') {
      params = { K: roundTo(rand(0.5, 2.5), 0.1), tau: roundTo(rand(0.3, 3), 0.1), omega: roundTo(rand(0.5, 3), 0.1) };
      tMax = Math.ceil(Math.max(6 * Math.PI / params.omega, 4 * params.tau));
      sigma = 0.025 * params.K;
    } else {
      // 70% underdamped, 15% critically damped, 15% overdamped
      const r = Math.random();
      const zeta = r < 0.7 ? roundTo(rand(0.15, 0.85), 0.05) : r < 0.85 ? 1 : roundTo(rand(1.2, 1.5), 0.05);
      params = { K: 1, zeta: Number(zeta.toFixed(2)), wn: roundTo(rand(1.5, 8), 0.1) };
      if (params.zeta < 1) {
        const peakTime = Math.PI / (params.wn * Math.sqrt(1 - params.zeta ** 2));
        tMax = Math.max(5 / (params.zeta * params.wn), 2.5 * peakTime);
      } else {
        // the slowest pole sets the settling time
        const slowRate = params.wn * (params.zeta - Math.sqrt(params.zeta ** 2 - 1));
        tMax = 5 / slowRate;
      }
      sigma = 0.02;
    }

    const data = [];
    for (let i = 0; i <= SAMPLES; i += 1) {
      const t = (tMax * i) / SAMPLES;
      data.push([t, CURVE[kind](t, params) + sigma * gaussian()]);
    }
    return { kind, params, tMax, data };
  }

  function guessedParams() {
    const g = {
      K: Number($('m4-gain').value),
      tau: Number($('m4-tau').value),
      zeta: Number($('m4-zeta').value),
      wn: Number($('m4-wn').value)
    };
    if (sensor.kind === 'first-sine') return { K: g.K, tau: g.tau, omega: sensor.params.omega };
    if (sensor.kind === 'first-step') return { K: g.K, tau: g.tau };
    return { K: 1, zeta: g.zeta, wn: g.wn };
  }

  function unknownKeys() {
    return sensor.kind === 'second' ? ['zeta', 'wn'] : ['K', 'tau'];
  }

  // --- 4. Chart ---
  function niceStep(raw) {
    const mag = 10 ** Math.floor(Math.log10(raw));
    const f = raw / mag;
    return (f <= 1 ? 1 : f <= 2 ? 2 : f <= 5 ? 5 : 10) * mag;
  }

  function line(x1, y1, x2, y2, stroke, extra = '') {
    return `<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="${stroke}" ${extra}/>`;
  }

  function pathFrom(fn, params, xOf, yOf, tMax) {
    let d = '';
    for (let i = 0; i <= 400; i += 1) {
      const t = (tMax * i) / 400;
      d += `${i === 0 ? 'M' : 'L'}${xOf(t).toFixed(2)},${yOf(fn(t, params)).toFixed(2)} `;
    }
    return d;
  }

  function drawChart() {
    const svg = $('m4-chart');
    const W = 900, H = 390, left = 70, right = 30, top = 40, bottom = 58;
    const plotW = W - left - right, plotH = H - top - bottom;
    const fn = CURVE[sensor.kind];

    const ys = sensor.data.map((d) => d[1]);
    const rawMax = Math.max(...ys), rawMin = sensor.kind === 'first-sine' ? Math.min(...ys) : 0;
    const step = niceStep((rawMax - rawMin) * 1.1 / 5);
    const yMax = Math.ceil(rawMax / step) * step;
    const yMin = rawMin < 0 ? Math.floor(rawMin / step) * step : 0;
    const xOf = (t) => left + (t / sensor.tMax) * plotW;
    const yOf = (v) => top + plotH * (1 - (v - yMin) / (yMax - yMin));

    let svgText = '';
    for (let v = yMin; v <= yMax + step / 2; v += step) {
      const y = yOf(v);
      svgText += line(left, y, W - right, y, '#e7eaf3', 'stroke-dasharray="4 5"');
      svgText += `<text x="${left - 12}" y="${y + 4}" text-anchor="end" font-size="13" fill="#64748b">${Number(v.toFixed(3))}</text>`;
    }
    const tStep = niceStep(sensor.tMax / 6);
    for (let t = 0; t <= sensor.tMax + 1e-9; t += tStep) {
      const x = xOf(t);
      svgText += line(x, top, x, H - bottom, '#eef0f6');
      svgText += `<text x="${x}" y="${H - bottom + 22}" text-anchor="middle" font-size="12" fill="#64748b">${Number(t.toFixed(2))}</text>`;
    }
    svgText += `<text x="${W / 2}" y="${H - 10}" text-anchor="middle" font-size="14" fill="#64748b">Time (s)</text>`;

    const dots = sensor.data.map(([t, y]) =>
      `<circle cx="${xOf(t).toFixed(1)}" cy="${yOf(y).toFixed(1)}" r="3.2" fill="#6d62e8" fill-opacity=".5"/>`).join('');

    let overlay = '';
    if (!revealed) {
      overlay += `<path d="${pathFrom(fn, guessedParams(), xOf, yOf, sensor.tMax)}" fill="none" stroke="#6d62e8" stroke-width="3" stroke-dasharray="9 7" clip-path="url(#m4-clip)"/>`;
    }
    let legend = `<circle cx="500" cy="22" r="4" fill="#6d62e8" fill-opacity=".6"/><text x="511" y="27" font-size="14" fill="#334155">Measured</text>`;
    if (revealed) {
      overlay += `<path d="${pathFrom(fn, guess, xOf, yOf, sensor.tMax)}" fill="none" stroke="#6d62e8" stroke-width="3" stroke-dasharray="9 7" clip-path="url(#m4-clip)"/>`;
      overlay += `<path d="${pathFrom(fn, sensor.params, xOf, yOf, sensor.tMax)}" fill="none" stroke="#16a6a0" stroke-width="3.5" clip-path="url(#m4-clip)"/>`;
      legend += `${line(610, 22, 640, 22, '#16a6a0', 'stroke-width="3.5"')}<text x="647" y="27" font-size="14" fill="#334155">True</text>`;
      legend += `${line(715, 22, 745, 22, '#6d62e8', 'stroke-width="3" stroke-dasharray="7 5"')}<text x="752" y="27" font-size="14" fill="#334155">Your guess</text>`;
    }
    if (!revealed) legend += `${line(610, 22, 640, 22, '#6d62e8', 'stroke-width="3" stroke-dasharray="7 5"')}<text x="647" y="27" font-size="14" fill="#334155">Your guess (live)</text>`;
    const clip = `<defs><clipPath id="m4-clip"><rect x="${left}" y="${top}" width="${plotW}" height="${plotH}"/></clipPath></defs>`;
    svg.innerHTML = `${clip}${svgText}${dots}${overlay}${legend}`;
    drawLogChart();
  }

  // Log method (first-order step): ln(1 - y/K) of the measured data against t is a line of slope -1/τ.
  function drawLogChart() {
    const panel = $('m4-log-panel');
    panel.hidden = sensor.kind !== 'first-step';
    if (panel.hidden) { $('m4-log-chart').innerHTML = ''; return; }
    const svg = $('m4-log-chart');
    const W = 900, H = 310, left = 70, right = 30, top = 28, bottom = 50;
    const plotW = W - left - right, plotH = H - top - bottom;
    const g = revealed ? guess : guessedParams();
    const yTop = 0.3, yBot = -3.2, tMax = sensor.tMax;
    const xOf = (t) => left + (t / tMax) * plotW;
    const yOf = (v) => top + plotH * (yTop - v) / (yTop - yBot);

    let s = '';
    for (const v of [0, -1, -2, -3]) {
      s += line(left, yOf(v), W - right, yOf(v), '#e7eaf3', 'stroke-dasharray="4 5"');
      s += `<text x="${left - 12}" y="${yOf(v) + 4}" text-anchor="end" font-size="13" fill="#64748b">${v}</text>`;
    }
    const tStep = niceStep(tMax / 6);
    for (let t = 0; t <= tMax + 1e-9; t += tStep) {
      s += line(xOf(t), top, xOf(t), H - bottom, '#eef0f6');
      s += `<text x="${xOf(t)}" y="${H - bottom + 22}" text-anchor="middle" font-size="12" fill="#64748b">${Number(t.toFixed(2))}</text>`;
    }
    s += `<text x="${W / 2}" y="${H - 8}" text-anchor="middle" font-size="14" fill="#64748b">Time (s)</text>`;
    const slopeLine = (tau, stroke, extra) => line(xOf(0), yOf(0), xOf(tMax), yOf(-tMax / tau), stroke, `stroke-width="3" ${extra}`);
    const dots = sensor.data
      .map(([t, y]) => [t, Math.log(1 - y / g.K)])
      .filter(([, v]) => Number.isFinite(v) && v > Math.log(0.05))
      .map(([t, v]) => `<circle cx="${xOf(t).toFixed(1)}" cy="${yOf(v).toFixed(1)}" r="3.2" fill="#6d62e8" fill-opacity=".5"/>`).join('');
    let lines = slopeLine(g.tau, '#6d62e8', 'stroke-dasharray="9 7"');
    let legend = `<circle cx="460" cy="14" r="4" fill="#6d62e8" fill-opacity=".6"/><text x="471" y="19" font-size="14" fill="#334155">ln(1 − y/K)</text>` +
      `${line(610, 14, 640, 14, '#6d62e8', 'stroke-width="3" stroke-dasharray="7 5"')}<text x="647" y="19" font-size="14" fill="#334155">Your slope</text>`;
    if (revealed) {
      lines += slopeLine(sensor.params.tau, '#16a6a0', '');
      legend += `${line(760, 14, 790, 14, '#16a6a0', 'stroke-width="3.5"')}<text x="797" y="19" font-size="14" fill="#334155">True</text>`;
    }
    const clip = `<defs><clipPath id="m4-log-clip"><rect x="${left}" y="${top}" width="${plotW}" height="${plotH}"/></clipPath></defs>`;
    svg.innerHTML = `${clip}${s}<g clip-path="url(#m4-log-clip)">${dots}${lines}</g>${legend}`;
    $('m4-slope-value').textContent = `${(-1 / g.tau).toFixed(3).replace('-', '−')} s⁻¹`;
  }

  // --- 5. Scoring ---
  function marks(trueValue, guessValue) {
    const err = Math.abs(guessValue - trueValue) / trueValue;
    const score = err <= FULL_MARKS_TOLERANCE ? 100
      : Math.max(0, 100 * (1 - (err - FULL_MARKS_TOLERANCE) / (ZERO_MARKS_ERROR - FULL_MARKS_TOLERANCE)));
    return { err, score };
  }

  function reveal() {
    if (revealed) return;
    revealed = true;
    guess = guessedParams();

    const keys = unknownKeys();
    const rows = keys.map((key) => {
      const info = PARAM_INFO[key];
      const m = marks(sensor.params[key], guess[key]);
      return { key, info, ...m, trueValue: sensor.params[key], guessValue: guess[key] };
    });
    const roundScore = Math.round(rows.reduce((s, r) => s + r.score, 0) / rows.length);
    session.total += roundScore;
    session.rounds += 1;

    $('m4-result-body').innerHTML = rows.map((r) => {
      const ok = r.err <= FULL_MARKS_TOLERANCE;
      return `<tr><td>${r.info.label}</td><td>${r.trueValue.toFixed(r.info.decimals)}${r.info.unit}</td>` +
        `<td>${r.guessValue.toFixed(r.info.decimals)}${r.info.unit}</td><td>${(r.err * 100).toFixed(1)}%</td>` +
        `<td style="color:${ok ? '#15803d' : '#dc2626'};font-weight:bold">${Math.round(r.score)} / 100</td></tr>`;
    }).join('');

    const feedback = $('m4-feedback');
    feedback.className = `prediction-feedback ${roundScore >= 100 ? 'correct' : 'incorrect'}`;
    feedback.textContent = roundScore >= 100
      ? 'Full marks! Every estimate is within 10% of the true value.'
      : roundScore >= 50
        ? 'Close. Compare your dashed curve with the true one and see where it drifts.'
        : 'Far off this time. Use a hint next round and read the curve step by step.';

    $('m4-notes').innerHTML = rows.map((r) => `<strong>${r.info.label}:</strong> ${explain(r)}`).join('<br>');
    $('m4-result').hidden = false;
    $('m4-submit').disabled = true;
    document.querySelectorAll('#page-module4 input[type="range"]').forEach((el) => { el.disabled = true; });
    updateScore(roundScore);
    drawChart();
  }

  // Short, parameter-specific explanation shown after the reveal.
  function explain(r) {
    const p = sensor.params, t = r.trueValue;
    const status = r.err <= FULL_MARKS_TOLERANCE ? 'Within 10%, full marks. ' : `Your guess was too ${r.guessValue > t ? 'high' : 'low'}. `;
    if (r.key === 'K') {
      return status + (sensor.kind === 'first-step'
        ? `K is the final value the curve settles at (true K = ${t.toFixed(2)}).`
        : 'K scales the steady-state amplitude: amplitude = K / √(1 + (ωτ)²).');
    }
    if (r.key === 'tau') {
      return status + (sensor.kind === 'first-step'
        ? `τ is the time at which the output reaches 63.2% of K, here ${t.toFixed(2)} s. On the log plot the points fall on a line of slope −1/τ = ${(-1 / t).toFixed(3)} s⁻¹.`
        : `Divide the steady-state amplitude by K to get M, then τ = √(1/M² − 1) / ω (true M = ${(1 / Math.sqrt(1 + (p.omega * t) ** 2)).toFixed(3)}).`);
    }
    if (r.key === 'zeta') {
      if (t >= 1 - 1e-6) return status + 'There is no overshoot, so ζ ≥ 1. The slower the tail, the larger ζ; compare your dashed curve with the data.';
      const Mp = Math.exp(-Math.PI * t / Math.sqrt(1 - t * t));
      return status + `The overshoot was ${(Mp * 100).toFixed(1)}%, and ζ = −ln(Mp) / √(π² + ln²(Mp)) = ${t.toFixed(2)}.`;
    }
    if (p.zeta >= 1 - 1e-6) return status + 'With no peak, ωn sets how fast the curve rises: a larger ωn rises and settles sooner (about 4/ωn near ζ = 1).';
    const tp = Math.PI / (t * Math.sqrt(1 - p.zeta ** 2));
    return status + `Read the peak time tp = ${tp.toFixed(2)} s, then ωn = π / (tp·√(1 − ζ²)) = ${t.toFixed(2)} rad/s.`;
  }

  function updateScore(roundScore) {
    $('m4-round-score').innerHTML = roundScore === null ? '--' : `${roundScore}<span>/ 100</span>`;
    $('m4-session-avg').innerHTML = session.rounds ? `${Math.round(session.total / session.rounds)}<span>/ 100</span>` : '--';
    $('m4-session-caption').textContent = `${session.total} points in total`;
    $('m4-rounds').textContent = String(session.rounds);
  }

  // --- 6. UI ---
  function describe() {
    const p = sensor.params;
    if (sensor.kind === 'first-step') return ['First-order sensor · step input', 'Estimate the gain K and the time constant τ from the curve.'];
    if (sensor.kind === 'first-sine') return ['First-order sensor · sine input', `Input: sin(${p.omega.toFixed(1)} t), ω = ${p.omega.toFixed(1)} rad/s. Estimate the gain K and the time constant τ.`];
    return ['Second-order sensor · step input', 'K = 1 is given. Estimate ζ and ωn, using overshoot and peak time if the curve overshoots.'];
  }

  function updateOutputs() {
    if (sensor && !revealed) queueMicrotask(drawChart);
    $('m4-gain-value').textContent = Number($('m4-gain').value).toFixed(2);
    $('m4-tau-value').textContent = `${Number($('m4-tau').value).toFixed(2)} s`;
    $('m4-zeta-value').textContent = Number($('m4-zeta').value).toFixed(2);
    $('m4-wn-value').textContent = `${Number($('m4-wn').value).toFixed(1)} rad/s`;
  }

  function startRound() {
    sensor = newSensor();
    revealed = false;
    hintCount = 0;
    guess = null;

    const [title, brief] = describe();
    $('m4-kind').textContent = title;
    $('m4-brief').textContent = brief;
    $('m4-chart-title').textContent = 'Hidden sensor response';
    $('m4-round-tag').textContent = `Round ${session.rounds + 1}`;

    const keys = unknownKeys();
    for (const [key, info] of Object.entries(PARAM_INFO)) {
      $(`${info.slider}-block`).hidden = !keys.includes(key);
    }
    document.querySelectorAll('#page-module4 input[type="range"]').forEach((el) => { el.disabled = false; });
    $('m4-gain').value = '1.5'; $('m4-tau').value = '2'; $('m4-zeta').value = '0.5'; $('m4-wn').value = '4';
    updateOutputs();

    $('m4-submit').disabled = false;
    $('m4-result').hidden = true;
    $('m4-feedback').textContent = '';
    $('m4-notes').innerHTML = '';
    $('m4-hint-text').textContent = '';
    $('m4-hint').textContent = `Show a hint (0 / ${HINTS[hintKey()].length})`;
    updateScore(null);
    drawChart();
  }

  function showHint() {
    const list = HINTS[hintKey()];
    if (hintCount < list.length) hintCount += 1;
    $('m4-hint-text').innerHTML = list.slice(0, hintCount).map((h) => `• ${h}`).join('<br>');
    $('m4-hint').textContent = `Show a hint (${hintCount} / ${list.length})`;
  }

  document.addEventListener('DOMContentLoaded', () => {
    ['m4-gain', 'm4-tau', 'm4-zeta', 'm4-wn'].forEach((id) => $(id).addEventListener('input', updateOutputs));
    $('m4-submit').addEventListener('click', reveal);
    $('m4-new').addEventListener('click', startRound);
    $('m4-hint').addEventListener('click', showHint);
    startRound();
  });
})();