/*
  Dynamics Lab — Module 1: First-order time response
  ---------------------------------------------------------
  Transfer function: G(s) = K / (τs + 1)
  Differential equation: τ * dy/dt + y = K * x

  For a unit step (A = 1): y(t) = KA (1 - exp(-t/τ))
  For a unit-slope ramp:   y(t) = KA (t - τ(1 - exp(-t/τ)))
  Normalized step error:  e(t) = exp(-t/τ)
  Therefore ln(e(t)) = -t/τ; its slope = -1/τ.

  Pure JavaScript and SVG, no server or chart library needed.
*/

'use strict';

const A = 1; // Input amplitude (step) or slope (ramp)
const DEFAULTS = { tau: 1.5, K: 1, input: 'step' };
const state = { ...DEFAULTS };

// Keep the same chart axes when sliders change so the curves visibly move.
// The 20-second window includes 3τ even at the maximum τ = 6 s.
const CHART_TIME_MAX = 20;
const STEP_Y_MAX = 3;
const RAMP_Y_MAX = 55;

const $ = (id) => document.getElementById(id);
const tauSlider = $('tau');
const gainSlider = $('gain');
const inputButtons = document.querySelectorAll('[data-input]');
const predictionFeedback = $('prediction-feedback');

// --- 1. Formulas from the lecture ---
function stepResponse(t, tau, K, amplitude = A) {
  return K * amplitude * (1 - Math.exp(-t / tau));
}

function rampResponse(t, tau, K, slope = A) {
  return K * slope * (t - tau * (1 - Math.exp(-t / tau)));
}

function outputAt(t) {
  return state.input === 'step'
    ? stepResponse(t, state.tau, state.K)
    : rampResponse(t, state.tau, state.K);
}

function idealOutputAt(t) {
  return state.input === 'step' ? state.K * A : state.K * A * t;
}

function stepErrorFraction(t, tau) {
  return Math.exp(-t / tau);
}

function fmt(value, decimals = 1) {
  return Number(value).toFixed(decimals);
}

// --- 2. Small SVG plotter, drawn on each slider movement ---
const svgNS = 'http://www.w3.org/2000/svg';
function svgNode(tag, attributes = {}, label = '') {
  const node = document.createElementNS(svgNS, tag);
  for (const [key, value] of Object.entries(attributes)) {
    node.setAttribute(key, String(value));
  }
  if (label !== '') node.textContent = label;
  return node;
}

function add(svg, tag, attributes, label = '') {
  const node = svgNode(tag, attributes, label);
  svg.appendChild(node);
  return node;
}

function makePath(points) {
  return points.map(([x, y], i) => `${i === 0 ? 'M' : 'L'}${x.toFixed(2)},${y.toFixed(2)}`).join(' ');
}

function drawAxes(svg, config) {
  const { x0, y0, width, height, xMax, yTicks, yPosition, xTickLabel, yTickLabel } = config;
  const bottom = y0 + height;
  const xPosition = (t) => x0 + (t / xMax) * width;

  for (let i = 0; i <= 6; i += 1) {
    const t = xMax * i / 6;
    const x = xPosition(t);
    add(svg, 'line', { x1: x, y1: y0, x2: x, y2: bottom, stroke: '#ecf0f6', 'stroke-width': 1 });
    add(svg, 'text', { x, y: bottom + 27, 'text-anchor': 'middle', 'font-size': 16, fill: '#9aa5b7' }, xTickLabel(t));
  }
  for (const tick of yTicks) {
    const y = yPosition(tick);
    add(svg, 'line', { x1: x0, y1: y, x2: x0 + width, y2: y, stroke: '#ecf0f6', 'stroke-width': 1 });
    add(svg, 'text', { x: x0 - 17, y: y + 5, 'text-anchor': 'end', 'font-size': 16, fill: '#9aa5b7' }, yTickLabel(tick));
  }
  add(svg, 'text', { x: x0 + width / 2, y: bottom + 57, 'text-anchor': 'middle', 'font-size': 16, 'font-weight': 600, fill: '#8692a7' }, 'Time (s)');
  return { xPosition, bottom };
}

function drawResponseChart() {
  const svg = $('response-chart');
  svg.replaceChildren();

  const x0 = 88, y0 = 22, width = 776, height = 290;
  const xMax = CHART_TIME_MAX;
  const yMax = state.input === 'step' ? STEP_Y_MAX : RAMP_Y_MAX;
  const yPosition = (v) => y0 + height * (1 - v / yMax);
  const { xPosition } = drawAxes(svg, {
    x0, y0, width, height, xMax,
    yTicks: [0, .25, .5, .75, 1].map((p) => p * yMax),
    yPosition,
    xTickLabel: (t) => fmt(t, t < 10 ? 1 : 0),
    yTickLabel: (v) => fmt(v, yMax < 15 ? 1 : 0)
  });

  // Build both curves using formulas (not manually specified coordinates).
  const outputPoints = [];
  const idealPoints = [];
  for (let i = 0; i <= 240; i += 1) {
    const t = (xMax * i) / 240;
    outputPoints.push([xPosition(t), yPosition(outputAt(t))]);
    idealPoints.push([xPosition(t), yPosition(idealOutputAt(t))]);
  }
  // On a true step, draw the discontinuity at t = 0.
  if (state.input === 'step') {
    idealPoints.unshift([xPosition(0), yPosition(0)]);
  }
  add(svg, 'path', { d: makePath(idealPoints), fill: 'none', stroke: '#a7b1c5', 'stroke-width': 3, 'stroke-dasharray': '9 8', 'stroke-linecap': 'round' });
  add(svg, 'path', { d: makePath(outputPoints), fill: 'none', stroke: '#6d62e8', 'stroke-width': 4, 'stroke-linejoin': 'round', 'stroke-linecap': 'round' });

  if (state.input === 'step') {
    // Two lecture reference points: 63.2% at τ, 95% at 3τ.
    for (const [multiple, caption] of [[1, 'τ · 63.2%'], [3, '3τ · 95%']]) {
      const t = multiple * state.tau;
      const x = xPosition(t);
      const y = yPosition(stepResponse(t, state.tau, state.K));
      add(svg, 'line', { x1: x, y1: y, x2: x, y2: y0 + height, stroke: '#bab4fc', 'stroke-dasharray': '5 7', 'stroke-width': 2 });
      add(svg, 'circle', { cx: x, cy: y, r: 8, fill: '#fff', stroke: '#6d62e8', 'stroke-width': 4 });
      const labelX = Math.min(x + 13, x0 + width - 110);
      add(svg, 'text', { x: labelX, y: Math.max(y - 17, y0 + 15), 'font-size': 16, 'font-weight': 700, fill: '#6156d6' }, caption);
    }
  } else {
    // Illustrate ramp tracking error close to steady state (at 5τ).
    const t = 5 * state.tau;
    const x = xPosition(t);
    const idealY = yPosition(idealOutputAt(t));
    const actualY = yPosition(rampResponse(t, state.tau, state.K));
    add(svg, 'line', { x1: x, y1: idealY, x2: x, y2: actualY, stroke: '#16a6a0', 'stroke-width': 3, 'stroke-dasharray': '4 4' });
    add(svg, 'circle', { cx: x, cy: actualY, r: 6, fill: '#16a6a0', stroke: '#fff', 'stroke-width': 2 });
    add(svg, 'text', { x: x - 12, y: actualY + 31, 'font-size': 16, 'font-weight': 700, 'text-anchor': 'end', fill: '#168d88' }, 'Lag ≈ τ');
  }
}

function drawErrorChart() {
  const svg = $('error-chart');
  svg.replaceChildren();

  const x0 = 92, y0 = 20, width = 775, height = 206;
  const xMax = CHART_TIME_MAX;
  // This is a natural-log y-axis: ln(error) has slope -1/τ.
  // Keep the same axis for all τ values; omit values below 0.001.
  const minLog = Math.log(0.001), maxLog = 0;
  const yPosition = (v) => y0 + height * (maxLog - Math.log(v)) / (maxLog - minLog);
  const { xPosition } = drawAxes(svg, {
    x0, y0, width, height, xMax,
    yTicks: [1, .1, .01, .001], yPosition,
    xTickLabel: (t) => fmt(t, t < 10 ? 1 : 0),
    yTickLabel: (v) => String(v)
  });

  const points = [];
  const plotUntil = Math.min(xMax, state.tau * Math.log(1000));
  for (let i = 0; i <= 240; i += 1) {
    const t = plotUntil * i / 240;
    points.push([xPosition(t), yPosition(stepErrorFraction(t, state.tau))]);
  }
  add(svg, 'path', { d: makePath(points), fill: 'none', stroke: '#16a6a0', 'stroke-width': 4, 'stroke-linecap': 'round' });
  add(svg, 'circle', { cx: xPosition(state.tau), cy: yPosition(Math.exp(-1)), r: 7, fill: '#16a6a0', stroke: '#fff', 'stroke-width': 3 });
  add(svg, 'text', { x: xPosition(state.tau) + 12, y: yPosition(Math.exp(-1)) - 12, 'font-size': 16, fill: '#118e89', 'font-weight': 700 }, 't = τ');
}

// --- 3. Metric cards and two required validation checks ---
function updateMetrics() {
  if (state.input === 'step') {
    $('metric-one-label').textContent = 'Output at τ';
    $('metric-one').innerHTML = `${fmt(100 * (1 - Math.exp(-1)))}<span>%</span>`;
    $('metric-one-caption').textContent = 'of the final output';
    $('metric-two-label').textContent = 'Output at 3τ';
    $('metric-two').innerHTML = `${fmt(100 * (1 - Math.exp(-3)))}<span>%</span>`;
    $('metric-two-caption').textContent = 'of the final output';
    $('metric-three-label').textContent = 'Time constant';
    $('metric-three').innerHTML = `${fmt(state.tau)}<span>s</span>`;
    $('metric-three-caption').textContent = '63.2% reached at τ';
  } else {
    const steadyStateError = state.K * A * state.tau;
    $('metric-one-label').textContent = 'Ramp time lag';
    $('metric-one').innerHTML = `${fmt(state.tau)}<span>s</span>`;
    $('metric-one-caption').textContent = 'output follows τ behind';
    $('metric-two-label').textContent = 'Steady-state error';
    $('metric-two').innerHTML = `${fmt(steadyStateError, 2)}`;
    $('metric-two-caption').textContent = 'approaches KAτ';
    $('metric-three-label').textContent = 'System gain';
    $('metric-three').innerHTML = `${fmt(state.K)}`;
    $('metric-three-caption').textContent = 'output-to-input scaling';
  }
  $('slope-value').textContent = `${fmt(-1 / state.tau, 3).replace('-', '−')} s⁻¹`;
}

function runValidation() {
  // Compare normalized output so changing K does not invalidate the test.
  const ratioTau = stepResponse(state.tau, state.tau, state.K) / (state.K * A);
  const ratioThreeTau = stepResponse(3 * state.tau, state.tau, state.K) / (state.K * A);
  const checks = [
    { actual: ratioTau, expected: .632, cell: 'validation-tau', status: 'validation-tau-status' },
    { actual: ratioThreeTau, expected: .950, cell: 'validation-three-tau', status: 'validation-three-tau-status' }
  ];
  let passed = 0;
  checks.forEach(({ actual, expected, cell, status }) => {
    const ok = Math.abs(actual - expected) < .001; // 0.1 percentage-point tolerance
    $(cell).textContent = `${fmt(actual * 100, 2)}%`;
    $(status).textContent = ok ? 'PASS' : 'FAIL';
    $(status).classList.toggle('fail', !ok);
    if (ok) passed += 1;
  });
  $('validation-count').textContent = `${passed} / ${checks.length} passed`;
}

function render() {
  $('tau-value').textContent = `${fmt(state.tau)} s`;
  $('gain-value').textContent = fmt(state.K);
  $('response-title').textContent = `${state.input === 'step' ? 'Step' : 'Ramp'} response`;
  $('chart-explanation').textContent = state.input === 'step'
    ? 'At t = τ, the step response reaches 63.2% of its final value; at 3τ, it reaches 95%. The axes stay fixed to show the speed and amplitude changes.'
    : 'For a ramp, the output follows the ideal signal with an asymptotic time lag τ and steady-state error KAτ. The axes stay fixed for comparison.';
  inputButtons.forEach((button) => {
    const selected = button.dataset.input === state.input;
    button.classList.toggle('active', selected);
    button.setAttribute('aria-pressed', String(selected));
  });
  drawResponseChart();
  drawErrorChart();
  updateMetrics();
  runValidation();
}

// --- 4. Predict & check: one click both tests and explains the prediction ---
const predictionResult = $('prediction-result');
const predictionHint = $('prediction-hint');
const predictionComparison = $('prediction-comparison');

function clearPrediction() {
  document.querySelectorAll('input[name="prediction"]').forEach((radio) => { radio.checked = false; });
  predictionResult.hidden = true;
  predictionHint.textContent = '';
  predictionFeedback.textContent = '';
  predictionComparison.hidden = false;
}

$('lock-prediction').addEventListener('click', () => {
  const selected = document.querySelector('input[name="prediction"]:checked');
  if (!selected) {
    predictionHint.textContent = 'Choose “It gets slower” or “It gets faster” first.';
    return;
  }

  // Move τ by one second (automatically) so the effect is immediately visible.
  // If already at the maximum, use a visible 4.5 -> 5.5 demonstration range.
  const wasAtMaximum = state.tau > 5;
  const previous = wasAtMaximum ? 4.5 : state.tau;
  const next = Number(Math.min(previous + 1, 6).toFixed(1));
  state.tau = next;
  tauSlider.value = String(next);
  render();

  const correct = selected.value === 'slower';
  predictionFeedback.className = `prediction-feedback ${correct ? 'correct' : 'incorrect'}`;
  predictionFeedback.textContent = correct
    ? 'Correct! Increasing τ makes the sensor respond more slowly. Watch the curve shift right.'
    : 'Not quite. A larger τ means a slower response. Notice the curve now rises more gradually.';
  $('prediction-before').textContent = `Before: τ = ${fmt(previous)} s`;
  $('prediction-after').textContent = `After: τ = ${fmt(next)} s`;
  predictionResult.hidden = false;
  predictionHint.textContent = wasAtMaximum
    ? 'Since τ was near its maximum, the demo compares 4.5 s to 5.5 s.'
    : '';
});

tauSlider.addEventListener('input', () => {
  state.tau = Number(tauSlider.value);
  clearPrediction();
  render();
});

gainSlider.addEventListener('input', () => {
  state.K = Number(gainSlider.value);
  render();
});

inputButtons.forEach((button) => {
  button.addEventListener('click', () => {
    state.input = button.dataset.input;
    render();
  });
});

$('reset-button').addEventListener('click', () => {
  Object.assign(state, DEFAULTS);
  tauSlider.value = String(state.tau);
  gainSlider.value = String(state.K);
  clearPrediction();
  render();
});

// --- 5. Single-page navigation: Home and four individual module tabs ---
const PAGE_NAMES = ['home', 'module1', 'module2', 'module3', 'module4'];
const navTabs = [...document.querySelectorAll('[data-page]')];

function navigateTo(name) {
  const pageName = PAGE_NAMES.includes(name) ? name : 'home';
  PAGE_NAMES.forEach((page) => {
    $(`page-${page}`).hidden = page !== pageName;
  });
  navTabs.forEach((tab) => {
    const isActive = tab.dataset.page === pageName;
    tab.classList.toggle('active', isActive);
    tab.setAttribute('aria-selected', String(isActive));
    tab.tabIndex = isActive ? 0 : -1;
  });
  window.scrollTo(0, 0);
}

function openPage(name) {
  const page = PAGE_NAMES.includes(name) ? name : 'home';
  if (window.location.hash === `#${page}`) navigateTo(page);
  else window.location.hash = page;
}

document.querySelectorAll('[data-page]').forEach((button) => {
  button.addEventListener('click', () => openPage(button.dataset.page));
  button.addEventListener('keydown', (event) => {
    if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(event.key)) return;
    event.preventDefault();
    const current = PAGE_NAMES.indexOf(button.dataset.page);
    const nextIndex = event.key === 'Home' ? 0 : event.key === 'End' ? PAGE_NAMES.length - 1
      : (current + (event.key === 'ArrowRight' ? 1 : -1) + PAGE_NAMES.length) % PAGE_NAMES.length;
    navTabs[nextIndex].focus();
    openPage(PAGE_NAMES[nextIndex]);
  });
});

document.querySelectorAll('[data-open]').forEach((button) => {
  button.addEventListener('click', () => openPage(button.dataset.open));
});

window.addEventListener('hashchange', () => navigateTo(window.location.hash.slice(1)));

render();
navigateTo(window.location.hash.slice(1) || 'home');
