/* Homework 2, P2 — A slow sensor in the feedback loop.
 * Reuses the existing Dynamics Lab UI classes without modifying HTML/CSS/other JS files.
 *
 * Model:
 *   Plant (1st order): dy/dt = (K * u - y) / tau
 *   Plant (2nd order): dy/dt = v, dv/dt = (u - b * v - k * y) / m
 *   Sensor (1st order): dym/dt = (y - ym) / taus   (ym = y when taus = 0)
 *   Controller sees ONLY sensor output ym:
 *     e = r - ym
 *     dI/dt = e
 *     dz/dt = (ym - z) / Tf   (Tf = 0.04 s derivative filter)
 *     u = clip(Kp * e + Ki * I - Kd * (ym - z) / Tf, lo, hi)
 */
(function (global) {
  'use strict';

  const defaults = {
    plant: 'first',
    K: 1,
    tau: 1,
    m: 1,
    b: 2,
    k: 4,
    kp: 2,
    ki: 6,
    kd: 0,
    taus: 0.35,
    r: 1,
    lo: -20,
    hi: 20,
    T: 15
  };

  const filterTime = 0.04;
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  function measured(s, p) {
    return p.taus <= 1e-6 ? s[0] : s[4];
  }

  function control(s, p) {
    const ym = measured(s, p);
    const e = p.r - ym;
    const raw = p.kp * e + p.ki * s[2] - p.kd * (ym - s[3]) / filterTime;
    return { raw, u: clamp(raw, p.lo, p.hi), ym, e };
  }

  function derivative(s, p) {
    const c = control(s, p);
    const dy = p.plant === 'first' ? (p.K * c.u - s[0]) / p.tau : s[1];
    const dv = p.plant === 'first' ? 0 : (c.u - p.b * s[1] - p.k * s[0]) / p.m;
    const dI = c.e;
    const dz = (c.ym - s[3]) / filterTime;
    const dym = p.taus <= 1e-6 ? dy : (s[0] - s[4]) / p.taus;
    return [dy, dv, dI, dz, dym];
  }

  function rk4(s, dt, p) {
    const a = derivative(s, p);
    const b = derivative(s.map((v, i) => v + dt * a[i] / 2), p);
    const c = derivative(s.map((v, i) => v + dt * b[i] / 2), p);
    const d = derivative(s.map((v, i) => v + dt * c[i]), p);
    return s.map((v, i) => v + dt * (a[i] + 2 * b[i] + 2 * c[i] + d[i]) / 6);
  }

  function simulate(input, refinement = 1) {
    const p = { ...defaults, ...input };
    const plantRate = p.plant === 'first'
      ? (1 + p.K * (p.kp + p.kd / filterTime)) / p.tau
      : Math.max(p.b / p.m, Math.sqrt((p.k + p.kp + p.kd / filterTime) / p.m));
    const sensorRate = p.taus > 1e-6 ? 1 / p.taus : 0;
    const rate = Math.max(plantRate, sensorRate);
    const dtTarget = Math.min(0.01, filterTime / 8, 0.08 / Math.max(1, rate)) / refinement;
    const n = Math.ceil(p.T / dtTarget);
    const dt = p.T / n;

    if (n > 800000) {
      throw new Error('This combination needs too many steps. Reduce the duration or increase τs.');
    }

    let s = [0, 0, 0, 0, 0];
    let clipped = 0;
    let failed = false;
    const rows = [];

    for (let i = 0; i <= n; i++) {
      const c = control(s, p);
      if (!s.every(Number.isFinite) || Math.abs(s[0]) > 1e6) {
        failed = true;
        break;
      }
      if (Math.abs(c.raw - c.u) > 1e-8) clipped++;
      rows.push({ t: i * dt, y: s[0], ym: c.ym, u: c.u });
      if (i < n) s = rk4(s, dt, p);
    }

    return {
      p,
      rows,
      dt,
      failed,
      clipped: clipped / (n + 1),
      metrics: analyze(rows, p, failed)
    };
  }

  function analyze(rows, p, failed) {
    const last = rows[rows.length - 1];
    const tail = rows.slice(Math.floor(rows.length * 0.9));
    const final = tail.reduce((acc, q) => acc + q.y, 0) / tail.length;
    let min = Infinity;
    let max = -Infinity;
    for (const q of tail) {
      min = Math.min(min, q.y);
      max = Math.max(max, q.y);
    }
    const scale = Math.max(Math.abs(final), Math.abs(p.r), 0.001);
    const steady = !failed && (max - min) < 0.001 * scale;

    let peak = -Infinity;
    let peakTime = null;
    let maxLag = 0;
    let lastOutside = -1;

    for (let i = 0; i < rows.length; i++) {
      const q = rows[i];
      if (q.y > peak) {
        peak = q.y;
        peakTime = q.t;
      }
      maxLag = Math.max(maxLag, Math.abs(q.y - q.ym));
      if (Math.abs(q.y - final) > 0.02 * Math.abs(final)) lastOutside = i;
    }

    let candidate = rows[lastOutside + 1]?.t;
    if (lastOutside >= 0 && lastOutside + 1 < rows.length) {
      const a = rows[lastOutside];
      const b = rows[lastOutside + 1];
      const boundary = final + (a.y > final ? 1 : -1) * 0.02 * Math.abs(final);
      if (b.y !== a.y) candidate = a.t + (b.t - a.t) * (boundary - a.y) / (b.y - a.y);
    }

    const settling = steady && final > 1e-8 && candidate !== undefined && (last.t - candidate) >= 0.5
      ? candidate
      : null;

    const refLevel = steady && Math.abs(final) > 1e-8 ? Math.abs(final) : Math.abs(p.r);
    const overshoot = !failed && refLevel > 1e-8
      ? Math.max(0, 100 * (peak - refLevel) / refLevel)
      : null;

    return {
      steady,
      final,
      peak,
      peakTime,
      maxLag,
      overshoot,
      settling,
      error: steady ? p.r - final : null,
      endError: p.r - last.y,
      endSensorGap: Math.abs(last.y - last.ym)
    };
  }

  function validations() {
    const base = { ...defaults, kp: 2, ki: 6, kd: 0, lo: -100, hi: 100, T: 25 };

    // 1. Ideal sensor limit (taus = 0):
    //    Closed-loop T(s) = (2s + 6) / (s^2 + 3s + 6), which has a zero at s = -3.
    //    Exact step response: y(t) = 1 - exp(-1.5t) * (cos(wd*t) - (0.5/wd)*sin(wd*t)), wd = sqrt(3.75).
    //    Peak occurs at tp = (pi + atan(-2 / (wd - 0.75/wd))) / wd = 1.1515 s -> Mp = 14.52%.
    const ideal = simulate({ ...base, taus: 0 });
    const wd = Math.sqrt(3.75);
    const tp = (Math.PI + Math.atan(-2 / (wd - 0.75 / wd))) / wd;
    const expectedMp = -Math.exp(-1.5 * tp) * (Math.cos(wd * tp) - (0.5 / wd) * Math.sin(wd * tp)) * 100;

    // 2. Slow sensor increases overshoot (taus = 0.25 s):
    //    Closed-loop T(s) = (2s + 6)(0.25s + 1) / (s(s + 1)(0.25s + 1) + (2s + 6)) -> Mp = 54.00%.
    const slow = simulate({ ...base, taus: 0.25 });

    // 3. Routh-Hurwitz critical sensor lag: taus_crit = (1 + K*Kp) / (K*Ki - K*Kp - 1) = 1.00 s.
    //    At taus = 1.00 s, sustained oscillation has frequency wn_osc = sqrt((1 + K*Kp)/taus) = sqrt(3) rad/s,
    //    giving period T_osc = 2*pi / sqrt(3) = 3.6276 s.
    const crit = simulate({ ...base, taus: 1.0, T: 30 });
    const crossings = [];
    for (let i = 1; i < crit.rows.length; i++) {
      const a = crit.rows[i - 1];
      const b = crit.rows[i];
      if (a.t > 10 && a.y < base.r && b.y >= base.r) {
        crossings.push(a.t + (b.t - a.t) * (base.r - a.y) / (b.y - a.y));
      }
    }
    const periods = [];
    for (let i = 1; i < crossings.length; i++) periods.push(crossings[i] - crossings[i - 1]);
    const avgPeriod = periods.length
      ? periods.reduce((sum, val) => sum + val, 0) / periods.length
      : NaN;
    const expectedPeriod = (2 * Math.PI) / Math.sqrt(3);

    // 4. Steady-state convergence: both true y and sensor ym reach r when stable
    const settled = simulate({ ...base, taus: 0.25, T: 30 });

    return [
      {
        name: 'Ideal sensor (τs = 0 s) — baseline PI overshoot',
        expected: expectedMp.toFixed(2) + '%',
        actual: ideal.metrics.overshoot.toFixed(2) + '%',
        pass: Math.abs(ideal.metrics.overshoot - expectedMp) < 0.05
      },
      {
        name: 'Slow sensor (τs = 0.25 s) — overshoot increases',
        expected: '> 14.5% (≈ 54.00%)',
        actual: slow.metrics.overshoot.toFixed(2) + '%',
        pass: slow.metrics.overshoot > ideal.metrics.overshoot + 35 && Math.abs(slow.metrics.overshoot - 54.0) < 0.1
      },
      {
        name: 'Critical lag (τs = 1.00 s) — sustained oscillation period',
        expected: expectedPeriod.toFixed(3) + ' s',
        actual: Number.isFinite(avgPeriod) ? avgPeriod.toFixed(3) + ' s' : '—',
        pass: Number.isFinite(avgPeriod) && Math.abs(avgPeriod - expectedPeriod) < 0.03
      },
      {
        name: 'Stable closed loop — true y and sensor ym both reach r',
        expected: '|r − y| < 0.001; |y − ym| < 0.001',
        actual: settled.metrics.endError.toExponential(1) + '; ' + settled.metrics.endSensorGap.toExponential(1),
        pass: settled.metrics.steady && Math.abs(settled.metrics.endError) < 0.001 && settled.metrics.endSensorGap < 0.001
      }
    ];
  }

  global.P2Engine = { defaults, simulate, validations };

  if (typeof document === 'undefined') return;

  function initP2() {
    const nav = document.querySelector('.main-nav');
    const main = document.querySelector('main.app-main');
    if (!nav || !main || document.getElementById('page-p2')) return;

    // Inject scoped styles matching P1 without modifying style.css
    if (!document.getElementById('p2-scoped-style')) {
      const style = document.createElement('style');
      style.id = 'p2-scoped-style';
      style.textContent = [
        '#page-p2 [hidden] { display: none !important; }',
        '#page-p2 p { line-height: 1.65; }',
        '#page-p2 .chart-box svg text { font-size: 14px; fill: #71788e; font-family: inherit; }',
        '#page-p2 .validation-panel p, #p2-feedback { font-size: 13px; color: #656a80; }',
        '#page-p2 summary { cursor: pointer; color: #6556be; font-weight: 650; margin-top: 15px; }',
        '#page-p2 .metric-value { font-size: 23px; }',
        '#page-p2 .p2-sensor-highlight { background: #f8f7ff; border: 1px solid #e4e0fa; border-radius: 12px; padding: 14px; margin-top: 12px; }',
        '#page-p2 .p2-sensor-highlight label { color: #5146c3; }',
        '#page-p2 .p2-legend { display: flex; flex-wrap: wrap; gap: 20px; font-size: 12px; margin-bottom: 0; }',
        '#page-p2 .p2-legend .k-true { color: #6d62e8; font-weight: 650; }',
        '#page-p2 .p2-legend .k-meas { color: #16a6a0; font-weight: 650; }',
        '#page-p2 .p2-legend .k-ref { color: #7d879b; }'
      ].join('\n');
      document.head.appendChild(style);
    }

    // Add Control · P2 tab next to Control · P1 using the exact .nav-tab class
    const controlTab = document.getElementById('tab-control');
    const tabP2 = document.createElement('button');
    tabP2.type = 'button';
    tabP2.id = 'tab-p2';
    tabP2.className = 'nav-tab';
    tabP2.setAttribute('role', 'tab');
    tabP2.setAttribute('aria-controls', 'page-p2');
    tabP2.setAttribute('aria-selected', 'false');
    tabP2.dataset.page = 'p2';
    tabP2.textContent = 'Control · P2';
    if (controlTab && controlTab.nextSibling) {
      nav.insertBefore(tabP2, controlTab.nextSibling);
    } else {
      nav.appendChild(tabP2);
    }

    // Add #page-p2 section using the exact .page-view class
    const root = document.createElement('section');
    root.id = 'page-p2';
    root.className = 'page-view';
    root.setAttribute('role', 'tabpanel');
    root.setAttribute('aria-labelledby', 'tab-p2');
    root.hidden = true;
    const controlPage = document.getElementById('page-control');
    if (controlPage && controlPage.nextSibling) {
      main.insertBefore(root, controlPage.nextSibling);
    } else {
      main.appendChild(root);
    }

    // Keep P2 page navigation in sync with module1.js's single-page router
    function activateP2() {
      document.querySelectorAll('.page-view').forEach((sec) => {
        sec.hidden = sec.id !== 'page-p2';
      });
      document.querySelectorAll('.main-nav [data-page]').forEach((tab) => {
        const active = tab.id === 'tab-p2';
        tab.classList.toggle('active', active);
        tab.setAttribute('aria-selected', String(active));
        tab.tabIndex = active ? 0 : -1;
      });
      window.scrollTo(0, 0);
    }

    function deactivateP2() {
      root.hidden = true;
      tabP2.classList.remove('active');
      tabP2.setAttribute('aria-selected', 'false');
      tabP2.tabIndex = -1;
    }

    tabP2.addEventListener('click', () => {
      if (window.location.hash === '#p2') activateP2();
      else window.location.hash = 'p2';
    });

    tabP2.addEventListener('keydown', (event) => {
      const allTabs = [...document.querySelectorAll('.main-nav [data-page]')];
      const idx = allTabs.indexOf(tabP2);
      if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(event.key)) return;
      event.preventDefault();
      const nextIdx = event.key === 'Home'
        ? 0
        : event.key === 'End'
          ? allTabs.length - 1
          : (idx + (event.key === 'ArrowRight' ? 1 : -1) + allTabs.length) % allTabs.length;
      allTabs[nextIdx].focus();
      allTabs[nextIdx].click();
    });

    document.querySelectorAll('[data-page], [data-open]').forEach((btn) => {
      if (btn.id === 'tab-p2') return;
      btn.addEventListener('click', () => {
        if ((btn.dataset.page || btn.dataset.open) !== 'p2') deactivateP2();
      });
    });

    window.addEventListener('hashchange', () => {
      if (window.location.hash.slice(1) === 'p2') activateP2();
      else deactivateP2();
    });

    const $ = (id) => document.getElementById('p2-' + id);
    let state = { ...defaults };
    let frame = 0;

    const sensorField = ['taus', 'Sensor time constant τs (s)', 0, 3, 0.05];
    const fields = [
      ['kp', 'Proportional gain Kp', 0, 30, 0.1],
      ['ki', 'Integral gain Ki', 0, 20, 0.1],
      ['kd', 'Derivative gain Kd', 0, 5, 0.05],
      ['r', 'Setpoint step', 0.1, 5, 0.1],
      ['K', 'Plant gain K', 0.1, 3, 0.1],
      ['tau', 'Plant time constant τ (s)', 0.2, 6, 0.1],
      ['m', 'Mass m (kg)', 0.5, 4, 0.1],
      ['b', 'Damping b (N·s/m)', 0.2, 20, 0.1],
      ['k', 'Spring k (N/m)', 1, 20, 0.1],
      ['lo', 'Control lower limit', -100, 0, 1],
      ['hi', 'Control upper limit', 0.1, 100, 0.1],
      ['T', 'Duration (s)', 5, 60, 1]
    ];

    function fieldHTML([id, label, min, max, step], extraClass = '') {
      return `<div class="p1-field ${extraClass}" data-field="${id}">` +
        `<label for="p2-${id}">${label}</label>` +
        `<div class="p1-input-pair">` +
        `<input type="range" id="p2-${id}-slider" aria-label="${label} slider" min="${min}" max="${max}" step="${step}" value="${state[id]}">` +
        `<input type="number" id="p2-${id}" min="${min}" max="${max}" step="${step}" value="${state[id]}">` +
        `</div></div>`;
    }

    root.innerHTML = `
      <div class="module-hero">
        <a class="breadcrumb-link" href="#home">← Back to home</a>
        <div class="module-title-row">
          <div>
            <p class="section-kicker">CONTROL / HOMEWORK 2 / P2</p>
            <h1>A slow sensor in the loop<span class="title-period">.</span></h1>
            <p>The controller only sees what the sensor reports. Increase τ<sub>s</sub> and watch lag turn into overshoot and oscillation.</p>
          </div>
          <span class="formula-badge">τ<sub>s</sub> dy<sub>m</sub>/dt + y<sub>m</sub> = y · e = r − y<sub>m</sub></span>
        </div>
      </div>

      <div class="p1-presets" aria-label="Sensor lag examples">
        <button type="button" data-preset="ideal">Ideal sensor · τs = 0 s</button>
        <button type="button" data-preset="mild">Moderate lag · τs = 0.25 s</button>
        <button type="button" data-preset="ring">Heavy ringing · τs = 0.60 s</button>
        <button type="button" data-preset="osc">Sustained oscillation · τs = 1.0 s</button>
        <button type="button" id="p2-reset">Reset</button>
      </div>

      <section class="panel p1-loop">
        <h3>Closed-loop block diagram with sensor dynamics</h3>
        <svg viewBox="0 0 900 185" role="img" aria-label="Setpoint r enters summing junction. Error e = r minus sensor output ym goes to PID, Limits, and Plant. True output y feeds back through a first-order sensor with time constant tau_s to produce ym.">
          <defs>
            <marker id="p2-arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">
              <path d="M0 0 L8 4 L0 8" fill="#6c63d9"/>
            </marker>
            <marker id="p2-arrow-teal" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">
              <path d="M0 0 L8 4 L0 8" fill="#16a6a0"/>
            </marker>
          </defs>
          <g fill="none" stroke="#6c63d9" stroke-width="2">
            <path d="M40 55H134 M176 55H245 M375 55H445 M575 55H645 M775 55H865" marker-end="url(#p2-arrow)"/>
            <circle cx="155" cy="55" r="20"/>
            <rect x="245" y="25" width="130" height="60" rx="12"/>
            <rect x="445" y="25" width="130" height="60" rx="12"/>
            <rect x="645" y="25" width="130" height="60" rx="12"/>
          </g>
          <g fill="none" stroke="#16a6a0" stroke-width="2.2">
            <path d="M815 55V138H555" marker-end="url(#p2-arrow-teal)"/>
            <rect x="415" y="108" width="140" height="58" rx="12" fill="#f2fbfa"/>
            <path d="M415 138H155V77" marker-end="url(#p2-arrow-teal)"/>
          </g>
          <g fill="#41385d" font-size="16" text-anchor="middle">
            <text x="67" y="41">r</text>
            <text x="155" y="61">Σ</text>
            <text x="126" y="44">+</text>
            <text x="175" y="101">−</text>
            <text x="208" y="41">e = r − ym</text>
            <text x="310" y="61">PID</text>
            <text x="510" y="61">Limits</text>
            <text x="607" y="41">u</text>
            <text x="710" y="61">Plant</text>
            <text x="842" y="41" fill="#6d62e8" font-weight="700">y (true)</text>
            <text x="485" y="133" fill="#117a75" font-weight="700">Sensor</text>
            <text x="485" y="153" fill="#117a75" font-size="13">1 / (τs s + 1)</text>
            <text x="285" y="128" fill="#117a75" font-weight="700">ym (measured)</text>
            <text x="485" y="180" font-size="14" fill="#656a80">Controller only sees ym · true output y can overshoot while ym is still catching up</text>
          </g>
        </svg>
      </section>

      <div class="workspace-grid">
        <aside class="controls-stack">
          <section class="panel controls-panel">
            <h3>Simulation controls</h3>
            ${fieldHTML(sensorField, 'p2-sensor-highlight')}
            <div class="p1-field">
              <label for="p2-plant">Plant model</label>
              <select id="p2-plant">
                <option value="first">First order — Module 1</option>
                <option value="second">Mass–spring–damper — Module 3</option>
              </select>
            </div>
            ${fields.map((f) => fieldHTML(f)).join('')}
            <p class="p1-model" id="p2-model"></p>
          </section>

          <section class="panel prediction-panel">
            <h3>Predict, then check</h3>
            <p>With K = τ = 1, Kp = 2, Ki = 6 and an ideal sensor (τs = 0 s), the loop settles smoothly with ~14.5% overshoot. What happens if we slow the sensor to <strong>τs = 0.60 s</strong> without changing the PID gains?</p>
            <label class="p1-choice"><input type="radio" name="p2-prediction" value="oscillate"> Overshoot grows sharply and the loop oscillates</label>
            <label class="p1-choice"><input type="radio" name="p2-prediction" value="slower"> It simply rises more slowly with no overshoot</label>
            <label class="p1-choice"><input type="radio" name="p2-prediction" value="same"> True output stays the same because the plant is unchanged</label>
            <button type="button" class="primary-button" id="p2-check">Check with a simulation →</button>
            <p id="p2-feedback" role="status"></p>
          </section>
        </aside>

        <div class="results-stack">
          <section class="panel chart-panel">
            <div class="chart-heading">
              <div>
                <p class="chart-eyebrow">TRUE VS. MEASURED RESPONSE</p>
                <h3>True output y &amp; sensor reading y<sub>m</sub></h3>
              </div>
              <span class="live-chip">● Live</span>
            </div>
            <p class="p2-legend">
              <span class="k-true">━ True output y</span>
              <span class="k-meas">━ Sensor reading y<sub>m</sub></span>
              <span class="k-ref">┄ Setpoint r</span>
            </p>
            <div class="chart-box">
              <svg id="p2-output" viewBox="0 0 900 350" role="img" aria-label="True output y, sensor reading ym, and setpoint r against time"></svg>
            </div>
            <p class="chart-footnote" id="p2-status" role="status"></p>
          </section>

          <div class="metrics-grid p1-metrics">
            <article class="metric-card">
              <span class="metric-label">True overshoot</span>
              <div class="metric-value" id="p2-metric-overshoot">—</div>
              <p>Peak y above target / final value</p>
            </article>
            <article class="metric-card">
              <span class="metric-label">Peak sensor lag</span>
              <div class="metric-value" id="p2-metric-lag">—</div>
              <p>Max |y − y<sub>m</sub>| hidden from PID</p>
            </article>
            <article class="metric-card">
              <span class="metric-label">Settling time</span>
              <div class="metric-value" id="p2-metric-settling">—</div>
              <p>True y within ±2% of final</p>
            </article>
            <article class="metric-card">
              <span class="metric-label">Steady-state error</span>
              <div class="metric-value" id="p2-metric-error">—</div>
              <p>Setpoint r − final true y</p>
            </article>
          </div>

          <section class="panel chart-panel">
            <p class="chart-eyebrow">ACTUATOR EFFORT</p>
            <h3>Control signal u</h3>
            <p class="p1-legend"><span>━ Applied u</span><span class="p1-limit-key">┄ Actuator limits</span></p>
            <div class="chart-box">
              <svg id="p2-control" viewBox="0 0 900 350" role="img" aria-label="Applied control signal u and actuator limits against time"></svg>
            </div>
            <p class="chart-footnote" id="p2-saturation"></p>
          </section>

          <section class="panel validation-panel">
            <h3>Why a slow sensor destabilizes the loop</h3>
            <p>A first-order sensor with time constant <strong>τ<sub>s</sub></strong> introduces a frequency-dependent phase lag of <strong>−tan<sup>−1</sup>(ωτ<sub>s</sub>)</strong> and attenuates high-frequency changes in the feedback path.</p>
            <p>Look closely at the first rise on the plot: when the <strong>true output y</strong> crosses the setpoint <strong>r</strong>, the <strong>sensor reading y<sub>m</sub></strong> is still below <strong>r</strong>. Because the controller sees <em>e = r − y<sub>m</sub> &gt; 0</em>, it keeps driving the actuator forward after the plant has already arrived—causing overshoot and oscillation.</p>
            <details>
              <summary>Equations &amp; implementation</summary>
              <p>Sensor ODE: dy<sub>m</sub>/dt = (y − y<sub>m</sub>) / τ<sub>s</sub> (when τ<sub>s</sub> = 0, y<sub>m</sub> = y identically).</p>
              <p>Controller ODEs: e = r − y<sub>m</sub>; dI/dt = e; dz/dt = (y<sub>m</sub> − z) / T<sub>f</sub> (T<sub>f</sub> = 0.04 s); u = clip(K<sub>p</sub>e + K<sub>i</sub>I − K<sub>d</sub>(y<sub>m</sub> − z)/T<sub>f</sub>, lo, hi).</p>
              <p>For K = τ = 1 with PI control, Routh–Hurwitz gives the exact stability limit τ<sub>s,crit</sub> = (1 + K<sub>p</sub>) / (K<sub>i</sub> − K<sub>p</sub> − 1) when K<sub>i</sub> &gt; K<sub>p</sub> + 1.</p>
            </details>
          </section>
        </div>
      </div>

      <section class="panel validation-panel p1-validation">
        <div class="chart-heading">
          <div>
            <p class="chart-eyebrow">ENGINEERING CHECK</p>
            <h3>Slow-sensor validation tests</h3>
          </div>
          <span class="small-tag" id="p2-test-count"></span>
        </div>
        <p>Reference tests use K = τ = 1, K<sub>p</sub> = 2, K<sub>i</sub> = 6 and wide actuator limits, independently of the current sliders.</p>
        <div class="table-scroll">
          <table>
            <thead>
              <tr><th>Check</th><th>Expected</th><th>Calculated</th><th>Status</th></tr>
            </thead>
            <tbody id="p2-tests"></tbody>
          </table>
        </div>
      </section>
    `;

    function plotOutput(rows, rVal) {
      const svg = $('output');
      const W = 900, H = 350, L = 76, R = 24, top = 24, B = 52;
      let low = 0, high = rVal;
      for (const q of rows) {
        low = Math.min(low, q.y, q.ym);
        high = Math.max(high, q.y, q.ym);
      }
      const pad = Math.max((high - low) * 0.12, 0.05);
      low -= pad;
      high += pad;
      const end = rows[rows.length - 1].t || state.T;
      const x = (t) => L + (t / end) * (W - L - R);
      const y = (v) => H - B - ((v - low) / (high - low)) * (H - top - B);

      let html = '';
      for (let i = 0; i <= 5; i++) {
        const t = (end * i) / 5;
        const v = low + ((high - low) * i) / 5;
        html += `<path d="M${L} ${y(v)}H${W - R}" stroke="#e9e7f0"/>` +
          `<text x="${L - 10}" y="${y(v) + 4}" text-anchor="end">${v.toFixed(Math.abs(v) > 100 ? 0 : 2)}</text>` +
          `<text x="${x(t)}" y="${H - B + 24}" text-anchor="middle">${t.toFixed(1)}</text>`;
      }

      // Setpoint reference line
      html += `<path d="M${L} ${y(rVal)}H${W - R}" stroke="#8791a8" stroke-width="2" stroke-dasharray="7 5"/>` +
        `<text x="${W - R - 5}" y="${y(rVal) - 6}" text-anchor="end" fill="#8791a8">r = ${rVal}</text>`;

      const stride = Math.max(1, Math.floor(rows.length / 2500));
      const pts = rows.filter((_, i) => i % stride === 0 || i === rows.length - 1);

      const truePath = pts.map((q, i) => `${i ? 'L' : 'M'}${x(q.t).toFixed(2)} ${y(q.y).toFixed(2)}`).join(' ');
      const measPath = pts.map((q, i) => `${i ? 'L' : 'M'}${x(q.t).toFixed(2)} ${y(q.ym).toFixed(2)}`).join(' ');

      // Sensor reading ym (teal) and True output y (purple)
      html += `<path d="${measPath}" fill="none" stroke="#16a6a0" stroke-width="2.5" stroke-dasharray="8 4"/>`;
      html += `<path d="${truePath}" fill="none" stroke="#6d62e8" stroke-width="2.8"/>`;

      // Highlight max lag point between true y and measured ym
      let maxIdx = 0;
      for (let i = 1; i < pts.length; i++) {
        if (Math.abs(pts[i].y - pts[i].ym) > Math.abs(pts[maxIdx].y - pts[maxIdx].ym)) maxIdx = i;
      }
      const gap = Math.abs(pts[maxIdx].y - pts[maxIdx].ym);
      if (gap > 0.03 * Math.max(1, Math.abs(rVal))) {
        const gx = x(pts[maxIdx].t);
        const gy1 = y(pts[maxIdx].y);
        const gy2 = y(pts[maxIdx].ym);
        html += `<line x1="${gx.toFixed(1)}" y1="${gy1.toFixed(1)}" x2="${gx.toFixed(1)}" y2="${gy2.toFixed(1)}" stroke="#e78d54" stroke-width="2" stroke-dasharray="3 3"/>`;
        html += `<circle cx="${gx.toFixed(1)}" cy="${gy1.toFixed(1)}" r="4" fill="#6d62e8"/>`;
        html += `<circle cx="${gx.toFixed(1)}" cy="${gy2.toFixed(1)}" r="4" fill="#16a6a0"/>`;
      }

      html += `<text x="${W / 2}" y="${H - 8}" text-anchor="middle">Time (s)</text>` +
        `<text x="${L}" y="16">Output (y vs. ym)</text>`;
      svg.innerHTML = html;
    }

    function plotControl(rows, refs) {
      const svg = $('control');
      const W = 900, H = 350, L = 76, R = 24, top = 24, B = 52;
      let low = 0, high = 0;
      for (const q of rows) {
        low = Math.min(low, q.u);
        high = Math.max(high, q.u);
      }
      refs.forEach((q) => {
        low = Math.min(low, q.value);
        high = Math.max(high, q.value);
      });
      const pad = Math.max((high - low) * 0.12, 0.05);
      low -= pad;
      high += pad;
      const end = rows[rows.length - 1].t || state.T;
      const x = (t) => L + (t / end) * (W - L - R);
      const y = (v) => H - B - ((v - low) / (high - low)) * (H - top - B);

      let html = '';
      for (let i = 0; i <= 5; i++) {
        const t = (end * i) / 5;
        const v = low + ((high - low) * i) / 5;
        html += `<path d="M${L} ${y(v)}H${W - R}" stroke="#e9e7f0"/>` +
          `<text x="${L - 10}" y="${y(v) + 4}" text-anchor="end">${v.toFixed(Math.abs(v) > 100 ? 0 : 2)}</text>` +
          `<text x="${x(t)}" y="${H - B + 24}" text-anchor="middle">${t.toFixed(1)}</text>`;
      }
      refs.forEach((q) => {
        html += `<path d="M${L} ${y(q.value)}H${W - R}" stroke="${q.color}" stroke-width="2" stroke-dasharray="7 5"/>` +
          `<text x="${W - R - 5}" y="${y(q.value) - 6}" text-anchor="end" fill="${q.color}">${q.label}</text>`;
      });
      const stride = Math.max(1, Math.floor(rows.length / 2500));
      const pts = rows.filter((_, i) => i % stride === 0 || i === rows.length - 1);
      html += `<path d="${pts.map((q, i) => `${i ? 'L' : 'M'}${x(q.t).toFixed(2)} ${y(q.u).toFixed(2)}`).join(' ')}" fill="none" stroke="#7563dc" stroke-width="2.6"/>` +
        `<text x="${W / 2}" y="${H - 8}" text-anchor="middle">Time (s)</text>` +
        `<text x="${L}" y="16">Control effort</text>`;
      svg.innerHTML = html;
    }

    function render() {
      try {
        const result = simulate(state);
        const m = result.metrics;
        plotOutput(result.rows, state.r);
        plotControl(result.rows, [
          { value: state.lo, color: '#ce8155', label: 'min ' + state.lo },
          { value: state.hi, color: '#ce8155', label: 'max ' + state.hi }
        ]);

        $('metric-overshoot').textContent = m.overshoot === null
          ? '—'
          : m.overshoot.toFixed(1) + '%' + (m.steady ? '' : ' (osc.)');
        $('metric-lag').textContent = m.maxLag.toFixed(3);
        $('metric-settling').textContent = m.settling === null ? '—' : m.settling.toFixed(2) + ' s';
        $('metric-error').textContent = m.error === null ? '—' : m.error.toFixed(4);

        let status;
        if (result.failed) {
          status = 'Simulation stopped: output exceeded the numerical safety range (unstable oscillation).';
        } else if (m.steady) {
          status = `Settled at y ≈ ${m.final.toFixed(4)} (sensor ym ≈ ${m.final.toFixed(4)}). Peak sensor lag |y − ym| = ${m.maxLag.toFixed(3)}.`;
        } else {
          status = `Loop is oscillating / not settled within ${state.T} s (peak y = ${m.peak.toFixed(3)}, max |y − ym| = ${m.maxLag.toFixed(3)}). Reduce τs or retune gains.`;
        }
        $('status').textContent = status;
        $('saturation').textContent = `Applied u is clipped to [${state.lo}, ${state.hi}]. Saturated during ${(100 * result.clipped).toFixed(1)}% of sampled instants.`;

        const plantDesc = state.plant === 'first'
          ? `Plant G(s) = ${state.K} / (${state.tau}s + 1)`
          : `Plant G(s) = 1 / (${state.m}s² + ${state.b}s + ${state.k})`;
        const sensorDesc = state.taus <= 1e-6
          ? 'Sensor H(s) = 1 (ideal, τs = 0 s)'
          : `Sensor H(s) = 1 / (${state.taus}s + 1)`;
        $('model').textContent = `${plantDesc} · ${sensorDesc}`;
      } catch (e) {
        $('status').textContent = e.message;
        ['overshoot', 'lag', 'settling', 'error'].forEach((k) => { $('metric-' + k).textContent = '—'; });
        $('output').innerHTML = '';
        $('control').innerHTML = '';
        $('saturation').textContent = '';
      }
    }

    function schedule() {
      cancelAnimationFrame(frame);
      frame = requestAnimationFrame(render);
    }

    const allFields = [sensorField, ...fields];

    function sync() {
      $('plant').value = state.plant;
      for (const [id] of allFields) {
        $(id).value = state[id];
        $(id + '-slider').value = state[id];
      }
      root.querySelectorAll('[data-field]').forEach((el) => {
        const id = el.dataset.field;
        if (id === 'taus') return;
        el.hidden = state.plant === 'first'
          ? ['m', 'b', 'k'].includes(id)
          : ['K', 'tau'].includes(id);
      });
      schedule();
    }

    for (const [id, , min, max] of allFields) {
      $(id + '-slider').addEventListener('input', () => {
        state[id] = Number($(id + '-slider').value);
        $(id).value = state[id];
        schedule();
      });
      $(id).addEventListener('input', () => {
        const v = $(id).valueAsNumber;
        if (Number.isFinite(v) && v >= min && v <= max) {
          state[id] = v;
          $(id + '-slider').value = v;
          schedule();
        }
      });
      $(id).addEventListener('change', () => {
        $(id).value = state[id];
      });
    }

    $('plant').addEventListener('change', () => {
      state.plant = $('plant').value;
      sync();
    });

    root.querySelectorAll('[data-preset]').forEach((button) => {
      button.addEventListener('click', () => {
        const kind = button.dataset.preset;
        const map = { ideal: 0, mild: 0.25, ring: 0.6, osc: 1.0 };
        state = {
          ...defaults,
          taus: map[kind] ?? defaults.taus,
          T: kind === 'osc' ? 25 : 15
        };
        $('feedback').textContent = '';
        sync();
      });
    });

    $('reset').addEventListener('click', () => {
      state = { ...defaults };
      $('feedback').textContent = '';
      root.querySelectorAll('[name="p2-prediction"]').forEach((el) => { el.checked = false; });
      sync();
    });

    $('check').addEventListener('click', () => {
      const answer = root.querySelector('[name="p2-prediction"]:checked');
      if (!answer) {
        $('feedback').textContent = 'Choose a prediction first.';
        return;
      }
      const before = simulate({ ...defaults, taus: 0 }).metrics;
      state = { ...defaults, taus: 0.6, T: 15 };
      const after = simulate(state).metrics;
      $('feedback').textContent =
        `${answer.value === 'oscillate' ? 'Correct!' : 'Not quite: overshoot grows and the loop oscillates.'} ` +
        `Overshoot jumped from ${before.overshoot.toFixed(1)}% (τs = 0 s) to ${after.overshoot.toFixed(1)}% (τs = 0.60 s) ` +
        `because ym lagged behind y by up to ${after.maxLag.toFixed(3)}.`;
      sync();
    });

    const checks = validations();
    $('tests').innerHTML = checks.map((c) =>
      `<tr><td>${c.name}</td><td>${c.expected}</td><td>${c.actual}</td>` +
      `<td><span class="${c.pass ? 'pass-chip' : 'p1-fail'}">${c.pass ? 'PASS' : 'FAIL'}</span></td></tr>`
    ).join('');
    $('test-count').textContent = `${checks.filter((c) => c.pass).length} / ${checks.length} passed`;

    sync();
    if (window.location.hash.slice(1) === 'p2') activateP2();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initP2);
  } else {
    initP2();
  }
})(typeof window !== 'undefined' ? window : globalThis);