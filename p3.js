
/* ENCS4380 Homework 2 — P3: Digital PID */

(function (global) {
    'use strict';

    const defaults = {
        ts: 0.1,
        kp: 2,
        ki: 1,
        kd: 0,
        K: 1,
        tau: 1,
        antiWindup: true,
        r: 1,
        uMin: 0,
        uMax: 2,
        duration: 15
    };

    const clamp = (v, lo, hi) =>
        Math.max(lo, Math.min(hi, v));

    // Digital PID controller
    function pidStep(p, y, integral, previousError) {

        const error = p.r - y;

        const P = p.kp * error;

        const nextIntegral = integral + error * p.ts;

        const D =
            p.kd * (error - previousError) / p.ts;

        const raw = P + p.ki * nextIntegral + D;

        const blocks =
            (raw > p.uMax && error > 0) ||
            (raw < p.uMin && error < 0);

        // Conditional integration
        if (!p.antiWindup || !blocks) {
            integral = nextIntegral;
        }

        const I = p.ki * integral;

        return {
            error,
            P,
            I,
            D,
            integral,
            u: clamp(P + I + D, p.uMin, p.uMax)
        };
    }

    // Plant simulation using Euler integration
    function simulatePID(input = {}) {

        const p = { ...defaults, ...input };

        let y = 0;
        let integral = 0;
        let previousError = 0;
        let u = 0;

        const output = [{ t: 0, y: 0 }];
        const control = [];

        for (
            let k = 0;
            k * p.ts < p.duration - 1e-9;
            k++
        ) {

            const t = k * p.ts;

            const c = pidStep(
                p,
                y,
                integral,
                previousError
            );

            integral = c.integral;
            previousError = c.error;
            u = c.u;

            control.push({
                t,
                u,
                I: c.I
            });

            const interval =
                Math.min(p.ts, p.duration - t);

            const steps = Math.ceil(
                interval / Math.min(0.005, p.tau / 20)
            );

            const dt = interval / steps;

            for (let j = 1; j <= steps; j++) {

                y += dt * (p.K * u - y) / p.tau;

                output.push({
                    t: Math.min(p.duration, t + j * dt),
                    y
                });
            }
        }

        control.push({
            ...control[control.length - 1],
            t: p.duration
        });

        return {
            p,
            output,
            control,
            metrics: calculateMetrics(output, p.r)
        };
    }

    // Calculate performance metrics
    function calculateMetrics(data, r) {

        const last = data[data.length - 1];

        function crossing(level) {

            const i = data.findIndex(q => q.y >= level);

            if (i < 0) return null;
            if (i === 0) return data[0].t;

            const a = data[i - 1];
            const b = data[i];

            return a.t +
                (b.t - a.t) *
                (level - a.y) / (b.y - a.y);
        }

        const t10 = crossing(0.1 * r);
        const t90 = crossing(0.9 * r);

        const peak = Math.max(
            ...data.map(q => q.y)
        );

        let outside = -1;

        data.forEach((q, i) => {
            if (Math.abs(q.y - r) > 0.02 * r) {
                outside = i;
            }
        });

        const candidate = data[outside + 1];

        const tail = data
            .filter(q => q.t >= last.t - 3)
            .map(q => q.y);

        const steady =
            Math.max(...tail) -
            Math.min(...tail) < 0.005 * r;

        return {
            rise:
                t10 === null || t90 === null
                    ? null
                    : t90 - t10,

            overshoot:
                Math.max(0, 100 * (peak - r) / r),

            settling:
                candidate && last.t - candidate.t >= 1
                    ? candidate.t
                    : null,

            error: steady ? r - last.y : null,

            final: last.y
        };
    }

    // Validation tests
    function validations() {

        const base = {
            K: 1,
            tau: 1,
            r: 1,
            ts: 0.01,
            uMin: -100,
            uMax: 100,
            duration: 25
        };

        const a = simulatePID({
            ...base,
            kp: 9,
            ki: 0,
            kd: 0
        });

        const t63 = a.output.find(
            q => q.y >= 0.9 * (1 - Math.exp(-1))
        ).t;

        const b = simulatePID({
            ...base,
            kp: 1,
            ki: 1,
            kd: 0
        });

        const wn = Math.sqrt(base.K / base.tau);

        const zeta =
            (1 + base.K) /
            (2 * Math.sqrt(base.tau * base.K));

        const c = pidStep({
            ...defaults,
            kp: 2,
            ki: 10,
            kd: 0
        }, 0, 0, 0);

        const d = pidStep({
            ...defaults,
            kp: 2,
            ki: 10,
            kd: 0,
            antiWindup: false
        }, 0, 0, 0);

        return [
            {
                name: 'P only: time constant',
                expected: '0.10 s',
                actual: t63.toFixed(3) + ' s',
                pass: Math.abs(t63 - 0.1) < 0.02
            },
            {
                name: 'P only: steady-state error',
                expected: '0.10',
                actual: (1 - a.metrics.final).toFixed(4),
                pass:
                    Math.abs(1 - a.metrics.final - 0.1)
                    < 0.001
            },
            {
                name: 'PI reference: continuous model',
                expected: 'ωn = 1, ζ = 1',
                actual:
                    `ωn = ${wn.toFixed(2)}, ζ = ${zeta.toFixed(2)}`,
                pass:
                    Math.abs(wn - 1) < 1e-9 &&
                    Math.abs(zeta - 1) < 1e-9
            },
            {
                name: 'Stable PI: step tracking',
                expected: '|error| < 0.001',
                actual:
                    Math.abs(1 - b.metrics.final)
                        .toExponential(2),
                pass:
                    Math.abs(1 - b.metrics.final) < 0.001
            },
            {
                name: 'Anti-windup during saturation',
                expected: 'I on = 0; I off > 0',
                actual:
                    `${c.I.toFixed(2)} / ${d.I.toFixed(2)}`,
                pass:
                    c.I === 0 &&
                    d.I > 0 &&
                    c.u === 2 &&
                    d.u === 2
            }
        ];
    }

    // Arduino code generator
    function generateArduinoCode(p) {

        return `// Digital PID Controller
// Educational Arduino Uno example
// Connect PWM to a suitable motor driver.
// Adapt sensor calibration to your hardware.

const float Kp = ${p.kp.toFixed(2)};
const float Ki = ${p.ki.toFixed(2)};
const float Kd = ${p.kd.toFixed(2)};
const float Ts = ${p.ts.toFixed(2)};

const bool antiWindup = ${p.antiWindup};

const float setpoint = 1.0;
const float uMin = 0.0;
const float uMax = 2.0;

const unsigned long sampleMs =
    ${Math.round(p.ts * 1000)}UL;

const int sensorPin = A0;
const int pwmPin = 9;

float integral = 0;
float previousError = 0;

unsigned long lastTime = 0;

void setup() {

    pinMode(sensorPin, INPUT);
    pinMode(pwmPin, OUTPUT);

    lastTime = millis() - sampleMs;
}

void loop() {

    unsigned long now = millis();

    if (now - lastTime < sampleMs)
        return;

    lastTime = now;

    // Example sensor calibration
    float y = 3.0f * analogRead(sensorPin) / 1023.0f;

    float error = setpoint - y;

    // Proportional term
    float P = Kp * error;

    // Integral term
    float nextIntegral = integral + error * Ts;

    // Derivative term
    float D = Kd * (error - previousError) / Ts;

    float raw = P + Ki * nextIntegral + D;

    // Anti-windup
    bool blocks =
        (raw > uMax && error > 0) ||
        (raw < uMin && error < 0);

    if (!antiWindup || !blocks) {
        integral = nextIntegral;
    }

    float I = Ki * integral;

    // Apply actuator limits
    float u = constrain(P + I + D, uMin, uMax);

    // Convert control signal to PWM
    int pwm = (int)(255.0f * u / uMax + 0.5f);

    analogWrite(pwmPin, pwm);

    previousError = error;
}`;
    }

    global.P3Engine = {
        defaults,
        pidStep,
        simulatePID,
        validations,
        generateArduinoCode
    };

    if (typeof document === 'undefined') return;

    function initP3() {

        const nav =
            document.querySelector('.main-nav');

        const main =
            document.querySelector('main.app-main');

        if (!nav || !main ||
            document.getElementById('page-p3')) {
            return;
        }

        // Create navigation tab
        const tab = document.createElement('button');

        tab.type = 'button';
        tab.id = 'tab-p3';
        tab.className = 'nav-tab';
        tab.dataset.page = 'p3';
        tab.textContent = 'Control · P3';

        tab.setAttribute('role', 'tab');
        tab.setAttribute('aria-controls', 'page-p3');
        tab.setAttribute('aria-selected', 'false');

        nav.appendChild(tab);

        // Create P3 page
        const root = document.createElement('section');

        root.id = 'page-p3';
        root.className = 'page-view';
        root.hidden = true;

        root.setAttribute('role', 'tabpanel');
        root.setAttribute('aria-labelledby', 'tab-p3');

        main.appendChild(root);

        const $ = id =>
            document.getElementById('p3-' + id);

        // Slider settings
        const fields = [
            ['ts', 'Sampling Time (Ts)', 0.01, 2, 0.01, ' s'],
            ['kp', 'Proportional Gain (Kp)', 0, 20, 0.1, ''],
            ['ki', 'Integral Gain (Ki)', 0, 20, 0.1, ''],
            ['kd', 'Derivative Gain (Kd)', 0, 5, 0.05, ''],
            ['K', 'Plant Gain (K)', 0.1, 3, 0.1, ''],
            ['tau', 'Plant Time Constant (τ)', 0.1, 6, 0.1, ' s']
        ];

        const fieldHTML =
            ([id, label, min, max, step, unit]) => `

            <div class="control-block">

                <div class="control-top">
                    <label for="p3-${id}">
                        ${label}
                    </label>

                    <output id="p3-${id}-value">
                        ${defaults[id].toFixed(2)}${unit}
                    </output>
                </div>

                <input type="range"
                       id="p3-${id}"
                       min="${min}"
                       max="${max}"
                       step="${step}"
                       value="${defaults[id]}">

                <div class="range-labels">
                    <span>${min}${unit}</span>
                    <span>${max}${unit}</span>
                </div>

            </div>
        `;

        const metricHTML = (id, label, desc) => `
            <article class="metric-card">

                <span class="metric-label">
                    ${label}
                </span>

                <div class="metric-value"
                     id="p3-${id}">—</div>

                <p>${desc}</p>

            </article>
        `;

        // Page interface
        root.innerHTML = `

            <div class="module-hero">

                <a class="breadcrumb-link" href="#home">
                    ← Back to home
                </a>

                <div class="module-title-row">

                    <div>
                        <p class="section-kicker">
                            CONTROL / HOMEWORK 2 / P3
                        </p>

                        <h1>
                            Digital PID<span class="title-period">.</span>
                        </h1>

                        <p>
                            Explore sampling time, digital control
                            and integral anti-windup.
                        </p>
                    </div>

                    <span class="formula-badge">
                        u[k] = P + I + D
                    </span>

                </div>
            </div>

            <section class="panel controls-panel">

                <div class="panel-heading">

                    <div class="panel-heading-icon purple">
                        ⚙
                    </div>

                    <div>
                        <h3>Simulation controls</h3>
                        <p>Tune your digital PID controller</p>
                    </div>

                </div>

                ${fields.map(fieldHTML).join('')}

                <label class="choice">
                    <input type="checkbox"
                           id="p3-anti-windup"
                           checked>
                    <span>Enable Anti-windup</span>
                </label>

                <p class="help-text" id="p3-model"></p>

                <div class="p1-presets">

                    <button type="button"
                            data-p3-preset="fast">
                        Fast sampling
                    </button>

                    <button type="button"
                            data-p3-preset="slow">
                        Slow sampling
                    </button>

                    <button type="button"
                            data-p3-preset="windup">
                        Windup example
                    </button>

                    <button type="button"
                            data-p3-preset="reset">
                        Reset
                    </button>

                </div>
            </section>

            <section class="panel chart-panel">

                <div class="chart-heading">
                    <div>
                        <p class="chart-eyebrow">
                            LIVE VISUALIZATION
                        </p>
                        <h3>System Response</h3>
                    </div>

                    <span class="live-chip">● Live</span>
                </div>

                <p class="help-text">
                    Purple: selected Ts ·
                    Teal dashed: fast reference (Ts = 0.01 s) ·
                    Gray dashed: setpoint
                </p>

                <div class="chart-box">
                    <svg id="p3-response-chart"
                         viewBox="0 0 900 350"
                         role="img"
                         aria-label="Plant output and fast-sampling reference">
                    </svg>
                </div>

                <p class="chart-footnote"
                   id="p3-comparison"></p>

            </section>

            <section class="panel chart-panel">

                <h3>Digital Control Signal</h3>

                <p class="help-text">
                    The signal is held constant between
                    sampling instants.
                    Limits: 0–2 (mapped to 0–100% PWM).
                </p>

                <div class="chart-box">
                    <svg id="p3-control-chart"
                         viewBox="0 0 900 350"
                         role="img"
                         aria-label="Stair-step control signal">
                    </svg>
                </div>

            </section>

            <section class="panel chart-panel">

                <h3>Integral Windup</h3>

                <p class="help-text">
                    Purple: integral term with current setting.
                    Teal dashed: opposite anti-windup setting.
                </p>

                <div class="chart-box">

                    <svg id="p3-integral-chart"
                         viewBox="0 0 900 350"
                         role="img"
                         aria-label="Integral term with and without anti-windup">
                    </svg>

                </div>

            </section>

            <div class="metrics-grid p1-metrics">

                ${metricHTML(
                    'rise',
                    'Rise Time',
                    '10% to 90% of setpoint'
                )}

                ${metricHTML(
                    'overshoot',
                    'Overshoot',
                    'Peak above setpoint'
                )}

                ${metricHTML(
                    'settling',
                    'Settling Time',
                    'Within ±2% of setpoint'
                )}

                ${metricHTML(
                    'error',
                    'Steady-State Error',
                    'Estimated over the final 3 s'
                )}

            </div>

            <section class="panel prediction-panel">

                <div class="prediction-heading">
                    <span class="sparkle">✦</span>
                    <h3>Predict &amp; check</h3>
                    <span class="mini-label">
                        QUICK CHALLENGE
                    </span>
                </div>

                <p>
                    For K = τ = 1, Kp = 2, Ki = 1,
                    Kd = 0, what happens when Ts
                    increases from 0.01 s to 2 s?
                </p>

                <label class="choice">
                    <input type="radio"
                           name="p3-prediction"
                           value="worse">
                    The output becomes more oscillatory.
                </label>

                <label class="choice">
                    <input type="radio"
                           name="p3-prediction"
                           value="same">
                    The response is unchanged.
                </label>

                <button type="button"
                        class="primary-button"
                        id="p3-check">

                    Check my prediction →

                </button>

                <p id="p3-feedback" role="status"></p>

            </section>

            <section class="panel">

                <p class="chart-eyebrow">
                    HARDWARE IMPLEMENTATION
                </p>

                <h3>Generated Arduino Code</h3>

                <p class="help-text">
                    Updates with the gains, sampling time
                    and anti-windup setting.
                    Educational analog sensor/PWM example.
                    Adapt calibration to your hardware.
                </p>

                <pre style="
                    background: #182235;
                    color: #e2e8f0;
                    padding: 20px;
                    border-radius: 12px;
                    overflow-x: auto;
                    max-width: 100%;
                    font-size: 13px;
                    line-height: 1.6;
                "><code id="p3-arduino-code"></code></pre>

                <button type="button"
                        id="p3-copy-code"
                        class="primary-button">

                    Copy Arduino Code

                </button>

                <p id="p3-copy-status" role="status"></p>

            </section>

            <section class="panel validation-panel">

                <h3>Validation Tests</h3>

                <p>
                    Independent reference checks:
                    K = τ = 1.
                    PI natural frequency and damping ratio
                    refer to the continuous model.
                    Wide actuator limits are used for
                    P/PI reference tests.
                </p>

                <div class="table-scroll">

                    <table>
                        <thead>
                            <tr>
                                <th>Test</th>
                                <th>Expected</th>
                                <th>Calculated</th>
                                <th>Status</th>
                            </tr>
                        </thead>

                        <tbody id="p3-tests"></tbody>

                    </table>

                </div>

            </section>
        `;

// Improve P3 page layout
const workspace = document.createElement('div');
workspace.className = 'workspace-grid';

const leftColumn = document.createElement('aside');
leftColumn.className = 'controls-stack';

const rightColumn = document.createElement('div');
rightColumn.className = 'results-stack';

const controls = root.querySelector('.controls-panel');
const prediction = root.querySelector('.prediction-panel');
const charts = root.querySelectorAll('.chart-panel');
const metrics = root.querySelector('.metrics-grid');

root.insertBefore(workspace, controls);

// Left column
leftColumn.appendChild(controls);
leftColumn.appendChild(prediction);

// Right column
charts.forEach(chart => {
    rightColumn.appendChild(chart);
});

rightColumn.appendChild(metrics);

workspace.appendChild(leftColumn);
workspace.appendChild(rightColumn);

// Make Arduino code scrollable
const codeBox = root.querySelector('pre');

codeBox.style.maxHeight = '400px';
codeBox.style.overflowY = 'auto';
codeBox.style.overflowX = 'auto';

// Scoped responsive styles for P3
const p3Style = document.createElement('style');

p3Style.textContent = `
    #page-p3 .workspace-grid {
        display: grid;
        grid-template-columns: minmax(0, 1fr) minmax(0, 2fr);
        gap: 24px;
        align-items: start;
        margin-bottom: 24px;
    }

    #page-p3 .controls-stack,
    #page-p3 .results-stack {
        display: flex;
        flex-direction: column;
        gap: 20px;
        min-width: 0;
    }

    #page-p3 .metrics-grid {
        grid-template-columns: repeat(2, minmax(0, 1fr));
    }

    #page-p3 > .panel {
        margin-bottom: 24px;
        padding: 24px;
        min-width: 0;
    }

    #page-p3 .chart-box svg {
        width: 100%;
        height: auto;
    }

    @media (max-width: 800px) {
        #page-p3 .workspace-grid {
            grid-template-columns: minmax(0, 1fr);
        }
    }

    @media (max-width: 480px) {
        #page-p3 .metrics-grid {
            grid-template-columns: minmax(0, 1fr);
        }
    }
`;

document.head.appendChild(p3Style);

        // SVG chart drawing
        function drawChart(id, series, refs, yLabel) {

            const W = 900;
            const H = 350;
            const L = 76;
            const R = 28;
            const top = 28;
            const B = 50;

            const end =
                series[0].data.at(-1).t;

            let lo = 0;
            let hi = 0;

            for (const s of series) {
                for (const q of s.data) {
                    lo = Math.min(lo, q[s.key]);
                    hi = Math.max(hi, q[s.key]);
                }
            }

            for (const ref of refs) {
                lo = Math.min(lo, ref.value);
                hi = Math.max(hi, ref.value);
            }

            const pad =
                Math.max((hi - lo) * 0.1, 0.05);

            lo -= pad;
            hi += pad;

            const x = t =>
                L + t / end * (W - L - R);

            const y = v =>
                H - B -
                (v - lo) / (hi - lo) *
                (H - top - B);

            let html = '';

            // Draw grid and labels
            for (let i = 0; i <= 5; i++) {

                const v = lo + (hi - lo) * i / 5;
                const t = end * i / 5;

                html += `

                    <path d="M${L} ${y(v)}H${W - R}"
                          stroke="#e5e8f2"
                          stroke-dasharray="4 4"/>

                    <text x="${L - 10}"
                          y="${y(v) + 4}"
                          text-anchor="end"
                          font-size="14"
                          fill="#71788e">
                        ${v.toFixed(2)}
                    </text>

                    <text x="${x(t)}"
                          y="${H - B + 24}"
                          text-anchor="middle"
                          font-size="14"
                          fill="#71788e">
                        ${t.toFixed(1)}
                    </text>

                `;
            }

            // Draw reference lines
            for (const ref of refs) {

                html += `

                    <path
                        d="M${L} ${y(ref.value)}H${W - R}"
                        stroke="${ref.color}"
                        stroke-width="2"
                        stroke-dasharray="7 5"/>

                    <text x="${W - R - 5}"
                          y="${y(ref.value) - 7}"
                          text-anchor="end"
                          font-size="14"
                          fill="${ref.color}">

                        ${ref.label}

                    </text>

                `;
            }

            // Draw data series
            for (const s of series) {

                const stride = s.stair
                    ? 1
                    : Math.max(
                        1,
                        Math.ceil(s.data.length / 1800)
                    );

                const pts = s.data.filter(
                    (q, i) =>
                        i % stride === 0 ||
                        i === s.data.length - 1
                );

                let path = '';

                pts.forEach((q, i) => {

                    const px = x(q.t).toFixed(2);
                    const py = y(q[s.key]).toFixed(2);

                    if (i === 0) {
                        path += `M${px} ${py} `;
                    } else if (s.stair) {
                        path += `H${px} V${py} `;
                    } else {
                        path += `L${px} ${py} `;
                    }
                });

                html += `

                    <path d="${path}"
                          fill="none"
                          stroke="${s.color}"
                          stroke-width="2.8"
                          ${s.dash
                              ? 'stroke-dasharray="7 5"'
                              : ''}/>

                `;
            }

            html += `

                <text x="${W / 2}"
                      y="${H - 8}"
                      text-anchor="middle"
                      fill="#71788e"
                      font-size="14">

                    Time (s)

                </text>

                <text x="${L}"
                      y="17"
                      fill="#71788e"
                      font-size="14">

                    ${yLabel}

                </text>

            `;

            $(id).innerHTML = html;
        }

        // Read current slider values
        function readParams() {

            const p = { ...defaults };

            for (const [id] of fields) {
                p[id] = Number($(id).value);
            }

            p.antiWindup = $('anti-windup').checked;

            return p;
        }

        // Update all results
        function renderP3() {

            const p = readParams();

            const a = simulatePID(p);

            const fast = simulatePID({
                ...p,
                ts: 0.01
            });

            const opposite = simulatePID({
                ...p,
                antiWindup: !p.antiWindup
            });

            const purple = '#7563e8';
            const teal = '#16a6a0';

            // System response
            drawChart(
                'response-chart',
                [
                    {
                        data: fast.output,
                        key: 'y',
                        color: teal,
                        dash: true
                    },
                    {
                        data: a.output,
                        key: 'y',
                        color: purple
                    }
                ],
                [
                    {
                        value: 1,
                        color: '#8791a8',
                        label: 'Setpoint = 1'
                    }
                ],
                'Output y(t)'
            );

            // Digital control signal
            drawChart(
                'control-chart',
                [
                    {
                        data: a.control,
                        key: 'u',
                        color: purple,
                        stair: true
                    }
                ],
                [
                    {
                        value: 0,
                        color: '#ce8155',
                        label: 'Lower limit = 0'
                    },
                    {
                        value: 2,
                        color: '#ce8155',
                        label: 'Upper limit = 2'
                    }
                ],
                'Control signal u[k]'
            );

            // Integral windup comparison
            drawChart(
                'integral-chart',
                [
                    {
                        data: opposite.control,
                        key: 'I',
                        color: teal,
                        stair: true,
                        dash: true
                    },
                    {
                        data: a.control,
                        key: 'I',
                        color: purple,
                        stair: true
                    }
                ],
                [],
                'Integral contribution Ki × integral'
            );

            // Performance metrics
            const m = a.metrics;

            $('rise').textContent =
                m.rise === null
                    ? 'Not reached'
                    : m.rise.toFixed(2) + ' s';

            $('overshoot').textContent =
                m.overshoot.toFixed(1) + '%';

            $('settling').textContent =
                m.settling === null
                    ? 'Not settled'
                    : m.settling.toFixed(2) + ' s';

            $('error').textContent =
                m.error === null
                    ? 'Not steady'
                    : m.error.toFixed(4);

            // Sampling time comparison
            $('comparison').textContent =
                `Overshoot: ${m.overshoot.toFixed(1)}% ` +
                `at Ts = ${p.ts.toFixed(2)} s vs ` +
                `${fast.metrics.overshoot.toFixed(1)}% ` +
                `at Ts = 0.01 s. ` +
                `Larger Ts can degrade performance; ` +
                `the effect depends on gains.`;

            // Plant model
            $('model').textContent =
                `G(s) = ${p.K} / (${p.tau}s + 1). ` +
                `Reference plant values are illustrative ` +
                `until measured project values are entered.`;

            // Generate Arduino code
            $('arduino-code').textContent =
                generateArduinoCode(p);

            $('copy-status').textContent = '';
        }

        // Synchronize controls with parameters
        function sync(p) {

            for (const [id, , , , , unit] of fields) {

                $(id).value = p[id];

                $(id + '-value').textContent =
                    p[id].toFixed(2) + unit;
            }

            $('anti-windup').checked = p.antiWindup;

            renderP3();
        }

        // Schedule rendering
        let frame = 0;

        function schedule() {

            cancelAnimationFrame(frame);

            frame = requestAnimationFrame(renderP3);
        }

        // Slider events
        for (const [id, , , , , unit] of fields) {

            $(id).addEventListener('input', () => {

                $(id + '-value').textContent =
                    Number($(id).value).toFixed(2) + unit;

                schedule();
            });
        }

        $('anti-windup')
            .addEventListener('change', schedule);

        // Preset buttons
        root.querySelectorAll('[data-p3-preset]')
            .forEach(btn => {

                btn.addEventListener('click', () => {

                    const presets = {
                        fast: {
                            ts: 0.01
                        },

                        slow: {
                            ts: 2
                        },

                        windup: {
                            ts: 0.1,
                            kp: 4,
                            ki: 10,
                            antiWindup: false
                        },

                        reset: {}
                    };

                    $('feedback').textContent = '';

                    sync({
                        ...defaults,
                        ...presets[btn.dataset.p3Preset]
                    });
                });
            });

        // Predict and Check
        $('check').addEventListener('click', () => {

            const answer = root.querySelector(
                '[name="p3-prediction"]:checked'
            );

            if (!answer) {

                $('feedback').textContent =
                    'Choose a prediction first.';

                return;
            }

            const before = simulatePID({
                ...defaults,
                ts: 0.01
            });

            const after = simulatePID({
                ...defaults,
                ts: 2
            });

            sync({
                ...defaults,
                ts: 2
            });

            $('feedback').textContent =
                `${answer.value === 'worse'
                    ? 'Correct!'
                    : 'Not quite.'} ` +

                `Overshoot changes from ` +
                `${before.metrics.overshoot.toFixed(1)}% ` +
                `to ${after.metrics.overshoot.toFixed(1)}%. ` +

                `The held control signal reacts less frequently.`;
                $('feedback').style.color =
    answer.value === 'worse' ? '#15803d' : '#dc2626';

$('feedback').style.fontWeight = '600';
        });

        // Copy Arduino code
        $('copy-code').addEventListener('click', async () => {

            try {

                await navigator.clipboard.writeText(
                    $('arduino-code').textContent
                );

                $('copy-status').textContent =
                    'Arduino code copied successfully!';

            } catch (e) {

                $('copy-status').textContent =
                    'Copy unavailable. Select and copy manually.';
            }
        });

        // Display validation results
        $('tests').innerHTML = validations()
            .map(c => `

                <tr>
                    <td>${c.name}</td>
                    <td>${c.expected}</td>
                    <td>${c.actual}</td>

                    <td>
                        <span class="${c.pass
                            ? 'pass-chip'
                            : 'p1-fail'}">

                            ${c.pass ? 'PASS' : 'FAIL'}

                        </span>
                    </td>
                </tr>

            `).join('');

        // Page navigation
        function activateP3() {

            document.querySelectorAll('.page-view')
                .forEach(page => {

                    page.hidden =
                        page.id !== 'page-p3';
                });

            document.querySelectorAll('.nav-tab')
                .forEach(button => {

                    const active =
                        button.id === 'tab-p3';

                    button.classList.toggle(
                        'active',
                        active
                    );

                    button.setAttribute(
                        'aria-selected',
                        String(active)
                    );

                    button.tabIndex = active ? 0 : -1;
                });

            window.scrollTo(0, 0);
        }

        tab.addEventListener('click', () => {

            window.location.hash = 'p3';

            activateP3();
        });

        window.addEventListener('hashchange', () => {

            if (window.location.hash === '#p3') {

                activateP3();

            } else {

                root.hidden = true;

                tab.classList.remove('active');

                tab.setAttribute(
                    'aria-selected',
                    'false'
                );

                tab.tabIndex = -1;
            }
        });

        // Keyboard navigation
        tab.addEventListener('keydown', event => {

            if (![
                'ArrowLeft',
                'ArrowRight',
                'Home',
                'End'
            ].includes(event.key)) {
                return;
            }

            event.preventDefault();

            const tabs = [
                ...nav.querySelectorAll('[data-page]')
            ];

            const i = tabs.indexOf(tab);

            const next =
                event.key === 'Home'
                    ? 0
                    : event.key === 'End'
                        ? tabs.length - 1
                        : (
                            i +
                            (event.key === 'ArrowRight' ? 1 : -1) +
                            tabs.length
                          ) % tabs.length;

            tabs[next].focus();
            tabs[next].click();
        });

        // Initial render
        sync(defaults);

        if (window.location.hash === '#p3') {
            activateP3();
        }
    }

    if (document.readyState === 'loading') {

        document.addEventListener(
            'DOMContentLoaded',
            initP3
        );

    } else {

        initP3();
    }

})(typeof window !== 'undefined' ? window : globalThis);
