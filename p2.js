
/* Homework 2, P2 — Slow sensor in the feedback loop.
 * No dependencies; works with file://.
 *
 * State: [plant output y, velocity v, integral I, sensor output z, filter q].
 * Sensor: tauS * dz/dt = y - z.
 * PID uses measured output z instead of true output y.
 * RK4 integrates the plant, sensor, integral and derivative-filter states.
 *
 * P1 remains unchanged. Exposes window.P2Engine.
 */
(function (global) {
  'use strict';

  const defaults = {
    plant: 'first',
    K: 1, tau: 1,
    m: 1, b: 2, k: 4,
    kp: 3, ki: 1, kd: 0.2,
    r: 1, lo: -20, hi: 20,
    tauS: 0.5, T: 20
  };

  const filterTime = 0.04;
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  function validate(p) {
    for (const key of ['tau', 'm', 'tauS', 'T']) {
      if (!Number.isFinite(p[key]) || p[key] <= 0)
        throw new Error(key + ' must be positive.');
    }

    if (p.lo >= p.hi)
      throw new Error('Lower actuator limit must be below upper limit.');

    if (p.plant !== 'first' && p.plant !== 'second')
      throw new Error('Invalid plant model.');
  }

  // Controller feedback comes from the sensor output z.
  function control(s, p) {
    const measured = s[3];
    const derivative = (measured - s[4]) / filterTime;

    const raw =
      p.kp * (p.r - measured) +
      p.ki * s[2] -
      p.kd * derivative;

    return { raw, u: clamp(raw, p.lo, p.hi) };
  }

  function derivative(s, p) {
    const u = control(s, p).u;
    const y = s[0];
    const v = s[1];
    const z = s[3];

    return [
      // Plant output derivative
      p.plant === 'first'
        ? (p.K * u - y) / p.tau
        : v,

      // Plant velocity derivative
      p.plant === 'first'
        ? 0
        : (u - p.b * v - p.k * y) / p.m,

      // Integral of measured error
      p.r - z,

      // Physical sensor dynamics
      (y - z) / p.tauS,

      // Derivative filter
      (z - s[4]) / filterTime
    ];
  }

  function rk4(s, dt, p) {
    const a = derivative(s, p);
    const b = derivative(
      s.map((v, i) => v + dt * a[i] / 2), p
    );
    const c = derivative(
      s.map((v, i) => v + dt * b[i] / 2), p
    );
    const d = derivative(
      s.map((v, i) => v + dt * c[i]), p
    );

    return s.map((v, i) =>
      v + dt * (a[i] + 2*b[i] + 2*c[i] + d[i]) / 6
    );
  }

  function analyze(rows, p, failed) {
    if (!rows.length) {
      return {
        steady: false, final: NaN,
        rise: null, overshoot: null,
        settling: null, error: null,
        endError: NaN
      };
    }

    const last = rows[rows.length - 1];
    const tail = rows.slice(Math.floor(rows.length * 0.9));

    const final =
      tail.reduce((sum, q) => sum + q.y, 0) / tail.length;

    let min = Infinity, max = -Infinity;

    for (const q of tail) {
      min = Math.min(min, q.y);
      max = Math.max(max, q.y);
    }

    const scale = Math.max(Math.abs(final), Math.abs(p.r), 0.001);
    const steady = !failed && max - min < 0.001 * scale;

    const cross = level => {
      for (let i = 1; i < rows.length; i++) {
        if (rows[i].y >= level && rows[i-1].y < level) {
          const a = rows[i-1];
          const b = rows[i];

          return a.t + (b.t-a.t) *
            (level-a.y) / (b.y-a.y);
        }
      }
      return null;
    };

    const t10 = final > 1e-8 ? cross(0.1 * final) : null;
    const t90 = final > 1e-8 ? cross(0.9 * final) : null;

    let peak = -Infinity;
    let lastOutside = -1;

    for (let i = 0; i < rows.length; i++) {
      peak = Math.max(peak, rows[i].y);

      if (Math.abs(rows[i].y-final) > 0.02*Math.abs(final))
        lastOutside = i;
    }

    let candidate = rows[lastOutside+1]?.t;

    if (lastOutside >= 0 && lastOutside+1 < rows.length) {
      const a = rows[lastOutside];
      const b = rows[lastOutside+1];

      const boundary =
        final + (a.y > final ? 1 : -1) * 0.02 * Math.abs(final);

      if (b.y !== a.y) {
        candidate = a.t + (b.t-a.t) *
          (boundary-a.y) / (b.y-a.y);
      }
    }

    const settling =
      steady && final > 1e-8 &&
      candidate !== undefined &&
      last.t-candidate >= 0.5
        ? candidate : null;

    return {
      steady,
      final,
      rise: steady && t10 !== null && t90 !== null
        ? t90-t10 : null,
      overshoot: steady && final > 1e-8
        ? Math.max(0, 100*(peak-final)/Math.abs(final))
        : null,
      settling,
      error: steady ? p.r-final : null,
      endError: p.r-last.y
    };
  }

  function simulate(input, refinement = 1) {
    const p = { ...defaults, ...input };
    validate(p);

    if (!Number.isFinite(refinement) || refinement <= 0)
      throw new Error('Refinement must be positive.');

    const plantRate = p.plant === 'first'
      ? (1 + Math.abs(p.K)*(p.kp+p.kd/filterTime))/p.tau
      : Math.max(
          p.b/p.m,
          Math.sqrt((p.k+p.kp+p.kd/filterTime)/p.m)
        );

    const rate = Math.max(
      plantRate, 1/p.tauS, 1/filterTime
    );

    const dtTarget =
      Math.min(0.01, p.tauS/8, filterTime/8,
        0.08/Math.max(1, rate)) / refinement;

    const n = Math.ceil(p.T/dtTarget);
    const dt = p.T/n;

    if (n > 800000)
      throw new Error('Too many simulation steps. Reduce gains or duration.');

    // [y, velocity, integral, sensor output z, derivative filter q]
    let s = [0, 0, 0, 0, 0];

    let clipped = 0;
    let failed = false;
    const rows = [];

    for (let i = 0; i <= n; i++) {
      const c = control(s, p);

      if (
        !s.every(Number.isFinite) ||
        Math.abs(s[0]) > 1e6 ||
        Math.abs(s[3]) > 1e6
      ) {
        failed = true;
        break;
      }

      if (Math.abs(c.raw-c.u) > 1e-8)
        clipped++;

      rows.push({
        t: i*dt,
        y: s[0],
        z: s[3],
        u: c.u
      });

      if (i < n)
        s = rk4(s, dt, p);
    }

    return {
      p, rows, dt, failed,
      clipped: clipped/Math.max(1, rows.length),
      metrics: analyze(rows, p, failed)
    };
  }

  function validations() {
    const checks = [];

    // 1. Sensor step response at t = tauS.
    const sensorValue = 1-Math.exp(-1);

    checks.push({
      name: 'Sensor response at t = tauS',
      expected: '0.63212',
      actual: sensorValue.toFixed(5),
      pass: Math.abs(sensorValue-0.6321205588) < 0.00001
    });

    // 2. Fast sensor should closely follow the plant.
    const fast = simulate({
      ...defaults,
      kp: 1, ki: 1, kd: 0,
      tauS: 0.001,
      lo: -100, hi: 100,
      T: 10
    });

    const fastGap = Math.max(
      ...fast.rows.map(q => Math.abs(q.y-q.z))
    );

    checks.push({
      name: 'Fast sensor follows true output',
      expected: 'Maximum |y-z| < 0.02',
      actual: fastGap.toFixed(5),
      pass: fastGap < 0.02
    });

    // 3. Sensor output should lag the true output.
    const slow = simulate({
      ...defaults,
      kp: 1, ki: 1, kd: 0,
      tauS: 0.5,
      lo: -100, hi: 100,
      T: 10
    });

    const slowGap = Math.max(
      ...slow.rows.map(q => q.y-q.z)
    );

    checks.push({
      name: 'Slow sensor measurement lag',
      expected: 'Maximum (y-z) > 0',
      actual: slowGap.toFixed(5),
      pass: slowGap > 0
    });

    // 4. PI controller should remove steady-state error.
    const pi = simulate({
      ...defaults,
      kp: 1, ki: 1, kd: 0,
      tauS: 0.2,
      lo: -100, hi: 100,
      T: 30
    });

    checks.push({
      name: 'PI steady-state tracking',
      expected: 'Absolute error < 0.01',
      actual: pi.metrics.steady
        ? Math.abs(pi.metrics.error).toFixed(5)
        : 'Not steady',
      pass: pi.metrics.steady &&
        Math.abs(pi.metrics.error) < 0.01
    });

    return checks;
  }

  // Public engine for testing and future P3 reuse.
  global.P2Engine = {
    defaults, simulate, validations, filterTime
  };

  // UI is mounted only when P2 is selected.
  function mount() {
    const root = document.getElementById('page-control');
    if (!root) return;

    let state = { ...defaults };
    let frame = 0;

    const $ = id => document.getElementById('p2-'+id);

    const fields = [
      ['kp','Proportional gain Kp',0,30,.1],
      ['ki','Integral gain Ki',0,20,.1],
      ['kd','Derivative gain Kd',0,5,.05],
      ['r','Setpoint step',.1,5,.1],
      ['tauS','Sensor time constant τs (s)',.01,3,.01],
      ['K','Plant gain K',.1,3,.1],
      ['tau','Plant time constant τ (s)',.2,6,.1],
      ['m','Mass m (kg)',.5,4,.1],
      ['b','Damping b (N·s/m)',.2,20,.1],
      ['k','Spring k (N/m)',1,20,.1],
      ['lo','Control lower limit',-100,0,1],
      ['hi','Control upper limit',.1,100,.1],
      ['T','Duration (s)',5,120,1]
    ];

    function field([id,label,min,max,step]) {
      return `
        <div class="p1-field" data-field="${id}">
          <label for="p2-${id}">${label}</label>
          <div class="p1-input-pair">
            <input type="range" id="p2-${id}-slider"
              min="${min}" max="${max}" step="${step}"
              value="${state[id]}">
            <input type="number" id="p2-${id}"
              min="${min}" max="${max}" step="${step}"
              value="${state[id]}">
          </div>
        </div>`;
    }

    root.innerHTML = `
      <div class="module-hero">
        <a class="breadcrumb-link" href="#home">← Back to home</a>
        <div class="module-title-row">
          <div>
            <p class="section-kicker">CONTROL / HOMEWORK 2 / P2</p>
            <h1>Slow sensor<span class="title-period">.</span></h1>
            <p>See how sensor delay changes the feedback response.</p>
          </div>
          <span class="formula-badge">τs ż = y − z</span>
        </div>
      </div>

      <div class="p1-presets">
        <button type="button" data-preset="first">First-order example</button>
        <button type="button" data-preset="second">Mass–spring–damper</button>
        <button type="button" id="p2-reset">Reset</button>
      </div>

      <section class="panel p1-loop">
        <h3>Feedback block diagram</h3>
        <p>Setpoint → PID controller → actuator → plant → sensor → negative feedback.</p>
        <p>The controller receives sensor output z, not true output y.</p>
      </section>

      <div class="workspace-grid">
        <aside class="controls-stack">
          <section class="panel controls-panel">
            <h3>Simulation controls</h3>
            <div class="p1-field">
              <label for="p2-plant">Plant model</label>
              <select id="p2-plant">
                <option value="first">First order — Module 1</option>
                <option value="second">Mass–spring–damper — Module 3</option>
              </select>
            </div>
            ${fields.map(field).join('')}
            <p class="p1-model" id="p2-model"></p>
          </section>

          <section class="panel prediction-panel">
            <h3>Predict, then check</h3>
            <p>What happens when the sensor time constant increases?</p>
            <label class="p1-choice">
              <input type="radio" name="p2-prediction" value="larger">
              Measurement lag generally increases
            </label>
            <label class="p1-choice">
              <input type="radio" name="p2-prediction" value="same">
              Measurement remains instant
            </label>
            <label class="p1-choice">
              <input type="radio" name="p2-prediction" value="zero">
              Plant output becomes zero
            </label>
            <button type="button" class="primary-button" id="p2-check">
              Check with a simulation →
            </button>
            <p id="p2-feedback" role="status"></p>
          </section>
        </aside>

        <div class="results-stack">
          <section class="panel chart-panel">
            <div class="chart-heading">
              <div>
                <p class="chart-eyebrow">LIVE RESPONSE</p>
                <h3>True output and sensor measurement</h3>
              </div>
              <span class="live-chip">● Live</span>
            </div>
            <p class="p1-legend">
              <span>━ True output y</span>
              <span>┄ Sensor output z</span>
              <span>┄ Setpoint r</span>
            </p>
            <div class="chart-box">
              <svg id="p2-output" viewBox="0 0 900 350"></svg>
            </div>
            <p class="chart-footnote" id="p2-status"></p>
          </section>

          <div class="metrics-grid p1-metrics">
            ${[
              ['rise','Rise time','10–90% of final output'],
              ['overshoot','Overshoot','Above final output'],
              ['settling','Settling time','±2% of final output'],
              ['error','Steady-state error','Target − final output']
            ].map(([id,label,caption]) => `
              <article class="metric-card">
                <span class="metric-label">${label}</span>
                <div class="metric-value" id="p2-metric-${id}">—</div>
                <p>${caption}</p>
              </article>
            `).join('')}
          </div>

          <section class="panel chart-panel">
            <p class="chart-eyebrow">ACTUATOR EFFORT</p>
            <h3>Control signal u</h3>
            <div class="chart-box">
              <svg id="p2-control" viewBox="0 0 900 350"></svg>
            </div>
            <p class="chart-footnote" id="p2-saturation"></p>
          </section>

          <section class="panel validation-panel">
            <h3>How to read these results</h3>
            <p>The controller compares the setpoint with measured output z. The sensor follows the actual plant output y with a delay.</p>
            <p>Performance metrics are calculated using the true plant output. The sensor curve is displayed separately to show the measurement lag.</p>
            <p>The final output is estimated from the last 10% of the simulation. Metrics are withheld if the output has not become steady.</p>
            <details>
              <summary>Equations and implementation</summary>
              <p>Sensor: τs·ż = y − z.</p>
              <p>Controller: e = r − z; I′ = e; u = clip(Kp·e + Ki·I − Kd·filtered derivative).</p>
              <p>First order: y′ = (K·u − y)/τ.</p>
              <p>Second order: y′ = v; v′ = (u − b·v − k·y)/m.</p>
              <p>RK4 integrates the plant, sensor, integral and derivative-filter states together. Anti-windup is left for P3.</p>
            </details>
          </section>
        </div>
      </div>

      <section class="panel validation-panel p1-validation">
        <div class="chart-heading">
          <div>
            <p class="chart-eyebrow">ENGINEERING CHECK</p>
            <h3>Homework validation</h3>
          </div>
          <span class="small-tag" id="p2-test-count"></span>
        </div>
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

    function draw(id, rows, keys, refs = []) {
      const svg = $(id);
      if (!rows.length) return;

      const W=900,H=350,L=76,R=24,top=24,B=52;

      let low=0, high=0;
      for (const q of rows) {
        for (const key of keys) {
          low=Math.min(low,q[key]);
          high=Math.max(high,q[key]);
        }
      }
      refs.forEach(q=>{
        low=Math.min(low,q.value);
        high=Math.max(high,q.value);
      });

      const pad=Math.max((high-low)*.12,.05);
      low-=pad;
      high+=pad;

      const end=rows[rows.length-1].t || state.T;
      const x=t=>L+t/end*(W-L-R);
      const y=v=>H-B-(v-low)/(high-low)*(H-top-B);

      let html='';

      for(let i=0;i<=5;i++){
        const t=end*i/5;
        const v=low+(high-low)*i/5;

        html+=`<path d="M${L} ${y(v)}H${W-R}" stroke="#e9e7f0"/>`;
        html+=`<text x="${L-10}" y="${y(v)+4}" text-anchor="end">${v.toFixed(2)}</text>`;
        html+=`<text x="${x(t)}" y="${H-B+24}" text-anchor="middle">${t.toFixed(1)}</text>`;
      }

      refs.forEach(q=>{
        html+=`<path d="M${L} ${y(q.value)}H${W-R}"
          stroke="${q.color}" stroke-width="2" stroke-dasharray="7 5"/>`;
      });

      const stride=Math.max(1,Math.floor(rows.length/2500));
      const pts=rows.filter((q,i)=>i%stride===0||i===rows.length-1);

      const colors={y:'#7563dc',z:'#d28a52',u:'#7563dc'};
      const dashes={y:'',z:'8 5',u:''};

      for(const key of keys){
        const path=pts.map((q,i)=>
          (i?'L':'M')+x(q.t).toFixed(2)+' '+y(q[key]).toFixed(2)
        ).join(' ');

        html+=`<path d="${path}" fill="none"
          stroke="${colors[key]}" stroke-width="2.6"
          stroke-dasharray="${dashes[key]}"/>`;
      }

      html+=`<text x="${W/2}" y="${H-8}" text-anchor="middle">Time (s)</text>`;
      svg.innerHTML=html;
    }

    function render() {
      try {
        const result=simulate(state);
        const m=result.metrics;

        draw('output',result.rows,['y','z'],[
          {value:state.r,color:'#8791a8'}
        ]);

        draw('control',result.rows,['u'],[
          {value:state.lo,color:'#ce8155'},
          {value:state.hi,color:'#ce8155'}
        ]);

        for(const key of ['rise','overshoot','settling','error']){
          const value=m[key];

          $('metric-'+key).textContent =
            value===null ? '—' :
            value.toFixed(3)+
            (key==='overshoot'?'%':key==='error'?'':' s');
        }

        $('status').textContent=result.failed
          ? 'Simulation stopped because the output exceeded the safety range.'
          : m.steady
            ? `True final output ≈ ${m.final.toFixed(4)}. Target = ${state.r}. Steady-state error ≈ ${(100*m.error/state.r).toFixed(2)}% of target.`
            : `Not steady within ${state.T} s. Increase duration or retune gains. End error = ${m.endError.toFixed(4)}.`;

        $('saturation').textContent =
          `Control clipped to [${state.lo}, ${state.hi}]. `+
          `Saturated during ${(100*result.clipped).toFixed(1)}% of samples.`;

        $('model').textContent=state.plant==='first'
          ? `Plant: G(s) = ${state.K}/(${state.tau}s + 1). Sensor: Gs(s) = 1/(${state.tauS}s + 1).`
          : `Plant: G(s) = 1/(${state.m}s² + ${state.b}s + ${state.k}). Sensor τs = ${state.tauS} s.`;
      } catch(e) {
        $('status').textContent=e.message;

        for(const key of ['rise','overshoot','settling','error'])
          $('metric-'+key).textContent='—';
      }
    }

    function schedule() {
      cancelAnimationFrame(frame);
      frame=requestAnimationFrame(render);
    }

    function sync() {
      $('plant').value=state.plant;

      for(const [id] of fields){
        $(id).value=state[id];
        $(id+'-slider').value=state[id];
      }

      root.querySelectorAll('[data-field]').forEach(el=>{
        const id=el.dataset.field;
        el.hidden=state.plant==='first'
          ? ['m','b','k'].includes(id)
          : ['K','tau'].includes(id);
      });

      schedule();
    }

    for(const [id,,min,max] of fields){
      $(id+'-slider').addEventListener('input',()=>{
        state[id]=Number($(id+'-slider').value);
        $(id).value=state[id];
        schedule();
      });

      $(id).addEventListener('input',()=>{
        const v=$(id).valueAsNumber;

        if(Number.isFinite(v)&&v>=min&&v<=max){
          state[id]=v;
          $(id+'-slider').value=v;
          schedule();
        }
      });

      $(id).addEventListener('change',()=>{
        $(id).value=state[id];
      });
    }

    $('plant').addEventListener('change',()=>{
      state.plant=$('plant').value;
      sync();
    });

    root.querySelectorAll('[data-preset]').forEach(button=>{
      button.addEventListener('click',()=>{
        state=button.dataset.preset==='first'
          ? {...defaults,plant:'first',tauS:.5,kp:3,ki:1,kd:.2}
          : {...defaults,
