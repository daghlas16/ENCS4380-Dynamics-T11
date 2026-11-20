/* P4 uses the Module 4 step-plant models and P1's filtered, saturated RK4 loop. */
(() => {
  'use strict';
  const standalone = document.documentElement.hasAttribute('data-p4-standalone');
  const initial = {kp: 1, ki: 0, kd: 0};
  const maxima = {kp: 6, ki: 4, kd: 1};
  const unitStep = {r: 1, lo: -3, hi: 3, T: 36};
  const tolerance = 0.001;

  // Routh-Hurwitz for the local closed loop at its unsaturated equilibrium.
  // With Ki=0 the disconnected integral state cancels from the polynomial.
  function stable(p) {
    const f = 0.04;
    const [m,b,k,g] = p.plant === 'first' ? [0,p.tau,1,p.K] : [p.m,p.b,p.k,1];
    const a4 = m*f, a3 = m+b*f, a2 = b+k*f+g*(p.kp*f+p.kd);
    const a1 = k+g*(p.kp+p.ki*f), a0 = g*p.ki;
    if (!a4) return !a0 || a2*a1 > a3*a0;
    const minor = a3*a2-a4*a1;
    return minor > 0 && (!a0 || minor*a1 > a3*a3*a0);
  }

  function measure(result) {
    const {rows, p} = result;
    if (result.failed || !rows.length || Math.abs(rows.at(-1).t-p.T)>1e-6) {
      throw new Error('Simulation incomplete. Adjust the gains and try again.');
    }
    let peak = 0, lastOutside = -1, t10 = null, t90 = null;
    for (let i=0; i<rows.length; i++) {
      const {t,y,u} = rows[i];
      if (![t,y,u].every(Number.isFinite)) throw new Error('Non-finite simulation output.');
      peak = Math.max(peak,y);
      if (Math.abs(y-p.r) > 0.02*p.r) lastOutside = i;
      if (i === 0) continue;
      const prev = rows[i-1];
      const cross = level => prev.t+(t-prev.t)*(level-prev.y)/(y-prev.y);
      if (t10 === null && y >= 0.1*p.r) t10 = cross(0.1*p.r);
      if (t90 === null && y >= 0.9*p.r) t90 = cross(0.9*p.r);
    }
    const locallyStable = stable(p);
    const steady = result.metrics.steady && locallyStable;
    const entry = rows[lastOutside+1];
    return {
      overshoot: Math.max(0,100*(peak-p.r)/p.r),
      settling: steady && entry && p.T-entry.t >= 1 ? entry.t : null,
      error: steady ? Math.abs(result.metrics.final-p.r)/p.r : null,
      rise: t90 === null ? null : t90-t10,
      locallyStable, steady
    };
  }

  function evaluate(m, limit) {
    const passes = [m.locallyStable && m.overshoot <= 10,
      m.settling !== null && m.settling <= limit,
      m.error !== null && m.error <= tolerance];
    return {passes, score: passes.reduce((n,pass,i)=>n+(pass ? [30,40,30][i] : 0),0)};
  }

  function simulate(plant, gains) {
    const result = window.P1Engine.simulate({...unitStep,...plant,...gains});
    return {result, metrics: measure(result)};
  }

  function drawPlant() {
    const uniform = (lo,hi) => lo+Math.random()*(hi-lo);
    if (Math.random() < 0.5) {
      return {plant:'first', K:uniform(0.5,2.5), tau:uniform(0.4,4)};
    }
    const wn = uniform(1.5,8), zeta = uniform(0.15,1.5);
    // Normalize Module 4's numerator wn² to the unit numerator used by P1.
    return {plant:'second', m:1/(wn*wn), b:2*zeta/wn, k:1};
  }

  function challenge(previous) {
    const witness = {kp:1,ki:1,kd:0.2};
    for (let attempt=0; ; attempt++) {
      // Bound retries with two distinct, reachable members of the first-order family.
      const plant = attempt < 16 ? drawPlant() :
        {plant:'first',K:previous?.K === 1 ? 1.2 : 1,tau:1};
      if (previous && Object.keys(plant).every(key=>plant[key] === previous[key])) continue;
      const {metrics} = simulate(plant,witness);
      if (metrics.settling === null) continue;
      const limit = Math.ceil(metrics.settling*1.3);
      if (evaluate(metrics,limit).score === 100) return {plant,limit};
    }
  }

  function plot(svg,rows,key,refs) {
    const W=900,H=350,L=76,R=24,top=24,B=52;
    let low=0,high=0;
    for(const q of rows){low=Math.min(low,q[key]);high=Math.max(high,q[key]);}
    refs.forEach(q=>{low=Math.min(low,q.value);high=Math.max(high,q.value);});
    const pad=Math.max((high-low)*.12,.05);low-=pad;high+=pad;
    const end=rows[rows.length-1].t||unitStep.T;
    const x=t=>L+t/end*(W-L-R), y=v=>H-B-(v-low)/(high-low)*(H-top-B);
    let html='';
    for(let i=0;i<=5;i++){
      const t=end*i/5, v=low+(high-low)*i/5;
      html+=`<path d="M${L} ${y(v)}H${W-R}" stroke="#e9e7f0"/><text x="${L-10}" y="${y(v)+4}" text-anchor="end">${v.toFixed(Math.abs(v)>100?0:2)}</text><text x="${x(t)}" y="${H-B+24}" text-anchor="middle">${t.toFixed(1)}</text>`;
    }
    refs.forEach(q=>{html+=`<path d="M${L} ${y(q.value)}H${W-R}" stroke="${q.color}" stroke-width="2" stroke-dasharray="7 5"/><text x="${W-R-5}" y="${y(q.value)-6}" text-anchor="end" fill="${q.color}">${q.label}</text>`;});
    const stride=Math.max(1,Math.floor(rows.length/2500));
    const pts=rows.filter((q,i)=>i%stride===0||i===rows.length-1);
    html+=`<path d="${pts.map((q,i)=>(i?'L':'M')+x(q.t).toFixed(2)+' '+y(q[key]).toFixed(2)).join(' ')}" fill="none" stroke="#7563dc" stroke-width="2.6"/><text x="${W/2}" y="${H-8}" text-anchor="middle">Time (s)</text><text x="${L}" y="16">${key==='y'?'Output':'Control effort'}</text>`;
    svg.innerHTML=html;
  }
  function mount() {
    const nav = document.querySelector('.main-nav');
    const main = document.querySelector('main');
    if (!nav || !main || document.getElementById('page-p4')?.dataset.p4Mounted) return;
    const tab = document.getElementById('tab-p4') || document.createElement('button');
    tab.type = 'button';
    tab.id = 'tab-p4';
    tab.className = 'nav-tab';
    tab.dataset.page = 'p4';
    tab.textContent = 'Control · P4';
    tab.setAttribute('role','tab');
    tab.setAttribute('aria-controls','page-p4');
    if (!tab.isConnected) nav.append(tab);
    const root = document.getElementById('page-p4') || document.createElement('section');
    root.dataset.p4Mounted = 'true';
    root.id = 'page-p4';
    root.className = 'page-view';
    root.hidden = true;
    root.setAttribute('role','tabpanel');
    root.setAttribute('aria-labelledby',tab.id);
    if (!root.isConnected) main.append(root);

    function syncPage() {
      const active = standalone || location.hash === '#p4';
      root.hidden = !active;
      tab.classList.toggle('active',active);
      tab.setAttribute('aria-selected',String(active));
      tab.tabIndex = active ? 0 : -1;
      if (!active) return;
      main.querySelectorAll('.page-view').forEach(page => { page.hidden = page !== root; });
      nav.querySelectorAll('[data-page]').forEach(other => {
        if (other === tab) return;
        other.classList.remove('active');
        other.setAttribute('aria-selected','false');
        other.tabIndex = -1;
      });
      window.scrollTo(0,0);
    }
    function openChallenge() {
      if (location.hash === '#p4') syncPage();
      else location.hash = 'p4';
    }
    tab.addEventListener('click',openChallenge);
    const actions = document.querySelector('#page-home .home-actions');
    if (actions && !actions.querySelector('[data-open="p4"]')) {
      const link = document.createElement('button');
      link.type = 'button';
      link.className = 'home-primary';
      link.textContent = 'Try the PID tuning challenge ↗';
      link.addEventListener('click',openChallenge);
      actions.append(link);
    }
    tab.addEventListener('keydown',event => {
      const tabs = [...nav.querySelectorAll('[data-page]')];
      const targets = {ArrowLeft:tabs[tabs.length-2], ArrowRight:tabs[0], Home:tabs[0], End:tab};
      if (!Object.hasOwn(targets,event.key)) return;
      event.preventDefault();
      targets[event.key].focus();
      targets[event.key].click();
    });
    window.addEventListener('hashchange',syncPage);
    const style = document.createElement('style');
    style.textContent = `
      #page-p4 p { line-height:1.55; }
      #page-p4 .p1-presets { margin:12px 0 0; }
      #page-p4 select { display:block; max-width:100%; padding:8px; margin:8px 0 12px; }
      #page-p4 .chart-box svg text { font:14px sans-serif; fill:#62677e; }
      #page-p4 .validation-panel p, #p4-observations { color:#565c70; font-size:13px; }
      #p4-input-note { color:#a13c2c; }
      #p4-score { color:#5146a9; white-space:nowrap; }`;
    document.head.append(style);

    const el = key => root.querySelector('#p4-'+key);
    const labels = {kp:'Proportional gain Kp',ki:'Integral gain Ki',kd:'Derivative gain Kd'};
    const field = key => `<div class="p1-field"><label for="p4-${key}">${labels[key]}</label>
      <div class="p1-input-pair"><input type="range" id="p4-${key}-range" aria-label="${labels[key]} slider"
      min="0" max="${maxima[key]}" step="0.05"><input type="number" id="p4-${key}" required min="0"
      max="${maxima[key]}" step="0.05"></div></div>`;
    root.innerHTML = `<header class="module-hero"><p class="section-kicker">CONTROL · P4</p>
      <div class="module-title-row"><h1>PID tuning challenge</h1><span class="formula-badge" id="p4-round"></span></div>
      <p>Drive a hidden plant from rest to a unit setpoint. Tune the gains until all three specifications pass.</p></header>
      <div class="workspace-grid"><aside class="controls-stack"><section class="panel controls-panel">
      <h3>Controller gains</h3>${Object.keys(labels).map(field).join('')}
      <p id="p4-input-note" role="status"></p><button class="primary-button" id="p4-test">Evaluate controller</button>
      <div class="p1-presets"><button id="p4-reset">Reset PID gains</button><button id="p4-new">New challenge</button></div></section>
      <section class="panel prediction-panel"><h3>Predict, then check</h3><p>On this plant, keep Kp = 1 and Kd = 0.
      What happens to the final error when Ki goes from 0 to 1?</p><label for="p4-choice">Your prediction</label>
      <select id="p4-choice"><option value="">Choose an outcome</option><option value="less">It decreases</option>
      <option value="same">It stays approximately equal</option><option value="more">It increases</option></select>
      <button class="reset-button" id="p4-check">Check prediction</button><p id="p4-feedback" role="status"></p></section></aside>
      <div class="results-stack"><section class="panel chart-panel"><p class="chart-eyebrow">LIVE RESPONSE</p>
      <h3>Output &amp; setpoint</h3><p class="p1-legend"><span>━ Output y</span><span>┄ Setpoint r = 1</span></p>
      <div class="chart-box"><svg id="p4-plot" viewBox="0 0 900 350" role="img" aria-label="Output and unit target versus time in seconds"></svg></div>
      <p id="p4-observations"></p></section>
      <section class="panel chart-panel"><h3>Actuator effort</h3>
      <p class="p1-legend"><span>━ Applied control u</span><span>┄ Actuator limits ±3</span></p>
      <div class="chart-box"><svg id="p4-effort" viewBox="0 0 900 350" role="img" aria-label="Applied control and actuator limits versus time"></svg></div></section><section class="panel validation-panel"><div class="chart-heading">
      <h3>Challenge specifications</h3><strong id="p4-score"></strong></div>
      <div class="table-scroll"><table><thead><tr><th>Specification</th><th>Target</th><th>Measured</th><th>Result</th></tr></thead>
      <tbody id="p4-results"></tbody></table></div><p>Overshoot earns 30 points, settling time 40, and final error 30.</p>
      <p id="p4-verdict" role="status"></p></section></div></div>`;

    let round, number = 0, frame = 0;
    function reset() {
      for (const key of Object.keys(initial)) el(key).value = el(key+'-range').value = initial[key];
      render(false);
    }
    function newRound() {
      round = challenge(round?.plant);
      el('round').textContent = 'Challenge '+(++number);
      el('choice').value = el('feedback').textContent = '';
      reset();
    }
    function clearResults(message) {
      el('plot').replaceChildren();
      el('effort').replaceChildren();
      el('observations').textContent = '';
      el('score').textContent = '0 / 100';
      showRows(['—','—','—'],[false,false,false]);
      el('verdict').textContent = message;
    }
    function showRows(values,passes) {
      const names = ['Overshoot','Settling time (±2%)','Absolute steady-state error'];
      const specs = ['≤ 10%', '≤ '+round.limit+' s','≤ 0.1%'];
      el('results').innerHTML = names.map((name,i)=>`<tr><th scope="row">${name}</th><td>${specs[i]}</td>
        <td>${values[i]}</td><td><span class="${passes[i]?'pass-chip':'p1-fail'}">${passes[i]?'PASS':'FAIL'}</span></td></tr>`).join('');
    }
    function render(submitted) {
      cancelAnimationFrame(frame);
      const gains = {};
      for (const key of Object.keys(initial)) {
        if (!el(key).validity.valid || !Number.isFinite(el(key).valueAsNumber)) {
          el('input-note').textContent = labels[key]+': enter 0–'+maxima[key]+' in steps of 0.05.';
          clearResults('Enter valid gains to evaluate.');
          return;
        }
        gains[key] = el(key).valueAsNumber;
      }
      el('input-note').textContent = '';
      try {
        const {result,metrics:m} = simulate(round.plant,gains);
        const {passes,score} = evaluate(m,round.limit);
        plot(el('plot'),result.rows,'y',[{value:1,color:'#69778c',label:'r = 1'}]);
        plot(el('effort'),result.rows,'u',[{value:3,color:'#986448',label:'+3'},{value:-3,color:'#986448',label:'−3'}]);
        el('observations').textContent = 'Rise time (10–90% of target): '+(m.rise === null?'not reached':m.rise.toFixed(3)+' s')+
          ' · Actuator ±3: clipped for '+(100*result.clipped).toFixed(1)+'% of samples.';
        showRows([m.overshoot.toFixed(3)+'%',m.settling === null?'Not confirmed':m.settling.toFixed(3)+' s',
          m.error === null?'Not steady':(100*m.error).toFixed(4)+'%'],passes);
        el('score').textContent = score+' / 100';
        el('verdict').textContent = submitted
          ? (score === 100 ? 'Challenge passed.' : 'Challenge failed.') : '';

      } catch (err) { clearResults(err.message); }
    }
    for (const key of Object.keys(initial)) {
      const num = el(key), slider = el(key+'-range');
      for (const input of [num,slider]) input.addEventListener('input',()=>{
        if (input === slider) num.value = slider.value;
        else if (num.validity.valid) slider.value = num.value;
        cancelAnimationFrame(frame);
        frame = requestAnimationFrame(()=>render(false));
      });
    }
    el('new').onclick = newRound;
    el('reset').onclick = reset;
    el('test').onclick = ()=>render(true);
    el('choice').onchange = ()=>{ el('feedback').textContent = ''; };
    el('check').onclick = ()=>{
      if (!el('choice').value) { el('feedback').textContent = 'Choose a prediction first.'; return; }
      const before = simulate(round.plant,initial).metrics.error;
      const after = simulate(round.plant,{...initial,ki:1}).metrics.error;
      if (before === null || after === null) {
        el('feedback').textContent = 'Inconclusive: at least one run lacks a steady tail at 36 s.'; return;
      }
      const outcome = Math.abs(after-before)<=tolerance?'same':after<before?'less':'more';
      el('feedback').textContent = (outcome===el('choice').value?'Correct. ':'Compare the two runs. ')+
        'Absolute error: '+(100*before).toFixed(3)+'% → '+(100*after).toFixed(3)+
        '%. Integral action accumulates persistent error until the actuator supplies the required steady input.';
    };
    newRound();
    syncPage();
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded',mount,{once:true});
  else mount();
})();
