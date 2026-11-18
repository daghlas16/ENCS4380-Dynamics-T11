/* Homework 2, P1 — PID playground. No dependencies; works with file://.
 * State: [plant output y, velocity v, integral of error I, filtered output z].
 * D acts on measured output with a 0.04 s filter to avoid setpoint derivative kick.
 * For a constant reference after its step this approximates Kd de/dt.
 * RK4 integrates the plant, integral, and derivative-filter states together.
 */
(function (global) {
  'use strict';
  const defaults = {plant:'first', K:1, tau:1, m:1, b:2, k:4, kp:9, ki:0, kd:0,
    r:1, lo:-20, hi:20, T:20};
  const filterTime = 0.04;
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  function control(s, p) {
    const raw = p.kp*(p.r-s[0]) + p.ki*s[2] - p.kd*(s[0]-s[3])/filterTime;
    return {raw, u:clamp(raw,p.lo,p.hi)};
  }
  function derivative(s,p) {
    const u = control(s,p).u;
    return [p.plant==='first' ? (p.K*u-s[0])/p.tau : s[1],
      p.plant==='first' ? 0 : (u-p.b*s[1]-p.k*s[0])/p.m,
      p.r-s[0], (s[0]-s[3])/filterTime];
  }
  function rk4(s,dt,p) {
    const a=derivative(s,p), b=derivative(s.map((v,i)=>v+dt*a[i]/2),p);
    const c=derivative(s.map((v,i)=>v+dt*b[i]/2),p);
    const d=derivative(s.map((v,i)=>v+dt*c[i]),p);
    return s.map((v,i)=>v+dt*(a[i]+2*b[i]+2*c[i]+d[i])/6);
  }
  function simulate(input, refinement=1) {
    const p={...defaults,...input};
    const rate = p.plant==='first' ? (1+p.K*(p.kp+p.kd/filterTime))/p.tau
      : Math.max(p.b/p.m, Math.sqrt((p.k+p.kp+p.kd/filterTime)/p.m));
    const dtTarget=Math.min(.01,filterTime/8,.08/Math.max(1,rate))/refinement;
    const n=Math.ceil(p.T/dtTarget), dt=p.T/n;
    if(n>800000) throw new Error('This combination needs too many steps. Reduce the duration or gains.');
    let s=[0,0,0,0], clipped=0, failed=false;
    const rows=[];
    for(let i=0;i<=n;i++) {
      const c=control(s,p);
      if(!s.every(Number.isFinite) || Math.abs(s[0])>1e6) {failed=true;break;}
      if(Math.abs(c.raw-c.u)>1e-8) clipped++;
      rows.push({t:i*dt,y:s[0],u:c.u});
      if(i<n) s=rk4(s,dt,p);
    }
    return {p,rows,dt,failed,clipped:clipped/(n+1),metrics:analyze(rows,p,failed)};
  }
  function analyze(rows,p,failed) {
    const last=rows[rows.length-1], tail=rows.slice(Math.floor(rows.length*.9));
    const final=tail.reduce((a,q)=>a+q.y,0)/tail.length;
    let min=Infinity,max=-Infinity;
    for(const q of tail){min=Math.min(min,q.y);max=Math.max(max,q.y);}
    const scale=Math.max(Math.abs(final),Math.abs(p.r),.001);
    const steady=!failed && max-min<.001*scale;
    const cross=level=>{
      for(let i=1;i<rows.length;i++) if(rows[i].y>=level && rows[i-1].y<level) {
        const a=rows[i-1],b=rows[i];return a.t+(b.t-a.t)*(level-a.y)/(b.y-a.y);
      }
      return null;
    };
    const a=final>1e-8 ? cross(.1*final):null, b=final>1e-8 ? cross(.9*final):null;
    let peak=0,lastOutside=-1;
    for(let i=0;i<rows.length;i++) {
      peak=Math.max(peak,rows[i].y);
      if(Math.abs(rows[i].y-final)>.02*Math.abs(final)) lastOutside=i;
    }
    let candidate=rows[lastOutside+1]?.t;
    if(lastOutside>=0 && lastOutside+1<rows.length) {
      const a=rows[lastOutside], b=rows[lastOutside+1];
      const boundary=final+(a.y>final?1:-1)*.02*Math.abs(final);
      if(b.y!==a.y) candidate=a.t+(b.t-a.t)*(boundary-a.y)/(b.y-a.y);
    }
    const settling=steady && final>1e-8 && candidate!==undefined && last.t-candidate>=.5 ? candidate:null;
    return {steady,final,rise:steady&&a!==null&&b!==null?b-a:null,
      overshoot:steady&&final>1e-8?Math.max(0,100*(peak-final)/Math.abs(final)):null,
      settling,error:steady?p.r-final:null,endError:p.r-last.y};
  }
  function validations() {
    const base={...defaults,lo:-100,hi:100,T:30};
    const po=simulate({...base,kp:9});
    const sample=po.rows.reduce((a,b)=>Math.abs(a.t-.1)<Math.abs(b.t-.1)?a:b);
    const tau=-sample.t/Math.log(1-sample.y/.9);
    const pi=simulate({...base,kp:1,ki:1});
    // The PI characteristic denominator is tau*s² + (1+K*Kp)*s + K*Ki.
    const wn=Math.sqrt(base.K/base.tau), zeta=(1+base.K)/(2*Math.sqrt(base.tau*base.K));
    const checks=[
      {name:'P: Kp=9 — time constant',expected:'0.100 s',actual:tau.toFixed(4)+' s',pass:Math.abs(tau-.1)<.001},
      {name:'P: Kp=9 — steady-state error',expected:'10.00%',actual:(po.metrics.error*100).toFixed(3)+'%',pass:Math.abs(po.metrics.error-.1)<.0001},
      {name:'PI: Kp=1, Ki=1 — ωn and ζ',expected:'1 rad/s; 1',actual:wn.toFixed(3)+' rad/s; '+zeta.toFixed(3),pass:Math.abs(wn-1)<1e-9&&Math.abs(zeta-1)<1e-9},
      {name:'PI: numerical response at t=1 s',expected:'1 − exp(−1) ≈ 0.63212',actual:pi.rows.reduce((a,b)=>Math.abs(a.t-1)<Math.abs(b.t-1)?a:b).y.toFixed(5),pass:Math.abs(pi.rows.reduce((a,b)=>Math.abs(a.t-1)<Math.abs(b.t-1)?a:b).y-(1-Math.exp(-1)))<.001}
    ];
    const pis=[[1,1],[2,3],[4,2]].map(([kp,ki])=>simulate({...base,kp,ki,T:60}));
    const err=Math.max(...pis.map(x=>Math.abs(x.metrics.endError)));
    checks.push({name:'PI: step error (3 stable gain settings)',expected:'≈ 0 (tolerance 0.001)',actual:err.toExponential(2),pass:err<.001});
    return checks;
  }
  // Exposed for numerical validation and future P2–P4 reuse, not automatic coupling.
  global.P1Engine={defaults,simulate,validations};
  if(typeof document==='undefined') return;
  const root=document.getElementById('page-control');
  if(!root)return;
  const $=id=>document.getElementById('p1-'+id);
  let state={...defaults}, frame=0;
  const fields=[['kp','Proportional gain Kp',0,30,.1],['ki','Integral gain Ki',0,20,.1],['kd','Derivative gain Kd',0,5,.05],
    ['r','Setpoint step',.1,5,.1],['K','Plant gain K',.1,3,.1],['tau','Time constant τ (s)',.2,6,.1],
    ['m','Mass m (kg)',.5,4,.1],['b','Damping b (N·s/m)',.2,20,.1],['k','Spring k (N/m)',1,20,.1],
    ['lo','Control lower limit',-100,0,1],['hi','Control upper limit',.1,100,.1],['T','Duration (s)',5,120,1]];
  function field([id,label,min,max,step]) {
    return `<div class="p1-field" data-field="${id}"><label for="p1-${id}">${label}</label><div class="p1-input-pair"><input type="range" id="p1-${id}-slider" aria-label="${label} slider" min="${min}" max="${max}" step="${step}" value="${state[id]}"><input type="number" id="p1-${id}" min="${min}" max="${max}" step="${step}" value="${state[id]}"></div></div>`;
  }
  root.innerHTML=`<div class="module-hero"><a class="breadcrumb-link" href="#home">← Back to home</a><div class="module-title-row"><div><p class="section-kicker">CONTROL / HOMEWORK 2 / P1</p><h1>PID playground<span class="title-period">.</span></h1><p>Set a target. Tune the controller. Watch feedback shape the response.</p></div><span class="formula-badge">e = r − y · negative feedback</span></div></div>
    <div class="p1-presets" aria-label="Controller examples"><button type="button" data-preset="p">P only · fast, with offset</button><button type="button" data-preset="pi">PI · remove the offset</button><button type="button" data-preset="pid">PID · damp the oscillation</button><button type="button" id="p1-reset">Reset</button></div>
    <section class="panel p1-loop"><h3>Closed-loop block diagram</h3><svg viewBox="0 0 900 165" role="img" aria-label="Setpoint r enters a summing junction. Error goes to PID, then actuator limits, then plant. Output y feeds back negatively."><defs><marker id="p1-arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0 0 L8 4 L0 8" fill="#6c63d9"/></marker></defs><g fill="none" stroke="#6c63d9" stroke-width="2"><path d="M40 60H134 M176 60H245 M375 60H445 M575 60H645 M775 60H865" marker-end="url(#p1-arrow)"/><circle cx="155" cy="60" r="20"/><rect x="245" y="30" width="130" height="60" rx="12"/><rect x="445" y="30" width="130" height="60" rx="12"/><rect x="645" y="30" width="130" height="60" rx="12"/><path d="M815 60V135H155V82" marker-end="url(#p1-arrow)"/></g><g fill="#41385d" font-size="17" text-anchor="middle"><text x="67" y="45">r</text><text x="155" y="66">Σ</text><text x="126" y="49">+</text><text x="175" y="106">−</text><text x="207" y="45">e</text><text x="310" y="66">PID</text><text x="510" y="66">Limits</text><text x="607" y="45">u</text><text x="710" y="66">Plant</text><text x="842" y="45">y</text><text x="470" y="158">Unity feedback · measured output = true output</text></g></svg></section>
    <div class="workspace-grid"><aside class="controls-stack"><section class="panel controls-panel"><h3>Simulation controls</h3><div class="p1-field"><label for="p1-plant">Plant model</label><select id="p1-plant"><option value="first">First order — Module 1</option><option value="second">Mass–spring–damper — Module 3</option></select></div>${fields.map(field).join('')}<p class="p1-model" id="p1-model"></p></section>
    <section class="panel prediction-panel"><h3>Predict, then check</h3><p>For K = τ = 1, what happens to the final error when we add Ki = 1 to a P controller with Kp = 1?</p><label class="p1-choice"><input type="radio" name="p1-prediction" value="zero"> It tends to zero</label><label class="p1-choice"><input type="radio" name="p1-prediction" value="same"> It stays at 50%</label><label class="p1-choice"><input type="radio" name="p1-prediction" value="larger"> It increases</label><button type="button" class="primary-button" id="p1-check">Check with a simulation →</button><p id="p1-feedback" role="status"></p></section></aside>
    <div class="results-stack"><section class="panel chart-panel"><div class="chart-heading"><div><p class="chart-eyebrow">LIVE RESPONSE</p><h3>Output &amp; setpoint</h3></div><span class="live-chip">● Live</span></div><p class="p1-legend"><span>━ Output y</span> <span>┄ Setpoint r</span></p><div class="chart-box"><svg id="p1-output" viewBox="0 0 900 350" role="img" aria-label="Output and setpoint against time"></svg></div><p class="chart-footnote" id="p1-status" role="status"></p></section>
    <div class="metrics-grid p1-metrics">${[['rise','Rise time','10–90% of final output'],['overshoot','Overshoot','Above final output'],['settling','Settling time','±2% of final output'],['error','Steady-state error','Setpoint − final output']].map(([id,label,caption])=>`<article class="metric-card"><span class="metric-label">${label}</span><div class="metric-value" id="p1-metric-${id}">—</div><p>${caption}</p></article>`).join('')}</div>
    <section class="panel chart-panel"><p class="chart-eyebrow">ACTUATOR EFFORT</p><h3>Control signal u</h3><p class="p1-legend"><span>━ Applied u</span><span class="p1-limit-key">┄ Actuator limits</span></p><div class="chart-box"><svg id="p1-control" viewBox="0 0 900 350" role="img" aria-label="Applied control signal and lower and upper limits against time"></svg></div><p class="chart-footnote" id="p1-saturation"></p></section>
    <section class="panel validation-panel"><h3>How to read these results</h3><p>P reacts to the current error. I accumulates error to remove a persistent offset. D reacts to output motion and can reduce overshoot. Their effects depend on the plant and the other gains.</p><p>Rise time, overshoot and settling time use the estimated <strong>final output</strong>, not the setpoint. A P controller can settle quickly but still miss its target: always check the steady-state error too.</p><p>Final output is estimated from the last 10% of the simulation. If that tail still varies, metrics show “—”; extend the duration. This is a numerical estimate, not a proof of stability.</p><details><summary>Equations &amp; implementation</summary><p>e = r − y; I′ = e; z′ = (y − z)/Tf; u = clip(Kp·e + Ki·I − Kd·(y − z)/Tf). Tf = 0.04 s.</p><p>First order: y′ = (K·u − y)/τ. Second order: y′ = v; v′ = (u − b·v − k·y)/m. Its DC gain is 1/k.</p><p>RK4 integrates all states from rest. D uses a filtered measurement to avoid derivative kick on a setpoint step. The small numerical integration step is not the digital controller sampling time from P3. Integral windup can occur during saturation; anti-windup belongs to P3.</p></details></section></div></div>
    <section class="panel validation-panel p1-validation"><div class="chart-heading"><div><p class="chart-eyebrow">ENGINEERING CHECK</p><h3>Homework validation</h3></div><span class="small-tag" id="p1-test-count"></span></div><p>Reference tests use K = τ = 1, wide actuator limits and a unit step, independently of the current sliders. Zero PI step error requires a stable loop and a reachable target.</p><div class="table-scroll"><table><thead><tr><th>Check</th><th>Expected</th><th>Calculated</th><th>Status</th></tr></thead><tbody id="p1-tests"></tbody></table></div></section>`;
  function plot(id,rows,key,refs) {
    const svg=$(id), W=900,H=350,L=76,R=24,top=24,B=52;
    let low=0,high=0;
    for(const q of rows){low=Math.min(low,q[key]);high=Math.max(high,q[key]);}
    refs.forEach(q=>{low=Math.min(low,q.value);high=Math.max(high,q.value);});
    const pad=Math.max((high-low)*.12,.05);low-=pad;high+=pad;
    const end=rows[rows.length-1].t||state.T;
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
  function render() {
    try {
      const result=simulate(state), m=result.metrics;
      plot('output',result.rows,'y',[{value:state.r,color:'#8791a8',label:'r = '+state.r}]);
      plot('control',result.rows,'u',[{value:state.lo,color:'#ce8155',label:'min '+state.lo},{value:state.hi,color:'#ce8155',label:'max '+state.hi}]);
      for(const key of ['rise','overshoot','settling','error']) {
        const value=m[key];$('metric-'+key).textContent=value===null?'—':value.toFixed(3)+(key==='overshoot'?'%':key==='error'?'':' s');
      }
      const status=result.failed?'Simulation stopped: output exceeded the numerical safety range.':m.steady?`Final output ≈ ${m.final.toFixed(4)}. Target = ${state.r}. Steady-state error ≈ ${(100*m.error/state.r).toFixed(2)}% of target.`:`Not steady within ${state.T} s. Increase the duration or retune the gains. End-of-window error = ${m.endError.toFixed(4)} (not a steady-state estimate).`;
      $('status').textContent=status;
      $('saturation').textContent=`Applied u is clipped to [${state.lo}, ${state.hi}]. Saturated during ${(100*result.clipped).toFixed(1)}% of sampled instants.`;
      $('model').textContent=state.plant==='first'?`G(s) = ${state.K} / (${state.tau}s + 1)`:`G(s) = 1 / (${state.m}s² + ${state.b}s + ${state.k}); ωn = ${Math.sqrt(state.k/state.m).toFixed(3)}, ζ = ${(state.b/(2*Math.sqrt(state.k*state.m))).toFixed(3)}`;
    }catch(e){$('status').textContent=e.message;for(const key of ['rise','overshoot','settling','error'])$('metric-'+key).textContent='—';$('output').innerHTML='';$('control').innerHTML='';$('saturation').textContent='';}
  }
  function schedule(){cancelAnimationFrame(frame);frame=requestAnimationFrame(render);}
  function sync(){
    $('plant').value=state.plant;
    for(const [id] of fields){$(id).value=state[id];$(id+'-slider').value=state[id];}
    root.querySelectorAll('[data-field]').forEach(el=>{
      const id=el.dataset.field;el.hidden=state.plant==='first'?['m','b','k'].includes(id):['K','tau'].includes(id);
    });schedule();
  }
  for(const [id,,min,max] of fields){
    $(id+'-slider').addEventListener('input',()=>{state[id]=Number($(id+'-slider').value);$(id).value=state[id];schedule();});
    $(id).addEventListener('input',()=>{const v=$(id).valueAsNumber;if(Number.isFinite(v)&&v>=min&&v<=max){state[id]=v;$(id+'-slider').value=v;schedule();}});
    $(id).addEventListener('change',()=>{$(id).value=state[id];});
  }
  $('plant').addEventListener('change',()=>{state.plant=$('plant').value;sync();});
  root.querySelectorAll('[data-preset]').forEach(button=>button.addEventListener('click',()=>{
    const kind=button.dataset.preset;
    state=kind==='p'?{...defaults}:kind==='pi'?{...defaults,kp:1,ki:1}:{...defaults,plant:'second',m:1,b:.4,k:4,kp:12,ki:4,kd:3,T:30};
    $('feedback').textContent='';sync();
  }));
  $('reset').addEventListener('click',()=>{state={...defaults};$('feedback').textContent='';root.querySelectorAll('[name="p1-prediction"]').forEach(el=>el.checked=false);sync();});
  $('check').addEventListener('click',()=>{
    const answer=root.querySelector('[name="p1-prediction"]:checked');
    if(!answer){$('feedback').textContent='Choose a prediction first.';return;}
    const before=simulate({...defaults,kp:1}).metrics;
    state={...defaults,kp:1,ki:1};const after=simulate(state).metrics;
    $('feedback').textContent=`${answer.value==='zero'?'Correct!':'Try again: the error tends to zero.'} P error: ${before.error.toFixed(3)} → PI error: ${after.error.toFixed(5)}. The controls and plots now show this PI experiment.`;sync();
  });
  const checks=validations();
  $('tests').innerHTML=checks.map(c=>`<tr><td>${c.name}</td><td>${c.expected}</td><td>${c.actual}</td><td><span class="${c.pass?'pass-chip':'p1-fail'}">${c.pass?'PASS':'FAIL'}</span></td></tr>`).join('');
  $('test-count').textContent=`${checks.filter(c=>c.pass).length} / ${checks.length} passed`;
  sync();
})(typeof window!=='undefined'?window:globalThis);
