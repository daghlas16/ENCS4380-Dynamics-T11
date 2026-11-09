
document.addEventListener("DOMContentLoaded", () => {

  // Get the sliders from index.html
  const tauSlider = document.getElementById("m2-tau");
  const frequencySlider = document.getElementById("m2-frequency");

  // Update calculations when sliders change
  function updateModule2() {

    // Read the values from the sliders
    const tau = Number(tauSlider.value);
    const omega = Number(frequencySlider.value);

    // 1. Calculate amplitude ratio
    const M = 1 / Math.sqrt(1 + (omega * tau) ** 2);

    // 2. Calculate phase lag in degrees
    const phase = -Math.atan(omega * tau) * 180 / Math.PI;

    // 3. Calculate dynamic error
    const error = M - 1;

    // Display slider values
    document.getElementById("m2-tau-value").textContent =
      tau.toFixed(1) + " s";

    document.getElementById("m2-frequency-value").textContent =
      omega.toFixed(1) + " rad/s";

    // Display calculated results
    document.getElementById("m2-amplitude").textContent =
      M.toFixed(3);

    document.getElementById("m2-phase").textContent =
      phase.toFixed(1) + "°";

    document.getElementById("m2-error").textContent =
      error.toFixed(3);
    drawSineWaves(omega, M, phase);
    const cutoff = 1 / tau;

document.getElementById("m2-cutoff").textContent =
  cutoff.toFixed(2) + " rad/s";

drawBodePlot(tau, omega);
  }

  function drawSineWaves(omega, M, phaseDegrees) {
    const svg = document.getElementById("m2-sine-chart");

    const width = 900;
    const height = 390;
    const left = 65;
    const right = 30;
    const top = 35;
    const bottom = 55;

    const plotWidth = width - left - right;
    const plotHeight = height - top - bottom;
    const centerY = top + plotHeight / 2;
    const amplitudeScale = plotHeight * 0.42;

    const phaseRadians = phaseDegrees * Math.PI / 180;

    // Display three sine-wave cycles
    const duration = 6 * Math.PI / omega;

    let inputPath = "";
    let outputPath = "";

    for (let i = 0; i <= 600; i++) {
      const t = duration * i / 600;
      const x = left + plotWidth * i / 600;

      const input = Math.sin(omega * t);
      const output = M * Math.sin(omega * t + phaseRadians);

      const inputY = centerY - input * amplitudeScale;
      const outputY = centerY - output * amplitudeScale;

      inputPath += `${i === 0 ? "M" : "L"}${x.toFixed(2)},${inputY.toFixed(2)} `;
      outputPath += `${i === 0 ? "M" : "L"}${x.toFixed(2)},${outputY.toFixed(2)} `;
    }

    let grid = "";

    // Horizontal grid lines
    for (let value = -1; value <= 1; value += 0.5) {
      const y = centerY - value * amplitudeScale;

      grid += `<line x1="${left}" y1="${y}"
        x2="${width - right}" y2="${y}"
        stroke="#e7eaf3" stroke-dasharray="4 5"/>`;

      grid += `<text x="${left - 12}" y="${y + 4}"
        text-anchor="end" font-size="13" fill="#64748b">
        ${value.toFixed(1)}
      </text>`;
    }

    // Time-axis markers
    for (let i = 0; i <= 6; i++) {
      const x = left + plotWidth * i / 6;
      const t = duration * i / 6;

      grid += `<line x1="${x}" y1="${top}"
        x2="${x}" y2="${height - bottom}"
        stroke="#eef0f6"/>`;

      grid += `<text x="${x}" y="${height - bottom + 22}"
        text-anchor="middle" font-size="12" fill="#64748b">
        ${t.toFixed(1)}
      </text>`;
    }

    svg.innerHTML = `
      ${grid}

      <path d="${inputPath}"
        fill="none"
        stroke="#7465e8"
        stroke-width="3"/>

      <path d="${outputPath}"
        fill="none"
        stroke="#14b8a6"
        stroke-width="3"/>

      <text x="${width / 2}" y="${height - 8}"
        text-anchor="middle" font-size="14" fill="#64748b">
        Time (s)
      </text>

      <line x1="580" y1="22" x2="610" y2="22"
        stroke="#7465e8" stroke-width="3"/>
      <text x="617" y="27" font-size="14" fill="#334155">
        Input
      </text>

      <line x1="715" y1="22" x2="745" y2="22"
        stroke="#14b8a6" stroke-width="3"/>
      <text x="752" y="27" font-size="14" fill="#334155">
        Output
      </text>
    `;
  }

function drawBodePlot(tau, selectedOmega) {
  const svg = document.getElementById("m2-bode-chart");

  const width = 900;
  const height = 390;
  const left = 75;
  const right = 35;
  const top = 35;
  const bottom = 65;

  const plotW = width - left - right;
  const plotH = height - top - bottom;

  const cutoff = 1 / tau;

  // Logarithmic frequency range around cutoff
  const minOmega = cutoff / 100;
  const maxOmega = cutoff * 100;

  const logMin = Math.log10(minOmega);
  const logMax = Math.log10(maxOmega);

  // Magnitude axis: 0 to -45 dB
  const minDb = -45;
  const maxDb = 0;

  function xScale(omega) {
    return left +
      (Math.log10(omega) - logMin) /
      (logMax - logMin) * plotW;
  }

  function yScale(db) {
    return top +
      (maxDb - db) / (maxDb - minDb) * plotH;
  }

  function magnitudeDb(omega) {
    const M = 1 / Math.sqrt(1 + (omega * tau) ** 2);
    return 20 * Math.log10(M);
  }

  let path = "";

  for (let i = 0; i <= 500; i++) {
    const logOmega = logMin +
      (logMax - logMin) * i / 500;

    const omega = 10 ** logOmega;
    const x = xScale(omega);
    const y = yScale(magnitudeDb(omega));

    path += `${i === 0 ? "M" : "L"}${x.toFixed(2)},${y.toFixed(2)} `;
  }

  let grid = "";

  // Horizontal dB grid lines
  for (let db = 0; db >= -45; db -= 5) {
    const y = yScale(db);

    grid += `
      <line x1="${left}" y1="${y}"
        x2="${width - right}" y2="${y}"
        stroke="#e7eaf3" stroke-dasharray="4 5"/>
      <text x="${left - 12}" y="${y + 4}"
        text-anchor="end" font-size="13"
        fill="#64748b">${db}</text>`;
  }

  // Log-frequency grid lines
  for (let exponent = -2; exponent <= 2; exponent++) {
    const omega = cutoff * 10 ** exponent;
    const x = xScale(omega);

    grid += `
      <line x1="${x}" y1="${top}"
        x2="${x}" y2="${height - bottom}"
        stroke="#edf0f5"/>
      <text x="${x}" y="${height - bottom + 22}"
        text-anchor="middle" font-size="12"
        fill="#64748b">${omega.toPrecision(2)}</text>`;
  }

  const cutoffX = xScale(cutoff);
  const cutoffY = yScale(magnitudeDb(cutoff));

  // Show selected frequency only when visible
  let selectedMarker = "";

  if (selectedOmega >= minOmega && selectedOmega <= maxOmega) {
    const selectedX = xScale(selectedOmega);
    const selectedY = yScale(magnitudeDb(selectedOmega));

    selectedMarker = `
      <circle cx="${selectedX}" cy="${selectedY}"
        r="7" fill="#14b8a6"
        stroke="white" stroke-width="3"/>`;
  }

  svg.innerHTML = `
    ${grid}

    <path d="${path}" fill="none"
      stroke="#7465e8" stroke-width="3.5"/>

    <line x1="${cutoffX}" y1="${top}"
      x2="${cutoffX}" y2="${height - bottom}"
      stroke="#f59e0b"
      stroke-width="2" stroke-dasharray="7 6"/>

    <circle cx="${cutoffX}" cy="${cutoffY}"
      r="6" fill="#f59e0b" stroke="white"
      stroke-width="2"/>

    <text x="${cutoffX + 12}" y="${cutoffY - 12}"
      font-size="14" font-weight="bold"
      fill="#b45309">Cutoff: -3 dB</text>

    ${selectedMarker}

    <text x="${width / 2}" y="${height - 10}"
      text-anchor="middle" font-size="14"
      fill="#64748b">Angular Frequency (rad/s) — Log Scale</text>

    <text x="18" y="${height / 2}"
      text-anchor="middle" font-size="14"
      fill="#64748b"
      transform="rotate(-90 18 ${height / 2})">
      Magnitude (dB)
    </text>
  `;
}

function runValidationTests() {
  const testTau = 0.2;
  const testOmega = 2;

  const amplitude = 1 / Math.sqrt(1 + (testOmega * testTau) ** 2);
  const phase = Math.atan(testOmega * testTau) * 180 / Math.PI;

  const maxError = 0.05;
  const maxFrequency = 100;

  const tauMax = Math.sqrt(1 / (1 - maxError) ** 2 - 1) /
    (2 * Math.PI * maxFrequency);

  const phase50 = Math.atan(2 * Math.PI * 50 * tauMax) *
    180 / Math.PI;

  const tests = [
    ["m2-test-amplitude", "m2-status-amplitude",
      amplitude, 0.93, 0.01, 3, ""],

    ["m2-test-phase", "m2-status-phase",
      phase, 21.8, 0.1, 1, "°"],

    ["m2-test-tau", "m2-status-tau",
      tauMax * 1000, 0.52, 0.01, 3, " ms"],

    ["m2-test-phase50", "m2-status-phase50",
      phase50, 9.3, 0.2, 1, "°"]
  ];

  tests.forEach(([valueId, statusId, actual, expected,
                  tolerance, decimals, unit]) => {
    const passed = Math.abs(actual - expected) <= tolerance;

    document.getElementById(valueId).textContent =
      actual.toFixed(decimals) + unit;

    const status = document.getElementById(statusId);
    status.textContent = passed ? "PASS" : "FAIL";
    status.style.color = passed ? "#15803d" : "#dc2626";
    status.style.fontWeight = "bold";
  });
}

function checkModule2Prediction() {

  // Get the selected answer
  const selected = document.querySelector(
    'input[name="m2-prediction"]:checked'
  );

  const result = document.getElementById(
    "m2-prediction-result"
  );

  if (!selected) {
    result.textContent = "Please select an answer first.";
    result.style.color = "#dc2626";
    return;
  }

  // Read the current frequency and time constant
  const tau = Number(tauSlider.value);
  const maxOmega = Number(frequencySlider.max);

  let oldOmega = Number(frequencySlider.value);

  // If frequency is already at maximum, reset it
  if (oldOmega >= maxOmega) {
    oldOmega = 2;
    frequencySlider.value = oldOmega;
  }

  // Calculate amplitude before changing frequency
  const oldM = 1 / Math.sqrt(1 + (oldOmega * tau) ** 2);

  // Double the frequency (without exceeding slider max)
  const newOmega = Math.min(maxOmega, oldOmega * 2);
  frequencySlider.value = newOmega;

  // Update charts and calculated values
  updateModule2();

  // Calculate the new amplitude ratio
  const newM = 1 / Math.sqrt(1 + (newOmega * tau) ** 2);

  if (selected.value === "decrease") {
    result.style.color = "#15803d";
    result.textContent = "Correct! The amplitude decreases. ";
  } else {
    result.style.color = "#dc2626";
    result.textContent = "Not quite. The amplitude decreases. ";
  }

  result.textContent +=
    `Frequency: ${oldOmega.toFixed(1)} → ${newOmega.toFixed(1)} rad/s. ` +
    `Amplitude ratio: ${oldM.toFixed(3)} → ${newM.toFixed(3)}.`;
}


  // Run calculations when sliders move
  tauSlider.addEventListener("input", updateModule2);
  frequencySlider.addEventListener("input", updateModule2);

  // Calculate initial values
  document.getElementById("m2-check-prediction")
  .addEventListener("click", checkModule2Prediction);
  runValidationTests();
  updateModule2();

});
