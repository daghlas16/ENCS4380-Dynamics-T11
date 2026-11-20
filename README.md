# Dynamics Lab: Interactive Dynamic Characteristics Toolkit

**ENCS4380 Interfacing Techniques, Homework 1** · Birzeit University · Instructor: Dr. Wasel Ghanem
**Group T11**

A sensor never reports the present. It reports the past, smoothed. This toolkit makes that delay visible: four interactive modules that run live in the browser, with sliders, instant plots, and a "Predict, then check" prompt in each one.

## Live demo

**https://daghlas16.github.io/ENCS4380-Dynamics-T11/**

It is a static site on GitHub Pages. Nothing needs a server, Binder, or a login, and it works on a phone.

---

## Team and module ownership

| Student | ID | Module |
| --- | --- | --- |
| Maen Khalil | 1230811 | Module 1: First-order time response |
| Malek Zeghari | 1230358 | Module 2: First-order frequency response |
| Yamen Daghlas | 1230020 | Module 3: Second-order time response (mass–spring–damper) |
| Ahmad Abu Farha | 1230318 | Module 4: "Guess the sensor" challenge |

Each module was tracked in its own GitHub Issue, assigned to its owner. Every member's work is visible in the commit history.

---

## Screenshots

### Home page
![Home page](docs/screenshots/home.png)

### Module 1: First-order time response
![Module 1 step response with 63.2% and 95% markers](docs/screenshots/module1-step.png)
![Module 1 ramp response and log error plot](docs/screenshots/module1-ramp-log.png)

### Module 2: First-order frequency response
![Module 2 input and output sine waves](docs/screenshots/module2-sine.png)
![Module 2 Bode magnitude plot with cutoff frequency](docs/screenshots/module2-bode.png)

### Module 3: Second-order time response
![Module 3 underdamped, critically damped and overdamped steps](docs/screenshots/module3-response.png)
![Module 3 poles and state-space panel](docs/screenshots/module3-poles-statespace.png)

### Module 4: Guess the sensor
![Module 4 hidden sensor challenge](docs/screenshots/module4-challenge.png)
![Module 4 reveal with true curve overlay](docs/screenshots/module4-reveal.png)

### On a phone
![Mobile view](docs/screenshots/mobile.png)

---

## Modules

### Module 1: First-order time response *(Maen Khalil)*

Model: `τ·ẏ + y = K·x`, with `G(s) = K / (τs + 1)`. Think of a thermometer moving from room air into tea.

- Step and ramp inputs.
- Interactive sliders for **K** and **τ**.
- Response plot, redrawn instantly.
- Markers at **63.2 %** (at t = τ) and **95 %** (at t = 3τ).
- Ramp lag (= τ) and steady-state error (= KAτ).
- Error-fraction plot on a **logarithmic axis**. It is a straight line with slope −1/τ.
- Predict and Check interaction.
- Validation tests.

### Module 2: First-order frequency response *(Malek Zeghari)*

Model: `M = 1 / √(1 + (ωτ)²)`, `φ = −tan⁻¹(ωτ)`.

- Sliders for time constant **τ** and angular frequency **ω**.
- Input and output sine waves on one plot, showing amplitude attenuation and phase lag.
- Bode magnitude plot with the cutoff frequency `ω = 1/τ` marked.
- Live **amplitude ratio**, **phase lag**, and **dynamic error** (`δ = M − 1`).
- Shows how changing frequency and τ changes the output.
- Predict and Check interaction.
- Validation tests.

### Module 3: Second-order time response *(Yamen Daghlas)*

Model: `G(s) = K·ωₙ² / (s² + 2ζωₙs + ωₙ²)`, with a mass–spring–damper (`m`, `b`, `k`) as the physical example.

- Underdamped, critically damped, and overdamped step responses.
- Sliders for mass, damping coefficient, and spring constant.
- Natural frequency ωₙ and damping ratio ζ calculated and displayed.
- Rise time, peak time, overshoot, and settling time read live off the curve.
- Poles shown in the s-plane.
- State-space panel: `A` and `B` update live, eigenvalues of `A` sit next to the poles, and a phase-plane path (x₁, x₂) shows a spiral for underdamped and no spiral for overdamped.
- Predict and Check interaction.
- Validation tests.

### Module 4: "Guess the sensor" challenge *(Ahmad Abu Farha)*

A hidden-sensor game where the user reads a curve and works backwards to the parameters, the way an engineer does with a real sensor.

- Random sensor generation for both first-order and second-order systems.
- Random parameters: **K** and **τ** (first order), **ζ** and **ωₙ** (second order).
- Response curve displayed with a small amount of noise.
- Step responses, plus first-order sine-input responses at a stated frequency.
- Interactive input fields for the user's parameter estimates.
- First-order estimation with the **63.2 % method** and the **logarithmic method**.
- Second-order estimation using **overshoot** and **peak time**.
- **Reveal Sensor** shows the actual hidden parameters.
- The actual response curve is overlaid on the user's estimated response.
- Score based on estimate accuracy: **within 10 % earns full marks**, falling to zero at 50 % error.
- **New Sensor** button starts a new round with fresh random parameters.
- Session score and number of rounds played are tracked and displayed.
- Hints on request that explain the identification methods.

---

## Validation tests

The page checks its own maths against the values from the lecture slides. Each test shows the expected value, the calculated value, and pass or fail.

| # | Test | Expected | Module |
| --- | --- | --- | --- |
| 1 | First-order step: y(τ) and y(3τ) | 0.632·KA and 0.95·KA | 1 |
| 2 | τ = 0.2 s, input sin 2t | amplitude ratio ≈ 0.93, phase lag ≈ 21.8° | 2 |
| 3 | Signals up to 100 Hz, at most 5 % dynamic error | τ_max ≈ 0.52 ms, phase lag ≈ 9.3° at 50 Hz | 2 |
| 4 | Second order, ζ = 0.5 | overshoot ≈ 16.3 % | 3 |
| 5 | Second order, ζ = 1 | no overshoot | 3 |
| 6 | State space, m = 1, b = 2, k = 4 (ζ = 0.5, ωₙ = 2 rad/s) | eigenvalues of A = −1 ± j1.732, equal to the poles | 3 |

---

## Tech and running locally

- Static site: HTML, CSS, and JavaScript, hosted on GitHub Pages.
- All responses are closed-form formulas from the course slides, plotted live in the browser.

To run it locally, clone the repo and open `index.html` in a browser (or serve the folder with `python -m http.server`).

```bash
git clone https://github.com/daghlas16/ENCS4380-Dynamics-T11.git
cd ENCS4380-Dynamics-T11
python -m http.server 8000   # then open http://localhost:8000
```

---

## AI usage

We used AI coding assistants for: *(e.g. debugging, layout and CSS help, drafting this README)*.
Every member can explain every line of code and every equation in their own module.

---

## References

- Lecture slides: *Static and Dynamic Characteristics*, ENCS4380.
- *State-Space Modeling* notes (2026 edition), Example 3 and Sections 6 and 7.1.
 ## Homework 2 
 
 ## P1 *(Maen Khalil)*

P1, P2 and P3 are implemented.
P4 and My system are separate group work.

- First-order plant from Module 1 (`K/(tau*s+1)`) and the physical mass–spring–damper from Module 3 (`1/(m*s²+b*s+k)`).
- Editable plant parameters, Kp/Ki/Kd, setpoint step, actuator limits and simulation duration.
- Negative-feedback block diagram; live output/setpoint and applied-control plots.
- Rise time (10–90%), overshoot and ±2% settling time relative to the estimated final output; steady-state error relative to the target. Estimates use the last 10% of the window and are withheld when the output has not become steady.
- Coupled RK4 integration of plant, error integral and derivative-filter states. Numerical step is selected from the plant/controller rates; no server or dependencies.
- Practical filtered derivative on measurement, Tf = 0.04 s, avoids setpoint derivative kick. Actuator clipping is applied inside integration. Anti-windup is intentionally left for P3.
- P, PI and PID presets; a working Predict, then check experiment.
- Five on-page checks cover all three homework validation requirements and the PI time response. Reference checks use K = tau = 1 and unsaturated, stable configurations.

Changed files: `index.html`, `module1.js` (navigation only), `style.css` (P1 additions), `README.md`.
New file: `p1.js`. Modules 2–4 are unchanged.

The Homework 1 modules continue to use their existing closed-form calculations; P1 uses RK4.
`window.P1Engine` exposes `simulate(parameters)`, `validations()` and `defaults` for future reuse.



## P2: First-Order Sensor Dynamics in a Feedback Control System (Yamen Daghlas)

First-order sensor model: `τs·ż = y − z`, with the sensor output used as the feedback signal instead of the true plant output.

- First-order plant from Module 1 (`K/(tau*s+1)`) and physical mass–spring–damper plant from Module 3 (`1/(m*s²+b*s+k)`).
- Adjustable sensor time constant `τs`, plant parameters, PID gains `Kp/Ki/Kd`, setpoint step, actuator limits and simulation duration.
- Negative-feedback block diagram showing the plant and sensor dynamics in the feedback path.
- Live plot comparing the true plant output `y`, measured sensor output `z`, and setpoint `r`.
- Applied-control signal plot with actuator limits.
- Rise time (10–90%), overshoot and ±2% settling time calculated from the true plant output; steady-state error relative to the target. Metrics use the estimated final output and are withheld when the response has not become steady.
- Coupled RK4 integration of the plant, sensor, integral error and derivative-filter states. The numerical step accounts for plant, sensor and controller rates.
- PID controller uses the measured sensor output `z` for feedback. The sensor follows `τs·ż = y − z`, so increasing `τs` makes the measurement respond more slowly.
- Filtered derivative on measurement, `Tf = 0.04 s`, to avoid setpoint derivative kick. Actuator clipping is applied inside integration. Anti-windup is left for P3.
- First-order and mass–spring–damper plant options.
- Predict and Check interaction demonstrating the effect of sensor time constant on measurement lag.
- Four validation checks cover the sensor step response, fast-sensor behavior, measurement lag and PI steady-state tracking.

Changed files: `index.html` (P2 script and navigation integration), `README.md`. New file: `p2.js`. P1 remains unchanged.

The Homework 1 modules continue to use their existing closed-form calculations. P2 uses RK4 to simulate the plant and sensor dynamics together. `window.P2Engine` exposes `simulate(parameters)`, `validations()`, `defaults` and `mount()` for future reuse.

## P3: Digital PID Controller (Malek Zeghari)

Digital PID implementation using discrete-time proportional, integral, and derivative calculations, with adjustable sampling time and optional anti-windup.

- First-order plant reused from Module 1 (K/(τs+1)).
- Adjustable sampling time Ts, PID gains Kp/Ki/Kd, plant gain K, and time constant τ.
- Numerical simulation of the first-order plant using Euler integration.
- Live system response plot comparing the selected sampling time with a fast-sampling reference (Ts = 0.01 s).
- Stair-step control signal showing the zero-order hold between sampling instants.
- Actuator saturation limits mapped to 0–100% PWM.
- Optional anti-windup using conditional integration to prevent excessive integral accumulation during saturation.
- Live comparison of the integral contribution with and without anti-windup.
- Rise time, overshoot, settling time, and estimated steady-state error.
- Fast sampling, slow sampling, windup example, and reset presets.
- Automatically generated Arduino PID code based on the current controller settings, with a copy button.
- Predict and Check interaction demonstrating the effect of increasing sampling time.
- Five validation checks covering P-only response, PI reference parameters, steady-state tracking, and anti-windup.
- Responsive interface for desktop and mobile devices.

Changed files: index.html (P3 script integration), README.md. New file: p3.js. Existing modules remain unchanged.

P3 uses discrete-time PID calculations combined with Euler integration of the plant. `window.P3Engine` exposes `simulatePID()`, `pidStep()`, `validations()`, `generateArduinoCode()` and `defaults` for future reuse.

The current plant parameters are reference values. The generated Arduino code is an educational example that requires sensor calibration and adaptation to the actual project hardware.


## P4: PID Tuning Challenge (Ahmad Abu Farha)

Tune a PID controller for a hidden plant to meet overshoot, settling-time and steady-state error targets.

- Random first-order or second-order plant based on the Homework 1 models. The plant stays fixed while the gains are adjusted.
- Adjustable Kp, Ki and Kd with live output, setpoint and actuator-limited control signal plots using P1’s RK4 simulation.
- Rise time, percentage overshoot, ±2% settling time and absolute steady-state error.
- PASS/FAIL for each specification: overshoot ≤ 10% (30 points), settling by the challenge deadline (40 points) and steady-state error ≤ 0.1% of the setpoint (30 points).
- New Challenge, Reset PID Gains and Evaluate Controller actions.
- Predict and Check interaction comparing steady-state error when Ki changes from 0 to 1 on the same plant.

