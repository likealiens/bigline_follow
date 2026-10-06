An advanced Proportional-Integral-Derivative (PID) closed-loop control system developed in TRIK Studio for high-precision autonomous line following using twin optical reflectance sensors.

---

## Technical Specifications & Setup

| Component | Port / Variable | Description |
| --- | --- | --- |
| **Left Light Sensor** | `A4`<br> | Calibrated to store baseline reading in `left`<br> |
| **Right Light Sensor** | `A3`<br> | Calibrated to store baseline reading in `right`<br> |
| **Left Drive Motor** | `M3`<br> | Differential steering motor (`30 + u` % power)

 |
| **Right Drive Motor** | `M4`<br> | Differential steering motor (`30 - u` % power)

 |
| **Control Delay** | `5 ms`<br> | Loop cycle pause block execution

 |

---

## Controller Architecture & Mathematics

The controller calculates real-time deviation from the line using normalized baseline differential readings and applies Proportional, Integral, and Derivative control.

```text
       +------------------+
       | Sensor Readings  |
       | (A4, A3 Ports)   |
       +--------+---------+
                |
                v
       +------------------+
       | Error Derivation | --> err = (sensorA4 - left) - (sensorA3 - right)
       +--------+---------+
                |
                v
       +------------------+
       | PID Control Loop | --> u = P + I + D
       +--------+---------+
                |
                v
       +------------------+
       | Output Saturation| --> Clamped steering u in [-30, +30]
       +--------+---------+
                |
     +----------+----------+
     |                     |
     v                     v
+----+----+           +----+----+
| Motor M3|           | Motor M4|
| 30 + u  |           | 30 - u  |
+---------+           +---------+

```

### 1. Error Calculation

To compensate for light intensity differences between sensors, ambient reference values `left` and `right` are acquired upon startup:


$$\text{err} = (\text{sensorA4} - \text{left}) - (\text{sensorA3} - \text{right})$$

### 2. Control Terms

* **Proportional ($P$):** Reacts instantly to the current magnitude of position error.



$$p = K_p \cdot \text{err}$$



* **Derivative ($D$):** Predicts future trajectory changes and dampens system oscillation.



$$d = K_d \cdot \frac{\text{err} - \text{last\_error}}{dt}$$



* **Integral ($I$):** Accumulates residual offset over real-world elapsed time ($i\_dt$).



$$i\_dt = \frac{\text{time}() - \text{last\_i\_time}}{1000}$$



$$\text{integral} = \text{integral} + (\text{err} \cdot i\_dt)$$



$$i = K_i \cdot \text{integral}$$




### 3. Non-Conditional Limiters (Algebraic Clamping)

To prevent integral windup and keep steering outputs within strict execution bounds without using conditional branching blocks, absolute-value algebraic saturation bounds are implemented:

* **Integral Anti-Windup ($\pm 60$ Limit):**

$$\text{integral} = \frac{\vert{}\text{integral} + 60\vert{} - \vert{}\text{integral} - 60\vert{}}{2}$$



* **Steering Output Saturation ($\pm 30$ Limit):**

$$u = \frac{\vert{}u + 30\vert{} - \vert{}u - 30\vert{}}{2}$$




### 4. Differential Drive Motor Mapping

The output correction $u$ modifies base speed (30%) across both drive wheels:

* $\text{Power}_{M3} = 30 + u$ %


* $\text{Power}_{M4} = 30 - u$ %



---

## Parameter Configuration

| Parameter | Value | Role / Purpose |
| --- | --- | --- |
| **$K_p$** | `1.2`<br> | Proportional gain for immediate error response

 |
| **$K_i$** | `0.06`<br> | Integral gain for steady-state error accumulation

 |
| **$K_d$** | `7.0`<br> | Derivative gain to suppress overshoot and damping

 |
| **$dt$** | `10`<br> | Fixed time scalar for derivative term

 |
| **Base Speed** | `30`<br> | Baseline forward percentage drive speed

 |
| **Integral Ceiling** | `±60`<br> | Saturation threshold to suppress integral windup

 |
| **Max Correction** | `±30`<br> | Steering range saturation bound

 |
| **Loop Delay** | `5 ms`<br> | Main control loop execution timer

 |

---

## Program Logic Flow

1. **Initialization Node (`f=`):**
* Captures startup ambient light values `left` and `right` from ports `A4` and `A3`.


* Sets coefficients $K_p = 1.2$, $K_i = 0.06$, $K_d = 7.0$, $dt = 10$.


* Initializes `last_error = 0`, `integral = 0`, and `last_i_time = time()`.




2. **Control Evaluation Node (`f=`):**
* Reads live sensor inputs and evaluates differential error `err`.


* Computes proportional ($p$) and derivative ($d$) components.


* Computes exact elapsed time $i\_dt$ using system `time()` ticks.


* Accumulates integrated error, applies algebraic clamping to $\pm 60$, and scales $i = K_i \cdot \text{integral}$.


* Sums control vector $u = p + i + d$ and clamps final output $u$ to $\pm 30$.


* Stores `last_error = err` for the next loop execution.




3. **Motor & Delay Execution Loop:**
* Passes power output `30 + u` % to left motor `M3`.


* Passes power output `30 - u` % to right motor `M4`.


* Executes a `5 ms` delay timer before cycling back to the control block.





---

## Results & Behavior

* **Smooth Path Centering:** Real-time proportional and derivative gains rapidly pull the robot back on track while avoiding sharp zig-zag oscillations on straight lines.


* **Controlled Saturation:** Limiting steering $u$ to $\pm30$ ensures individual motor power remains bounded between $0\%$ and $60\%$, keeping forward momentum steady through tight turns without triggering motor reversals.
You can watch the real life demonstration here: 
https://www.youtube.com/shorts/58peG8V484E
