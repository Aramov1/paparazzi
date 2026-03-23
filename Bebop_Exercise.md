# Bebop MAV Exercise — Implementation Reference

> Codebase: `conf/airframes/tudelft/bebop_mav_exercise.xml`
> Branch: `actually-final-avoider`
> Platform: Parrot Bebop running Paparazzi autopilot in `AP_MODE_NAV`

---

## 1. System Architecture

The system is built as a **sensor-fusion + state-machine pipeline**. Raw camera frames are processed by independent vision modules, their outputs are fused into a single obstacle signal, and a navigation state machine converts that signal into heading and waypoint commands.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Front Camera (MT9F002)                               │
│                     520 × 240 px · YUV422 · up to 20 FPS                   │
└──────────┬──────────────────┬──────────────────┬───────────────────────────┘
           │                  │                  │
           ▼                  ▼                  ▼
   ┌──────────────┐  ┌──────────────────┐  ┌──────────────────┐
   │ cv_opticflow │  │ cv_edge_detection│  │ cv_detect_contour│
   │  (L-K flow)  │  │  (custom Canny)  │  │  (tree detector) │
   └──────┬───────┘  └────────┬─────────┘  └────────┬─────────┘
          │                   │                      │
          └───────────────────┴──────────────────────┘
                              │
                              ▼
                   ┌─────────────────────┐
                   │   obstacle_avoider  │  ← sensor fusion, runs every loop
                   │  (fusion + ABI pub) │
                   └──────────┬──────────┘
                              │  ABI VISUAL_DETECTION
                              │  quality (obstacle?) + pixel_x (turn direction)
                              ▼
          ┌───────────────────────────────────────────┐
          │          waypoint_navigation (4 Hz)        │
          │          5-state avoidance machine         │
          └──────────────────┬────────────────────────┘
                             │ nav.heading, WP_GOAL
                             ▼
          ┌───────────────────────────────────────────┐
          │      cyberzoo_perimeter_waypoints          │
          │      (perimeter path, updates WP_PATH)    │
          └───────────────────────────────────────────┘
```

---

## 2. Hardware & Camera Configuration

| Parameter | Value |
|-----------|-------|
| Platform  | Parrot Bebop |
| Camera    | MT9F002 front camera |
| Resolution | 520 × 240 px (OUTPUT_HEIGHT × OUTPUT_WIDTH) |
| Pixel format | YUV422 |
| Target FPS | 20 (on-board), streamed at 10 via `video_capture` |
| Zoom | 1.25× |
| Bottom camera | Available for RTP streaming only |

Camera gains (tuned for CyberZoo lighting):

| Channel | Gain |
|---------|------|
| Green 1 & 2 | 10.0 |
| Blue | 12.5 |
| Red | 9.0 |
| Target exposure | 30 |

---

## 3. Vision Modules

### 3.1 Optical Flow — `cv_opticflow`

**File:** `sw/airborne/modules/computer_vision/opticflow/`
**Config:** `OPTICFLOW_CAMERA = front_camera`, max 25 corners, subpixel factor 200, Shi-Tomasi corners (method 0)

Runs Lucas-Kanade sparse optical flow on the front camera. Key outputs (protected by `opticflow_mutex`):

| Output | Meaning |
|--------|---------|
| `div_size` | Size divergence — proxy for looming/time-to-collision |
| `flow_vectors[]` | Per-corner flow vectors (x, y displacement in subpixels) |
| `tracked_cnt` | Number of successfully tracked corners |
| `fps` | Measured processing frame rate |

Derotation correction factors (X: 0.8, Y: 0.85) compensate for rotational body motion bleeding into translational flow estimates. Horizon limited to 10 frames (`MAX_HORIZON`).

---

### 3.2 Edge Detection — `cv_edge_detection`

**Files:** `sw/airborne/modules/computer_vision/edge_detection.cpp` + `cv_edge_detection.h`
**Config:** `EDGE_DETECTION_GRAYSCALE = 1`, `EDGE_DRAW = 1`

A full **manual Canny pipeline** implemented in C++ with no OpenCV dependency, operating directly on the YUV422 camera buffer. It runs a complete 6-step pipeline per frame:

#### Step 1 — Green Floor Detection (fused with blur)

Scans each pixel's YUV values to identify the green CyberZoo floor:
- **Criteria:** `30 < Y < green_thresh_value (160)`, `U < 115`, `V < 120`
- Tracks: `horizonRow`, `leftBound`, `rightBound` (bounding box of floor)
- Counts floor pixels per horizontal third → `floor_area_left/center/right`
- If fewer than `floor_min_pixels (500)` green pixels found: sets all floor areas to 99999 (i.e., "no obstacle from floor signal")

The detected floor boundary constrains subsequent edge detection to the region likely containing obstacles, reducing false positives from background clutter.

#### Step 2 — Gaussian Blur (2-pass separable, 5-tap kernel)

Horizontal pass kernel: `[1 4 6 4 1] / 16`
Vertical pass kernel: `[1 4 6 4 1] / 16`

Applied only within `[leftBound - floor_margin, rightBound + floor_margin]` and below `horizonRow`, keeping computation tight.

#### Step 3 — Sobel Gradient

3×3 Sobel operators in X and Y. Gradient magnitude: `|Gx| + |Gy|` (L1 norm, fast). Gradient direction binned into 4 classes (0°, 45°, 90°, 135°).

#### Step 4+5 — Non-Maximum Suppression + Double Threshold (fused)

NMS suppresses pixels not at a local maximum along the gradient direction. Double threshold:
- `high_thresh = 3 × edge_thresh (default 30)` → strong edge (255)
- `low_thresh = edge_thresh` → weak edge (128)

#### Step 5b — Hysteresis

Two-pass (forward then backward): weak edge pixels (128) adjacent to any strong edge (255) are promoted to strong. The forward pass (top-to-bottom, left-to-right) handles edges connected above/left; the backward pass (bottom-to-top, right-to-left) handles edges connected below/right, ensuring full bidirectional connectivity.

#### Step 6 — Count & Draw

Remaining strong edges counted per horizontal third:

| Output | Description |
|--------|-------------|
| `edge_count_left` | Strong edge pixels in left third |
| `edge_count_center` | Strong edge pixels in center third |
| `edge_count_right` | Strong edge pixels in right third |
| `edge_count_total` | Sum of all three |

If `EDGE_DRAW = 1`, strong edge pixels have their Y channel set to 255 (white overlay on video stream).

Timing is logged to `/data/ftp/internal_000/edges.log`.

**GCS-tunable parameters:**

| Parameter | Default | Range | Meaning |
|-----------|---------|-------|---------|
| `edge_thresh` | 30 | 10–100 | Low threshold for hysteresis |
| `green_thresh_value` | 160 | 100–220 | Max Y for green floor detection |
| `floor_margin` | 180 | 0–300 | Margin (px) added to floor bounding box |
| `floor_min_pixels` | 500 | 0–2000 | Min green pixels to trust floor signal |
| `edge_draw` | 1 | 0/1 | Overlay edges on video |

---

### 3.3 Contour / Tree Detection — `cv_detect_contour`

**Files:** `sw/airborne/modules/computer_vision/detect_contour.c`, `opencv_contour.cpp`
**Config:** `DETECT_CONTOUR_CAMERA = front_camera`, `DETECT_CONTOUR_FPS = 1`

Runs at 1 FPS (slow, heavier computation). Detects vertical obstacle contours (trees/poles) using OpenCV contour analysis. Output is a global `cont_est` struct (protected by `contour_mutex`):

| Field | Meaning |
|-------|---------|
| `contour_d_x` | Detection confidence / presence (≥ 0 means tree visible) |
| `contour_d_y` | Lateral offset of tree in image (positive = right, negative = left) |
| `contour_d_z` | Vertical offset |

---

## 4. Obstacle Avoider — `obstacle_avoider`

**File:** `sw/airborne/modules/obstacle_avoider/obstacle_avoider.c`
**Sender ID:** 43

Runs every control loop. Reads all three vision sources (thread-safe copies) and fuses them into a single binary obstacle flag and a turn direction vote.

### 4.1 Signal 1: Opticflow TTC + Regional Divergence

**Activation guard:** `fps ≥ OA_MIN_FPS (5)`, `tracked_cnt ≥ 4`, `|div_size| > OA_MIN_DIVERGENCE (0.003)`

```
TTC = 1 / (|div_size| × fps)
```

- `TTC < OA_WARNING_TTC (6.0s)` → **obstacle detected**
- Regional divergence computed for left third (x: 0..width/3) and right third (x: 2×width/3..width) using `get_divergence_region()` with min 50 flow vectors
- If `|div_right| > OA_REGION_MIN_DIVERGENCE (0.007)` or `|div_left| > 0.007`:
  - More divergence on right → turn left (vote--)
  - More divergence on left → turn right (vote++)

A safety TTC threshold `OA_SAFETY_TTC (1.5s)` is defined but not currently used in the fusion logic (reserved for future emergency stop).

### 4.2 Signal 2: Edge Count

- `edge_count_center > OA_EDGE_OBSTACLE_THRESHOLD (6000)` → **obstacle detected**
- Compare `|edge_count_right - edge_count_left|` vs `threshold/4 (1500)`:
  - More edges right → turn left (vote--)
  - More edges left → turn right (vote++)

### 4.3 Signal 3: Floor Area

- `floor_area_center < OA_FLOOR_MIN_AREA (2000)` → **obstacle detected** (floor occluded by obstacle)
- Compare `|floor_area_right - floor_area_left|` vs `threshold/2 (1000)`:
  - More floor right → turn right (vote++) — go toward open space
  - More floor left → turn left (vote--)

### 4.4 Signal 4: Tree Contour

- `contour_d_x ≥ 0.0` → **obstacle detected** (tree visible)
- `contour_d_y > 0` → tree is right → turn left (vote--)
- `contour_d_y ≤ 0` → tree is left → turn right (vote++)

### 4.5 ABI Publication

```c
AbiSendMsgVISUAL_DETECTION(sender_id=43,
    pixel_x = turn_vote,    // + = right, - = left
    quality  = obstacle ? OA_OBSTACLE_QUALITY (100000) : 0,
    ...others unused...)
```

The quality threshold in navigation is `0.18 × width × height ≈ 22,464`. `OA_OBSTACLE_QUALITY = 100,000` is intentionally large to always exceed this threshold.

### 4.6 GCS-Tunable Thresholds

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `OA_WARNING_TTC` | 6.0 s | TTC below which obstacle is flagged |
| `OA_SAFETY_TTC` | 1.5 s | (Reserved) Emergency TTC threshold |
| `OA_MIN_FPS` | 5.0 | Min opticflow FPS to trust signal |
| `OA_MIN_DIVERGENCE` | 0.003 | Min div_size to process TTC |
| `OA_IMG_WIDTH` | 272 | Expected image width for region split |
| `OA_REGION_MIN_DIVERGENCE` | 0.007 | Min regional div to vote on direction |
| `OA_EDGE_OBSTACLE_THRESHOLD` | 6000 | Center edge count obstacle threshold |
| `OA_FLOOR_MIN_AREA` | 2000 | Min center floor pixels (below = obstacle) |

---

## 5. Navigation State Machine — `waypoint_navigation`

**File:** `sw/airborne/modules/cyberzoo_navigation/waypoint_navigation.c`
**Run rate:** 4 Hz

Subscribes to `VISUAL_DETECTION` ABI messages (from any sender, `ABI_BROADCAST`). Maps the ABI fields:
- `quality` → `color_count` (obstacle "count")
- `pixel_x` → `sensor_turn_vote` (direction hint)

### 5.1 Hysteresis Counter

```
if color_count ≤ clear_threshold:  obstacle_free_confidence++
if color_count ≥ detect_threshold: obstacle_free_confidence -= 2
else:                              obstacle_free_confidence--
Bound(obstacle_free_confidence, 0, oa_rejoin_clear_confidence=8)
```

Thresholds:
- `color_count_threshold = oa_color_count_frac (0.18) × width × height`
- `clear_color_count_threshold = oa_clear_color_count_frac (0.18) × width × height`

This asymmetric update (`+1` vs `-2`) makes the system **faster to react to obstacles** than to clear them — a deliberate safety bias.

### 5.2 State Diagram

```
                    ┌────────────────┐
              ┌────▶│     SAFE       │◀────────────────────────┐
              │     │  follow path   │                         │
              │     └───────┬────────┘                         │
              │             │ obstacle detected                 │
              │             ▼                                   │
              │     ┌────────────────┐                         │
              │     │ OBSTACLE_FOUND │                         │
              │     │  freeze WPs,   │                         │
              │     │  choose turn   │                         │
              │     └───────┬────────┘                         │
              │             │ immediately                       │
              │             ▼                                   │
              │     ┌────────────────────┐                     │
              │     │ SEARCH_FOR_SAFE_   │                     │
              │     │    HEADING         │                     │
              │     │  rotate heading    │                     │
              │     │  until clear       │    rejoin geometry  │
              │     └───────┬────────────┘    + time + conf    │
              │             │ confidence ≥ 8                   │
              │             ▼                                   │
              │     ┌────────────────┐                         │
              │     │  REJOIN_PATH   │─────────────────────────┘
              │     │  steer back to │
              │     │  WP_PATH       │
              │     └────────────────┘
              │
              │  ┌─────────────────┐
              │  │  OUT_OF_BOUNDS  │
              └──│  rotate until   │
                 │  inside arena   │
                 └─────────────────┘
                 (entered from SEARCH when WP_TRAJECTORY exits arena)
```

### 5.3 State Descriptions

#### SAFE
- Calls `setGoalToPathWaypoint()` → copies WP_PATH position into WP_GOAL
- Calls `setHeadingToPathWaypointLimited(20°)` — slews heading toward WP_PATH at max 20°/cycle, respecting the blocked sector
- Transitions to `OBSTACLE_FOUND` when `color_count ≥ threshold` or `obstacle_free_confidence == 0`

#### OBSTACLE_FOUND
- Moves WP_TRAJECTORY and WP_GOAL to current drone position (stop in place)
- Records avoidance start position (`rejoin_start_x/y`)
- Marks current heading as the "blocked sector center" (remembered for `oa_blocked_sector_time_s = 4.0s`)
- Chooses turn direction via `chooseAvoidanceHeadingIncrement()`:
  1. **Edge-aware**: if within `oa_inner_edge_margin_m (0.4m)` of geofence edge → turn away from edge (geometrically computed from WP__OZ1–OZ4 polygon)
  2. **Sensor vote**: `sensor_turn_vote > 0` → turn right, `< 0` → turn left
  3. **Path-toward**: turn whichever direction brings heading closer to WP_PATH
- Resets `obstacle_free_confidence = 0`, transitions immediately to `SEARCH_FOR_SAFE_HEADING`

#### SEARCH_FOR_SAFE_HEADING
- While `obstacle_free_confidence == 0`: increments heading by `heading_increment (15°)`
- Moves WP_TRAJECTORY 0.7m ahead and checks `InsideObstacleZone()`
  - If outside bounds → `OUT_OF_BOUNDS`
- Moves WP_GOAL to WP_TRAJECTORY (drone drifts slightly forward while rotating — known issue, see comments in code)
- When `confidence ≥ 8` → `REJOIN_PATH`

#### REJOIN_PATH
- **First `oa_rejoin_forward_cycles (4)` cycles**: continues along current safe heading (WP_TRAJECTORY forward), ensuring safe direction is confirmed before turning back
- **After that**: calls `setGoalToPathWaypoint()` and `setHeadingToPathWaypointLimited(20°)` to steer back toward perimeter path
- Re-obstacle detection during rejoin → `OBSTACLE_FOUND`
- Transitions to `SAFE` when all of:
  - `rejoin_counter ≥ rejoin_hold_cycles` (time ≥ 1.5s)
  - `obstacle_free_confidence ≥ 8`
  - `isRejoinGeometrySatisfied()` (geometry check, see below)

#### OUT_OF_BOUNDS
- Rotates heading by `heading_increment` each cycle
- Moves WP_TRAJECTORY forward, checks bounds
- When WP_TRAJECTORY back inside arena → `SEARCH_FOR_SAFE_HEADING`

### 5.4 Rejoin Geometry Check

`isRejoinGeometrySatisfied()` verifies three conditions:

1. **Cross-track distance** to the line from `rejoin_start` to `WP_PATH` < `oa_rejoin_corridor_width_m (0.8m)`
2. **Heading error** to WP_PATH < `oa_rejoin_heading_error_deg (20°)`
3. **Forward probe** of `oa_rejoin_probe_distance_m (1.2m)` toward WP_PATH remains `InsideObstacleZone()`

### 5.5 Blocked Sector Memory

After encountering an obstacle, the heading at impact is stored as `blocked_heading_center`. For the next `oa_blocked_sector_time_s (4s)` = 16 cycles, any heading within `±oa_blocked_sector_half_angle_deg (35°)` is considered blocked. `setHeadingToPathWaypointLimited()` bypasses blocked headings by routing around the sector edge.

### 5.6 Navigation Parameters

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `oa_color_count_frac` | 0.18 | Obstacle detect fraction of image area |
| `oa_clear_color_count_frac` | 0.18 | Clear threshold fraction |
| `oa_safe_max_speed` | 0.3 m/s | Cruise speed |
| `oa_stop_max_speed` | 0.0 m/s | Speed on obstacle (not currently enforced) |
| `oa_heading_slew_deg` | 20° | Max heading change per cycle in SAFE/REJOIN |
| `oa_rejoin_hold_time_s` | 1.5 s | Minimum time in REJOIN before SAFE |
| `oa_rejoin_corridor_width_m` | 0.8 m | Cross-track tolerance for rejoin |
| `oa_rejoin_heading_error_deg` | 20° | Heading tolerance for rejoin |
| `oa_rejoin_probe_distance_m` | 1.2 m | Forward probe distance for bounds check |
| `oa_blocked_sector_half_angle_deg` | 35° | Half-width of blocked return sector |
| `oa_blocked_sector_time_s` | 4.0 s | Blocked sector memory duration |
| `oa_rejoin_clear_confidence` | 8 | Clear samples needed before SAFE rejoin |
| `oa_inner_edge_margin_m` | 0.4 m | Distance to geofence to activate edge-aware turn |
| `oa_rejoin_forward_cycles` | 4 | Cycles to continue safe heading before steering to path |

---

## 6. Perimeter Following — `cyberzoo_perimeter_waypoints`

**File:** `sw/airborne/modules/cyberzoo_navigation/cyberzoo_perimeter_waypoints.c`

Provides the **nominal path** that the navigation module follows when no obstacle is detected. The CyberZoo inner geofence is a quadrilateral defined by four waypoints: `WP__OZ1`, `WP__OZ2`, `WP__OZ3`, `WP__OZ4`.

**Algorithm (runs every periodic cycle):**

1. Get current edge: `start = OZn`, `end = OZ(n+1)`, compute unit direction `[ux, uy]`
2. Project drone position onto current edge → scalar `s` (along-track distance)
3. **Advance edge** if `dist_to_corner < corner_radius (1.0m)` OR `s > len - corner_radius`
4. **Lookahead point**: `t = start + (s + lookahead_distance) × direction`
5. Move `WP_PATH` to the lookahead point (in integer ENU coordinates)

The result is a smooth perimeter-following path that loops continuously through all 4 edges. The navigation state machine targets `WP_PATH` in `SAFE` and `REJOIN_PATH` states.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `cz_perimeter_lookahead` | 1.2 m | How far ahead to place WP_PATH |
| `cz_perimeter_corner_radius` | 1.0 m | Distance to corner triggering edge advance |

---

## 7. Waypoints

Three dynamic waypoints are used during flight:

| Waypoint | Role |
|----------|------|
| `WP_PATH` | Current lookahead target on perimeter (set by `cyberzoo_perimeter_waypoints`) |
| `WP_GOAL` | Active navigation target (commanded to autopilot) |
| `WP_TRAJECTORY` | Test point 0.7m ahead — checked against arena bounds before committing |
| `WP__OZ1`–`WP__OZ4` | Inner geofence corners (static, from flight plan) |

---

## 8. Telemetry — `OA_STATUS` Message (ID 250)

Custom Paparazzi telemetry message sent at `DefaultPeriodic`. Viewable in GCS.

| Field | Type | Meaning |
|-------|------|---------|
| `ttc` | float (s) | Time-to-collision from opticflow |
| `div_left` | float | Regional divergence, left third |
| `div_right` | float | Regional divergence, right third |
| `div_size` | float | Raw size divergence from opticflow |
| `tracked_cnt` | uint16 | Lucas-Kanade tracked corner count |
| `edge_left` | int32 | Edge pixels, left third |
| `edge_center` | int32 | Edge pixels, center third |
| `edge_right` | int32 | Edge pixels, right third |
| `floor_left` | int32 | Floor area pixels, left third |
| `floor_center` | int32 | Floor area pixels, center third |
| `floor_right` | int32 | Floor area pixels, right third |
| `contour_dy` | float | Tree lateral offset (+ = right) |
| `turn_vote` | int32 | Combined turn vote (+ = right, − = left) |
| `obstacle` | uint8 | 1 = obstacle detected, 0 = clear |
| `nav_state` | uint8 | 0=SAFE, 1=OBS_FOUND, 2=SEARCH, 3=REJOIN, 4=OOB |
| `obstacle_free_confidence` | int16 | 0–8 confidence counter |

---

## 9. Known Limitations & Open Issues

1. **Forward drift during SEARCH_FOR_SAFE_HEADING**: The drone continues moving slightly forward while rotating to find a safe heading (noted in a comment in `waypoint_navigation.c`). This can cause it to encroach on an obstacle before the turn is complete.

2. **`oa_stop_max_speed`**: The 0 m/s commanded speed on obstacle detection is defined but `NavSetMaxSpeed()` is only called with `oa_safe_max_speed`, so the stop command is never actually applied.

3. ~~**Rejoin blocked-sector bypass is commented out**~~ — **Fixed** (see §11, ISSUE 9).

4. **Contour detection at 1 FPS**: The tree detector runs very slowly. Between frames the stale `contour_d_x` value persists, but `find_contour()` resets `contour_d_x = -1.0f` whenever no tree is detected, so 20 avoider cycles reading the same positive value are correct — the tree is still there. When it disappears the next contour frame resets detection.

5. ~~**Thread safety for edge counts**~~ — **Fixed** (see §11, BUG 4).


