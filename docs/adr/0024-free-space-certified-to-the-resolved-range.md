# 0024. Free space certified to the range the sampling resolves

Status: Proposed. The owner chose this direction in chat on 2026-11-24 ("Triển khai tiếp") after the evidence
of issue #189 and the critique below (issue #190). It amends ADR 0020 D2 (free distance of a ray) and ADR 0022
decision 1 items b, c, e and h. Values marked TBD stay TBD.

## Context
Evidence in Gazebo SITL with bench values (#189, records outside the repository):
- The leveler is correct: recomputing every published map with an ideal leveler and the EKF attitude gives the
  published sector within 0.5 m in 95 % of the maps.
- The free distance of D2 is where one ray leaves the band, `room / tan(elevation)`. It needs a ray within
  `atan(h / r)` of the horizon (0.36 deg for 40 m with `height_up_m` 0.25) and its error is `(r^2 / h) dtheta`.
  With 1 deg rings it flickers 14-40 m with pitch even with the true attitude, and an EKF tilt error of 0.1-0.2 deg
  moves it by up to 16 m. On a 12 m/s approach the cap followed it: 6.5 m/s peak and 5-6 stop-and-go dips.
- A thin beam (0.3 m tall) at the lidar height: with a 0.53 deg EKF error its hits were leveled above the band and
  counted as free, and rays just under it certified 40 m (over-claim). At 27-35 m it spans 0.5-0.6 deg and no ray
  of a 1 deg grid hits it at all; the rays around it certify free space beyond it.
- At Livox-equivalent density (360 x 56 rays per 100 ms, about 201,600 points/s, a grid) a 40 m wall at 15 m
  read 2.2-4.7 m short in every phase, holds included: the rays just above the band hit the wall but certify the
  band only to `room / tan(elevation)`, and the top slice of the band between there and the wall is not sampled.
- Adding a free-only room `H_min` to D2 doubled the over-claims on the thin beam (12 to 25): it assumes an
  obstacle in the band sticks out of it on the certified side, which an object in the middle of the band does
  not.

Physics: a sensor with angular gaps `g_v` (vertical) and `g_h` (horizontal) crosses, at range `r`, every object at
least `r tan(g_v)` tall inside the band, and puts a column on every object at least `r tan(g_h)` wide. The rays are
physical: an attitude error does not remove one, it moves the leveled height of a return, which the allowance of
decision 3 covers. No certification rule can claim more without a smaller gap.

## Decision
1. Certification. A sector whose halves are covered (each holds at least `n_free` rays, free or obstacle; the
   upper half alone in the rear class) is certified free to one platform value, `certified_range_m`, at most
   40 m. Its distance is the smaller of the confirmed obstacle distance and `certified_range_m`; it is clear when
   no obstacle is confirmed and `certified_range_m` is 40 m. Coverage is unchanged (SYS-CP-029). The per-ray
   band-exit distance of D2 is removed: a ray no longer certifies a distance, it only counts for the coverage of
   its half.
2. Derivation of `certified_range_m` (TBD, owner). With `h_obj` and `w_obj` the smallest obstacle height and
   width that must be seen in every frame, `dz` the largest vertical origin motion in one frame (0.07 m in the 81
   SITL runs of ADR 0022 item i), `H = height_up_m + height_down_m` and `n_obstacle` the confirming returns:
   - rings on the part of the object inside the band: `k(r) = floor((min(h_obj, H) - dz) / (r tan g_v))`;
   - columns in one sector: `c(r) = floor(min(w_obj / 2, r tan 5 deg) / (r tan g_h))`, because a sector confirms
     only its own returns and an object on a sector boundary is split;
   - `certified_range_m` is the largest `r`, at most 40 m, with `k(r) >= 1` and `k(r) * c(r) >= n_obstacle`
     (neither count grows with `r`, so every shorter range passes too).
   The allowance does not enter: it widens the band test (decision 3) instead.
   `g_v` and `g_h` are properties of the sensor in one frame: for the SITL grid its steps; for the Mid-360S they
   are measured from real frames (OPEN-023), as the non-repetitive pattern is not a grid.
3. Band test with the allowance. A return at horizontal distance `d` and height `z` is obstacle evidence when
   `[z - d tan(dtheta_b), z + d tan(dtheta_b)]` meets the band. `attitude_allowance_rad` is the `dtheta_b` of
   ADR 0022 item h (TBD, owner, #156; measured EKF tilt error in SITL: p99 0.68, max 0.85 deg).
4. The frame origin bounds are removed (ADR 0022 decision 1 items b, c, e): the certified range does not depend
   on where a ray leaves the band, so the room `h - dz_b` has no use. The vertical origin motion enters `dz` in
   the derivation. Returns stay shifted exactly by the origin motion (item a) with the lever arm (item d).
5. Non-claim. An object smaller than `h_obj` tall or `w_obj` wide is not claimed in every frame; the Livox pattern
   may still hit it over several frames. The hazard table and the ODD state it (docs-sync).

Evidence of the rule, the same records recomputed offline with a given certified range (bench values, allowance
0.9 deg):

| Case | Result |
| --- | --- |
| Thin beam 0.3 or 0.5 m tall, certified range 15.1 m (1 deg grid) | 0 over-claims in 673 maps |
| Beam 1.0 m tall, certified range 30.1 m; the same with the 0.3 m beam | 0 over-claims; 1 over-claim on the 0.3 m beam |
| Tall wall at 15 m, tilt to 36.6 deg and 161 deg/s, Livox density, certified range 29 m | published minus truth: median -0.001 m, p95 0.049 m, frame-to-frame p95 0.008 m, 0 over-claims |

The node runs of PR #191 (SITL, certified range 27 and 40 m) are in the PR. Those two ranges were set before the
per-sector width rule of decision 2 was written; by that rule they need wider smallest objects than first assumed
(28.6 m for 1.0 x 3.0 m and 40 m for 1.0 x 5.0 m).

Trade-offs of the parameters (`tools/scenarios/certified_range.py --table` prints them for any inputs; `g_h`
1.0 deg, `dz` 0.07 m, band 0.85 m, `n_obstacle` 3, the bench stopping model):

| `h_obj` | `w_obj` | `g_v` | Certified range | Top speed |
| --- | --- | --- | --- | --- |
| 0.5 m | 1.5 m | 1.07 deg (Livox-like) | 14.3 m | 5.5 m/s |
| 1.0 m | 1.5 m | 1.07 deg | 20.9 m | 7.4 m/s |
| 1.0 m | 3.0 m | 1.07 deg | 28.6 m | 9.3 m/s |
| 1.0 m | 5.0 m | 1.07 deg | 40 m (cap) | 11.6 m/s |
| 1.0 m | 1.5 m | 0.5 deg | 29.8 m | 9.5 m/s |
| 1.0 m | 3.0 m | 0.5 deg | 40 m (cap) | 11.6 m/s |

- Range against the smallest obstacle: each metre of certified range needs a larger smallest obstacle (mostly a
  wider one: a sector confirms only its own returns, and at long range the band crosses one ring) or smaller gaps.
  Objects below `h_obj` / `w_obj` are a non-claim per frame (decision 5).
- The band height caps the range for any object: one ring must cross the band, `(H - dz) / tan(g_v)`, 41.8 m for
  0.85 m at 1.07 deg.
- Allowance against false stops: a larger allowance counts more band-edge returns as obstacles (more false stops,
  fewer over-claims); it must cover the measured attitude error (SITL EKF: p99 0.68, max 0.85 deg; target #156).
  It does not change the certified range.
- `n_obstacle` against thin poles: fewer confirming returns resolve narrower objects at the cost of noise
  rejection; a taller band crosses more rings at the cost of overhangs that stop the vehicle.
- The wire cap: the qualified range is 40 m (ICD-CP-03, checked by the cp_brake input check). It allows 11.6 m/s
  with the bench stopping model; 12 m/s needs 41.9 m. A longer qualified range needs ICD-CP-03 and the cp_brake
  input check to change (fork PR and pointer PR), and a sensor that returns from the darkest claimed object at
  that range (the Mid-360S range depends on reflectivity; not measured here).

Alternatives not chosen: `H_min` (above); more rays (not possible on the Mid-360S; a denser simulated sensor would
hide the problem); carrying free evidence over frames (a new mechanism; not needed for the cases above).

## Consequences
- Names: `SectorMapConfig::certified_range_m` and `attitude_allowance_rad`, YAML `sector_map.certified_range_m`
  and `sector_map.attitude_allowance_rad`, both required, null (TBD) in the platform file. Removed:
  `RayFrame::origin_rise_bound_m` and `origin_fall_bound_m`, the two bound arguments of
  `SectorMapBuilder::reset`, the `origin_rise_m` output of `RayLeveler::level`, the per-ray free distances.
- ICD-CP-06 loses its two frame fields; ADR 0020 D2, ADR 0022 and OPEN-031 / OPEN-036 are updated by the
  docs-sync PR. The `(r^2 / h) dtheta` over-claim of ADR 0022 item h no longer exists: the certified range does
  not depend on a ray's elevation; the attitude error now acts on the band test, bounded by the allowance.
- The speed claim is set by two owner choices (`h_obj`, `w_obj`) and one measurement (the gaps); the allowance
  (owner, measured attitude error) sets how conservative the band test is.
