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

Physics: a sensor with angular gaps `g_v` (vertical) and `g_h` (horizontal) and an attitude allowance `dtheta_b`
crosses, at range `r`, every in-band object at least `r tan(g_v + dtheta_b)` tall, and puts a column on every
object at least `r tan(g_h)` wide. No certification rule can claim more without a smaller gap.

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
   - height: `r_h = (h_obj - dz) / tan(g_v + dtheta_b)`;
   - width: the largest `r` with `ceil(n_obstacle / k(r)) * r * tan(g_h) <= w_obj`, where
     `k(r) = floor(H / (r tan g_v))` is the number of rings that cross the band at `r` (at least 1);
   - `certified_range_m = min(r_h, r_w, 40 m)`.
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

Evidence of the rule, the same records recomputed (bench values, `dtheta_b` 0.9 deg):

| Case | Result |
| --- | --- |
| Thin beam 0.3 m or `h_obj` tall, `h_obj` 0.5 m (1 deg grid) | 0 over-claims in 673 maps |
| Same, `h_obj` 1.0 m | 0 over-claims on a 1.0 m beam, 1 on the 0.3 m beam (below `h_obj`) |
| Tall wall at 15 m, tilt to 36.6 deg and 161 deg/s, Livox density, `h_obj` 1.0 m (29 m) | published minus truth: median -0.001 m, p95 0.049 m, frame-to-frame p95 0.008 m, 0 over-claims |
| Same, `h_obj` 0.5 m (14.5 m) | capped at 14.5 m for the wall at 15 m, 0 over-claims |

Bench examples of the derivation (`g_v` 1.07, `g_h` 1.0, `dtheta_b` 0.9 deg, `H` 0.85 m, `n_obstacle` 3): at 15 m
an object 0.52 m tall or 0.26 m wide is resolved; at 30 m 1.03 m tall or 1.57 m wide. The speed then follows from
the cap law: about 10 m/s for 29 m with `CPB_T_LAT` 0.95 s, `CPB_A_BRK` 3 m/s^2 and the 2 m margin.

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
- The speed claim is set by three owner choices (`h_obj`, `w_obj`, `dtheta_b`) and one measurement (the gaps).
