# 2D Planning and Control: from a Bicycle Model to RRT* + Pure Pursuit

[![Tests](https://github.com/asiansumer/2d-planning-control/actions/workflows/tests.yml/badge.svg?branch=main)](https://github.com/asiansumer/2d-planning-control/actions/workflows/tests.yml)

[中文版](README_zh.md)

[Results](#results) · [Findings](#findings) · [Run locally](#how-to-run)

![RRT* grows a tree, the path is shortened, then the car follows it](results/hero.gif)

This is a small 2D self-driving simulation that I built from scratch with only NumPy and Matplotlib. The car plans a path around obstacles with RRT*, then drives it in closed loop with Pure Pursuit for steering and a PI controller for speed, on top of a kinematic bicycle model. Every function has a pytest test (30 in total).

None of the algorithms are new. A*, RRT, RRT*, Pure Pursuit and PID are all classic, and libraries for them already exist. What I wanted was to write each one myself, connect them into one loop, and then run experiments to see where each part breaks. Most of what I learned came from those experiments, so most of this README is about them.

## How it works

```
map + obstacles
  -> inflate obstacles by a safety margin (1.5 m)
  -> RRT* finds a path (segment-level collision checks)
  -> shortcut (drop points that can see each other) + resample every 0.2 m
  -> every 0.1 s:  PI speed control  +  Pure Pursuit steering  ->  bicycle model step
  -> log lateral error, time, and whether the car body hits anything
```

I built it in seven stages, each one on top of the last:

| Stage | What I built | File |
|---|---|---|
| 1 | The world: circle obstacles and collision checks | `geometry.py` |
| 2 | Kinematic bicycle model, one step per call | `bicycle.py` |
| 3 | P / PI / PD controllers | `control.py` |
| 4 | Pure Pursuit path following | `pure_pursuit.py` |
| 5 | A* on a grid | `astar.py` |
| 6 | RRT, then RRT* | `rrt.py` |
| 7 | Put it all together, run three scenarios, measure | `path_utils.py`, `simulate.py`, `scenarios.py` |

## Results

Three scenarios, cruise speed 5 m/s, car body treated as a circle of radius 0.9 m:

| Scenario | Path length (m) | Time (s) | Mean lateral error (m) | Max lateral error (m) | Reached | Collision |
|---|---:|---:|---:|---:|:---:|:---:|
| Narrow gap | 30.5 | 6.8 | 0.10 | 0.31 | yes | no |
| U-turn | 43.2 | 9.5 | 0.27 | 1.60 | yes | no |
| Random obstacles | 36.2 | 7.9 | 0.14 | 0.46 | yes | no |

| Narrow gap | U-turn | Random obstacles |
|---|---|---|
| ![](results/narrow.gif) | ![](results/u_turn.gif) | ![](results/random.gif) |

The U-turn has a max error several times larger than the others. That turned out to be the most interesting result in the project (Finding 1).

## Findings

### Finding 1: the U-turn path is one the car cannot drive

![Same U-turn at 4 m/s and 8 m/s](results/uturn_speed.gif)

My first guess was that the car was too fast, so I slowed it down. At 4 m/s the max error is still 1.64 m, almost the same as at 5 m/s. At 8 m/s the car hits the wall.

The real reason is geometry. I limit steering to ±35°, and with a 2.5 m wheelbase the tightest circle the car can drive has a radius of 2.5 / tan 35° ≈ 3.6 m. RRT* only knows about points and straight lines, so the path it gives around the end of the wall turns much tighter than that. No controller can follow a turn the car physically cannot make, and going slower doesn't change the turning radius.

To fix this properly the planner has to know how the car moves. That is what Hybrid A* and other kinematic planners do, and it is the next thing I want to try.

### Finding 2: tracking accurately is not the same as being safe

![Collision rate and tracking error vs safety margin](results/margin_tradeoff.png)

Before planning, I grow every obstacle by a margin. I ran 12 random maps for each margin. With small margins (0.3 and 0.6 m) the planned path hugs the obstacles, the tracking error is the lowest (about 0.05 m), and the car still hits something on every single map. The car body is 0.9 m from its centre to its edge, so a margin below that is unsafe no matter how well the controller tracks. At 1.5 m there were no collisions. Going bigger makes paths longer and the error a bit larger, and at 2.1 m RRT* could only find a path on 9 of the 12 maps because some gaps close up.

So I use 1.5 m: the 0.9 m car half-width, plus 0.6 m for the corners that Pure Pursuit cuts.

### Finding 3: the look-ahead distance has to grow with speed

![Mean error over speed and look-ahead distance](results/lookahead_heatmap.png)

![Pure Pursuit at 12 m/s with three look-ahead distances](results/lookahead.gif)

Pure Pursuit steers towards a point Ld metres ahead on the path. At 12 m/s, Ld = 1 m makes the car overshoot and spin in circles (mean error 2.27 m), Ld = 10 m cuts every corner (1.05 m), and Ld = 3 m works well (0.20 m). The heatmap shows the good region (green) moving to the right as speed goes up. That is why the final controller uses Ld = max(2, 0.8 · v) instead of a fixed number.

### Finding 4: RRT* keeps improving, RRT stops at the first path

| RRT vs RRT* (same map, same seed) | Path length vs iterations |
|---|---|
| ![](results/rrt_vs_rrt_star.png) | ![](results/rrt_star_convergence.png) |

RRT stops as soon as it reaches the goal, so the path is whatever the random tree happened to grow, full of zigzags. RRT* does two extra things for every new point: it picks the parent that gives the cheapest total cost, and it re-wires nearby points through the new point if that makes them cheaper. On the narrow-gap map (8 seeds), RRT averages 37.2 m. RRT* gets to 34.2 m after 300 iterations and 30.8 m after 4000, and the spread between seeds gets much smaller.

One bug I found here: at first I only checked whether each new point was inside an obstacle, not the line to it. Paths could then jump straight through the edge of an obstacle. All checks now sample along the whole segment every 0.1 m.

### Finding 5: A* on a grid vs RRT*

![A* and RRT* on the same inflated map](results/astar_vs_rrt_star.png)

On the same map, A* on a 1 m grid with 4 neighbours gives 40.0 m. It is guaranteed to be the shortest path *on the grid*, but it can only move up/down/left/right, so it makes staircases and sharp 90° corners. RRT* plus shortcutting gives 30.5 m with smooth diagonals. A* is exact and repeatable, while RRT* works in continuous space and is random, so each one is better at something different.

### Finding 6: speed control, P → PI → PD

![P, PI and PD responses](results/pid.png)

My speed model is `v += (a - drag · v) · dt`, with target 10 m/s.

- **P only** never reaches the target: with kp = 0.5 it settles at 5 m/s, with kp = 2 at 8 m/s. At steady state the throttle `kp · error` exactly cancels the drag, so some error always has to remain. With kp = 20 the system diverges: each step multiplies the error by `1 - (kp + drag) · dt`, and once that number is outside (-1, 1) it blows up. This is the discrete-time unit circle from my control course.
- **PI** removes the error. ki = 1 settles at 10 m/s with no overshoot, and ki = 5 is faster but overshoots to 12.2.
- **D** didn't help speed control. In this first-order model, adding D made the overshoot slightly worse (12.2 → 12.6). It does matter for a second-order problem: stopping at a line 20 m ahead (position controlled through acceleration). Without D (kd = 0) the car swings between 0 and 40 m forever. With kd = 2 it stops at 20 m.

So the final car uses PI for speed.

## How to run

The demo uses synthetic maps: no dataset, trained weights or GPU is needed. Run everything from the project root.

### Set up

Create a Python 3 virtual environment and install the dependencies (NumPy, Matplotlib, Pillow for GIF output, and pytest):

```bash
git clone https://github.com/asiansumer/2d-planning-control.git
cd 2d-planning-control
python3 -m venv .venv
source .venv/bin/activate  # macOS / Linux
python -m pip install -r requirements.txt
```

On Windows PowerShell, activate with `.venv\Scripts\Activate.ps1` instead. To leave the environment later, run `deactivate`.

### Tests and demo

Run the tests (30 of them):

```bash
python -m pytest -q
```

The [Tests workflow](https://github.com/asiansumer/2d-planning-control/actions/workflows/tests.yml) runs this suite with Python 3.11 on pushes to `main` and on pull requests. It can also be started manually from the Actions tab.

Run the three scenarios. This prints the results table and saves `results/{narrow,u_turn,random}.png/.gif`:

```bash
python -m scripts.run_scenarios
```

Make all the figures in this README (about 2 minutes). Add names to make only some of them: `hero uturn lookahead margin convergence pid astar`.

```bash
python -m scripts.make_figures
python -m scripts.make_figures pid
```

On a server without a display, prefix plotting commands with `MPLBACKEND=Agg`, for example `MPLBACKEND=Agg python -m scripts.run_scenarios`.

## Project structure

```
geometry.py       collision checks, segment checks, obstacle inflation, angles
bicycle.py        kinematic bicycle model
control.py        P / PI / PD
pure_pursuit.py   steering towards a look-ahead point
astar.py          grid A*
rrt.py            RRT and RRT*
path_utils.py     path length, shortcut, resampling, lateral error
simulate.py       closed-loop simulation and metrics
scenarios.py      the three test maps + planning pipeline
scripts/          one script per stage, plus run_scenarios and make_figures
tests/            pytest tests for every module
results/          figures and GIFs
```

## Limitations and what I'd do next

- **The planner ignores how the car turns.** This is the cause of the U-turn problem. Next step: Hybrid A* or Reeds-Shepp curves, so every planned path is drivable.
- **Constant cruise speed.** The car should slow down before sharp turns. A simple fix is to set the target speed from the path curvature.
- **The car is a circle and obstacles don't move.** Real planners use the car's rectangle and predict moving objects.
- **Pure Python is slow.** RRT* with 4000 iterations takes about 1.5 s on one map because the nearest-neighbour search is a plain loop. I plan to rewrite A* or RRT* in C++ in this repo and compare the speed.
- **Kinematic model only.** No tyre slip, so results at high speed are optimistic.
