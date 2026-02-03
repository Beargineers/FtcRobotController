# Documentation for the autonomous programs pseudo-code

## Overview

The autonomous program is defined as a string of command characters that are interpreted sequentially. Each character represents an action the robot should perform during the autonomous period.

## Command Reference

### Starting Position Commands

These commands **must** appear at the beginning of the program string to define the starting position and initial operating zone:

| Command | Description |
|---------|-------------|
| `F` | Start from the **North** (front) position. Sets the shooting point to the north shooting zone. |
| `B` | Start from the **South** (back) position. Sets the shooting point to the south shooting zone. |

### Collection Commands

These commands instruct the robot to collect game pieces from specific locations:

| Command | Description |
|---------|-------------|
| `1` | Collect from **spike mark 1** (farthest from the goal) |
| `2` | Collect from **spike mark 2** (center) |
| `3` | Collect from **spike mark 3** (closest to the goal) |
| `0` | Collect from the **box** repeatedly until 3 artifacts are held, then shoot |
| `4` | Open the ramp, collect from it, and shoot |

### Special Actions

| Command | Description |
|---------|-------------|
| `/` | **Force shoot now** - Shoot immediately before proceeding to the next action (see below) |
| `R` | **Open ramp** - Navigate to the ramp position and wait briefly |
| `P` | **Push alliance bot** - Push an alliance partner robot slightly from the starting position |
| ` ` | **No-op** - Spaces are ignored and can be used for readability |

## Execution Behavior

1. **One load at a time**: The robot can only hold one load at a time. Shooting happens automatically before collecting a new load.

2. **Implicit vs explicit shooting**: Collection commands (`1`, `2`, `3`) implicitly trigger a shoot before collecting if there's already a load. The `/` command forces a shoot *before* non-collection actions like `R`. For example:
   - `23` = collect 2, shoot, collect 3
   - `2R1` = collect 2, open ramp, shoot, collect 1
   - `2/R1` = collect 2, shoot, open ramp, collect 1

3. **Initial load**: The robot's pre-loaded pieces are shot when the first collection command is encountered.

4. **Zone switching**: Using `F` or `B` mid-program changes the active shooting zone for subsequent shots.

5. **Time limit**: The program runs within a 29-second time limit. After completion (or timeout), the robot parks at an approach position based on its last operating zone.

## Examples

| Program | Description |
|---------|-------------|
| `B00000` | Start south, repeatedly collect from box and shoot |
| `F2R31` | Start north, collect 2, open ramp, shoot, collect 3, shoot, collect 1, shoot |
| `F2/R31` | Start north, collect 2, shoot, open ramp, collect 3, shoot, collect 1, shoot |

