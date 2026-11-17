# Jitterbug

Jitterbug is a fixed rate leader/follower teleop layer for SO-101 arms, written from scratch in Rust. The more complete technical report and discovery is on the [blog](https://www.conjeevaram.net/jitterbug)!

![hero](assets/jitterbug.gif)

## Results

The data below was collected over 80 interleaved 30 second runs at a 200 Hz target against LeRobot's teleop.

| | LeRobot | Jitterbug |
|---|---|---|
| rate, median over 80 runs | 195.3 Hz | 200 Hz |
| wake lateness p50 / p99 | 75 / 227 us | 5 / 8 us |
| ticks a full period or more late | 1 of 469,367 | 0 of 480,000 |

Lateness is per tick, pooled over the 80 runs. LeRobot's is measured against its own per tick deadline, Jitterbug's against the absolute grid.

## Layout

```
src/
  main.rs              cli flags, opens both arms, runs the loop, writes to csv
  teleop.rs            run(), high level loop
  teleop/
    pacer.rs           absolute timing grid
    session.rs         tracks per run state and executes tasks for one tick
    command.rs         defining target pose from multiple sources
    tests.rs           mock servo, run level tests
  safety.rs            clamp to joint limits, cap motion / tick
  servo.rs             bus operations as a trait, so tests can use a mock
  bus.rs               serial port and Feetech bus transactions
  protocol.rs          packet encode and decode
  rt.rs                timer slack, core pinning, cpu time (uses libc)
benchmark/
  run_ab.sh            interleaved LeRobot vs Jitterbug runs for data collection
  lerobot_instrumented.py   LeRobot's teleop loop with per tick timing logged
```

## Usage

```
cargo build --release
cargo test        # no hardware needed, runs against a mock servo
./target/release/jitterbug <leader_port> <follower_port> <ticks> <out.csv> [--hz=200] [--pin=CORE] [--max-speed=2000]
```

`--max-speed` is a per tick motion cap in servo ticks / second (2000 is 10 ticks per tick at 200 Hz). Joint limits are read from the servos at startup, three failed leader reads in a row abort the run, and torque is released on exit and on Ctrl-C.

we don't configure the servos. Run lerobot against the arms once first so its `configure()` has alr written the PID gains, return delay and accel.

The csv has one row per tick: `late_us, read_us, write_us, core, missed, trk, e2e`, then per joint leader, goal and measured positions (`l1..l6`, `g1..g6`, `m1..m6`). `trk` is the worst joint error between goal and follower, `e2e` between leader and follower.

To reproduce the comparison:

```
export LEROBOT_EXTRA='--robot.id=my_follower --teleop.id=my_leader'
ROUNDS=80 SECS=30 benchmark/run_ab.sh
```

`JB_FLAGS` passes extra flags to jitterbug, for example `JB_FLAGS=--pin=20`.
