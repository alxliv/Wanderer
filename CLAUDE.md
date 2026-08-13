# Wanderer — working rules

Read this before editing anything in this repo.

## Documentation

**Docs state what is. Git holds history.** Never write "superseded",
"previously", "earlier revisions", "this was moved", "formerly", "no longer",
or struck-through items marked *settled* / *resolved*. Nobody reading these
files wants to know what used to be here. If the past ever matters,
`git log -S` finds it.

**Fix the source of truth.** Never add an amendment file, a shim, a
compatibility note, or a "the following supersedes X" section. Edit the
document or the code that is wrong.

**Plain English.** No jargon where a common word works — "right" and "left",
not starboard and port; "to the rear", not astern. Expand an abbreviation the
first time it appears (mdps, ARW, ODR).

**No filler.** Every sentence must add something. Delete any sentence that
restates the one before it in different words, and any closing flourish that
tells the reader what they just read.

## Frame convention — non-negotiable

**Z points down. The coordinate system is right-handed. All layers, always** —
base, pilot, airframe, IMU, world model, simulator, tools.

That makes **a positive angular velocity a turn to the RIGHT** (clockwise seen
from above); heading increases turning right. Body velocity to wheel
velocities is therefore:

```
v_left  = v + omega * track / 2
v_right = v - omega * track / 2
```

A device's **driver** converts that device's output into this frame —
`motors_set()`, `encoders_sample()`, `imu_sample()`. Sign constants
(`MOTOR_*_SIGN`, `ENC_*_SIGN`, `IMU_YAW_SIGN`) exist only to cancel physical
wiring or mounting. Two modules that count in opposite directions is a bug in
one of them, not a reason for a third sign constant.

See `docs/Wanderer_Command_Architecture.md` §2a.

## Code

Pure C or simple C++, no unnecessary abstraction. Singleton FSMs are C modules
with file-static state, not classes. Integer return codes (0 = success,
negative = refused). Plain function pointers. State-as-function-pointer FSM
pattern.

Design, implement, test, commit — in that order. Golden vectors in
`protocol/cockpit_vectors.txt` are consumed by both the C++ and Python suites;
keep both green.

Firmware owns its calibration numbers. The pilot queries them, never keeps a
copy.
