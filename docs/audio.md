# Audio

Sound model, mixer, assets and capture analysis. All documentation: [index](README.md).

Recorded sounds (`assets/audio/`, 54 WAV files, 48 kHz / 16-bit) driven by the simulation. The mixer is our own
code on top of [miniaudio](https://miniaud.io) 0.11.25, of which only the low-level parts are used: `ma_device`
(Core Audio playback, f32 stereo 48 kHz, 10 ms period), `ma_decoder` (WAV loading) and `ma_encoder` (capture).
The `ma_engine` / node graph / resource manager are compiled out (`MA_NO_ENGINE`, `MA_NO_NODE_GRAPH`,
`MA_NO_RESOURCE_MANAGER`): the 12 engine layers must change rate and gain together, sample-accurately, from one
parameter set, and the offline capture must run the exact same code without a device or extra threads.

- **Threads.** Once per frame the game thread publishes an `AudioFrame` (listener, vehicle pose, rpm, throttle,
  gear, per-wheel contact data, time of day, pause, settings) and the events of the frame into a mailbox
  (`std::mutex`, held only for a copy). The device callback takes it with `try_lock` and never waits for the game;
  if the lock is busy it keeps the previous frame. The game never blocks on the audio thread either (stats come back
  through the same `try_lock` mailbox). Events are detected per physics tick in `VehicleAudioFeed`.
- **Engine.** 12 loops: 3 rpm bands × off-load / full-load × exhaust mic and engine-bay mic (two mics of the same
  recorded engine). Each loop plays at `rate = rpm × pitch / rpm_recorded`; bands crossfade with equal power on
  log rpm (idle → mid 1400–2000 rpm, mid → high 3200–4800 rpm of the played engine speed), throttle blends off-load
  and full-load. `pitch` (default 0.80, Audio panel) lowers the sound of the recorded small inline-4 towards the
  simulated 800–4800 rpm truck engine. The recorded level differences between layers (up to 40 dB) are compressed
  into a 12 dB spread (panel: "Level spread"). Starter recording on start and on every reset, crossfaded into the
  loops after it catches (or at once when the throttle opens); gear clunks when a gear engages; a quiet
  synthesized transmission whine at the propeller-shaft mesh frequency.
- **Electric drive (synthesized, `ev` profile).** No recordings: per motor six sine oscillators placed at its axle
  (±1.35 m along the body, 0.25 m low) on the engine bus. Electrical order 8 × shaft rpm / 60 (4 pole pairs) and its
  2nd harmonic, slot order 48, reduction gear mesh (21 teeth on the motor shaft, second stage 17 teeth at 1 / 3.6 of
  the speed) and the inverter carrier at 3.2 kHz with a sideband at carrier + 2 × the electrical frequency, faded out
  by 12 m/s. Level follows |motor torque| / max torque to the 0.7 power with a 0.12 floor; under regen the weight
  moves toward the slot order; tones above 5 kHz are rolled off. No starter, idle or gear sounds for the EV.
  Road hum (only with `tireSound.roadGain` > 0, hard surfaces only): per wheel band-limited noise 70–450 Hz,
  amplitude-modulated at the tread block rate, on the tire bus.
- **Tires.** From the per-wheel contact samples (surface, load, slip ratio, slip speed, rolling speed): squeal on
  concrete / low grip / rock, proportional to `squealSlip` — the longitudinal and lateral slip ratios combined on an
  ellipse (onset 0.55 longitudinal / 0.40 lateral, full at 1.7× that) times the sliding speed (2.5..9 m/s), gated by
  ground speed (1..2.5 m/s) and wheel load — so pulling away (the SUV's 2H full-throttle launch spins its rear
  wheels at ~5 m/s, ratio ~0.5), braking up to ~0.3 pedal, ABS stops and normal turns are silent, while lock-up,
  handbrake and power slides squeal. Envelope per wheel: attack 0.03 s, release 0.04 s, so a squeal stops within
  ~0.2 s of the slide. A wheel that lands after at least 0.05 s in the air (bounce, jump, rollover) does not squeal for
  0.08 s and reaches full squeal drive after 0.2 s, and the drive fades out between 37° and 60° of body tilt (no squeal
  on the side or roof); short lifts during a slide are not touchdowns. Test `audio_squeal_shape` checks the slide
  squeal against the v0.29.0 values (level, onset step, roughness, centroid, one onset chirp). One voice per wheel (random loop start and pitch per slide, pitch from sliding speed and
  load, an onset chirp that fades out with the slide instead of playing its 1.7 s file to the end). The vehicle's
  `audioProfile` sets the tire sound (`src/vehicle/TireSound.h`): `sporty` plays the recordings as they are (a road
  tire); `trailforge_suv` plays them at 0.78× pitch, 0.7× level and a darker low-pass (1.1–3.2 kHz instead of
  2.5–12 kHz) and adds a synthesized scrub per wheel — white noise band-limited to 120–900 Hz, amplitude-modulated at
  the rate the tread blocks pass the patch (sliding speed / 0.07 m), with the same gate and envelope as the squeal.
  Test `audio_tire_profiles` (real recordings, one wheel at 6 m/s, full squeal): sporty −26.0 dBFS, spectral centroid
  1473 Hz, 27.9 % of the energy below 1 kHz, 6.7 % above 3 kHz; SUV −26.6 dBFS, 1048 Hz, 44.3 %, 0.8 %. Gravel and
  dirt roll and scrub on packed dirt and rock, mud slosh plus
  random squelches, grass rustle and swishes, a water splash when a wheel drops into a puddle (contact below the
  puddle's water level inside its rectangle).
- **Body.** What a hard landing sounds like is the whole vehicle, not a tire: normal bumps add nothing to the rolling
  and surface sounds. Two synthesized layers on the impacts bus, shaped per vehicle by `bodySound`
  (`src/vehicle/BodySound.h`), with no fixed resonances (fixed decaying sines read as a twanging rubber band or a
  stick in a barrel): a **thud** — one low mode (SUV 130 Hz, buggy 170 Hz, coupe 150 Hz) that falls by 20 dB within
  two periods plus a noise hit low-passed between 0.5 kHz (light) and 2.6 kHz (full; buggy 0.8–4 kHz), 10 ms — and a
  **rattle** of sparse noise ticks, each band-passed (Q 1.2–1.5) around its own random frequency (SUV 0.45–2.6 kHz,
  buggy 1.2–6 kHz, coupe 0.7–3.5 kHz), at most one per 18 ms, rate and level following a shake envelope (time
  constant 0.09–0.12 s). A **body jolt** — the felt vertical acceleration of the body (velocity change of the center of
  mass plus gravity, along the body's up axis, low-passed 15 ms) rising through 2.3 g (intensity 0 at 2.3 g, 1 at
  6 g; re-armed below 1.6 g, at most every 0.25 s, the event waits up to 50 ms for the peak of the jolt) — plays a
  thud at 0.8× its intensity and the rattle; a **bump stop** — a suspension reaching it while still compressing at
  ≥ 1.2 m/s (full at 3.5 m/s; 0.4 s per wheel, 0.1 s between wheels) — plays a thud at the wheel and a light rattle.
  A thud within 70 ms of the previous one plays only if it is 1.3× harder (four wheels and the body landing are one
  hit). Bump stops above half intensity add the first clunk of a recorded car-body slam (`suspension_bottom_out_*`,
  high-passed at 250 Hz, faded out after 40 ms with a 50 ms time constant). Body impacts from a Jolt contact listener
  on the chassis body (impulse threshold 1000 N·s, at most every 0.2 s, see the simplifications below) play a metal
  recording faded after 60 ms (70 ms time constant), a thud and the rattle; a repeated recorded slam or impact
  within 0.15 s plays only if 1.3× louder. Chassis creak at large articulation only below 6 m/s with the body upright
  (tilt < 37°) on at least three wheels (not on its side or roof). Tests: `audio_landing` (no rattle and no knock
  crawling over the rock field at 10 km/h or rolling over the bump sequence at 15 km/h; at most 2 events/s over the
  bump sequence and the twister at 30 km/h; a body jolt and a bump stop rendered alone have < 35 % of their energy
  below 200 Hz), `audio_body_shape` (shipped recordings, per preset: a thud alone falls by 20 dB within 60 ms, a jolt
  within 120 ms, a landing within 80 ms, a slam within 150 ms, an impact within 180 ms — v0.29.0: 125–245, 240–335,
  155–300, 195–260 and 410 ms; strongest narrow spectral line <= 10 dB above its neighbours, v0.29.0 24–29 dB for a
  knock; a washboard of 15 hits in 1.5 s within 20 dB of its peak <= 40 % of the time, v0.29.0 79–88 %) and
  `audio_rollover` (the SUV tipped over at full lock on the skid strip: no creak while tilted or off its wheels).
- **Ambience.** Wind by speed (from 3 m/s, full at 30 m/s), day and night beds crossfaded by the sun elevation
  (−6° … +6°).
- **Space.** Every source has a position on the vehicle (exhaust, bay, wheels); gain and equal-power pan are
  relative to the camera (inverse distance from 5 m, silent beyond 200 m). No Doppler, no occlusion, no reverb.
- **Pause.** In the pause menu (and the debug freeze) the gameplay sounds (engine, tires, body, wind) ramp to
  silence and their voices freeze with the simulation. The day / night ambience beds keep playing at full level:
  they are not driven by the simulation, and the brief asks for no ducking. No music plays in pause. The world
  behind the intro and the start menu is treated the same way (gameplay sounds silent).
- **Menu sounds.** UI sounds (`assets/audio/ui/`) for move, select, back, setting changes, menu open and close, on
  their own bus (UI volume).
- **Music.** `assets/music/intro.mp3` plays from the intro through the start menu and fades out over 3 s when
  gameplay starts (1.5 s fade-in; it loops if the menu stays open longer than the track). It is decoded by
  `ma_decoder` (MP3) in the app layer (`AudioSystem`) and streamed from memory through the `MusicSource` interface,
  so the mixer stays free of miniaudio; it is only decoded while audible. The music bus has its own volume; the
  effects volume scales engine, tires, impacts and ambience.
- **Output.** 10 Hz DC blocker, 2 ms look-ahead limiter at −1.5 dBFS and a hard clamp at −1 dBFS.
- **Without a device.** `--no-audio` skips everything; if no device can be opened or the assets are missing, the
  game logs it and runs silent. Tests and the headless build never open a device.

**Audio panel (F3):** master / engine / tires / impacts / ambience & wind / UI / music volumes, mute, pitch factor, level
spread, whine; read-outs: played rpm, load, layer weights and playback rates, tire layers, wind, day/night, active
voices, one-shots, output peak, limiter, audio-thread time per callback and in % of real time.

**Offline capture.** `trailforge --script tools/scenarios/audio_check.txt --audio-capture /tmp/trailforge_audio_check.wav`
renders the mix without a device: every physics tick produces exactly one tick of audio (paused frames render frame
time), so the sound lines up with the simulation regardless of the frame rate. The mixer parameters of every tick go
to `out.wav.csv`. `tools/audio/analyze_capture.py out.wav` (needs numpy; uses ffmpeg and sox if installed) prints
sample/true peak, clipping, integrated loudness, engine pitch tracking against the simulated rpm, the skid band level
while braking, the day/night spectrum balance and the body / suspension events (count, rate, level against the
500 ms before each event, spectral centroid of the event's own sound: the spectrum of the 250 ms after the onset minus
the one before it), and writes spectrograms `/tmp/trailforge_audio_*.png`. `tools/scenarios/landing_audio.txt`
drives the proving ground for it: a rock crawl and the bump sequence at ~15 km/h (no knock, no rattle expected), the
bump sequence and the twister at ~30 km/h, the steps, and the tube-frame buggy over the bumps. The
capture is deterministic for the same tick sequence, but scenario input is timed in real time, so two runs differ
slightly.

**Assets.** `assets/audio/manifest.json` lists every file with its source, license, processing and verification;
`assets/audio/CREDITS.md` has the attributions (53 files CC0 1.0, one CC BY 4.0 — credit required, see the file).
`tools/audio/fetch_freesound.py` rebuilds the set from Freesound (needs `FREESOUND_API_KEY` in the environment,
ffmpeg and sox; the key is never written to disk). The game reads the assets from `assets/audio/` inside the app
bundle or next to the binary, and in development builds also from the source tree (see [Build and run](building.md#build-and-run)).
