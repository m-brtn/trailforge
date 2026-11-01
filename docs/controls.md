# Controls

Keyboard and gamepad bindings, cameras, menus, the vehicle page, reverse and driver aid keys. All documentation: [index](README.md).

| Action | Keyboard | Gamepad |
|---|---|---|
| Throttle / brake (stopped: release S and press again for reverse; keys ramp, see below) | W / S (or ↑ / ↓) | RT / LT |
| Steering | A / D (or ← / →) | left stick |
| Handbrake | Space | A (South) |
| Forward ↔ reverse, manual (only below 1 m/s) | Q | X (West) |
| Low range (only below 1 m/s; part-time 4WD: 4H ↔ 4L, from 2H to 4L; electric: none, a message says so) | L | Y (North) |
| Transfer case 2H → 4H → 4L (part-time 4WD only; 4L in / out only below 1 m/s); electric: drive mode NORMAL ↔ OFF-ROAD | V | D-pad left |
| Reset to the last start point (on a level: the nearest road point) | R | Back / View |
| Map (full screen; the game pauses; see [Map, waypoint and compass](levels.md#map-waypoint-and-compass)) | M | LB |

Keyboard pedals are a driver-input model (`src/input/KeyboardPedals.h`, the constants are in `KeyboardPedalSettings`),
not a force: a held W goes to 0.5 within 0.15 s and to 1.0 after 0.8 s, a held S to 0.4 within 0.1 s and to 1.0
after 0.7 s; releasing the key drops the pedal at once. A key is on/off, so without a ramp every tap was a full
pedal: the 2H SUV spun its rear wheels on every keyboard launch and locked them on every short brake tap. Measured
(`trailforge_probe keyboard`, concrete lane, instant → ramp): held W max slip 4.80 → 0.92 m/s, sliding 2.23 → 0 s,
marks 0.77 → 0, 0-30 km/h 1.90 → 1.73 s; a 0.25 s S tap at 50 km/h 7.57 → 1.17 m/s, marks 0.77 → 0; a held S stops in
1.83 s / 14.5 m instead of 1.75 s / 13.5 m and still locks the rear wheels (equal front / rear brake torque); a
0.4 s tap (pedal 0.66) still locks them. These numbers predate v0.24.0 (front-biased brakes, faster foot-brake rise). Gamepad triggers are analog and not ramped; scenarios that set pedal values
directly bypass the ramp.
| Camera: follow → close follow → hood → free | C | RB |
| Camera orbit (follow and close follow; swings back behind a moving vehicle 1.75 s after release) / look around (hood; springs back) | — | right stick |
| ABS on / off | B | D-pad up |
| Traction control: off → on → off-road | T | D-pad right |
| Diff locks: open → rear → rear + center → all | X | D-pad down |
| Headlights | H | — |
| Diagnostics | F1 | — |
| Graphics panel (quality, look, cosmetic dirt) | F2 | — |
| Audio panel (volumes, mute, engine pitch, read-outs) | F3 | — |
| Pause menu (Resume, Map, Reset vehicle, Garage, Settings, Quit to main menu, Quit game) | Escape | Start / Menu |
| Debug freeze (physics stops, no menu) / single physics step while frozen | P / N | — |
| Camera orbit / zoom | RMB + mouse / wheel | — |
| Free camera | the cluster that does not drive (arrow keys with W A S D driving, W A S D with the arrows), PgUp/PgDn, Shift — faster | — |
| Menus: select / back | Enter (Return on a Mac) / Esc or Backspace (Delete on a Mac), or the ◀ BACK button | A (South) / B (East) |

**Close follow camera.** A nearer, steeper chase view (the second step of the C / RB cycle; the follow view is
unchanged). It is sized from the vehicle's collision bounds in body space (`Vehicle::bodyBounds`): 0.565 body
lengths behind the rear bumper, 1.65 roof heights above the body origin, looking 12° down at a point half a body
length ahead of the front bumper, so the road shows over the roof and the horizon sits in the upper third. The rig
tilts with the body's smoothed pitch, so on a climb the roof hides as much of the road as on flat ground. The
shipped SUV and EV share the kit collision boxes (4.6 m long, roof 1.30 m above the body origin), so both get the
same pose: measured in `tools/scenarios/camera.txt` on the start pad, 2.6 m behind the bumper and 2.9–2.95 m above
the ground (follow view: 3.2 m above the ground, 7.3 m behind the body origin); the ground straight ahead is in
the view and not hidden by the collision shape from 12–12.25 m in front of the bumper (query `cam_road_ahead`).
Smoothing, terrain collision, the 0.5 m ground clearance and the mouse / right-stick orbit are the follow camera's;
the mouse wheel zooms only the follow view. Captures `/tmp/trailforge_cam_1*_close_*.png` (flat, driving, orbit,
both vehicles on the 12° climb).

**Hood camera.** A rigid view from the roof line in front of the windshield, mounted in body space from
`camera.hood` in vehicle.json (SUV and EV [0, 1.24, 0.62] m, the roof line of the kit bodies; [0, 0.92, 0.38] /
[0, 0.86, 0.30] m in v0.15.0–v0.22.0 for the traced bodies; pitch −1°, near plane 5 cm), so it pitches and rolls
with the body. On flat ground the hood fills the
bottom 19% (SUV) and 20% (EV) of the 60° view, also on the proving-ground slope
(`/tmp/trailforge_cam_0*_hood_*.png`). The view is not a cockpit: there is no interior model.

**Drive keys.** Settings → Controls → DRIVE KEYS chooses the driving cluster: W A S D (default) or the arrow keys. The
other cluster moves the free camera; no other binding uses either cluster, so nothing clashes (CTest
`controls_layout` checks that every key is bound once in both layouts). The keyboard pedal ramp applies to whichever
keys drive. The HUD hint line follows the choice. The menus accept both clusters for navigation in either layout.
The table above names the W A S D layout.

**Controls page.** Below DRIVE KEYS a vector-drawn keyboard shows only the bound keys, in their ANSI positions (function
row, letters, Space, Shift, Return, Backspace, PgUp / PgDn, the inverted-T arrows, and the mouse), each with a callout;
the driving keys are drawn bold. Key cap wording follows the platform the game runs on (macOS: RETURN, DELETE, PAGE UP;
Windows / Linux: ENTER, BACKSPACE, PG UP); there is no switch for it. Below GAMEPAD BUTTONS a drawn controller shows
every bound control with a callout. Menu navigation keys and buttons are not listed (they are the same everywhere
and shown in the menu footer). Button names come from the connected pad (`SDL_GetGamepadType`: PS3 / PS4 / PS5 ->
PlayStation names and shapes — cross, circle, square, triangle, L1 / R1 / L2 / R2, Create / Options; everything else,
and no pad, -> Xbox names A / B / X / Y, LB / RB / LT / RT, View / Menu); AUTO can be overridden with XBOX or
PLAYSTATION. Nintendo pads get Xbox names, which do not match their printed letters. No gamepad has been connected to
this build by a person: the pad diagram, the automatic choice with a real pad and the gamepad bindings are untested
by hand (CTest `pad_glyph_detect` covers the type mapping only; the right-stick camera is checked with a virtual SDL
pad). With several pads connected, the newest one drives; unplugging it falls back to another. Bindings, key labels and glyph names live in
`src/input/ControlLayout.h` (no SDL); `Input` maps them to SDL keys and buttons.

**Menus.** The game starts with an intro (logo over `intro.mp3`; any key, click or gamepad button skips it), then
the start menu: Start, Garage, Settings, Quit. Up / Down (arrows, W / S, D-pad or left stick) move, Enter / Space / gamepad A
select, Left / Right (A / D, D-pad, stick) change a setting, Esc / Backspace / gamepad B go back; the mouse hovers and
clicks rows (clicking the arrows of a setting steps it down / up). Every page below the start / pause menu has a
◀ BACK button at the top left above the page title: clickable, reached with Up from the first row (Down returns to
it), never the default focus. Esc during play opens the pause menu. Settings
(shared by both menus): graphics preset, resolution scale, grass distance, tree detail distance, VSync, master / music / effects / UI volume, the driver
aid defaults applied on Start and on Reset vehicle (ABS, TC mode, diff locks, auto reverse), the MAP rows (minimap on /
off, minimap size small / medium / large, minimap orientation turns with the vehicle / north up, compass bar on / off) and the Controls page
(drive keys W A S D / arrows, gamepad button names auto / Xbox / PlayStation, keyboard and gamepad diagrams).
The TC default "VEHICLE DEFAULT" (the default setting) uses each vehicle's `aids.traction`: on for the electric SUV,
off for the gasoline SUV; OFF / ON / OFF-ROAD force one mode for every vehicle.
They are saved to `settings.cfg` (keys include `grass_distance` / `tree_distance` in m, 0 = the preset's value, `traction_mode` -1 vehicle default / 0 off / 1 on / 2 off-road,
`drive_keys` 0 / 1, `pad_glyphs` 0 auto / 1 Xbox / 2 PlayStation, `minimap` / `minimap_north_up` / `compass` 0 / 1 and
`minimap_size` 0..2; an old `traction = 0` line reads as the vehicle
default)
in the user data directory (`trailforge --paths`) when a Settings page is closed and
loaded on start; unknown keys and bad values are ignored. The F1–F3 debug panels still work in gameplay and override
the settings for the session; they are hidden while a menu is open.

**Vehicle page.** With more than one vehicle in the catalog, Start opens SELECT VEHICLE: a strip of cards (name and
GASOLINE / ELECTRIC), the description and a spec list (mass, peak power, torque, drivetrain, battery, tires), with
the live vehicle as the preview under the garage orbit camera. Left / Right (A / D, D-pad, stick), the mouse wheel
(one vehicle per notch) or a click on a card change the vehicle (it respawns in place); Enter on VEHICLE or DRIVE, or
a click on the selected card, starts the game; Esc goes back. The choice is saved as `vehicle = <id>` in
`settings.cfg` and used on the next start; `--vehicle ID` selects a vehicle at startup and is remembered the same way
(not in scenario mode). The garage edits the running
vehicle and also switches it (its VEHICLE row), which updates the saved choice. The gamepad path is the same menu
input as the keys and is untested with a real pad.

**Reverse.** W is the forward pedal and S the backward pedal (RT / LT on a gamepad). While moving forward, S brakes,
and an S press that started while moving keeps braking through the stop and holds the vehicle for as long as it is
held, also on a slope (CTest `auto_reverse`: 0.8 mm of creep in 6 s on the 12° hill). Reverse engages only after S is
released and pressed again at a standstill (|forward speed| < 0.5 m/s, held 0.25 s); then S becomes the reverse
throttle. In reverse it is symmetric: W brakes the backward roll and holds; released and pressed again it engages
drive. Holding both pedals never switches. Q still toggles the direction by hand below 1 m/s; while moving it is
refused with an on-screen hint "Stop to change direction". The switch runs on physics ticks in simulated time, so it
does not depend on the frame rate. Auto reverse can be turned off (F1 → Tuning → Simulation → "Auto reverse"; then
S only brakes and Q + W drive backwards).

**Driver aids and differentials.** B toggles ABS, T cycles traction control (off → on → off-road), X cycles the
diff locks (none → rear → rear + center → all). The bottom HUD shows ABS / TC lamps (dim = off, green = armed,
blinking amber = acting) and the effective mode of each differential (F / C / R: open, LSD, LOCK). ABS and TC
switches survive R and teleports; the diff locks reset to none. ABS is off by default; TC starts in the vehicle's
default mode (on for the electric SUV, off for the gasoline SUV, see the Settings page). Details and limits are
under [Driver aids](vehicles.md#driver-aids).
While ImGui owns the keyboard or mouse (focus in a field or hovering a window), the vehicle does not receive that input.
F1–F3, P and Escape always work (Escape opens the pause menu, which takes all input until it is closed).

To type a number into an ImGui slider on macOS use **Cmd+click** (in ImGui's macOS mode Ctrl+click acts as a right click);
on other platforms use Ctrl+click.
