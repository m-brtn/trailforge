"""Shared vehicle data for the Blender body scripts (no bpy import at module level, so it also runs in plain Python).

Reads a vehicle.json (assets/vehicles/<id>/ or a test folder; wheels, suspension, steering) and blueprint.json (concept frame, rest hub
height), and derives what the body has to clear: the wheel sweep (tire over the full suspension travel and the
steering lock, the same volume the game validates in src/vehicle/WheelSweep.cpp) and the wheelhouse (arch) cutters.

Frames: game body space is +X left, +Y up, +Z forward (m), the glTF frame. Blender is Z up; the glTF exporter (+Y up)
maps Blender (x, y, z) to glTF (x, z, -y), so a game point (x, y, z) is Blender (x, -z, y): the vehicle faces -Y.
"""

import json
import math
import os

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))

WHEELHOUSE_MARGIN = 0.025  # m, tire sweep to arch cutter (src/vehicle/VehicleBlueprint.h kWheelhouseMargin)
FENDER_SIDE_MARGIN = 0.012  # m, tire outer face to the widest flare


def blender(x, y, z):
    """Game body-space point -> Blender coordinates (tuple)."""
    return (x, -z, y)


class Wheel:
    def __init__(self, index, x, z, radius, width, hub_low, hub_high, hub_rest, steer_deg):
        self.index = index            # 0 FL, 1 FR, 2 RL, 3 RR
        self.x, self.z = x, z
        self.radius, self.width = radius, width
        self.hub_low, self.hub_high, self.hub_rest = hub_low, hub_high, hub_rest
        self.steer = math.radians(steer_deg)

    @property
    def tag(self):
        return ("front" if self.index < 2 else "rear") + "_" + ("left" if self.index % 2 == 0 else "right")

    @property
    def reach(self):
        """Farthest tire point from the axle in the (z, y) plane over the steering range (BlueprintBody)."""
        r, h, s = self.radius, 0.5 * self.width, self.steer
        if s < 1e-4:
            return r
        u = h * math.cos(s) / (r * math.sin(s))
        return math.hypot(r, h) if u <= 1.0 else r * math.cos(s) + h * math.sin(s)

    @property
    def arch_radius(self):
        return self.reach + WHEELHOUSE_MARGIN

    @property
    def arch_top(self):
        return self.hub_high + self.arch_radius

    @property
    def inner_x(self):
        """|x| of the steered tire's inner edge minus the margin: the body is free inboard of it."""
        return abs(self.x) - 0.5 * self.width * math.cos(self.steer) - self.radius * math.sin(self.steer) - WHEELHOUSE_MARGIN

    @property
    def outer_x(self):
        return abs(self.x) + 0.5 * self.width + FENDER_SIDE_MARGIN

    def clearance(self, p):
        """Signed clearance of a game-space point to this wheel's sweep (same formula as WheelSweep::clearance)."""
        px, py, pz = p
        hub = min(max(py, self.hub_low), self.hub_high)
        dy, dx, dz = py - hub, px - self.x, pz - self.z
        steps = 15 if self.steer > 0 else 1
        best = 1e9
        for s in range(steps):
            a = -self.steer + 2 * self.steer * s / (steps - 1) if steps > 1 else 0.0
            axial = abs(dx * math.cos(a) - dz * math.sin(a)) - 0.5 * self.width
            along = dx * math.sin(a) + dz * math.cos(a)
            best = min(best, max(axial, math.hypot(along, dy) - self.radius))
        return best


class Vehicle:
    def __init__(self, vehicle, repo=REPO):
        """vehicle: a catalog id (assets/vehicles/<id>), or a path to a vehicle.json or its folder (test vehicles)."""
        path = vehicle if vehicle.endswith(".json") else os.path.join(vehicle, "vehicle.json")
        if not os.path.exists(path):
            path = os.path.join(repo, "assets", "vehicles", vehicle, "vehicle.json")
        self.json_path = os.path.abspath(path)
        self.dir = os.path.dirname(self.json_path)
        with open(self.json_path) as f:
            self.doc = json.load(f)
        self.id = self.doc.get("id", os.path.basename(self.dir))
        # The concept frame: the vehicle's own blueprint, else the one of its procedural visual model.
        bp_path = os.path.join(self.dir, "blueprint.json")
        if not os.path.exists(bp_path):
            bp_path = os.path.join(repo, "assets", "vehicles", self.doc.get("visualModel", self.id), "blueprint.json")
        self.blueprint = json.load(open(bp_path)) if os.path.exists(bp_path) else None
        g = self.doc["geometry"]
        self.wheel_base = g["wheelBase"]
        self.attach = g["attachHeight"]
        self.rest_hub = self.blueprint["scale"]["restHubY"] if self.blueprint else None
        steer = self.doc["steering"]["maxAngleDeg"]
        # Garage ride height (vs. base, m; src/vehicle/VehicleSetup.cpp): attachHeight = base - value. The body must
        # clear the wheels at both ends of the range, as trailforge_vehiclec checks.
        ride = self.doc.get("garage", {}).get("ranges", {}).get("suspension.rideHeight", {})
        self.ride_range = (min(ride.get("min", -0.05), 0.0), max(ride.get("max", 0.08), 0.0))
        self.wheels = []
        for i in range(4):
            axle = self.doc["axles"]["front" if i < 2 else "rear"]
            w, s = axle["wheel"], axle["suspension"]
            x = (0.5 if i % 2 == 0 else -0.5) * axle["track"]
            z = (0.5 if i < 2 else -0.5) * self.wheel_base
            low = self.attach - self.ride_range[1] - s["maxLength"]
            high = self.attach - self.ride_range[0] - s["minLength"]
            rest = self.rest_hub if self.rest_hub is not None else 0.5 * (low + high)
            self.wheels.append(Wheel(i, x, z, w["radius"], w["width"], low, high, rest, steer if i < 2 else 0.0))
        # Concept axle z (front) for the blueprint's concept -> body mapping.
        self.concept_axle_z = 0.5 * self.blueprint["scale"]["conceptWheelBase"] if self.blueprint else 0.5 * self.wheel_base

    # ---- concept frame (blueprint) -> body space, as BlueprintBody::bodyZ / bodyY
    def body_z(self, zc):
        a = abs(zc)
        az = 0.5 * self.wheel_base
        if a <= self.concept_axle_z:
            return zc * az / self.concept_axle_z
        return math.copysign(az + a - self.concept_axle_z, zc)

    def body_y(self, yc):
        return yc + (self.rest_hub or 0.0)

    @property
    def inner_x(self):
        return min(w.inner_x for w in self.wheels)

    def travel_up(self, axle):
        """Rest length - min length: how far the wheel rises above its rest position (base ride height)."""
        w = self.wheels[2 * axle]
        s = self.doc["axles"]["front" if axle == 0 else "rear"]["suspension"]
        return (self.attach - w.hub_rest) - s["minLength"]

    def clearance(self, p):
        return min(w.clearance(p) for w in self.wheels)

    def summary(self):
        lines = [f"vehicle {self.id}: wheelbase {self.wheel_base} m, attachHeight {self.attach} m, rest hub y {self.rest_hub}, "
                 f"garage ride height {self.ride_range[0]:+.2f}..{self.ride_range[1]:+.2f} m"]
        for w in self.wheels:
            lines.append(f"  {w.tag}: hub ({w.x:.3f}, {w.hub_rest:.3f}, {w.z:.3f}) r {w.radius} w {w.width} "
                         f"hub y {w.hub_low:.3f}..{w.hub_high:.3f} (rest length - min length {self.travel_up(w.index // 2):.3f} m) "
                         f"steer {math.degrees(w.steer):.0f} deg, arch radius {w.arch_radius:.3f}, arch top y {w.arch_top:.3f}")
        lines.append(f"  body free inboard of |x| {self.inner_x:.3f}")
        return "\n".join(lines)


if __name__ == "__main__":
    import sys
    print(Vehicle(sys.argv[1] if len(sys.argv) > 1 else "trailforge_suv").summary())
