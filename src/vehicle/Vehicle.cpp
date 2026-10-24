#include "vehicle/Vehicle.h"

#include "physics/PhysicsWorld.h"
#include "vehicle/VehicleShape.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <stdexcept>

namespace tf {
namespace {

constexpr float kDegToRad = 3.14159265358979f / 180.0f;
constexpr float kRadToDeg = 180.0f / 3.14159265358979f;

} // namespace

Vehicle::Vehicle(PhysicsWorld& world, const SurfaceTable& surfaces, const VehicleConfig& config,
                 JPH::RVec3Arg position, JPH::QuatArg rotation)
    : m_world(world), m_surfaces(surfaces), m_config(config) {
    using namespace JPH;
    const VehicleConfig& c = m_config;

    // ---- Body shape (matches the visual body model)
    StaticCompoundShapeSettings compound;
    for (const BodyBox& b : kBodyBoxes)
        compound.AddShape(Vec3(b.center[0], b.center[1], b.center[2]), Quat::sIdentity(),
                          new BoxShape(Vec3(b.halfExtents[0], b.halfExtents[1], b.halfExtents[2]), kBodyConvexRadius));
    ShapeSettings::ShapeResult compoundResult = compound.Create();
    if (compoundResult.HasError())
        throw std::runtime_error(std::string("Vehicle shape: ") + compoundResult.GetError().c_str());
    const RefConst<Shape> rawShape = compoundResult.Get();

    // ---- Center of mass: move the shape COM to the given point in body space
    const Vec3 desiredCom(0.0f, c.comHeight, c.comForward);
    const Vec3 comOffset = desiredCom - rawShape->GetCenterOfMass();
    ShapeSettings::ShapeResult offsetResult = OffsetCenterOfMassShapeSettings(comOffset, rawShape).Create();
    if (offsetResult.HasError())
        throw std::runtime_error(std::string("Vehicle COM: ") + offsetResult.GetError().c_str());
    const RefConst<Shape> bodyShape = offsetResult.Get();

    // ---- Mass and inertia: inertia from the shape (uniform density), scaled to mass x multiplier
    MassProperties mp = rawShape->GetMassProperties();
    mp.ScaleToMass(c.mass);
    mp.mInertia = Mat44::sScale(Vec3::sReplicate(c.inertiaScale)) * mp.mInertia;
    mp.mInertia(3, 3) = 1.0f;

    BodyCreationSettings bcs(bodyShape, position, rotation, EMotionType::Dynamic, Layers::Moving);
    bcs.mOverrideMassProperties = EOverrideMassProperties::MassAndInertiaProvided;
    bcs.mMassPropertiesOverride = mp;
    bcs.mFriction = 0.5f;           // body friction (when grounding the underbody / rolling over)
    bcs.mLinearDamping = 0.0f;      // no hidden "air" drag
    bcs.mAngularDamping = 0.02f;
    bcs.mAllowSleeping = false;     // telemetry is always up to date
    m_body = world.bodies().CreateBody(bcs);
    if (m_body == nullptr)
        throw std::runtime_error("Vehicle: body limit reached");
    m_bodyId = m_body->GetID();
    world.bodies().AddBody(m_bodyId, EActivation::Activate);

    // ---- Wheels
    VehicleConstraintSettings vs;
    vs.mUp = Vec3::sAxisY();
    vs.mForward = Vec3::sAxisZ();
    vs.mMaxPitchRollAngle = JPH_PI; // rollover prevention disabled: the vehicle can roll over

    const float hx = 0.5f * c.trackWidth;
    const float hz = 0.5f * c.wheelBase;
    const Vec3 positions[kWheelCount] = {
        Vec3(hx, c.attachHeight, hz), Vec3(-hx, c.attachHeight, hz),
        Vec3(hx, c.attachHeight, -hz), Vec3(-hx, c.attachHeight, -hz)};
    for (int i = 0; i < kWheelCount; ++i) {
        Ref<WheelSettingsWV> w = new WheelSettingsWV;
        w->mPosition = positions[i];
        w->mSuspensionDirection = Vec3(0, -1, 0);
        w->mSteeringAxis = Vec3(0, 1, 0);
        w->mWheelUp = Vec3(0, 1, 0);
        w->mWheelForward = Vec3(0, 0, 1);
        w->mSuspensionMinLength = c.suspensionMinLength;
        w->mSuspensionMaxLength = c.suspensionMaxLength;
        w->mSuspensionPreloadLength = c.suspensionPreload;
        w->mRadius = c.wheelRadius;
        w->mWidth = c.wheelWidth;
        w->mInertia = 0.5f * c.wheelMass * c.wheelRadius * c.wheelRadius;
        w->mAngularDamping = 0.2f;
        const bool front = i < 2;
        w->mMaxSteerAngle = front ? c.maxSteerAngleDeg * kDegToRad : 0.0f;
        m_wheelSettings[i] = w;
        vs.mWheels.push_back(w.GetPtr());
    }

    vs.mAntiRollBars.resize(2);
    vs.mAntiRollBars[0].mLeftWheel = 0;
    vs.mAntiRollBars[0].mRightWheel = 1;
    vs.mAntiRollBars[1].mLeftWheel = 2;
    vs.mAntiRollBars[1].mRightWheel = 3;

    // ---- Engine, gearbox, differentials (four-wheel drive: two axles at 50% each)
    auto* cs = new WheeledVehicleControllerSettings;
    vs.mController = cs;
    cs->mEngine.mMaxTorque = c.tuning.engineMaxTorque;
    cs->mEngine.mMinRPM = c.engineMinRPM;
    cs->mEngine.mMaxRPM = c.engineMaxRPM;
    cs->mEngine.mInertia = 0.6f;
    cs->mTransmission.mMode = ETransmissionMode::Auto;
    cs->mTransmission.mGearRatios.assign(std::begin(c.gearRatios), std::end(c.gearRatios));
    cs->mTransmission.mReverseGearRatios = {c.reverseRatio};
    cs->mTransmission.mShiftUpRPM = c.shiftUpRPM;
    cs->mTransmission.mShiftDownRPM = c.shiftDownRPM;
    cs->mTransmission.mSwitchTime = 0.35f;
    cs->mTransmission.mClutchReleaseTime = 0.25f;
    cs->mTransmission.mSwitchLatency = 0.5f;
    cs->mTransmission.mClutchStrength = c.tuning.clutchStrength;
    cs->mDifferentials.resize(2);
    cs->mDifferentials[0].mLeftWheel = 0;
    cs->mDifferentials[0].mRightWheel = 1;
    cs->mDifferentials[1].mLeftWheel = 2;
    cs->mDifferentials[1].mRightWheel = 3;
    for (auto& d : cs->mDifferentials) {
        d.mDifferentialRatio = c.differentialRatio;
        d.mEngineTorqueRatio = 0.5f;
    }

    m_constraint = new VehicleConstraint(*m_body, vs);
    m_controller = static_cast<WheeledVehicleController*>(m_constraint->GetController());

    // Cylinder shape cast for the wheel (available in Jolt 5.6).
    m_tester = new VehicleCollisionTesterCastCylinder(Layers::Moving, 0.1f);
    m_constraint->SetVehicleCollisionTester(m_tester);

    // Per-wheel grip = tire curve x surface coefficient at the contact point.
    m_constraint->SetCombineFriction(
        [this](uint wheel, float& ioLong, float& ioLat, const Body& body, const SubShapeID& subShape) {
            const SurfaceId id = m_surfaces.lookup(body.GetShape()->GetMaterial(subShape));
            const SurfaceProperties& s = m_surfaces.get(id);
            float slip = 0.0f;
            if (wheel < kWheelCount) {
                slip = curveSlip(wheel);
                ioLong = staticGripFriction(wheel, m_wheelSettings[wheel]->mLongitudinalFriction.GetValue(slip), slip);
                m_contactSurface[wheel] = id;
            }
            ioLong *= s.longitudinalFriction;
            ioLat *= s.lateralFriction;
            if (wheel < kWheelCount) combinedSlip(wheel, slip, ioLong, ioLat);
        });

    world.system().AddConstraint(m_constraint);
    world.system().AddStepListener(m_constraint);

    m_controls.abs = c.tuning.aids.absEnabled;
    m_controls.traction = c.tuning.aids.traction;
    applyTuning(c.tuning);
    m_telemetry.direction = m_direction;
    m_telemetry.controls = m_controls;
}

Vehicle::~Vehicle() {
    m_world.system().RemoveStepListener(m_constraint);
    m_world.system().RemoveConstraint(m_constraint);
    m_constraint = nullptr;
    m_world.bodies().RemoveBody(m_bodyId);
    m_world.bodies().DestroyBody(m_bodyId);
}

void Vehicle::buildTireCurves(const VehicleTuning& t) {
    for (auto& w : m_wheelSettings) {
        w->mLongitudinalFriction.Clear();
        w->mLongitudinalFriction.Reserve(4);
        w->mLongitudinalFriction.AddPoint(0.0f, 0.0f);
        w->mLongitudinalFriction.AddPoint(t.tirePeakSlipRatio, t.tirePeakLongFriction);
        w->mLongitudinalFriction.AddPoint(1.0f, t.tireSlideLongFriction);
        w->mLongitudinalFriction.AddPoint(10.0f, t.tireSlideLongFriction);

        w->mLateralFriction.Clear();
        w->mLateralFriction.Reserve(4);
        w->mLateralFriction.AddPoint(0.0f, 0.0f);
        w->mLateralFriction.AddPoint(t.tirePeakSlipAngleDeg, t.tirePeakLatFriction);
        w->mLateralFriction.AddPoint(30.0f, t.tireSlideLatFriction);
        w->mLateralFriction.AddPoint(90.0f, t.tireSlideLatFriction);
    }
}

// Jolt caps the longitudinal tire impulse at mu(slip) x load, with the slip ratio of the step start. The curve is
// 0 at zero slip, so a wheel the solver has just brought to grip gets a zero cap on the next step, spins up freely
// under drive torque (a 1st-gear wheel gains ~3 m/s of slip in one 1/120 s step), slides, grips again: a period-2
// cycle that reads as wheelspin on every launch, from rest up to ~8 m/s. A tire that has not broken away (slip
// below the peak) may therefore pass up to the peak friction; sliding wheels keep the curve value.
float Vehicle::staticGripFriction(JPH::uint wheel, float curveFriction, float slip) const {
    const VehicleTuning& t = m_config.tuning;
    if (!t.tireStaticGrip) return curveFriction;
    if (slip >= t.tirePeakSlipRatio) return curveFriction;
    return std::max(curveFriction, t.tirePeakLongFriction);
}

// Slip ratio for the tire curve. Jolt divides the slip speed by max(|v|, 0.001 m/s), so near a standstill any wheel
// rotation reads as a full slide (sliding mu) and a hill start can never stay at the peak. Here the denominator has
// a floor of tireSlipSpeedFloor (a common low-speed treatment in tire models); above it this equals Jolt's value.
float Vehicle::curveSlip(JPH::uint wheel) const {
    const VehicleTuning& t = m_config.tuning;
    const auto* w = static_cast<const JPH::WheelWV*>(m_constraint->GetWheel(wheel));
    if (t.tireSlipSpeedFloor <= 0.0f) return w->mLongitudinalSlip;
    JPH::Vec3 v = m_body->GetPointVelocity(w->GetContactPosition()) - w->GetContactPointVelocity();
    const float vLong = v.Dot(w->GetContactLongitudinal());
    const float rolling = w->GetAngularVelocity() * w->GetSettings()->mRadius;
    return std::abs(rolling - vLong) / std::max(std::abs(vLong), t.tireSlipSpeedFloor);
}

// Jolt limits the longitudinal and lateral tire impulses independently, so a locked wheel keeps its full side grip
// and steers as well as a rolling one. Beyond the peak (normalized slip > 1) the two limits are scaled by the share
// of each normalized slip, so the friction force points along the combined slip like a sliding contact patch.
// Blended in above 1-3 m/s: at a crawl Jolt's slip angle and slip ratio are numerically undefined.
void Vehicle::combinedSlip(JPH::uint wheel, float slip, float& ioLong, float& ioLat) const {
    const VehicleTuning& t = m_config.tuning;
    if (!t.tireCombinedSlip) return;
    const auto* w = static_cast<const JPH::WheelWV*>(m_constraint->GetWheel(wheel));
    const float sx = slip / std::max(t.tirePeakSlipRatio, 1e-3f);
    const float sy = w->mLateralSlip * kRadToDeg / std::max(t.tirePeakSlipAngleDeg, 0.1f);
    const float s = std::sqrt(sx * sx + sy * sy);
    if (s <= 1.0f) return;
    JPH::Vec3 v = m_body->GetPointVelocity(w->GetContactPosition()) - w->GetContactPointVelocity();
    v -= w->GetContactNormal() * v.Dot(w->GetContactNormal());
    const float blend = std::clamp((v.Length() - 1.0f) / 2.0f, 0.0f, 1.0f);
    ioLong *= 1.0f + blend * (sx / s - 1.0f);
    ioLat *= 1.0f + blend * (sy / s - 1.0f);
}

void Vehicle::applyTuning(const VehicleTuning& t) {
    m_config.tuning = t;
    // We own the WheelSettings; Jolt keeps pointers to them and reads them every step.
    for (int i = 0; i < kWheelCount; ++i) {
        auto& w = m_wheelSettings[i];
        const bool front = i < 2;
        w->mSuspensionSpring.mMode = JPH::ESpringMode::FrequencyAndDamping;
        w->mSuspensionSpring.mFrequency = front ? t.frontSpringFrequency : t.rearSpringFrequency;
        w->mSuspensionSpring.mDamping = front ? t.frontDamping : t.rearDamping;
        w->mMaxBrakeTorque = t.maxBrakeTorque;
        w->mMaxHandBrakeTorque = front ? 0.0f : t.maxHandBrakeTorque;
    }
    buildTireCurves(t);

    auto& bars = m_constraint->GetAntiRollBars();
    bars[0].mStiffness = t.antiRollFront;
    bars[1].mStiffness = t.antiRollRear;

    m_controller->GetEngine().mMaxTorque = t.engineMaxTorque;
    m_controller->GetTransmission().mClutchStrength = t.clutchStrength;
    applyDifferentials();
}

DiffMode Vehicle::effectiveDiff(int which) const {
    const VehicleTuning& t = m_config.tuning;
    // Locker stages: 1 rear, 2 rear + center, 3 all.
    const int stageNeeded[3] = {3, 1, 2};
    if (m_controls.diffLocks >= stageNeeded[which]) return DiffMode::Locked;
    return which == 0 ? t.frontDiff : which == 1 ? t.rearDiff : t.centerDiff;
}

// Jolt's differentials are torque splitters: open = fixed 50/50, limited slip = the split moves toward the slower
// side as the speed ratio approaches mLimitedSlipRatio, where all torque goes to the slower side. "Locked" uses
// the same model with lockedSlipRatio (1.05): the faster side loses its drive torque at a 5 % speed difference. The
// two sides are not coupled kinematically, so a locked axle neither scrubs in turns nor drags a lifted wheel along.
void Vehicle::applyDifferentials() {
    const VehicleTuning& t = m_config.tuning;
    const float locked = std::max(t.lockedSlipRatio, 1.001f);
    auto ratio = [&](DiffMode m, float lsd) {
        return m == DiffMode::LimitedSlip ? std::max(lsd, 1.001f) : m == DiffMode::Locked ? locked : FLT_MAX;
    };
    auto& diffs = m_controller->GetDifferentials();
    const float finalDrive = m_config.differentialRatio * (m_lowRange ? m_config.lowRangeRatio : 1.0f);
    diffs[0].mLimitedSlipRatio = ratio(effectiveDiff(0), t.limitedSlipRatio);
    diffs[1].mLimitedSlipRatio = ratio(effectiveDiff(1), t.limitedSlipRatio);
    for (auto& d : diffs) d.mDifferentialRatio = finalDrive;
    m_controller->SetDifferentialLimitedSlipRatio(ratio(effectiveDiff(2), t.centerLimitedSlipRatio));
    m_telemetry.frontDiff = effectiveDiff(0);
    m_telemetry.rearDiff = effectiveDiff(1);
    m_telemetry.centerDiff = effectiveDiff(2);
}

void Vehicle::setAidControls(const AidControls& controls) {
    m_controls = controls;
    m_controls.diffLocks = std::clamp(m_controls.diffLocks, 0, AidControls::kDiffLockStages - 1);
    applyDifferentials();
    m_telemetry.controls = m_controls;
}

bool Vehicle::toggleDirection() {
    if (std::abs(m_telemetry.forwardSpeed) > kShiftSpeedLimit) return false;
    m_direction = m_direction == DriveDirection::Forward ? DriveDirection::Reverse : DriveDirection::Forward;
    m_telemetry.direction = m_direction;
    return true;
}

bool Vehicle::toggleLowRange() {
    if (std::abs(m_telemetry.forwardSpeed) > kShiftSpeedLimit) return false;
    m_lowRange = !m_lowRange;
    applyDifferentials();
    m_telemetry.lowRange = m_lowRange;
    return true;
}

void Vehicle::prePhysics(const DriverCommand& cmd, float dt) {
    m_command = cmd;
    // Jolt's automatic keeps the clutch engaged in any gear, and engine rpm never drops below
    // the minimum. At idle the engine drags the wheels through the clutch with torque clutchStrength*dw
    // (up to ~840 N*m at the engine shaft), which after the gear ratios exceeds the brake torque:
    // under full brake the wheels kept turning and the vehicle crept until the gearbox shifted to neutral.
    // While the driver brakes without throttle, the clutch is disengaged (like a slipping torque converter),
    // and near a stop the gearbox is in neutral; with throttle the gearbox is automatic again.
    JPH::VehicleTransmission& tr = m_controller->GetTransmission();
    if (cmd.throttle <= 0.0f && (cmd.brake > 0.05f || cmd.handBrake > 0.05f)) {
        tr.mMode = JPH::ETransmissionMode::Manual;
        const bool nearStop = std::abs(m_telemetry.forwardSpeed) < kShiftSpeedLimit;
        tr.Set(nearStop ? 0 : tr.GetCurrentGear(), 0.0f);
    } else {
        tr.mMode = JPH::ETransmissionMode::Auto;
    }
    // Driver aids, from the wheel state of the previous tick. Jolt brakes each wheel with
    // brake input x WheelSettingsWV::mMaxBrakeTorque, so the per-wheel torque goes into mMaxBrakeTorque (read by
    // Jolt every step) with the brake input at 1 whenever any wheel brakes.
    AidInput in;
    for (int i = 0; i < kWheelCount; ++i) {
        const WheelTelemetry& w = m_telemetry.wheels[i];
        in.wheels[i] = {w.contact, w.groundSpeed, w.rollingSpeed};
    }
    in.throttle = cmd.throttle;
    in.brake = cmd.brake;
    in.maxBrakeTorque = m_config.tuning.maxBrakeTorque;
    in.dt = dt;
    m_aidOut = m_aids.update(m_config.tuning.aids, m_controls, in);
    float maxTorque = 0.0f;
    for (int i = 0; i < kWheelCount; ++i) maxTorque = std::max(maxTorque, m_aidOut.brakeTorque[i]);
    for (int i = 0; i < kWheelCount; ++i) m_wheelSettings[i]->mMaxBrakeTorque = m_aidOut.brakeTorque[i];

    tr.mClutchStrength = clutchStrength(m_aidOut.throttle);
    // Jolt automatic: the sign of the forward input selects forward or reverse.
    const float forward = (m_direction == DriveDirection::Forward ? 1.0f : -1.0f) * m_aidOut.throttle;
    m_controller->SetDriverInput(forward, cmd.steer, maxTorque > 0.0f ? 1.0f : 0.0f, cmd.handBrake);
}

// Jolt clamps the engine at its minimum rpm, so while the clutch slips at launch the clamp, not the engine, would
// supply S * (engine - clutch speed): up to ~840 N*m at the engine shaft at any throttle, 1.6x the maximum engine
// torque. Close to the minimum rpm the clutch therefore passes at most what the engine makes at this throttle (at
// least the creep torque), like a torque converter at stall; 400 rpm above it the full strength applies again and
// the clutch may pull the flywheel down (shifts, engine braking).
float Vehicle::clutchStrength(float throttle) const {
    const VehicleTuning& t = m_config.tuning;
    if (!t.launchClutchLimit) return t.clutchStrength;
    const JPH::VehicleEngine& engine = m_controller->GetEngine();
    const float rpm = engine.GetCurrentRPM();
    const float slip = (rpm - m_controller->GetWheelSpeedAtClutch()) * (2.0f * 3.14159265f / 60.0f); // rad/s
    if (slip <= 1.0f) return t.clutchStrength;
    const float available = std::max(engine.GetTorque(std::clamp(throttle, 0.0f, 1.0f)), t.creepTorque);
    const float limited = std::min(t.clutchStrength, available / slip);
    const float x = std::clamp((rpm - m_config.engineMinRPM) / 400.0f, 0.0f, 1.0f);
    return limited + (t.clutchStrength - limited) * x * x * (3.0f - 2.0f * x);
}

void Vehicle::postPhysics(float dt) {
    using namespace JPH;
    VehicleTelemetry& t = m_telemetry;
    const BodyInterface& bi = m_world.bodies();
    const Quat rot = bi.GetRotation(m_bodyId);
    const Vec3 vel = bi.GetLinearVelocity(m_bodyId);
    t.speed = vel.Length();
    t.forwardSpeed = vel.Dot(rot.RotateAxisZ());
    t.upDot = rot.RotateAxisY().GetY();
    t.engineRPM = m_controller->GetEngine().GetCurrentRPM();
    t.gear = m_controller->GetTransmission().GetCurrentGear();
    t.clutchFriction = m_controller->GetTransmission().GetClutchFriction();
    t.shifting = m_controller->GetTransmission().IsSwitchingGear();
    t.direction = m_direction;
    t.lowRange = m_lowRange;
    t.command = m_command;
    t.controls = m_controls;
    t.absActive = m_aidOut.absActive;
    t.tcActive = m_aidOut.tcActive;
    t.throttleFactor = m_aidOut.throttleFactor;
    t.engineThrottle = m_aidOut.throttle;

    const RMat44 bodyXf = bi.GetWorldTransform(m_bodyId);
    float steerSum = 0.0f;
    for (int i = 0; i < kWheelCount; ++i) {
        const auto* w = static_cast<const WheelWV*>(m_constraint->GetWheel(i));
        const WheelSettings* s = w->GetSettings();
        WheelTelemetry& wt = t.wheels[i];
        wt.contact = w->HasContact();
        wt.bumpStop = w->HasHitHardPoint();
        wt.suspensionLength = w->GetSuspensionLength();
        wt.compression = s->mSuspensionMaxLength - wt.suspensionLength;
        const float travel = s->mSuspensionMaxLength - s->mSuspensionMinLength;
        wt.compressionRatio = travel > 0.0f ? wt.compression / travel : 0.0f;
        // Jolt lambdas are impulses (N*s) accumulated over the step; force = impulse / dt.
        wt.load = dt > 0.0f ? w->GetSuspensionLambda() / dt : 0.0f;
        wt.longitudinalForce = dt > 0.0f ? w->GetLongitudinalLambda() / dt : 0.0f;
        wt.lateralForce = dt > 0.0f ? w->GetLateralLambda() / dt : 0.0f;
        wt.longitudinalSlip = w->mLongitudinalSlip;
        wt.lateralSlipDeg = w->mLateralSlip * kRadToDeg;
        wt.angularVelocity = w->GetAngularVelocity();
        wt.steerAngleDeg = w->GetSteerAngle() * kRadToDeg;
        wt.combinedLongFriction = wt.contact ? w->mCombinedLongitudinalFriction : 0.0f;
        wt.suspensionTop = Vec3(bodyXf * s->mPosition);
        wt.suspensionDir = bodyXf.Multiply3x3(s->mSuspensionDirection);
        wt.rollingSpeed = wt.angularVelocity * s->mRadius;
        wt.brakeTorque = m_aidOut.brakeTorque[i];
        wt.absPressure = m_aidOut.absPressure[i];
        wt.tcBrake = m_aidOut.tcBrake[i];
        {
            // Heading of the wheel: body forward turned by the steer angle about the body up axis.
            const Vec3 heading = rot * (Quat::sRotation(Vec3::sAxisY(), w->GetSteerAngle()) * Vec3::sAxisZ());
            const RVec3 hub = RVec3(wt.suspensionTop + wt.suspensionDir * wt.suspensionLength);
            if (w->HasContact()) {
                const Vec3 rel = Vec3(bi.GetPointVelocity(m_bodyId, w->GetContactPosition())) - w->GetContactPointVelocity();
                wt.groundSpeed = rel.Dot(w->GetContactLongitudinal());
            } else {
                wt.groundSpeed = Vec3(bi.GetPointVelocity(m_bodyId, hub)).Dot(heading);
            }
        }
        if (wt.contact) {
            wt.surface = m_contactSurface[i];
            wt.contactPosition = Vec3(w->GetContactPosition());
            wt.contactNormal = w->GetContactNormal();
            wt.contactLongitudinal = w->GetContactLongitudinal();
            wt.contactLateral = w->GetContactLateral();
            wt.contactSpeed = Vec3(bi.GetPointVelocity(m_bodyId, w->GetContactPosition())).Length();
            wt.slipValid = wt.contactSpeed > 0.5f || std::abs(wt.angularVelocity * s->mRadius) > 0.5f;

            WheelContactSample& c = m_contacts[i];
            const Vec3 n = wt.contactNormal;
            // The cylinder cast may report its contact anywhere across the tread; the patch center is the wheel
            // center projected onto the contact plane.
            const Vec3 hub = wt.suspensionTop + wt.suspensionDir * wt.suspensionLength;
            c.patchCenter = hub - n * (hub - wt.contactPosition).Dot(n);
            c.normal = n;
            c.longitudinal = wt.contactLongitudinal;
            c.lateral = wt.contactLateral;
            Vec3 v = Vec3(bi.GetPointVelocity(m_bodyId, RVec3(c.patchCenter)));
            v -= n * v.Dot(n);
            c.velocity = v;
            c.load = wt.load;
            c.longitudinalForce = wt.longitudinalForce;
            c.lateralForce = wt.lateralForce;
            c.rollingSpeed = wt.angularVelocity * s->mRadius;
            c.longitudinalSlipSpeed = c.rollingSpeed - v.Dot(c.longitudinal);
            c.lateralSlipSpeed = v.Dot(c.lateral);
        } else {
            wt.load = 0.0f;
            wt.longitudinalForce = 0.0f;
            wt.lateralForce = 0.0f;
        }
        WheelContactSample& c = m_contacts[i];
        c.contact = wt.contact;
        c.surface = wt.surface;
        c.width = s->mWidth;
        c.radius = s->mRadius;
        if (i < 2) steerSum += wt.steerAngleDeg;
    }
    t.steerAngleDeg = 0.5f * steerSum;
}

JPH::RMat44 Vehicle::bodyTransform() const {
    return m_world.bodies().GetWorldTransform(m_bodyId);
}

JPH::RMat44 Vehicle::wheelTransform(int wheel) const {
    return m_constraint->GetWheelWorldTransform(JPH::uint(wheel), JPH::Vec3::sAxisX(), JPH::Vec3::sAxisY());
}

} // namespace tf
