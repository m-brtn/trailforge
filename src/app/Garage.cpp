#include "app/Garage.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string_view>

namespace tf {
namespace {

// Setup names: letters, digits, space and - _ . ( ) + (safe in file names and JSON).
bool nameChar(char c) {
    const unsigned char u = static_cast<unsigned char>(c);
    return (u >= '0' && u <= '9') || (u >= 'A' && u <= 'Z') || (u >= 'a' && u <= 'z') ||
           std::string_view(" -_.()+").find(c) != std::string_view::npos;
}

int wrap(int v, int n) { return ((v % n) + n) % n; }

std::string trim(const std::string& s) {
    const auto a = s.find_first_not_of(" \t");
    if (a == std::string::npos) return {};
    const auto b = s.find_last_not_of(" \t");
    return s.substr(a, b - a + 1);
}

} // namespace

Garage::Garage(const std::vector<VehicleDefinition>& catalog, std::string setupDir)
    : m_catalog(catalog), m_setupDir(std::move(setupDir)) {
    buildRows();
    if (!m_catalog.empty()) selectVehicle(0);
}

void Garage::buildRows() {
    m_rows.clear();
    m_rows.push_back({GarageRowKind::Vehicle});
    m_rows.push_back({GarageRowKind::Setup});
    m_rows.push_back({GarageRowKind::Kit});
    for (const SetupParamInfo& p : setupParams()) m_rows.push_back({GarageRowKind::Param, &p});
    for (GarageAction a : {GarageAction::Save, GarageAction::SaveAsNew, GarageAction::Rename, GarageAction::Delete,
                           GarageAction::ResetFactory, GarageAction::Back})
        m_rows.push_back({GarageRowKind::Action, nullptr, a});
}

std::string Garage::filePath() const {
    if (m_setupDir.empty() || m_catalog.empty()) return {};
    return (std::filesystem::path(m_setupDir) / (vehicle().id + ".json")).string();
}

void Garage::say(std::string msg) {
    m_message = std::move(msg);
    ++m_messageSerial;
}

void Garage::selectVehicle(int index) {
    if (m_catalog.empty()) return;
    m_vehicle = std::clamp(index, 0, int(m_catalog.size()) - 1);
    m_file = SetupFile{};
    m_file.vehicleId = vehicle().id;
    const std::string path = filePath();
    std::error_code ec;
    if (!path.empty() && std::filesystem::exists(path, ec)) {
        std::string error;
        SetupFile loaded;
        if (loadSetupFile(path, loaded, &error) && loaded.vehicleId == vehicle().id) {
            m_file = loaded;
        } else {
            say("Setups of " + vehicle().name + " not loaded: " + (error.empty() ? "wrong vehicle id" : error));
            std::printf("Garage: %s\n", m_message.c_str());
        }
    }
    if (!m_file.selected.empty() && !m_file.find(m_file.selected)) m_file.selected.clear();
    loadActive();
}

bool Garage::selectVehicle(const std::string& id) {
    for (std::size_t i = 0; i < m_catalog.size(); ++i) {
        if (m_catalog[i].id != id) continue;
        selectVehicle(int(i));
        return true;
    }
    say("No vehicle '" + id + "'");
    return false;
}

void Garage::loadActive() {
    if (const VehicleSetup* s = m_file.find(m_file.selected); s && !m_file.selected.empty()) {
        m_working = *s;
    } else {
        m_working = VehicleSetup{};
    }
    rebuildConfig();
}

void Garage::rebuildConfig() {
    if (m_catalog.empty()) return;
    const VehicleDefinition& def = vehicle();
    m_warnings.clear();
    std::vector<std::string> errors;
    if (!configWithKit(def, m_working.kit, m_kitBase, errors)) m_kitBase = def.config;
    if (!buildSetupConfig(def, m_working, m_config, m_warnings)) m_config = m_kitBase;
    for (const std::string& w : m_warnings) std::printf("Garage: %s\n", w.c_str());
}

bool Garage::persist() {
    const std::string path = filePath();
    if (path.empty()) return true;
    std::string error;
    if (saveSetupFile(path, m_file, &error)) return true;
    say("Could not save setups: " + error);
    std::printf("Garage: %s\n", m_message.c_str());
    return false;
}

bool Garage::dirty() const {
    const VehicleSetup* s = factoryActive() ? nullptr : m_file.find(m_file.selected);
    if (!s) return !m_working.values.empty() || !m_working.kit.empty();
    return s->values != m_working.values || s->kit != m_working.kit;
}

ParamRange Garage::range(const SetupParamInfo& p) const {
    return setupParamRange(p, vehicle(), vehicle().findKit(m_working.kit), m_kitBase);
}

float Garage::fraction(const SetupParamInfo& p) const {
    const ParamRange r = range(p);
    if (r.max <= r.min) return 0.5f;
    return std::clamp((value(p) - r.min) / (r.max - r.min), 0.0f, 1.0f);
}

bool Garage::setValue(const std::string& key, float v) {
    const SetupParamInfo* p = findSetupParam(key);
    if (!p || m_catalog.empty()) {
        say("Unknown setup parameter '" + key + "'");
        return false;
    }
    const ParamRange r = range(*p);
    v = std::clamp(v, r.min, r.max);
    if (p->kind == SetupParamKind::Choice) v = std::round(v);
    const float base = p->get(m_kitBase, m_kitBase);
    const float before = value(*p);
    if (std::abs(v - base) <= 1e-6f * std::max(1.0f, std::abs(base))) m_working.values.erase(key);
    else m_working.values[key] = v;
    rebuildConfig();
    return value(*p) != before;
}

bool Garage::stepValue(const std::string& key, int dir) {
    const SetupParamInfo* p = findSetupParam(key);
    if (!p || m_catalog.empty()) {
        say("Unknown setup parameter '" + key + "'");
        return false;
    }
    const float cur = value(*p);
    if (p->kind == SetupParamKind::Choice) {
        const int n = int(p->choices.size());
        return setValue(key, float(wrap(int(std::lround(cur)) + (dir == 0 ? 1 : dir), n)));
    }
    if (dir == 0) return false;
    // Snap to the step grid so repeated steps land on round numbers.
    float next = std::round((cur + p->step * float(dir)) / p->step) * p->step;
    const ParamRange r = range(*p);
    if (dir > 0 && cur < r.max && next > r.max) next = r.max;
    if (dir < 0 && cur > r.min && next < r.min) next = r.min;
    return setValue(key, next);
}

bool Garage::selectSetup(const std::string& name) {
    const bool discarded = dirty();
    if (name.empty() || name == "Factory") {
        m_file.selected.clear();
    } else if (m_file.find(name)) {
        m_file.selected = name;
    } else {
        say("No setup named '" + name + "'");
        return false;
    }
    loadActive();
    persist();
    say((factoryActive() ? std::string("Factory setup") : "Setup '" + name + "'") +
        (discarded ? " (unsaved changes discarded)" : ""));
    return true;
}

bool Garage::setKit(const std::string& kitId) {
    if (!kitId.empty() && !vehicle().findKit(kitId)) {
        say("No kit '" + kitId + "' for " + vehicle().name);
        return false;
    }
    m_working.kit = kitId;
    rebuildConfig();
    // Values outside the new kit's ranges are clamped by buildSetupConfig; keep the stored ones consistent.
    for (auto it = m_working.values.begin(); it != m_working.values.end();) {
        const SetupParamInfo* p = findSetupParam(it->first);
        if (!p) {
            ++it;
            continue;
        }
        const ParamRange r = range(*p);
        it->second = std::clamp(it->second, r.min, r.max);
        ++it;
    }
    rebuildConfig();
    return true;
}

std::string Garage::uniqueName(const std::string& stem) const {
    for (int i = 1;; ++i) {
        const std::string n = stem + " " + std::to_string(i);
        if (!m_file.find(n)) return n;
    }
}

bool Garage::save() {
    if (factoryActive()) return saveAsNew();
    VehicleSetup* s = m_file.find(m_file.selected);
    s->values = m_working.values;
    s->kit = m_working.kit;
    if (!persist()) return false;
    say("Saved '" + s->name + "'");
    return true;
}

bool Garage::saveAsNew() {
    VehicleSetup s = m_working;
    s.name = uniqueName("Setup");
    m_file.setups.push_back(s);
    m_file.selected = s.name;
    m_working.name = s.name;
    if (!persist()) return false;
    say("Saved as '" + s.name + "'");
    return true;
}

std::string Garage::nameProblem(const std::string& raw) const {
    const std::string name = trim(raw);
    if (name.empty()) return "THE NAME IS EMPTY";
    if (name.size() > kMaxNameLength) return "AT MOST " + std::to_string(kMaxNameLength) + " CHARACTERS";
    for (char c : name)
        if (!nameChar(c)) return "ONLY LETTERS, DIGITS, SPACE AND - _ . ( ) +";
    std::string lower = name;
    for (char& c : lower) c = char(std::tolower(static_cast<unsigned char>(c)));
    if (lower == "factory") return "'FACTORY' IS RESERVED";
    if (name != m_file.selected && m_file.find(name)) return "A SETUP NAMED '" + name + "' ALREADY EXISTS";
    return {};
}

bool Garage::rename(const std::string& raw) {
    const std::string name = trim(raw);
    if (factoryActive()) {
        say("The factory setup cannot be renamed");
        return false;
    }
    if (const std::string problem = nameProblem(name); !problem.empty()) {
        say(problem);
        return false;
    }
    m_file.find(m_file.selected)->name = name;
    m_file.selected = name;
    m_working.name = name;
    if (!persist()) return false;
    say("Renamed to '" + name + "'");
    return true;
}

bool Garage::deleteActive() {
    if (factoryActive()) {
        say("The factory setup cannot be deleted");
        return false;
    }
    const std::string name = m_file.selected;
    std::erase_if(m_file.setups, [&](const VehicleSetup& s) { return s.name == name; });
    m_file.selected.clear();
    loadActive();
    persist();
    say("Deleted '" + name + "', factory setup active");
    return true;
}

void Garage::resetToFactory() {
    m_working.values.clear();
    m_working.kit.clear();
    rebuildConfig();
    say(factoryActive() ? "Factory values" : "Factory values (save to keep them in '" + m_file.selected + "')");
}

bool Garage::selectRow(GarageRowKind kind, const std::string& key) {
    for (std::size_t i = 0; i < m_rows.size(); ++i) {
        const GarageRow& r = m_rows[i];
        if (r.kind != kind) continue;
        if (kind == GarageRowKind::Param && key != r.param->key) continue;
        if (kind == GarageRowKind::Action && key != actionLabel(r.action)) continue;
        m_cursor = int(i);
        return true;
    }
    return false;
}

const char* Garage::actionLabel(GarageAction a) {
    switch (a) {
    case GarageAction::Save: return "SAVE SETUP";
    case GarageAction::SaveAsNew: return "SAVE AS NEW";
    case GarageAction::Rename: return "RENAME";
    case GarageAction::Delete: return "DELETE";
    case GarageAction::ResetFactory: return "RESET TO FACTORY";
    case GarageAction::Back: return "BACK";
    }
    return "";
}

const char* Garage::groupTitle(int row) const {
    if (row < 0 || row >= int(m_rows.size())) return nullptr;
    const GarageRow& r = m_rows[std::size_t(row)];
    if (row == 0) return "VEHICLE";
    if (r.kind == GarageRowKind::Param) {
        const GarageRow& prev = m_rows[std::size_t(row - 1)];
        if (prev.kind != GarageRowKind::Param || std::string(prev.param->group) != r.param->group) return r.param->group;
    }
    if (r.kind == GarageRowKind::Action && m_rows[std::size_t(row - 1)].kind != GarageRowKind::Action) return "SETUPS";
    return nullptr;
}

std::string Garage::valueText(const GarageRow& row) const {
    switch (row.kind) {
    case GarageRowKind::Vehicle: return vehicle().name;
    case GarageRowKind::Setup:
        return (factoryActive() ? std::string("FACTORY") : m_file.selected) + (dirty() ? " *" : "");
    case GarageRowKind::Kit:
        if (vehicle().kits.empty()) return "NONE (NO KITS YET)";
        if (const VehicleKit* k = vehicle().findKit(m_working.kit)) return k->name;
        return "NONE";
    case GarageRowKind::Param: {
        const SetupParamInfo& p = *row.param;
        const float v = value(p);
        if (p.kind == SetupParamKind::Choice) {
            const int i = std::clamp(int(std::lround(v)), 0, int(p.choices.size()) - 1);
            return p.choices[std::size_t(i)];
        }
        char buf[48];
        const float shown = v * p.displayScale;
        std::snprintf(buf, sizeof(buf), "%.*f%s%s", p.decimals, double(shown == 0.0f ? 0.0f : shown), p.unit[0] ? " " : "",
                      p.unit);
        return buf;
    }
    case GarageRowKind::Action: return {};
    }
    return {};
}

namespace {
// Suspension rows: axle index and whether the row is a spring (frequency) or a damping ratio; -1 = other row.
int suspensionAxle(const SetupParamInfo& p, bool& spring) {
    const std::string k = p.key;
    spring = k.ends_with(".frequency");
    if (!spring && !k.ends_with(".compression") && !k.ends_with(".rebound")) return -1;
    if (k.starts_with("suspension.front.")) return kFrontAxle;
    if (k.starts_with("suspension.rear.")) return kRearAxle;
    return -1;
}
} // namespace

std::string Garage::valueNote(const GarageRow& row) const {
    if (row.kind != GarageRowKind::Param) return {};
    bool spring = false;
    const int a = suspensionAxle(*row.param, spring);
    if (a < 0) return {};
    const float m = sprungCornerMass(m_config, a);
    const float f = m_config.tuning.axles[std::size_t(a)].springFrequency;
    char buf[48];
    if (spring) std::snprintf(buf, sizeof(buf), "%.0f N/mm", double(wheelRateFromFrequency(f, m) / 1000.0f));
    else std::snprintf(buf, sizeof(buf), "%.1f kNs/m", double(value(*row.param) * criticalDamping(f, m) / 1000.0f));
    return buf;
}

std::string Garage::helpText(const GarageRow& row) const {
    if (row.kind != GarageRowKind::Param) return {};
    bool spring = false;
    const int a = suspensionAxle(*row.param, spring);
    if (a < 0) return {};
    char buf[200];
    if (spring)
        std::snprintf(buf, sizeof(buf),
                      "NATURAL FREQUENCY: ~1-1.5 HZ SOFT OFF-ROAD, 2-3 HZ STIFF ROAD / RACE. N/MM: WHEEL RATE PER "
                      "CORNER FOR %.0f KG SPRUNG",
                      double(sprungCornerMass(m_config, a)));
    else
        std::snprintf(buf, sizeof(buf), "DAMPING RATIO (1 = CRITICAL): 0.2-0.4 COMFORT, 0.5-0.7 SPORTY. KNS/M: "
                                        "EQUIVALENT DAMPER COEFFICIENT PER CORNER");
    return buf;
}

std::string Garage::rangeText(const SetupParamInfo& p) const {
    if (p.kind == SetupParamKind::Choice) return {};
    const ParamRange r = range(p);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f .. %.*f", p.decimals, double(r.min * p.displayScale), p.decimals,
                  double(r.max * p.displayScale));
    return buf;
}

GarageResult Garage::activate(int row, int dir) {
    GarageResult res;
    const GarageRow& r = m_rows[std::size_t(row)];
    switch (r.kind) {
    case GarageRowKind::Vehicle:
        if (m_catalog.size() < 2) {
            if (dir != 0) say("Only one vehicle so far");
            break;
        }
        selectVehicle(wrap(m_vehicle + (dir == 0 ? 1 : dir), int(m_catalog.size())));
        say(vehicle().name);
        res.sound = GarageSound::Adjust;
        break;
    case GarageRowKind::Setup: {
        std::vector<std::string> names = {""};
        for (const VehicleSetup& s : m_file.setups) names.push_back(s.name);
        if (names.size() < 2) {
            say("No saved setups yet: change values, then SAVE SETUP");
            res.sound = GarageSound::Error;
            break;
        }
        const int cur = int(std::find(names.begin(), names.end(), m_file.selected) - names.begin());
        selectSetup(names[std::size_t(wrap(cur + (dir == 0 ? 1 : dir), int(names.size())))]);
        res.sound = GarageSound::Adjust;
        break;
    }
    case GarageRowKind::Kit: {
        const auto& kits = vehicle().kits;
        if (kits.empty()) {
            say("No kits for " + vehicle().name + " yet");
            res.sound = GarageSound::Error;
            break;
        }
        int cur = 0;
        for (std::size_t i = 0; i < kits.size(); ++i)
            if (kits[i].id == m_working.kit) cur = int(i) + 1;
        const int next = wrap(cur + (dir == 0 ? 1 : dir), int(kits.size()) + 1);
        setKit(next == 0 ? std::string() : kits[std::size_t(next - 1)].id);
        res.sound = GarageSound::Adjust;
        break;
    }
    case GarageRowKind::Param:
        if (stepValue(r.param->key, dir)) res.sound = GarageSound::Adjust;
        break;
    case GarageRowKind::Action:
        if (dir != 0) break;
        switch (r.action) {
        case GarageAction::Save: res.sound = save() ? GarageSound::Confirm : GarageSound::Error; break;
        case GarageAction::SaveAsNew: res.sound = saveAsNew() ? GarageSound::Confirm : GarageSound::Error; break;
        case GarageAction::Rename:
            if (factoryActive()) {
                say("The factory setup cannot be renamed: SAVE SETUP first");
                res.sound = GarageSound::Error;
                break;
            }
            openModal(GarageModal::Rename, 0);
            res.sound = GarageSound::Confirm;
            break;
        case GarageAction::Delete:
            if (factoryActive()) {
                say("The factory setup cannot be deleted");
                res.sound = GarageSound::Error;
                break;
            }
            openModal(GarageModal::ConfirmDelete, 1); // destructive: Cancel has the focus
            res.sound = GarageSound::Confirm;
            break;
        case GarageAction::ResetFactory:
            openModal(GarageModal::ConfirmReset, 1);
            res.sound = GarageSound::Confirm;
            break;
        case GarageAction::Back:
            res.close = true;
            res.sound = GarageSound::Back;
            break;
        }
        break;
    }
    return res;
}

void Garage::openModal(GarageModal m, int focus) {
    m_modal = m;
    m_modalFocus = focus;
    m_modalError.clear();
    m_renameText = m == GarageModal::Rename ? m_file.selected : std::string();
    m_renameSelected = m == GarageModal::Rename;
}

GarageResult Garage::handleModal(const GarageInput& in) {
    GarageResult res;
    if (in.modalHover == 0 || in.modalHover == 1) m_modalFocus = in.modalHover;
    if (m_modal == GarageModal::Rename) {
        // Typed text: the preselected name is replaced by the first character (or cleared by Backspace); characters
        // outside the safe set and anything past the length limit are refused with a visible reason.
        bool edited = false;
        for (char c : in.text) {
            if (m_renameSelected) {
                m_renameText.clear();
                m_renameSelected = false;
            }
            if (!nameChar(c)) {
                m_modalError = "ONLY LETTERS, DIGITS, SPACE AND - _ . ( ) +";
                res.sound = GarageSound::Error;
                continue;
            }
            if (m_renameText.size() >= kMaxNameLength) {
                m_modalError = "AT MOST " + std::to_string(kMaxNameLength) + " CHARACTERS";
                res.sound = GarageSound::Error;
                continue;
            }
            m_renameText += c;
            edited = true;
        }
        if (in.backspace) {
            if (m_renameSelected) m_renameText.clear();
            else if (!m_renameText.empty()) m_renameText.pop_back();
            m_renameSelected = false;
            edited = true;
        }
        if (in.left || in.right) m_renameSelected = false; // caret to the end, keep the text
        if (edited && res.sound != GarageSound::Error) {
            m_modalError = nameProblem(m_renameText);
            if (m_modalError.rfind("A SETUP", 0) != 0 && m_modalError.rfind("ONLY", 0) != 0) m_modalError.clear();
        }
    } else if (in.left || in.right || in.up || in.down) {
        m_modalFocus = 1 - m_modalFocus;
        res.sound = GarageSound::Move;
    }

    int button = in.modalClick;
    if (button < 0 && in.accept) button = m_modal == GarageModal::Rename ? 0 : m_modalFocus;
    if (button < 0 && in.back && !in.backspace) button = 1;
    if (button == 1) {
        say(m_modal == GarageModal::Rename ? "Rename cancelled" : "Cancelled");
        m_modal = GarageModal::None;
        res.sound = GarageSound::Back;
    } else if (button == 0) {
        switch (m_modal) {
        case GarageModal::Rename:
            if (std::string problem = nameProblem(m_renameText); !problem.empty()) {
                m_modalError = problem;
                res.sound = GarageSound::Error;
                return res;
            }
            if (!rename(m_renameText)) {
                m_modalError = "COULD NOT SAVE THE SETUP FILE";
                res.sound = GarageSound::Error;
                return res;
            }
            break;
        case GarageModal::ConfirmDelete: deleteActive(); break;
        case GarageModal::ConfirmReset: resetToFactory(); break;
        case GarageModal::None: break;
        }
        m_modal = GarageModal::None;
        res.sound = GarageSound::Confirm;
    }
    return res;
}

GarageResult Garage::handle(const GarageInput& in) {
    GarageResult res;
    if (m_catalog.empty()) {
        res.close = in.back;
        return res;
    }
    if (m_modal != GarageModal::None) {
        res = handleModal(in);
        res.configChanged = needsApply();
        return res;
    }

    const int n = int(m_rows.size());
    if (in.hover >= 0 && in.hover < n && in.hover != m_cursor) {
        m_cursor = in.hover;
        res.sound = GarageSound::Move;
    }
    if (in.up || in.down) {
        m_cursor = wrap(m_cursor + (in.down ? 1 : -1), n);
        res.sound = GarageSound::Move;
    }
    if (in.click >= 0 && in.click < n) {
        m_cursor = in.click;
        const GarageRow& r = m_rows[std::size_t(m_cursor)];
        if (r.kind == GarageRowKind::Param && r.param->kind == SetupParamKind::Number && in.clickFraction >= 0.0f) {
            const ParamRange rg = range(*r.param);
            const float raw = rg.min + (rg.max - rg.min) * std::clamp(in.clickFraction, 0.0f, 1.0f);
            const float snapped = std::round(raw / r.param->step) * r.param->step;
            if (setValue(r.param->key, snapped)) res.sound = GarageSound::Adjust;
        } else {
            const GarageResult a = activate(m_cursor, in.clickDir);
            res.close = a.close;
            if (a.sound != GarageSound::None) res.sound = a.sound;
        }
    } else if (in.left || in.right) {
        const GarageResult a = activate(m_cursor, in.right ? 1 : -1);
        if (a.sound != GarageSound::None) res.sound = a.sound;
    } else if (in.accept) {
        const GarageResult a = activate(m_cursor, 0);
        res.close = a.close;
        if (a.sound != GarageSound::None) res.sound = a.sound;
    } else if (in.back) {
        res.close = true;
        res.sound = GarageSound::Back;
    }
    res.configChanged = needsApply();
    return res;
}

} // namespace tf
