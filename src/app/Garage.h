#pragma once

#include "vehicle/VehicleSetup.h"

#include <algorithm>
#include <string>
#include <vector>

namespace tf {

// Garage model: vehicle and setup selection, kit, fine-tune values, persistence of the setups. Pure C++ (no SDL,
// no ImGui) so it is testable headless; the Application feeds it menu input, pushes config() into the simulation
// when configChanged is returned, and MenuUI draws it.
enum class GarageRowKind { Vehicle, Setup, Kit, Param, Action };
enum class GarageAction { Save, SaveAsNew, Rename, Delete, ResetFactory, Back };

struct GarageRow {
    GarageRowKind kind;
    const SetupParamInfo* param = nullptr; // Param rows
    GarageAction action = GarageAction::Back;
};

struct GarageInput {
    bool up = false, down = false, left = false, right = false;
    bool accept = false, back = false;
    int hover = -1;          // row under a moving mouse
    int click = -1;          // row clicked
    int clickDir = 0;        // -1 / +1: a value arrow was clicked
    float clickFraction = -1.0f; // 0..1: position on a parameter's slider bar (click or drag), -1 = none
    std::string text;        // typed text (rename)
    bool backspace = false;
};

enum class GarageSound { None, Move, Confirm, Back, Adjust, Error };

struct GarageResult {
    bool configChanged = false; // config() differs from what was pushed last
    bool close = false;         // back out of the garage
    GarageSound sound = GarageSound::None;
};

class Garage {
public:
    // setupDir: directory of <vehicle id>.json files; empty = setups are kept in memory only.
    Garage(const std::vector<VehicleDefinition>& catalog, std::string setupDir);

    bool empty() const { return m_catalog.empty(); }
    int vehicleIndex() const { return m_vehicle; }
    const VehicleDefinition& vehicle() const { return m_catalog[std::size_t(m_vehicle)]; }
    // Selects a vehicle and loads its setup file; the setup saved as selected becomes active.
    void selectVehicle(int index);
    bool selectVehicle(const std::string& id);

    const VehicleConfig& config() const { return m_config; } // base -> kit -> working values
    const VehicleConfig& kitBase() const { return m_kitBase; } // base -> kit (reference of the values)
    const VehicleSetup& working() const { return m_working; }
    const SetupFile& setups() const { return m_file; }
    bool factoryActive() const { return m_file.selected.empty(); }
    const std::string& activeName() const { return m_file.selected; } // empty = factory
    bool dirty() const;                // working values differ from the active setup as saved
    const std::vector<std::string>& warnings() const { return m_warnings; }

    const std::vector<GarageRow>& rows() const { return m_rows; }
    int cursor() const { return m_cursor; }
    void setCursor(int row) { m_cursor = std::clamp(row, 0, int(m_rows.size()) - 1); }
    bool renaming() const { return m_renaming; }
    const std::string& renameText() const { return m_renameText; }
    const std::string& message() const { return m_message; }
    int messageSerial() const { return m_messageSerial; } // increments with every new message

    GarageResult handle(const GarageInput& in);

    // Direct operations (scenario commands, tests). Return false (with a message) when refused.
    bool setValue(const std::string& key, float value); // stored units; clamped to the range
    bool stepValue(const std::string& key, int dir);
    bool selectSetup(const std::string& name);           // "" or "Factory" = factory
    bool setKit(const std::string& kitId);               // "" = no kit
    bool save();                                         // into the active setup (a new one when factory)
    bool saveAsNew();
    bool rename(const std::string& name);
    bool deleteActive();
    void resetToFactory();                               // working values back to the definition (not saved)
    bool selectRow(GarageRowKind kind, const std::string& key = {}); // cursor to a row (scenario)

    // Display helpers (MenuUI).
    ParamRange range(const SetupParamInfo& p) const;
    float value(const SetupParamInfo& p) const { return setupValue(p, m_config, m_kitBase); }
    float fraction(const SetupParamInfo& p) const;      // 0..1 position within the range
    std::string valueText(const GarageRow& row) const;
    std::string rangeText(const SetupParamInfo& p) const;
    static const char* actionLabel(GarageAction a);
    const char* groupTitle(int row) const; // section heading above this row, or nullptr

    // The configuration pushed into the simulation last (the application compares against it).
    void markApplied() { m_applied = m_config; }
    bool needsApply() const { return !(m_applied == m_config); }

private:
    const std::vector<VehicleDefinition>& m_catalog;
    std::string m_setupDir;
    int m_vehicle = 0;
    SetupFile m_file;
    VehicleSetup m_working;
    VehicleConfig m_kitBase, m_config, m_applied;
    std::vector<std::string> m_warnings;
    std::vector<GarageRow> m_rows;
    int m_cursor = 0;
    bool m_renaming = false;
    std::string m_renameText;
    std::string m_message;
    int m_messageSerial = 0;

    void buildRows();
    void rebuildConfig();
    void loadActive();
    bool persist();
    std::string uniqueName(const std::string& stem) const;
    std::string filePath() const;
    void say(std::string msg);
    GarageResult activate(int row, int dir);
};

} // namespace tf
