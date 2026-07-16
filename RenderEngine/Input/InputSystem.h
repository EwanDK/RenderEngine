#pragma once
#include "raylib.h"
#include <unordered_map>
#include <vector>
#include <cstdint>

namespace Input {

// Modifier bitfield
enum Modifier : uint8_t {
    MOD_NONE  = 0,
    MOD_CTRL  = 1 << 0,
    MOD_SHIFT = 1 << 1,
    MOD_ALT   = 1 << 2,
};

// A key combination: modifier flags + a primary key
struct KeyChord {
    uint8_t modifiers = MOD_NONE;
    int key = 0;

    bool operator==(const KeyChord& o) const {
        return modifiers == o.modifiers && key == o.key;
    }
};

struct KeyChordHash {
    size_t operator()(const KeyChord& c) const {
        return std::hash<int>()(c.key) ^ (static_cast<size_t>(c.modifiers) << 16);
    }
};

// Abstract actions — extend this as the engine grows
enum class ActionID : int {
    // Debug visualization
    ToggleClusters,
    ToggleParticles,
    ToggleContacts,
    ToggleSprings,
    Water,
    DebugNudge,

    // Spectator camera movement
    CamForward,
    CamBack,
    CamLeft,
    CamRight,
    CamUp,
    CamDown,

    COUNT  // keep last
};

// How the action was triggered this frame
enum class InputEvent : int {
    Pressed,
    Released,
    Held,
};

struct InputAction {
    ActionID action;
    InputEvent type;
};

class InputSystem {
public:
    InputSystem();

    // Rebind a key chord to an action (replaces any previous binding on that chord)
    void Bind(KeyChord chord, ActionID action);

    // Remove all bindings for a given action
    void Unbind(ActionID action);

    // Remove the binding on a specific chord
    void Unbind(KeyChord chord);

    // Poll raylib input and return all actions that fired this frame.
    // Call once per frame, before processing actions.
    std::vector<InputAction> Poll();

private:
    uint8_t GetCurrentModifiers() const;

    std::unordered_map<KeyChord, ActionID, KeyChordHash> bindings;
};

} // namespace Input
