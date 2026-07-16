#include "InputSystem.h"
#include <algorithm>

namespace Input {

InputSystem::InputSystem() {
    // Default bindings — debug visualization
    Bind({ MOD_NONE, KEY_F1 }, ActionID::ToggleClusters);
    Bind({ MOD_NONE, KEY_F2 }, ActionID::ToggleParticles);
    Bind({ MOD_NONE, KEY_F3 }, ActionID::ToggleContacts);
    Bind({ MOD_NONE, KEY_F4 }, ActionID::ToggleSprings);
    Bind({ MOD_NONE, KEY_F5 }, ActionID::Water);
    Bind({ MOD_NONE, KEY_F9 }, ActionID::DebugNudge);

    // Default bindings — spectator camera (physical QWERTY positions)
    // On AZERTY: W=Z, A=Q, S=S, D=D, Q=A, E=E
    Bind({ MOD_NONE, KEY_W }, ActionID::CamForward);
    Bind({ MOD_NONE, KEY_S }, ActionID::CamBack);
    Bind({ MOD_NONE, KEY_A }, ActionID::CamLeft);
    Bind({ MOD_NONE, KEY_D }, ActionID::CamRight);
    Bind({ MOD_NONE, KEY_Q }, ActionID::CamUp);
    Bind({ MOD_NONE, KEY_E }, ActionID::CamDown);
}

void InputSystem::Bind(KeyChord chord, ActionID action) {
    bindings[chord] = action;
}

void InputSystem::Unbind(ActionID action) {
    for (auto it = bindings.begin(); it != bindings.end(); ) {
        if (it->second == action)
            it = bindings.erase(it);
        else
            ++it;
    }
}

void InputSystem::Unbind(KeyChord chord) {
    bindings.erase(chord);
}

uint8_t InputSystem::GetCurrentModifiers() const {
    uint8_t mods = MOD_NONE;
    if (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL))
        mods |= MOD_CTRL;
    if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT))
        mods |= MOD_SHIFT;
    if (IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT))
        mods |= MOD_ALT;
    return mods;
}

std::vector<InputAction> InputSystem::Poll() {
    std::vector<InputAction> actions;
    uint8_t mods = GetCurrentModifiers();

    // Drain the pressed-key queue (GetKeyPressed returns 0 when empty)
    for (int key = GetKeyPressed(); key != 0; key = GetKeyPressed()) {
        KeyChord chord{ mods, key };
        auto it = bindings.find(chord);
        if (it != bindings.end()) {
            actions.push_back({ it->second, InputEvent::Pressed });
        }
    }

    // Check held and released state for all bound keys
    for (const auto& [chord, action] : bindings) {
        // Skip if modifiers don't match current state
        if (chord.modifiers != mods)
            continue;

        if (IsKeyReleased(chord.key)) {
            actions.push_back({ action, InputEvent::Released });
        } else if (IsKeyDown(chord.key)) {
            actions.push_back({ action, InputEvent::Held });
        }
    }

    return actions;
}

} // namespace Input
