// Minimal immediate-mode UI (panel, buttons, sliders) with an embedded
// Cyrillic-capable font.
#pragma once
#include "raylib.h"

namespace ui {

bool Init();
void Shutdown();

// Call once per frame before any widget
void BeginFrame();
// True when the mouse is over a panel (as laid out last frame) or a slider
// is being dragged: the 3D scene must ignore the mouse then.
bool WantsMouse();

void Panel(Rectangle r);
bool Button(Rectangle r, const char* label, bool highlighted = false);
// Returns true while the value changes
bool Slider(Rectangle r, const char* label, const char* valueText, float* value, float minValue, float maxValue);
void Label(const char* text, Vector2 pos, float size, Color color);
Vector2 Measure(const char* text, float size);

}  // namespace ui
