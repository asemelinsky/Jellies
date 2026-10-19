#include "ui.h"

#include <vector>
#include "font_data.h"

namespace ui {
namespace {

Font gSmall, gLarge;
bool gFontsLoaded = false;
std::vector<Rectangle> gPanels, gPanelsPrev;
int gActiveSlider = -1;
int gSliderCounter = 0;

const Color kAccent = {255, 170, 60, 255};
const Color kText = {245, 245, 250, 255};

Font LoadSized(int size) {
    std::vector<int> cps;
    for (int c = 32; c < 127; c++) cps.push_back(c);
    for (int c = 0x400; c < 0x460; c++) cps.push_back(c);
    for (int c : {0x490, 0x491, 0xB0, 0xB2, 0xB7, 0xD7, 0x2013, 0x2014, 0x2019, 0x2026, 0x02BC, 0x2190, 0x2192})
        cps.push_back(c);
    Font f = LoadFontFromMemory(".ttf", kFontData, (int)kFontDataSize, size, cps.data(), (int)cps.size());
    SetTextureFilter(f.texture, TEXTURE_FILTER_BILINEAR);
    return f;
}

const Font& Pick(float size) { return size > 22 ? gLarge : gSmall; }

bool Hover(Rectangle r) { return CheckCollisionPointRec(GetMousePosition(), r); }

}  // namespace

bool Init() {
    gSmall = LoadSized(36);
    gLarge = LoadSized(56);
    gFontsLoaded = gSmall.texture.id > 0 && gLarge.texture.id > 0;
    if (!gFontsLoaded) gSmall = gLarge = GetFontDefault();
    return gFontsLoaded;
}

void Shutdown() {
    if (gFontsLoaded) {
        UnloadFont(gSmall);
        UnloadFont(gLarge);
    }
    gFontsLoaded = false;
}

void BeginFrame() {
    gPanelsPrev.swap(gPanels);
    gPanels.clear();
    gSliderCounter = 0;
    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) gActiveSlider = -1;
}

bool WantsMouse() {
    if (gActiveSlider >= 0) return true;
    Vector2 m = GetMousePosition();
    for (const Rectangle& r : gPanelsPrev)
        if (CheckCollisionPointRec(m, r)) return true;
    return false;
}

void Panel(Rectangle r) {
    gPanels.push_back(r);
    DrawRectangleRounded(r, 0.06f, 8, Color{18, 22, 34, 185});
    DrawRectangleRoundedLinesEx(r, 0.06f, 8, 1.0f, Color{255, 255, 255, 40});
}

Vector2 Measure(const char* text, float size) { return MeasureTextEx(Pick(size), text, size, 0.5f); }

void Label(const char* text, Vector2 pos, float size, Color color) {
    DrawTextEx(Pick(size), text, Vector2{pos.x + 1, pos.y + 1}, size, 0.5f, Color{0, 0, 0, (unsigned char)(color.a / 2)});
    DrawTextEx(Pick(size), text, pos, size, 0.5f, color);
}

bool Button(Rectangle r, const char* label, bool highlighted) {
    bool hover = Hover(r) && gActiveSlider < 0;
    bool pressed = hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
    Color bg = highlighted ? Color{255, 170, 60, 200} : Color{255, 255, 255, (unsigned char)(hover ? 60 : 28)};
    if (hover && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) bg.a = (unsigned char)(bg.a + 30);
    DrawRectangleRounded(r, 0.3f, 8, bg);
    const float size = 18;
    Vector2 m = Measure(label, size);
    Label(label, Vector2{r.x + (r.width - m.x) * 0.5f, r.y + (r.height - m.y) * 0.5f}, size,
          highlighted ? Color{30, 20, 10, 255} : kText);
    return pressed;
}

bool Slider(Rectangle r, const char* label, const char* valueText, float* value, float minValue, float maxValue) {
    const int id = gSliderCounter++;
    const float size = 17;
    Label(label, Vector2{r.x, r.y}, size, kText);
    Vector2 vm = Measure(valueText, size);
    Label(valueText, Vector2{r.x + r.width - vm.x, r.y}, size, kAccent);

    Rectangle track = {r.x, r.y + 26, r.width, 6};
    Rectangle hit = {r.x - 6, r.y + 16, r.width + 12, 26};
    if (gActiveSlider < 0 && Hover(hit) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) gActiveSlider = id;

    bool changed = false;
    if (gActiveSlider == id) {
        float t = (GetMousePosition().x - track.x) / track.width;
        t = t < 0 ? 0 : (t > 1 ? 1 : t);
        float nv = minValue + t * (maxValue - minValue);
        if (nv != *value) {
            *value = nv;
            changed = true;
        }
    }
    float t = (*value - minValue) / (maxValue - minValue);
    DrawRectangleRounded(track, 1.0f, 6, Color{255, 255, 255, 45});
    DrawRectangleRounded(Rectangle{track.x, track.y, track.width * t, track.height}, 1.0f, 6, kAccent);
    bool hot = gActiveSlider == id || Hover(hit);
    DrawCircleV(Vector2{track.x + track.width * t, track.y + 3}, hot ? 9.0f : 7.5f, kText);
    return changed;
}

}  // namespace ui
