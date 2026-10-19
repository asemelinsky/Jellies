#include "basket.h"

namespace basket {
namespace {
bool gEnabled = false;
}

void Init(World&) {}
void Shutdown(World&) {}
void SetEnabled(bool on) { gEnabled = on; }
bool Enabled() { return gEnabled; }
void Reset() {}
void Update(World&, float) {}
void AppendSceneObjects(std::vector<SceneObject>&) {}
void DrawHUD() {}

}  // namespace basket
