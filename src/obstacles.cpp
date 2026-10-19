#include "obstacles.h"

namespace obstacles {
namespace {
bool gEnabled = false;
}

void Init(World&) {}
void Shutdown(World&) {}
void SetEnabled(bool on) { gEnabled = on; }
bool Enabled() { return gEnabled; }
void Update(const World&, float) {}
void AppendSceneObjects(std::vector<SceneObject>&) {}

}  // namespace obstacles
