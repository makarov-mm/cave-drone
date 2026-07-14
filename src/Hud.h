#pragma once
#include "LineRenderer.h"
#include "Drone.h"
#include "Math.h"
#include <string_view>

// FPV HUD in the style of industrial drone ground stations: crosshair,
// artificial horizon that rolls with the vehicle, compass ribbon, speed
// and altitude readouts, mission status line. Everything is drawn as
// screen-space lines through the shared LineRenderer with an orthographic
// projection; text uses a built-in vector line font, so the HUD needs no
// textures and stays zero-dependency.
namespace Hud
{
    // Text metrics: glyph height = size, advance = 0.78 * size
    [[nodiscard]] float TextWidth(std::string_view text, float size);
    void DrawText(LineRenderer& lines, float x, float y, float size,
                  std::string_view text, const Vec3& color);

    // paneW/paneH in pixels; drone provides attitude and speed
    void Draw(LineRenderer& lines, int paneW, int paneH,
              const Drone& drone, int droneIndex, std::string_view status);
}
