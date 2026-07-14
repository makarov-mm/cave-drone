#include "Hud.h"
#include <array>
#include <cmath>
#include <format>
#include <numbers>

namespace
{
    // Vector line font on a unit cell (x right, y up). Each glyph is a list
    // of segments (x1, y1, x2, y2), terminated by the segment count.
    struct Glyph
    {
        char character;
        int segmentCount;
        float segments[10][4];
    };

    constexpr std::array kFont = std::to_array<Glyph>({
        {'0', 4, {{0,0,0,1},{0,1,1,1},{1,1,1,0},{1,0,0,0}}},
        {'1', 1, {{0.5f,0,0.5f,1}}},
        {'2', 5, {{0,1,1,1},{1,1,1,0.5f},{1,0.5f,0,0.5f},{0,0.5f,0,0},{0,0,1,0}}},
        {'3', 4, {{0,1,1,1},{1,1,1,0},{0,0.5f,1,0.5f},{0,0,1,0}}},
        {'4', 3, {{0,1,0,0.5f},{0,0.5f,1,0.5f},{1,1,1,0}}},
        {'5', 5, {{1,1,0,1},{0,1,0,0.5f},{0,0.5f,1,0.5f},{1,0.5f,1,0},{1,0,0,0}}},
        {'6', 5, {{1,1,0,1},{0,1,0,0},{0,0,1,0},{1,0,1,0.5f},{1,0.5f,0,0.5f}}},
        {'7', 2, {{0,1,1,1},{1,1,1,0}}},
        {'8', 5, {{0,0,0,1},{0,1,1,1},{1,1,1,0},{1,0,0,0},{0,0.5f,1,0.5f}}},
        {'9', 5, {{1,0,1,1},{1,1,0,1},{0,1,0,0.5f},{0,0.5f,1,0.5f},{1,0,0,0}}},
        {'A', 4, {{0,0,0,1},{1,0,1,1},{0,1,1,1},{0,0.45f,1,0.45f}}},
        {'B', 6, {{0,0,0,1},{0,1,0.9f,1},{0.9f,1,0.9f,0.55f},{0,0.55f,1,0.55f},{1,0.55f,1,0},{1,0,0,0}}},
        {'C', 3, {{1,1,0,1},{0,1,0,0},{0,0,1,0}}},
        {'D', 5, {{0,0,0,1},{0,1,0.7f,1},{0.7f,1,1,0.6f},{1,0.6f,0.7f,0},{0.7f,0,0,0}}},
        {'E', 4, {{1,1,0,1},{0,1,0,0},{0,0,1,0},{0,0.5f,0.8f,0.5f}}},
        {'F', 3, {{1,1,0,1},{0,1,0,0},{0,0.5f,0.8f,0.5f}}},
        {'G', 5, {{1,1,0,1},{0,1,0,0},{0,0,1,0},{1,0,1,0.45f},{1,0.45f,0.5f,0.45f}}},
        {'H', 3, {{0,0,0,1},{1,0,1,1},{0,0.5f,1,0.5f}}},
        {'I', 3, {{0.2f,1,0.8f,1},{0.5f,1,0.5f,0},{0.2f,0,0.8f,0}}},
        {'J', 3, {{1,1,1,0},{1,0,0,0},{0,0,0,0.3f}}},
        {'K', 3, {{0,0,0,1},{0,0.5f,1,1},{0,0.5f,1,0}}},
        {'L', 2, {{0,1,0,0},{0,0,1,0}}},
        {'M', 4, {{0,0,0,1},{0,1,0.5f,0.5f},{0.5f,0.5f,1,1},{1,1,1,0}}},
        {'N', 3, {{0,0,0,1},{0,1,1,0},{1,0,1,1}}},
        {'O', 4, {{0,0,0,1},{0,1,1,1},{1,1,1,0},{1,0,0,0}}},
        {'P', 4, {{0,0,0,1},{0,1,1,1},{1,1,1,0.5f},{1,0.5f,0,0.5f}}},
        {'Q', 5, {{0,0,0,1},{0,1,1,1},{1,1,1,0},{1,0,0,0},{0.6f,0.4f,1.05f,-0.05f}}},
        {'R', 5, {{0,0,0,1},{0,1,1,1},{1,1,1,0.5f},{1,0.5f,0,0.5f},{0.35f,0.5f,1,0}}},
        {'S', 5, {{1,1,0,1},{0,1,0,0.5f},{0,0.5f,1,0.5f},{1,0.5f,1,0},{1,0,0,0}}},
        {'T', 2, {{0,1,1,1},{0.5f,1,0.5f,0}}},
        {'U', 3, {{0,1,0,0},{0,0,1,0},{1,0,1,1}}},
        {'V', 2, {{0,1,0.5f,0},{0.5f,0,1,1}}},
        {'W', 4, {{0,1,0.25f,0},{0.25f,0,0.5f,0.6f},{0.5f,0.6f,0.75f,0},{0.75f,0,1,1}}},
        {'X', 2, {{0,0,1,1},{0,1,1,0}}},
        {'Y', 3, {{0,1,0.5f,0.5f},{1,1,0.5f,0.5f},{0.5f,0.5f,0.5f,0}}},
        {'Z', 3, {{0,1,1,1},{1,1,0,0},{0,0,1,0}}},
        {'.', 1, {{0.4f,0,0.6f,0}}},
        {'-', 1, {{0.2f,0.5f,0.8f,0.5f}}},
        {'/', 1, {{0,0,1,1}}},
        {':', 2, {{0.45f,0.25f,0.55f,0.25f},{0.45f,0.72f,0.55f,0.72f}}},
        {'%', 3, {{0,0,1,1},{0,0.75f,0.25f,1},{0.75f,0,1,0.25f}}},
    });

    const Glyph* FindGlyph(char c)
    {
        if (c >= 'a' && c <= 'z')
            c = static_cast<char>(c - 'a' + 'A');
        for (const Glyph& glyph : kFont)
            if (glyph.character == c)
                return &glyph;
        return nullptr;
    }
}

namespace Hud
{

float TextWidth(std::string_view text, float size)
{
    return static_cast<float>(text.size()) * size * 0.78f;
}

void DrawText(LineRenderer& lines, float x, float y, float size,
              std::string_view text, const Vec3& color)
{
    float cursor = x;
    for (char character : text)
    {
        const Glyph* glyph = FindGlyph(character);
        if (glyph != nullptr)
            for (int i = 0; i < glyph->segmentCount; ++i)
            {
                const float* s = glyph->segments[i];
                lines.AddLine(
                    Vec3{cursor + s[0] * size * 0.62f, y + s[1] * size, 0.0f},
                    Vec3{cursor + s[2] * size * 0.62f, y + s[3] * size, 0.0f},
                    color);
            }
        cursor += size * 0.78f;
    }
}

void Draw(LineRenderer& lines, int paneW, int paneH,
          const Drone& drone, int droneIndex, std::string_view status)
{
    const float w = static_cast<float>(paneW);
    const float h = static_cast<float>(paneH);
    const float cx = w * 0.5f;
    const float cy = h * 0.5f;
    const Vec3 green{0.30f, 1.00f, 0.45f};
    const Vec3 dim{0.18f, 0.60f, 0.28f};

    // Attitude from the physical orientation
    const Quat& q = drone.Orientation();
    Vec3 fwd = q.Rotate(Vec3{0.0f, 0.0f, 1.0f});
    Vec3 right = q.Rotate(Vec3{1.0f, 0.0f, 0.0f});
    Vec3 up = q.Rotate(Vec3{0.0f, 1.0f, 0.0f});
    float roll = std::atan2(right.y, up.y);
    float pitch = std::asin(Clamp(fwd.y, -1.0f, 1.0f));
    float headingDeg = std::atan2(fwd.x, fwd.z) * (180.0f / std::numbers::pi_v<float>);
    if (headingDeg < 0.0f)
        headingDeg += 360.0f;

    // Crosshair with a gap in the middle
    const float ch = 14.0f, gap = 5.0f;
    lines.AddLine({cx - ch, cy, 0}, {cx - gap, cy, 0}, green);
    lines.AddLine({cx + gap, cy, 0}, {cx + ch, cy, 0}, green);
    lines.AddLine({cx, cy - ch, 0}, {cx, cy - gap, 0}, green);
    lines.AddLine({cx, cy + gap, 0}, {cx, cy + ch, 0}, green);

    // Artificial horizon: a bar that stays level with the world, so it
    // counter-rotates against the vehicle roll and shifts with pitch
    {
        float len = w * 0.16f;
        float c = std::cos(-roll), s = std::sin(-roll);
        float offsetY = -pitch * (h * 0.45f);
        Vec3 dir{c * len, s * len, 0.0f};
        Vec3 mid{cx, cy + offsetY, 0.0f};
        lines.AddLine(mid - dir, mid - dir * 0.25f, green);
        lines.AddLine(mid + dir * 0.25f, mid + dir, green);
        // End ticks pointing down
        Vec3 tick{s * 8.0f, -c * 8.0f, 0.0f};
        lines.AddLine(mid - dir, mid - dir + tick, green);
        lines.AddLine(mid + dir, mid + dir + tick, green);
    }

    // Compass ribbon along the top: ticks every 15 degrees inside +-50
    {
        float ribbonY = h - 34.0f;
        float halfSpan = 50.0f;
        float pxPerDeg = (w * 0.42f) / (2.0f * halfSpan);
        lines.AddLine({cx - halfSpan * pxPerDeg, ribbonY, 0},
                      {cx + halfSpan * pxPerDeg, ribbonY, 0}, dim);
        for (int deg = 0; deg < 360; deg += 15)
        {
            float rel = static_cast<float>(deg) - headingDeg;
            while (rel > 180.0f) rel -= 360.0f;
            while (rel < -180.0f) rel += 360.0f;
            if (std::fabs(rel) > halfSpan)
                continue;
            float x = cx + rel * pxPerDeg;
            bool major = deg % 45 == 0;
            lines.AddLine({x, ribbonY, 0}, {x, ribbonY + (major ? 10.0f : 5.0f), 0},
                          major ? green : dim);
            if (deg % 90 == 0)
            {
                constexpr std::string_view names = "NESW"; // 0 / 90 / 180 / 270
                DrawText(lines, x - 5.0f, ribbonY + 13.0f, 13.0f,
                         names.substr(static_cast<size_t>(deg / 90), 1), green);
            }
        }
        // Heading readout under the center caret
        lines.AddLine({cx, ribbonY - 2.0f, 0}, {cx - 5.0f, ribbonY - 9.0f, 0}, green);
        lines.AddLine({cx, ribbonY - 2.0f, 0}, {cx + 5.0f, ribbonY - 9.0f, 0}, green);
        const std::string headingText =
            std::format("{:03d}", static_cast<int>(headingDeg + 0.5f));
        DrawText(lines, cx - TextWidth(headingText, 13.0f) * 0.5f, ribbonY - 26.0f, 13.0f,
                 headingText, green);
    }

    // Speed (left) and altitude (right) blocks
    {
        float y = cy - 8.0f;
        DrawText(lines, 26.0f, y + 26.0f, 11.0f, "SPD M/S", dim);
        DrawText(lines, 26.0f, y, 20.0f, std::format("{:4.1f}", drone.Speed()), green);

        DrawText(lines, w - 118.0f, y + 26.0f, 11.0f, "ALT M", dim);
        DrawText(lines, w - 118.0f, y, 20.0f, std::format("{:5.1f}", drone.Position().y), green);
    }

    // Status line and vehicle id at the bottom
    {
        DrawText(lines, 26.0f, 20.0f, 13.0f, std::format("UAV {}", droneIndex + 1), green);
        DrawText(lines, w - TextWidth(status, 13.0f) - 26.0f, 20.0f, 13.0f, status, green);
    }
}

} // namespace Hud
