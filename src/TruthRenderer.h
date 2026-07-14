#pragma once
#include "GlLoader.h"
#include "Shader.h"
#include <expected>
#include <string>
#include "World.h"
#include "Math.h"
#include <vector>

// Renders the ground-truth cave for the FPV pane: what an onboard camera
// would actually see, as opposed to the reconstructed map in the other
// pane. The cave is pitch black except for the drone's headlight, so the
// FPV picture is lit rock around the vehicle fading into darkness, exactly
// like footage from industrial inspection drones.
//
// The world is static, so the surface is meshed once per generation into
// 16^3 chunks and drawn with frustum culling.
class TruthRenderer
{
public:
    [[nodiscard]] std::expected<void, std::string> Init();
    void Shutdown();

    // Meshes the whole ground-truth surface. Call after World::Generate.
    void Build(const World& world);

    // lightPos is the drone position (the headlight).
    void Draw(const Mat4& viewProj, const Vec3& lightPos) const;

private:
    struct ChunkMesh
    {
        GLuint vao = 0;
        GLuint vbo = 0;
        int vertexCount = 0;
        Vec3 aabbMin;
        Vec3 aabbMax;
    };

    Shader m_shader;
    std::vector<ChunkMesh> m_meshes;

    void FreeMeshes();
};
