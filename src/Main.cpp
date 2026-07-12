#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "App.h"

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{
    App app;
    if (!app.Init())
    {
        MessageBoxA(nullptr, "Failed to initialize (window / OpenGL 3.3 context).",
                    "CaveDroneSim", MB_ICONERROR);
        return 1;
    }
    return app.Run();
}
