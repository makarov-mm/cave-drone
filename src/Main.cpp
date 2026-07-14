#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "App.h"

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{
    App app;
    if (auto initialized = app.Init(); !initialized)
    {
        MessageBoxA(nullptr, initialized.error().c_str(), "CaveDroneSim", MB_ICONERROR);
        return 1;
    }
    return app.Run();
}
