#pragma once
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

struct InputState
{
    bool keys[256] = {};
    bool keysPressed[256] = {};   // edge-triggered, cleared each frame
    bool mouseDown = false;
    int  mouseX = 0, mouseY = 0;
    int  mouseDeltaX = 0, mouseDeltaY = 0;
    int  wheelDelta = 0;
};

class Window
{
public:
    Window() = default;
    ~Window();
    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    bool Create(const char* title, int width, int height);
    bool PumpMessages();           // returns false when the app should quit
    void SwapBuffers() const;
    void SetTitle(const char* title) const;
    void EndFrameInput();          // clears per-frame input deltas

    int Width() const { return m_width; }
    int Height() const { return m_height; }
    InputState& Input() { return m_input; }

private:
    HWND m_hwnd = nullptr;
    HDC m_hdc = nullptr;
    HGLRC m_hglrc = nullptr;
    int m_width = 0;
    int m_height = 0;
    InputState m_input;

    static LRESULT CALLBACK WndProcStatic(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);
    LRESULT WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);
    bool CreateGlContext();
};
