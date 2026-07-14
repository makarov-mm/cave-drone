#include "Window.h"
#include "GlLoader.h"
#include <algorithm>
#include <windowsx.h>

#define WGL_CONTEXT_MAJOR_VERSION_ARB 0x2091
#define WGL_CONTEXT_MINOR_VERSION_ARB 0x2092
#define WGL_CONTEXT_PROFILE_MASK_ARB  0x9126
#define WGL_CONTEXT_CORE_PROFILE_BIT_ARB 0x00000001

typedef HGLRC (WINAPI *PFNWGLCREATECONTEXTATTRIBSARB)(HDC, HGLRC, const int*);
typedef BOOL (WINAPI *PFNWGLSWAPINTERVALEXT)(int);

Window::~Window()
{
    if (m_hglrc != nullptr)
    {
        wglMakeCurrent(nullptr, nullptr);
        wglDeleteContext(m_hglrc);
    }
    if (m_hdc != nullptr && m_hwnd != nullptr)
        ReleaseDC(m_hwnd, m_hdc);
}

LRESULT CALLBACK Window::WndProcStatic(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    Window* self = reinterpret_cast<Window*>(GetWindowLongPtrA(hwnd, GWLP_USERDATA));
    if (msg == WM_NCCREATE)
    {
        CREATESTRUCTA* cs = reinterpret_cast<CREATESTRUCTA*>(lParam);
        SetWindowLongPtrA(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(cs->lpCreateParams));
        self = reinterpret_cast<Window*>(cs->lpCreateParams);
    }
    if (self != nullptr)
        return self->WndProc(hwnd, msg, wParam, lParam);
    return DefWindowProcA(hwnd, msg, wParam, lParam);
}

LRESULT Window::WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg)
    {
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    case WM_SIZE:
        m_width = LOWORD(lParam);
        m_height = HIWORD(lParam);
        return 0;
    case WM_KEYDOWN:
        if (wParam < 256)
        {
            if (!m_input.keys[wParam])
                m_input.keysPressed[wParam] = true;
            m_input.keys[wParam] = true;
        }
        if (wParam == VK_ESCAPE)
            PostQuitMessage(0);
        return 0;
    case WM_KEYUP:
        if (wParam < 256)
            m_input.keys[wParam] = false;
        return 0;
    case WM_LBUTTONDOWN:
        m_input.mouseDown = true;
        SetCapture(hwnd);
        return 0;
    case WM_LBUTTONUP:
        m_input.mouseDown = false;
        ReleaseCapture();
        return 0;
    case WM_MOUSEMOVE:
    {
        int x = GET_X_LPARAM(lParam);
        int y = GET_Y_LPARAM(lParam);
        if (m_input.mouseDown)
        {
            m_input.mouseDeltaX += x - m_input.mouseX;
            m_input.mouseDeltaY += y - m_input.mouseY;
        }
        m_input.mouseX = x;
        m_input.mouseY = y;
        return 0;
    }
    case WM_MOUSEWHEEL:
        m_input.wheelDelta += GET_WHEEL_DELTA_WPARAM(wParam) / WHEEL_DELTA;
        return 0;
    default:
        return DefWindowProcA(hwnd, msg, wParam, lParam);
    }
}

std::expected<void, std::string> Window::Create(const char* title, int width, int height)
{
    m_width = width;
    m_height = height;

    WNDCLASSA wc = {};
    wc.style = CS_OWNDC;
    wc.lpfnWndProc = WndProcStatic;
    wc.hInstance = GetModuleHandleA(nullptr);
    wc.hCursor = LoadCursorA(nullptr, reinterpret_cast<LPCSTR>(IDC_ARROW));
    wc.lpszClassName = "CaveDroneSimWindow";
    if (RegisterClassA(&wc) == 0)
        return std::unexpected("RegisterClass failed");

    RECT rect = {0, 0, width, height};
    AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW, FALSE);

    m_hwnd = CreateWindowA(
        wc.lpszClassName, title, WS_OVERLAPPEDWINDOW | WS_VISIBLE,
        CW_USEDEFAULT, CW_USEDEFAULT,
        rect.right - rect.left, rect.bottom - rect.top,
        nullptr, nullptr, wc.hInstance, this);
    if (m_hwnd == nullptr)
        return std::unexpected("CreateWindow failed");

    m_hdc = GetDC(m_hwnd);
    if (!CreateGlContext())
        return std::unexpected("OpenGL 3.3 context creation failed");
    if (auto loaded = LoadGlFunctions(); !loaded)
        return std::unexpected(loaded.error());
    return {};
}

bool Window::CreateGlContext()
{
    PIXELFORMATDESCRIPTOR pfd = {};
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    pfd.iLayerType = PFD_MAIN_PLANE;

    int format = ChoosePixelFormat(m_hdc, &pfd);
    if (format == 0 || SetPixelFormat(m_hdc, format, &pfd) == FALSE)
        return false;

    // Legacy context first, to obtain wglCreateContextAttribsARB
    HGLRC legacy = wglCreateContext(m_hdc);
    if (legacy == nullptr)
        return false;
    wglMakeCurrent(m_hdc, legacy);

    auto wglCreateContextAttribsARB = reinterpret_cast<PFNWGLCREATECONTEXTATTRIBSARB>(
        reinterpret_cast<void*>(wglGetProcAddress("wglCreateContextAttribsARB")));

    if (wglCreateContextAttribsARB != nullptr)
    {
        const int attribs[] = {
            WGL_CONTEXT_MAJOR_VERSION_ARB, 3,
            WGL_CONTEXT_MINOR_VERSION_ARB, 3,
            WGL_CONTEXT_PROFILE_MASK_ARB, WGL_CONTEXT_CORE_PROFILE_BIT_ARB,
            0};
        HGLRC modern = wglCreateContextAttribsARB(m_hdc, nullptr, attribs);
        if (modern != nullptr)
        {
            wglMakeCurrent(m_hdc, modern);
            wglDeleteContext(legacy);
            m_hglrc = modern;
        }
        else
        {
            m_hglrc = legacy;
        }
    }
    else
    {
        m_hglrc = legacy;
    }

    auto wglSwapIntervalEXT = reinterpret_cast<PFNWGLSWAPINTERVALEXT>(
        reinterpret_cast<void*>(wglGetProcAddress("wglSwapIntervalEXT")));
    if (wglSwapIntervalEXT != nullptr)
        wglSwapIntervalEXT(1);

    return true;
}

bool Window::PumpMessages()
{
    MSG msg;
    while (PeekMessageA(&msg, nullptr, 0, 0, PM_REMOVE))
    {
        if (msg.message == WM_QUIT)
            return false;
        TranslateMessage(&msg);
        DispatchMessageA(&msg);
    }
    return true;
}

void Window::SwapBuffers() const
{
    ::SwapBuffers(m_hdc);
}

void Window::SetTitle(const char* title) const
{
    SetWindowTextA(m_hwnd, title);
}

void Window::EndFrameInput()
{
    m_input.mouseDeltaX = 0;
    m_input.mouseDeltaY = 0;
    m_input.wheelDelta = 0;
    std::ranges::fill(m_input.keysPressed, false);
}
