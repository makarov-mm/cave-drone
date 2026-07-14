#include "GlLoader.h"

GLuint (APIENTRY *glCreateShader)(GLenum) = nullptr;
void   (APIENTRY *glShaderSource)(GLuint, GLsizei, const GLchar* const*, const GLint*) = nullptr;
void   (APIENTRY *glCompileShader)(GLuint) = nullptr;
void   (APIENTRY *glGetShaderiv)(GLuint, GLenum, GLint*) = nullptr;
void   (APIENTRY *glGetShaderInfoLog)(GLuint, GLsizei, GLsizei*, GLchar*) = nullptr;
void   (APIENTRY *glDeleteShader)(GLuint) = nullptr;
GLuint (APIENTRY *glCreateProgram)(void) = nullptr;
void   (APIENTRY *glAttachShader)(GLuint, GLuint) = nullptr;
void   (APIENTRY *glLinkProgram)(GLuint) = nullptr;
void   (APIENTRY *glGetProgramiv)(GLuint, GLenum, GLint*) = nullptr;
void   (APIENTRY *glGetProgramInfoLog)(GLuint, GLsizei, GLsizei*, GLchar*) = nullptr;
void   (APIENTRY *glUseProgram)(GLuint) = nullptr;
void   (APIENTRY *glDeleteProgram)(GLuint) = nullptr;
GLint  (APIENTRY *glGetUniformLocation)(GLuint, const GLchar*) = nullptr;
void   (APIENTRY *glUniformMatrix4fv)(GLint, GLsizei, GLboolean, const GLfloat*) = nullptr;
void   (APIENTRY *glUniform1f)(GLint, GLfloat) = nullptr;
void   (APIENTRY *glUniform3f)(GLint, GLfloat, GLfloat, GLfloat) = nullptr;
void   (APIENTRY *glGenVertexArrays)(GLsizei, GLuint*) = nullptr;
void   (APIENTRY *glBindVertexArray)(GLuint) = nullptr;
void   (APIENTRY *glDeleteVertexArrays)(GLsizei, const GLuint*) = nullptr;
void   (APIENTRY *glGenBuffers)(GLsizei, GLuint*) = nullptr;
void   (APIENTRY *glBindBuffer)(GLenum, GLuint) = nullptr;
void   (APIENTRY *glBufferData)(GLenum, GLsizeiptr, const void*, GLenum) = nullptr;
void   (APIENTRY *glBufferSubData)(GLenum, GLintptr, GLsizeiptr, const void*) = nullptr;
void   (APIENTRY *glDeleteBuffers)(GLsizei, const GLuint*) = nullptr;
void   (APIENTRY *glEnableVertexAttribArray)(GLuint) = nullptr;
void   (APIENTRY *glVertexAttribPointer)(GLuint, GLint, GLenum, GLboolean, GLsizei, const void*) = nullptr;

namespace
{
    void* GetProc(const char* name)
    {
        void* p = reinterpret_cast<void*>(wglGetProcAddress(name));
        if (p == nullptr || p == reinterpret_cast<void*>(1) ||
            p == reinterpret_cast<void*>(2) || p == reinterpret_cast<void*>(3) ||
            p == reinterpret_cast<void*>(-1))
        {
            HMODULE module = GetModuleHandleA("opengl32.dll");
            p = module ? reinterpret_cast<void*>(GetProcAddress(module, name)) : nullptr;
        }
        return p;
    }
}

#define LOAD_GL(name) \
    name = reinterpret_cast<decltype(name)>(GetProc(#name)); \
    if (name == nullptr) \
        return std::unexpected(std::string("missing OpenGL entry point: ") + #name);

std::expected<void, std::string> LoadGlFunctions()
{
    LOAD_GL(glCreateShader)
    LOAD_GL(glShaderSource)
    LOAD_GL(glCompileShader)
    LOAD_GL(glGetShaderiv)
    LOAD_GL(glGetShaderInfoLog)
    LOAD_GL(glDeleteShader)
    LOAD_GL(glCreateProgram)
    LOAD_GL(glAttachShader)
    LOAD_GL(glLinkProgram)
    LOAD_GL(glGetProgramiv)
    LOAD_GL(glGetProgramInfoLog)
    LOAD_GL(glUseProgram)
    LOAD_GL(glDeleteProgram)
    LOAD_GL(glGetUniformLocation)
    LOAD_GL(glUniformMatrix4fv)
    LOAD_GL(glUniform1f)
    LOAD_GL(glUniform3f)
    LOAD_GL(glGenVertexArrays)
    LOAD_GL(glBindVertexArray)
    LOAD_GL(glDeleteVertexArrays)
    LOAD_GL(glGenBuffers)
    LOAD_GL(glBindBuffer)
    LOAD_GL(glBufferData)
    LOAD_GL(glBufferSubData)
    LOAD_GL(glDeleteBuffers)
    LOAD_GL(glEnableVertexAttribArray)
    LOAD_GL(glVertexAttribPointer)
    return {};
}
