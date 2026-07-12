#pragma once
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <GL/gl.h>
#include <cstddef>

typedef char GLchar;
typedef ptrdiff_t GLsizeiptr;
typedef ptrdiff_t GLintptr;

#define GL_ARRAY_BUFFER      0x8892
#define GL_STATIC_DRAW       0x88E4
#define GL_DYNAMIC_DRAW      0x88E8
#define GL_FRAGMENT_SHADER   0x8B30
#define GL_VERTEX_SHADER     0x8B31
#define GL_COMPILE_STATUS    0x8B81
#define GL_LINK_STATUS       0x8B82
#define GL_INFO_LOG_LENGTH   0x8B84

extern GLuint (APIENTRY *glCreateShader)(GLenum type);
extern void   (APIENTRY *glShaderSource)(GLuint shader, GLsizei count, const GLchar* const* strings, const GLint* lengths);
extern void   (APIENTRY *glCompileShader)(GLuint shader);
extern void   (APIENTRY *glGetShaderiv)(GLuint shader, GLenum pname, GLint* params);
extern void   (APIENTRY *glGetShaderInfoLog)(GLuint shader, GLsizei maxLen, GLsizei* length, GLchar* infoLog);
extern void   (APIENTRY *glDeleteShader)(GLuint shader);
extern GLuint (APIENTRY *glCreateProgram)(void);
extern void   (APIENTRY *glAttachShader)(GLuint program, GLuint shader);
extern void   (APIENTRY *glLinkProgram)(GLuint program);
extern void   (APIENTRY *glGetProgramiv)(GLuint program, GLenum pname, GLint* params);
extern void   (APIENTRY *glGetProgramInfoLog)(GLuint program, GLsizei maxLen, GLsizei* length, GLchar* infoLog);
extern void   (APIENTRY *glUseProgram)(GLuint program);
extern void   (APIENTRY *glDeleteProgram)(GLuint program);
extern GLint  (APIENTRY *glGetUniformLocation)(GLuint program, const GLchar* name);
extern void   (APIENTRY *glUniformMatrix4fv)(GLint location, GLsizei count, GLboolean transpose, const GLfloat* value);
extern void   (APIENTRY *glUniform1f)(GLint location, GLfloat v0);
extern void   (APIENTRY *glUniform3f)(GLint location, GLfloat v0, GLfloat v1, GLfloat v2);
extern void   (APIENTRY *glGenVertexArrays)(GLsizei n, GLuint* arrays);
extern void   (APIENTRY *glBindVertexArray)(GLuint array);
extern void   (APIENTRY *glDeleteVertexArrays)(GLsizei n, const GLuint* arrays);
extern void   (APIENTRY *glGenBuffers)(GLsizei n, GLuint* buffers);
extern void   (APIENTRY *glBindBuffer)(GLenum target, GLuint buffer);
extern void   (APIENTRY *glBufferData)(GLenum target, GLsizeiptr size, const void* data, GLenum usage);
extern void   (APIENTRY *glDeleteBuffers)(GLsizei n, const GLuint* buffers);
extern void   (APIENTRY *glEnableVertexAttribArray)(GLuint index);
extern void   (APIENTRY *glVertexAttribPointer)(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void* pointer);

// Must be called with a current OpenGL context. Returns false if any entry point is missing.
bool LoadGlFunctions();
