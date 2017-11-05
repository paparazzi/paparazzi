/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/lib/opengl/opengl.h
 *
 * Handles all the OpenGL ES and EGL setup and shared
 * functions. This is developped for the Mali GPU
 * especially for vision processing.
 */

#ifndef _CV_LIB_OPENGL_H
#define _CV_LIB_OPENGL_H

#include "std.h"
#include "GLES2/gl2.h"
#include "GLES2/gl2ext.h"
#include "EGL/egl.h"

struct opengl_t {
  EGLDisplay display;
  EGLSurface surface;
  EGLContext context;
  EGLConfig config;
  GLuint texture; // For now here
  GLuint programObject; // For now here
};
extern struct opengl_t opengl;

bool opengl_init(void);
void opengl_free(void);
bool opengl_linkProgram(void);
bool opengl_createProgram(void);
GLuint opengl_shader_load(const char *shaderSrc, GLenum type);
GLuint opengl_shader_load_file(const char *shaderFile, GLenum type);

#endif /* _CV_LIB_OPENGL_H */
