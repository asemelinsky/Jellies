// Shaders are written as desktop GLSL 330. In the browser (WebGL 2) the same
// source is compiled as GLSL ES 3.00, which needs a different version line and
// explicit default precision.
#pragma once
#include <string>

inline std::string PortGLSL(const char* src) {
    std::string s(src ? src : "");
#if defined(PLATFORM_WEB)
    const std::string from = "#version 330";
    size_t p = s.find(from);
    if (p != std::string::npos) {
        size_t eol = s.find('\n', p);
        if (eol == std::string::npos) eol = s.size();
        s.replace(p, eol - p,
                  "#version 300 es\n"
                  "precision highp float;\n"
                  "precision highp int;\n"
                  "precision highp sampler2D;");
    }
#endif
    return s;
}
