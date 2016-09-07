

#ifndef MARKER_CHECKERS_H
#define MARKER_CHECKERS_H

#ifdef __cplusplus
extern "C" {
#endif

struct resultsc {
    bool detected;
    int x;
    int y;
};

void init_detect_checkers(void);

struct resultsc opencv_detect_checkers(char *img, int width, int height, int dt);

#ifdef __cplusplus
}
#endif


#endif // MARKER_CHECKERS_H
