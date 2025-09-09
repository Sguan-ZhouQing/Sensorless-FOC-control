#ifndef __FAST_SIN_H
#define __FAST_SIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define PI 3.14159265358979323846f

// 声明函数
float f1(float x);
float f2(float x);
float fast_sin(float x);
float fast_cos(float x);
void fast_sin_cos(float x, float *sin_x, float *cos_x);

#ifdef __cplusplus
}
#endif

#endif // FAST_SIN_H

