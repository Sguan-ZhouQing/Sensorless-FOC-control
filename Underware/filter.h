#ifndef __FILTER_H
#define __FILTER_H

float low_pass_filter(float input, float last_output, float alpha);
float kalman_filter_std(float input, float r, float q);


#endif // FILTER_H
