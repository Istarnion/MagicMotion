#ifndef UTILS_H_
#define UTILS_H_

#ifdef MIN
#undef MIN
#endif
#define MIN(a, b) ((a) <= (b) ? (a) : (b))

#ifdef MAX
#undef MAX
#endif
#define MAX(a, b) ((a) >= (b) ? (a) : (b))

// Linearly interpolate between a and b
#define LERP(a, b, t) ((a)*(1.0f-(t)) + (b)*(t))

#endif /* end of include guard: UTILS_H_ */

