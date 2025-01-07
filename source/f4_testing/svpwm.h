
#ifndef __SVPWM_H__
#define __SVPWM_H__

#include <math.h>

#define SQRT3VAL (1.73205080757F)

#define RAD2DEG(rad) ((rad) * (180.0 / M_PI))
#define DEG2RAD(deg) ((deg) * (M_PI / 180.0))

// TODO convert all of this to fixed point math

static inline void clarke_transform(int32_t ia, int32_t ib, int32_t *ialphap, int32_t *ibetap)
{
    // for balanced systems, ia + ib + ic = 0, so clarke reduces to 2x2 transform
    int32_t ialpha = ia;
    int32_t ibeta = ia * (1 / SQRT3VAL) + ib * (2 / SQRT3VAL);
    *ialphap = ialpha;
    *ibetap = ibeta;
}

static inline void park_transform(int32_t vd, int32_t vq, float teta, int32_t *valphap, int32_t *vbetap)
{
    // Park: CW rotation matrix by teta to make alpha/beta time-invariant
    // teta: rotor flux position (bring to stator reference frame)
    int32_t valpha = vd * cos(teta) + vq * -sin(teta);
    int32_t vbeta = vd * sin(teta) + vq * cos(teta);
    *valphap = valpha;
    *vbetap = vbeta;
}

static inline int svpwm_get_sector(int32_t alpha, int32_t beta)
{
    int32_t ang = RAD2DEG(atan2(alpha, beta));
    // TODO this sucks
    if (ang < 0)
        ang += 360;
    while (ang >= 360)
        ang -= 360;

    if (ang >=   0 && ang <  60) return 1;
    if (ang >=  60 && ang < 120) return 2;
    if (ang >= 120 && ang < 180) return 3;
    if (ang >= 180 && ang < 240) return 4;
    if (ang >= 240 && ang < 300) return 5;
    if (ang >= 300 && ang < 360) return 6;

    return 0;
}

static inline void svpwm_calc_weight(int32_t alpha, int32_t beta, int32_t *t1p, int32_t *t2p, int *sectorp)
{
    int sector;
    double t1;
    double t2;

    sector = svpwm_get_sector(alpha, beta);

    switch (sector)
    {
    case 1: // 1 0 0, 1 1 0
        t2 = (alpha) * (2 / SQRT3VAL);
        t1 = (beta) - t2 / 2;
        break;
    case 2: // 1 1 0, 0 1 0
        t2 = alpha * (1 / SQRT3VAL) - beta;
        t1 = (beta) * (2) + t2;
        break;
    case 3: // 0 1 0, 0 1 1
        t1 = (alpha) * (2 / SQRT3VAL);
        t2 = (-t1 / 2) - (beta);
        break;
    case 4: // 0 1 1, 0 0 1
        t2 = (alpha) * -(2 / SQRT3VAL);
        t1 = (-t2 / 2) - (beta);
        break;
    case 5: // 0 0 1, 1 0 0
        t1 = (alpha) * (-1 / SQRT3VAL) - beta;
        t2 = (beta) * (2) + t1;
        break;
    case 6: // 0 0 1, 1 0 0
        t1 = (alpha) * (-2 / SQRT3VAL);
        t2 = (beta) - (t1 / 2);
        break;
    default:
        t1 = 0;
        t2 = 0;
        break;
    }

    *t1p = (int32_t)t1; // [0, vdc]
    *t2p = (int32_t)t2; // [0, vdc]
    *sectorp = sector;
}

static inline void svpwm_calc_duty(int32_t t1, int32_t t2, int sector, int32_t vdc, int32_t *s1p, int32_t *s2p, int32_t *s3p)
{
    double t0, t7;
    double s1, s2, s3;

    t0 = vdc - (t1 + t2); // calc amount of zero time
    t0 = t0 / 2; // split zero times evenly between t0 and t7
    t7 = t0;

    switch (sector)
    {   case 1: // 1 0 0, 1 1 0
            s1 = (t1 + t2) + t7;
            s2 = t2 + t7;
            s3 = t7;
        break;
        case 2: // 1 1 0, 0 1 0
            s1 = t1 + t7;
            s2 = (t1 + t2) + t7;
            s3 = t7;
        break;
        case 3: // 0 1 0, 0 1 1
            s1 = t7;
            s2 = (t1 + t2) + t7;
            s3 = t2 + t7;
        break;
        case 4: // 0 1 1, 0 0 1
            s1 = t7;
            s2 = t1 + t7;
            s3 = (t1 + t2) + t7;
        break;
        case 5: // 0 0 1, 1 0 1
            s1 = t2 + t7;
            s2 = t7;
            s3 = (t1 + t2) + t7;
        break;
        case 6: // 1 0 1, 1 0 0
            s1 = (t1 + t2) + t7;
            s2 = t7;
            s3 = t1 + t7;
        break;
        default:
            s1 = 0;
            s2 = 0;
            s3 = 0;
        break;
    }

    *s1p = (int32_t)s1;
    *s2p = (int32_t)s2;
    *s3p = (int32_t)s3;
}

#define LUT_COUNT 128

static int32_t sin_wav_lut1[LUT_COUNT] = {0, 6, 12, 18, 24, 30, 37, 42, 48, 54, 60, 65, 70, 75, 80, 85, 90, 94, 98, 102, 106, 109, 112, 115, 117, 120, 122, 123, 125, 126, 126, 127, 127, 127, 126, 126, 125, 123, 122, 120, 117, 115, 112, 109, 106, 102, 98, 94, 90, 85, 80, 75, 70, 65, 60, 54, 48, 42, 37, 30, 24, 18, 12, 6, 0, -7, -13, -19, -25, -31, -38, -43, -49, -55, -61, -66, -71, -76, -81, -86, -91, -95, -99, -103, -107, -110, -113, -116, -118, -121, -123, -124, -126, -127, -127, -128, -128, -128, -127, -127, -126, -124, -123, -121, -118, -116, -113, -110, -107, -103, -99, -95, -91, -86, -81, -76, -71, -66, -61, -55, -49, -43, -38, -31, -25, -19, -13, -7}; // +0
static int32_t sin_wav_lut2[LUT_COUNT] = {110, 107, 103, 99, 95, 91, 87, 82, 77, 72, 67, 61, 56, 50, 44, 39, 33, 26, 20, 14, 8, 2, -5, -11, -17, -23, -29, -36, -41, -47, -53, -59, -64, -70, -75, -80, -85, -89, -94, -98, -102, -105, -109, -112, -115, -117, -120, -122, -124, -125, -126, -127, -128, -128, -128, -128, -127, -126, -125, -123, -121, -119, -117, -114, -111, -108, -104, -100, -96, -92, -88, -83, -78, -73, -68, -62, -57, -51, -45, -40, -34, -27, -21, -15, -9, -3, 4, 10, 16, 22, 28, 35, 40, 46, 52, 58, 63, 69, 74, 79, 84, 88, 93, 97, 101, 104, 108, 111, 114, 116, 119, 121, 123, 124, 125, 126, 127, 127, 127, 127, 126, 125, 124, 122, 120, 118, 116, 113}; // +2pi/3

#endif // __SVPWM_H__
