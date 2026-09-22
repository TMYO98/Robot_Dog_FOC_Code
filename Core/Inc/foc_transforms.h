/*
 * foc_transforms.h
 *
 * Clarke / Park / inverse Park transforms for FOC.
 *
 * Convention:
 *   Clarke:      ialpha = ia
 *                ibeta  = (ia + 2*ib) / sqrt(3)     (assumes ia + ib + ic = 0)
 *   Park:        id =  ialpha*cos(theta) + ibeta*sin(theta)
 *                iq = -ialpha*sin(theta) + ibeta*cos(theta)
 *   Inverse Park: valpha = vd*cos(theta) - vq*sin(theta)
 *                 vbeta  = vd*sin(theta) + vq*cos(theta)
 *   Inverse Clarke: va = valpha
 *                   vb = -valpha/2 + (sqrt(3)/2)*vbeta
 *                   vc = -valpha/2 - (sqrt(3)/2)*vbeta
 *
 * theta is the electrical angle in radians.
 *
 * Created on: Sep 21, 2026
 *      Author: CT
 */

#ifndef INC_FOC_TRANSFORMS_H_
#define INC_FOC_TRANSFORMS_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void FOC_ClarkeTransform(float ia, float ib, float ic, float *ialpha, float *ibeta);
void FOC_ParkTransform(float ialpha, float ibeta, float theta, float *id, float *iq);
void FOC_InverseParkTransform(float id, float iq, float theta, float *ialpha, float *ibeta);
void FOC_InverseClarkeTransform(float valpha, float vbeta, float *va, float *vb, float *vc);

#ifdef __cplusplus
}
#endif

#endif /* INC_FOC_TRANSFORMS_H_ */
