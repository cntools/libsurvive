//
//  quaternion.h
//  Game
//
//  Created by user on 9/8/16.
//  Copyright © 2016 user. All rights reserved.
//

#ifndef quaternion_h
#define quaternion_h

typedef struct {
    float i,j,k,r;
} Quaternion;
typedef struct {
    float x,y,z,theta;
} AxisAngle;

typedef struct {
    Quaternion rot;   // orientation
    Quaternion pos;   // positon
} QuaternionBone;

#define printq2(name,quat) printf("%s i %f j %f k %f r %f\n", name, quat.i, quat.j, quat.k, quat.r)

#define QuaternionNormalize(quat) { \
    float inv_len= 1.0f / sqrt( quat.i*quat.i + quat.j*quat.j + quat.k*quat.k + quat.r*quat.r ); \
    quat.i *= inv_len; \
    quat.j *= inv_len; \
    quat.k *= inv_len; }

#define QuaternionSet(res,I,J,K,R) { res.i=I; res.j=J; res.k=K; res.r=R; }

#define AxisAngleOfQuaternion(res,q) { \
    float len = sqrt(q.i*q.i + q.j*q.j + q.k*q.k); \
    if (len > 0.0001) { \
        float inv_len = 1.0f / len;  \
        res.x = q.i * inv_len; \
        res.y = q.j * inv_len; \
        res.z = q.k * inv_len; \
    } else { \
        res.x=1.0f;  res.y=0.0f; res.z=0.0f; \
    } \
    res.theta = 2.0 * acos( q.r ); }

#define QuaternionOfAxisAngle(res,aa) QuaternionSetAxisAngle(res, (aa).x, (aa).y, (aa).z, (aa).theta)

#define QuaternionSetAxisAngle(res,x,y,z,theta) { \
    float inv_lenXYZ = 1.0f / sqrt( (x)*(x) + (y)*(y) + (z)*(z) );       \
    float cos_half_theta = cos( 0.5f*(theta) );     \
    float sin_half_theta = sin( 0.5f*(theta) );     \
    res.i = (x) * sin_half_theta * inv_lenXYZ; \
    res.j = (y) * sin_half_theta * inv_lenXYZ; \
    res.k = (z) * sin_half_theta * inv_lenXYZ; \
    res.r =       cos_half_theta; }

#define QuaternionIdentity(q) { q.i=0.0f; q.j=0.0f; q.k=0.0f; q.r=1.0f; }

#define QuaternionInv(res,a) { \
    res.i = -(a).i; \
    res.j = -(a).j; \
    res.k = -(a).k; \
    res.r =  (a).r; }

#define QuaternionMult(res,a,b) { \
    res.i =  (a).i*(b).r + (a).j*(b).k - (a).k*(b).j + (a).r*(b).i; \
    res.j = -(a).i*(b).k + (a).j*(b).r + (a).k*(b).i + (a).r*(b).j; \
    res.k =  (a).i*(b).j - (a).j*(b).i + (a).k*(b).r + (a).r*(b).k; \
    res.r = -(a).i*(b).i - (a).j*(b).j - (a).k*(b).k + (a).r*(b).r; }

#define QuaternionSub(res,a,b) { \
    res.i = (a).i - (b).i; \
    res.j = (a).j - (b).j; \
    res.k = (a).k - (b).k; \
    res.r = (a).r - (b).r; }

#define QuaternionAdd(res,a,b) { \
    res.i = (a).i + (b).i; \
    res.j = (a).j + (b).j; \
    res.k = (a).k + (b).k; \
    res.r = (a).r + (b).r; }

#define QuaternionRot(res,q,p) { \
    Quaternion q_inv,left;      \
    QuaternionInv(q_inv,q);     \
    QuaternionMult(left,q,p);   \
    QuaternionMult(res,left,q_inv); }

#define QuaternionSlerp(res,delta,u,v) { \
    Quaternion rot,u_inv;        \
    float theta,inv_s,scale;     \
    QuaternionInv(u_inv,u);      \
    QuaternionMult(rot,v,u_inv); \
    if (rot.r < 0.998 && rot.r > -0.998) { \
        theta  = acos(rot.r);        \
        inv_s  = 1.0f / sin(theta);  \
        theta *= delta;              \
        scale  = inv_s * sin(theta); \
        rot.i *= scale;              \
        rot.j *= scale;              \
        rot.k *= scale;              \
        rot.r  = cos(theta);         \
        QuaternionMult(res,rot,u);   \
    } else {     \
        res = u; \
    } }

/*
#define QuaternionSlerp(res,delta,u,v) { \
    Quaternion rot,u_inv;        \
    float theta,inv_s,scale;     \
    QuaternionInv(u_inv,u);      \
    QuaternionMult(rot,v,u_inv); \
    if (rot.r < 0.998) { \
        printq2("slerp v",v); \
        theta  = acos(rot.r);        \
        inv_s  = 1.0f / sin(theta);  \
        printf("slerp theta %f inv_s %f\n", theta, inv_s); \
        theta *= delta;              \
        scale  = inv_s * sin(theta); \
        rot.i *= scale;              \
        rot.j *= scale;              \
        rot.k *= scale;              \
        rot.r  = cos(theta);         \
        QuaternionMult(res,rot,u);   \
    } else {     \
        res = u; \
    } \
    printq2("slerp res", res); }
*/

#define QuaternionBoneTransform(out,in,bone) {       \
    Quaternion subtracted,rotated;              \
    QuaternionSub(subtracted,in,bone.pos);      \
    QuaternionRot(rotated,bone.rot,subtracted); \
    QuaternionAdd(out,rotated,bone.pos); }

#endif /* quaternion_h */
