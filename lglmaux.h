/*
** $Id: lglmaux.h $
** Supplemental functions for Lua libraries
** See Copyright Notice in lua.h
*/

#ifndef lglmaux_h
#define lglmaux_h

#include "lua.h"

/* MAXNUMBER2STR for vector and matrix types. */
#define LUAGLM_MAXNUMBER2STR	768

/* Return the type string associated with the GLM value at idx. */
LUAI_FUNC const char *luaglm_typename (lua_State *L, int idx);

/*
** Push onto the stack a formatted string of the vector/matrix value at idx,
** returning a pointer to created string.
*/
LUAI_FUNC const char *luaglm_pushstring (lua_State *L, int idx);

/* String formatting fast-path. */
LUAI_FUNC int luaglm_format (lua_State *L, int idx, char *buff, size_t sz);

/*
** One-at-a-time hash the value at the given index: string values are hashed,
** boolean and numeric values are cast to lua_Integer, otherwise zero.
*/
LUAI_FUNC lua_Unsigned (luaglm_tohash) (lua_State *L, int idx, int ignore_case);

/*
** {==================================================================
** GLSL Library Functions
** https://registry.khronos.org/OpenGL/specs/gl/GLSLangSpec.4.60.pdf
** ===================================================================
*/

/* Chapter 8. Built-In Functions */
LUAI_FUNC int luaglm_vec (lua_State *L);
LUAI_FUNC int luaglm_vec2 (lua_State *L);
LUAI_FUNC int luaglm_vec3 (lua_State *L);
LUAI_FUNC int luaglm_vec4 (lua_State *L);
LUAI_FUNC int luaglm_quat (lua_State *L);
LUAI_FUNC int luaglm_mat2x2 (lua_State *L);
LUAI_FUNC int luaglm_mat3x3 (lua_State *L);
LUAI_FUNC int luaglm_mat4x4 (lua_State *L);

/* 8.1. Angle and Trigonometry Functions */
LUAI_FUNC int luaglm_rad (lua_State *L);
LUAI_FUNC int luaglm_deg (lua_State *L);
LUAI_FUNC int luaglm_sin (lua_State *L);
LUAI_FUNC int luaglm_cos (lua_State *L);
LUAI_FUNC int luaglm_tan (lua_State *L);
LUAI_FUNC int luaglm_asin (lua_State *L);
LUAI_FUNC int luaglm_acos (lua_State *L);
LUAI_FUNC int luaglm_atan (lua_State *L);
LUAI_FUNC int luaglm_sinh (lua_State *L);
LUAI_FUNC int luaglm_cosh (lua_State *L);
LUAI_FUNC int luaglm_tanh (lua_State *L);
LUAI_FUNC int luaglm_asinh (lua_State *L);
LUAI_FUNC int luaglm_acosh (lua_State *L);
LUAI_FUNC int luaglm_atanh (lua_State *L);

/* 8.2. Exponential Functions */
LUAI_FUNC int luaglm_pow (lua_State *L);
LUAI_FUNC int luaglm_exp (lua_State *L);
LUAI_FUNC int luaglm_log (lua_State *L);
LUAI_FUNC int luaglm_exp2 (lua_State *L);
LUAI_FUNC int luaglm_log2 (lua_State *L);
LUAI_FUNC int luaglm_sqrt (lua_State *L);
LUAI_FUNC int luaglm_invsqrt (lua_State *L);

/* 8.3. Common Functions */
LUAI_FUNC int luaglm_abs (lua_State *L);
LUAI_FUNC int luaglm_sign (lua_State *L);
LUAI_FUNC int luaglm_floor (lua_State *L);
LUAI_FUNC int luaglm_trunc (lua_State *L);
LUAI_FUNC int luaglm_round (lua_State *L);
LUAI_FUNC int luaglm_ceil (lua_State *L);
LUAI_FUNC int luaglm_fract (lua_State *L);
LUAI_FUNC int luaglm_mod (lua_State *L);
LUAI_FUNC int luaglm_modf (lua_State *L);
LUAI_FUNC int luaglm_min (lua_State *L);
LUAI_FUNC int luaglm_max (lua_State *L);
LUAI_FUNC int luaglm_clamp (lua_State *L);
LUAI_FUNC int luaglm_mix (lua_State *L);
LUAI_FUNC int luaglm_step (lua_State *L);
LUAI_FUNC int luaglm_smoothstep (lua_State *L);
LUAI_FUNC int luaglm_isnan (lua_State *L);
LUAI_FUNC int luaglm_isinf (lua_State *L);
LUAI_FUNC int luaglm_fma (lua_State *L);
LUAI_FUNC int luaglm_frexp (lua_State *L);
LUAI_FUNC int luaglm_ldexp (lua_State *L);
#if LUAI_IS32INT
/* LUAI_FUNC int luaglm_floatBitsToInt (lua_State *L); */
/* LUAI_FUNC int luaglm_intBitsToFloat (lua_State *L); */
/* LUAI_FUNC int luaglm_floatBitsToUint (lua_State *L); */
/* LUAI_FUNC int luaglm_uintBitsToFloat (lua_State *L); */
#endif

/* 8.4. Floating-Point Pack and Unpack Functions */
/* LUAI_FUNC int luaglm_packUnorm2x16 (lua_State *L); */
/* LUAI_FUNC int luaglm_packSnorm2x16 (lua_State *L); */
/* LUAI_FUNC int luaglm_packUnorm4x8 (lua_State *L); */
/* LUAI_FUNC int luaglm_packSnorm4x8 (lua_State *L); */
/* LUAI_FUNC int luaglm_unpackUnorm2x16 (lua_State *L); */
/* LUAI_FUNC int luaglm_unpackSnorm2x16 (lua_State *L); */
/* LUAI_FUNC int luaglm_unpackUnorm4x8 (lua_State *L); */
/* LUAI_FUNC int luaglm_unpackSnorm4x8 (lua_State *L); */
/* LUAI_FUNC int luaglm_packHalf2x16 (lua_State *L); */
/* LUAI_FUNC int luaglm_unpackHalf2x16 (lua_State *L); */
/* LUAI_FUNC int luaglm_packDouble2x32 (lua_State *L); */
/* LUAI_FUNC int luaglm_unpackDouble2x32 (lua_State *L); */

/* 8.5. Geometric Functions */
LUAI_FUNC int luaglm_length (lua_State *L);
LUAI_FUNC int luaglm_distance (lua_State *L);
LUAI_FUNC int luaglm_dot (lua_State *L);
LUAI_FUNC int luaglm_cross (lua_State *L);
LUAI_FUNC int luaglm_normalize (lua_State *L);
LUAI_FUNC int luaglm_faceforward (lua_State *L);
LUAI_FUNC int luaglm_reflect (lua_State *L);
LUAI_FUNC int luaglm_refract (lua_State *L);

/* 8.6. Matrix Functions */
LUAI_FUNC int luaglm_matrixcompmult (lua_State *L);
LUAI_FUNC int luaglm_outerproduct (lua_State *L);
LUAI_FUNC int luaglm_transpose (lua_State *L);
LUAI_FUNC int luaglm_det (lua_State *L);
LUAI_FUNC int luaglm_inverse (lua_State *L);

/* 8.7. Vector Relational Functions */
LUAI_FUNC int luaglm_lessThan (lua_State *L);
LUAI_FUNC int luaglm_lessThanEqual (lua_State *L);
LUAI_FUNC int luaglm_greaterThan (lua_State *L);
LUAI_FUNC int luaglm_greaterThanEqual (lua_State *L);
LUAI_FUNC int luaglm_equal (lua_State *L);
LUAI_FUNC int luaglm_notEqual (lua_State *L);
LUAI_FUNC int luaglm_any (lua_State *L);
LUAI_FUNC int luaglm_all (lua_State *L);
LUAI_FUNC int luaglm_not (lua_State *L);

/* 8.8. Integer Functions */
/* LUAI_FUNC int luaglm_uaddCarry (lua_State *L); */
/* LUAI_FUNC int luaglm_usubBorrow (lua_State *L); */
/* LUAI_FUNC int luaglm_umulExtended (lua_State *L); */
/* LUAI_FUNC int luaglm_imulExtended (lua_State *L); */
LUAI_FUNC int luaglm_bitfieldExtract (lua_State *L);
LUAI_FUNC int luaglm_bitfieldInsert (lua_State *L);
LUAI_FUNC int luaglm_bitfieldReverse (lua_State *L);
LUAI_FUNC int luaglm_bitCount (lua_State *L);
LUAI_FUNC int luaglm_findLSB (lua_State *L);
LUAI_FUNC int luaglm_findMSB (lua_State *L);

/* 8.9. Texture Functions */
/* 8.10. Atomic-Counter Functions */
/* 8.11. Image Functions */
/* 8.12. Fragment Processing Functions */
/* 8.13. Geometry Shader Functions */
/* 8.14. Fragment Processing Functions */
/* 8.15. Noise Functions (deprecated starting with version 4.4 of GLSL) */

/* 8.X. Extensions */
LUAI_FUNC int luaglm_approx (lua_State *L);

LUAI_FUNC int luaglm_angle (lua_State *L);
LUAI_FUNC int luaglm_quat_axis (lua_State *L);
LUAI_FUNC int luaglm_quat_for (lua_State *L);
LUAI_FUNC int luaglm_slerp (lua_State *L);
/* LUAI_FUNC int luaglm_squad (lua_State *L); */

LUAI_FUNC int luaglm_translate (lua_State *L);
LUAI_FUNC int luaglm_scale (lua_State *L);
LUAI_FUNC int luaglm_rotate (lua_State *L);
LUAI_FUNC int luaglm_frustum (lua_State *L);
LUAI_FUNC int luaglm_ortho (lua_State *L);
LUAI_FUNC int luaglm_perspective (lua_State *L);
LUAI_FUNC int luaglm_project (lua_State *L);
LUAI_FUNC int luaglm_unproject (lua_State *L);
LUAI_FUNC int luaglm_lookat (lua_State *L);
LUAI_FUNC int luaglm_billboard (lua_State *L);
LUAI_FUNC int luaglm_compose (lua_State *L);
LUAI_FUNC int luaglm_decompose (lua_State *L);

LUAI_FUNC int luaglm_from_euler (lua_State *L);
LUAI_FUNC int luaglm_to_euler (lua_State *L);

/* }================================================================== */

#endif
