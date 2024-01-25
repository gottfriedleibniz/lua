/*
** $Id: lglmext.hpp $
** Extensions to cglm:
**  1. glm/detail/type_half.{hpp,inl} implemented for cglm
**  2. glm/gtx/string_cast.{hpp,inl} implemented for cglm
**  3. glm/gtx/hash.{hpp,inl} implemented for cglm
**  4. Missing GLSL built-in functions
**  5. SIMD improvements for cglm and glsl functions
** See Copyright Notice in lua.h
*/

#ifndef lglmext_h
#define lglmext_h

#include <stdio.h>
#include <stddef.h>
#include <cglm/cglm.h>

/*!
 * @brief round towards even
 */
CGLM_INLINE float glm_roundf(float x) {
  return isfinite(x) ? x - remainderf(x, 1.0f) : x;
}

/*!
 * @brief calculate sin and cos
 */
CGLM_INLINE void glm_sincos(float x, float *sin, float *cos) {
  *sin = sinf(x);
  *cos = cosf(x);
}

/*!
 * @brief asinf wrapper that sanitizes its domain
 */
CGLM_INLINE float glm_asin(float x) {
  return asinf(glm_clamp(x, -1.0f, 1.0f));
}

/*!
 * @brief acosf wrapper that sanitizes its domain
 */
CGLM_INLINE float glm_acos(float x) {
  return acosf(glm_clamp(x, -1.0f, 1.0f));
}

/*!
 * @brief acoshf wrapper that sanitizes its domain
 */
CGLM_INLINE float glm_acosh(float x) {
  return x <= 1.0f ? 0.0f : acoshf(x);
}

/*!
 * @brief atanhf wrapper that sanitizes its domain
 */
CGLM_INLINE float glm_atanh(float x) {
  return atanhf(glm_clamp(x, -1.0f, 1.0f));
}

/*!
 * @brief generic log implementation
 */
CGLM_INLINE float glm_log(float x, float base) {
  if (base == 2.0f) return log2f(x);
  if (base == 10.0f) return log10f(x);
  return logf(x) / logf(base);
}

/*
** {==================================================================
** type_half.h
** ===================================================================
*/

/* types.h */
typedef uint16_t float16;
typedef float16 hvec4[4];

/* On non-GNU compilers assume AVX2 implies F16C */
#if !defined(CGLM_SIMD_FP16)
#if defined(CGLM_SIMD_x86) \
  && (defined(__F16C__) || (!defined(__GNUC__) && defined(__AVX2__)))
  #define CGLM_SIMD_FP16 1
#elif defined(CGLM_SIMD_ARM) && (__ARM_FP & 2) \
  && (defined(__ARM_FP16_FORMAT_IEEE) || defined(__ARM_FP16_FORMAT_ALTERNATIVE))
  #define CGLM_SIMD_FP16 1
#endif
#endif

/* _mm_storeu_si64: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=87558 */
#if (defined(__GNUC__) && __GNUC__ >= 9)          \
  || (defined(__clang__) && __clang_major__ >= 8) \
  || (defined(_MSC_VER) && _MSC_VER >= 1910)      \
  || (defined(__INTEL_COMPILER) && defined(__x86_64__))
  #define CGLM_HALF_INTRINSICS_x86 1
#else
  #define CGLM_HALF_INTRINSICS_x86 0
#endif

/*!
 * @brief convert float to half float
 */
CGLM_INLINE float16 glm_to_half(float v) {
#if defined(CGLM_SIMD_x86) && defined(CGLM_SIMD_FP16)
#if defined(_MSC_VER)
  __m128i cvt0 = _mm_cvtps_ph(_mm_set_ss(v), _MM_FROUND_TO_NEAREST_INT);
  return (float16)_mm_extract_epi16(cvt0, 0);
#else
  return _cvtss_sh(v, _MM_FROUND_TO_NEAREST_INT);
#endif
#elif defined(CGLM_SIMD_ARM) && defined(CGLM_SIMD_FP16)
  return vget_lane_u16(vreinterpret_u16_f16(vcvt_f16_f32(glmm_set1(v))), 0);
#else
  uint32_t a, sign, half;
  union {
    float f;
    uint32_t bits;
  } conv;

  conv.f = v;
  sign = (conv.bits & 0x80000000U) >> 16U;
  a = conv.bits & 0x7FFFFFFFU;
  if (a >= 0x47800000) /* Exponent overflow/NaN -> Signed Inf/NaN */
    half = 0x7C00U | ((a > 0x7F800000) ? (0x200 | ((a >> 13U) & 0x3FFU)) : 0U);
  else if (a <= 0x33000000U) /* Exponent underflow -> Signed zero */
    half = 0;
  else if (a < 0x38800000U) { /* Exponent underflow -> Subnormal */
    uint32_t shift = 125U - (a >> 23U);
    uint32_t sig = 0x800000U | (a & 0x7FFFFFU);
    uint32_t last = (sig & ((1U << shift) - 1)) != 0;
    half = ((sig >> (shift + 1)) | last) & ((sig >> shift) & 1U);
  }
  else {
    uint32_t sig = a + 0xC8000000U;
    half = ((sig + 0x0FFFU + ((sig >> 13U) & 1U)) >> 13U) & 0x7FFFU;
  }
  return (float16)(sign | half);
#endif
}

/*!
 * @brief convert half float to float
 */
CGLM_INLINE float glm_from_half(float16 v) {
#if defined(CGLM_SIMD_x86) && defined(CGLM_SIMD_FP16)
#if defined(_MSC_VER)
  return _mm_cvtss_f32(_mm_cvtph_ps(_mm_cvtsi32_si128((int)v)));
#else
  return _cvtsh_ss(v);
#endif
#elif defined(CGLM_SIMD_ARM) && defined(CGLM_SIMD_FP16)
  return vgetq_lane_f32(vcvt_f32_f16(vreinterpret_f16_u16(vdup_n_u16(v))), 0);
#else
  uint32_t a, sign, exponent, mantissa;
  union {
    float f;
    uint32_t bits;
  } conv;

  sign = ((uint32_t)v & 0x8000u) << 16;
  exponent = (uint32_t)(v & 0x7C00u);
  mantissa = (uint32_t)(v & 0x03FFu);
  if (exponent == 0x7C00u) /* Signed Inf/NaN */
    a = 0x7F800000u | (mantissa << 13);
  else if (exponent != 0x0000u) /* Normalized */
    a = (uint32_t)((v & 0x7FFFu) + 0x1C000u) << 13;
  else if (mantissa == 0) /* Signed-Zero */
    a = 0;
  else { /* Subnormal */
    mantissa <<= 1;
    while ((mantissa & 0x0400u) == 0) {
      mantissa <<= 1;
      exponent++;
    }
    a = ((0x70 - exponent) << 23) | ((mantissa & 0x03FFu) << 13);
  }
  conv.bits = sign | a;
  return conv.f;
#endif
}

/*!
 * @brief initialize a packed vector.
 */
CGLM_INLINE void glm_vec4_pack(vec4 input, hvec4 dest) {
#if defined(CGLM_SIMD_x86) && defined(CGLM_SIMD_FP16) && CGLM_HALF_INTRINSICS_x86
  _mm_storeu_si64(dest, _mm_cvtps_ph(glmm_load(input), _MM_FROUND_TO_NEAREST_INT));
#elif defined(CGLM_SIMD_ARM) && defined(CGLM_SIMD_FP16)
  vst1_u16(dest, vreinterpret_u16_f16(vcvt_f16_f32(glmm_load(input))));
#else
  dest[0] = glm_to_half(input[0]);
  dest[1] = glm_to_half(input[1]);
  dest[2] = glm_to_half(input[2]);
  dest[3] = glm_to_half(input[3]);
#endif
}

/*!
 * @brief unpack a packed vector.
 */
CGLM_INLINE void glm_vec4_unpack(hvec4 input, vec4 dest) {
#if defined(CGLM_SIMD_x86) && defined(CGLM_SIMD_FP16) && CGLM_HALF_INTRINSICS_x86
  glmm_store(dest, _mm_cvtph_ps(_mm_loadu_si64(input)));
#elif defined(CGLM_SIMD_ARM) && defined(CGLM_SIMD_FP16)
  glmm_store(dest, vcvt_f32_f16(vreinterpret_f16_u16(vld1_u16(input))));
#else
  dest[0] = glm_from_half(input[0]);
  dest[1] = glm_from_half(input[1]);
  dest[2] = glm_from_half(input[2]);
  dest[3] = glm_from_half(input[3]);
#endif
}

/* }================================================================== */

/*
** {==================================================================
** simd.h
** ===================================================================
*/

/* Masks */
#define CGLM_MASK_SIGN 0x80000000u
#define CGLM_MASK_MANT 0x807FFFFFu
#define CGLM_MASK_ABS  0x7FFFFFFFu
#define CGLM_MASK_LIMT 0x4B000000u
#define CGLM_MASK_PINF 0x7F800000u
#define CGLM_MASK_NINF 0xFF800000u

#if defined(CGLM_SIMD_x86)
/* Masking */
#define glmm_setbits1(x) _mm_castsi128_ps(_mm_set1_epi32((int)x))
#define glmm_setbits(w, z, y, x) _mm_castsi128_ps(_mm_set_epi32((int)w, (int)z, (int)y, (int)x))
#define glmm_maskstore(p, a) glmm_store(p, _mm_and_ps(a, glmm_set1(1.0f)))

/* Abstractions */
#define glmm_casti _mm_castps_si128
#define glmm_cvtt _mm_cvttps_epi32
#define glmm_setzero _mm_setzero_ps
#define glmm_add _mm_add_ps
#define glmm_sub _mm_sub_ps
#define glmm_mul _mm_mul_ps
#define glmm_div _mm_div_ps
#define glmm_sqrt _mm_sqrt_ps
#define glmm_eq _mm_cmpeq_ps
#define glmm_neq _mm_cmpneq_ps
#define glmm_lt _mm_cmplt_ps
#define glmm_le _mm_cmple_ps
#define glmm_gt _mm_cmpgt_ps
#define glmm_ge _mm_cmpge_ps
#define glmm_and _mm_and_ps
#define glmm_andnot _mm_andnot_ps
#define glmm_or _mm_or_ps
#define glmm_xor _mm_xor_ps
#define glmm_pmax _mm_max_ps
#define glmm_pmin _mm_min_ps

/* Integer */
#define glmm_128i __m128i
#define glmm_icast _mm_castsi128_ps
#define glmm_iset1 _mm_set1_epi32
#define glmm_isetzero _mm_setzero_si128
#define glmm_icvt _mm_cvtepi32_ps
#define glmm_iadd _mm_add_epi32
#define glmm_isub _mm_sub_epi32
#define glmm_ieq(x, y) glmm_icast(_mm_cmpeq_epi32((x), (y)))
#define glmm_iand _mm_and_si128
#define glmm_iandnot _mm_andnot_si128
#define glmm_isll _mm_slli_epi32
#define glmm_isrl _mm_srli_epi32
#ifdef CGLM_ALL_UNALIGNED
  #define glmm_iload(p) _mm_loadu_si128(p)
  #define glmm_istore(p, a) _mm_storeu_si128((__m128i *)p, a)
#else
  #define glmm_iload(p) _mm_load_si128(p)
  #define glmm_istore(p, a) _mm_store_si128((__m128i *)p, a)
#endif

CGLM_INLINE __m128 glmm_load2(vec2 v) {
#if defined(__SSE2__)
  return _mm_castpd_ps(_mm_load_sd((const double *)v));
#else
  return _mm_set_ps(0.0f, 0.0f, v[1], v[0]);
#endif
}

CGLM_INLINE __m128 glmm_load2h(vec2 v) {
  return _mm_set_ps(v[1], v[0], v[1], v[0]);
}

CGLM_INLINE void glmm_store2(vec2 dest, __m128 v) {
#if defined(__SSE2__)
  _mm_store_sd((double *)dest, _mm_castps_pd(v));
#else
  dest[0] = _mm_cvtss_f32(glmm_splat_x(v));
  dest[1] = _mm_cvtss_f32(glmm_splat_y(v));
#endif
}

/*!
 * @brief alternate implementation of glmm_load3
 */
CGLM_INLINE __m128 glmm_load3u(vec3 v) {
#if defined(__SSE2__)
  __m128 low = _mm_castpd_ps(_mm_load_sd((const double *)v));
  __m128 high = _mm_load_ss(&v[2]);
  return _mm_movelh_ps(low, high);
#else
  return _mm_set_ps(0.0f, v[2], v[1], v[0]);
#endif
}

/*!
 * @brief alternate implementation of glmm_store3
 */
CGLM_INLINE void glmm_store3u(vec3 dest, __m128 v) {
#if defined(__SSE2__)
  _mm_store_sd((double *)dest, _mm_castps_pd(v));
  _mm_store_ss(&dest[2], glmm_splat_z(v));
#else
  CGLM_ALIGN(16) vec4 v4;
  _mm_store_ps(v4, v);
  glm_vec3(v4, dest);
#endif
}

static inline __m128 glmm_select(__m128 a, __m128 b, __m128 mask) {
#if defined(__SSE4_1__) /* mask components are all-or-nothing */
  return _mm_blendv_ps(a, b, mask);
#else
  return _mm_or_ps(_mm_and_ps(mask, b), _mm_andnot_ps(mask, a));
#endif
}

static inline bool glmm_all(__m128 v) {
  return _mm_movemask_ps(_mm_cmpneq_ps(v, _mm_setzero_ps())) == 0x0F;
}

static inline bool glmm_any(__m128 v) {
  return _mm_movemask_ps(_mm_cmpneq_ps(v, _mm_setzero_ps())) != 0;
}

static inline bool glmm_eqv(__m128 x, __m128 y) {
  return _mm_movemask_ps(_mm_cmpeq_ps(x, y)) == 0x0F;
}

static inline __m128 glmm_isnan(__m128 v) {
  return _mm_cmpneq_ps(v, v);
}

static inline __m128 glmm_isinf(__m128 v) {
  return _mm_cmpeq_ps(glmm_abs(v), glmm_setbits1(CGLM_MASK_PINF));
}

static inline __m128 glmm_signbit(__m128 v) {
  return _mm_and_ps(v, glmm_setbits1(CGLM_MASK_SIGN));
}

static inline __m128 glmm_sign(__m128 v) { /* 1.0 : -1.0 */
  return _mm_or_ps(glmm_signbit(v), glmm_set1(1.0f));
}

static inline __m128 glmm_negate(__m128 v) {
  return _mm_xor_ps(v, glmm_float32x4_SIGNMASK_NEG);
}

static inline __m128 glmm_approx(__m128 x, __m128 y, __m128 eps) {
  __m128 sub0 = _mm_sub_ps(x, y);
  __m128 neg0 = _mm_sub_ps(_mm_setzero_ps(), sub0);
  return _mm_cmple_ps(_mm_max_ps(neg0, sub0), eps);
}

#if defined(__AVX__)
static inline __m256 glmm256_approx(__m256 x, __m256 y, __m256 eps) {
  __m256 sub0 = _mm256_sub_ps(x, y);
  __m256 neg0 = _mm256_sub_ps(_mm256_setzero_ps(), sub0);
  return _mm256_cmp_ps(_mm256_max_ps(neg0, sub0), eps, _CMP_LE_OQ);
}
#endif

static inline __m128 glmm_invsqrt(__m128 v) {
#if 1
  return _mm_div_ps(glmm_set1(1.0f), _mm_sqrt_ps(v));
#else
  __m128 rsqrt0 = _mm_rsqrt_ps(v);
  __m128 mul0 = _mm_mul_ps(_mm_mul_ps(v, rsqrt0), rsqrt0);
  return _mm_mul_ps(
    _mm_mul_ps(glmm_set1(0.5f), rsqrt0),
    _mm_sub_ps(glmm_set1(3.0f), mul0)
  );
#endif
}

static inline __m128 glmm_normalize(__m128 v) {
  return _mm_div_ps(v, _mm_sqrt_ps(glmm_vdot(v, v)));
}

static inline __m128 glmm_clamp(__m128 v, __m128 minVal, __m128 maxVal) {
  return _mm_min_ps(_mm_max_ps(v, minVal), maxVal);
}

static inline __m128 glmm_round(__m128 v) {
#if defined(__SSE4_1__)
  return _mm_round_ps(v, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC);
#else
  __m128 limit = glmm_setbits1(CGLM_MASK_LIMT);
  __m128 sgn0 = glmm_setbits1(CGLM_MASK_SIGN);
  __m128 or0 = _mm_or_ps(_mm_and_ps(sgn0, v), limit); /* +/- limit */
  __m128 sub0 = _mm_sub_ps(_mm_add_ps(v, or0), or0);
  __m128 cmp0 = _mm_cmplt_ps(glmm_abs(v), limit);
  return glmm_select(v, sub0, cmp0);
#endif
}

static inline __m128 glmm_floor(__m128 v) {
#if defined(__SSE4_1__)
  return _mm_floor_ps(v);
#else
  __m128 rnd0 = glmm_round(v);
  __m128 cmp0 = _mm_cmplt_ps(v, rnd0);
  __m128 and0 = _mm_and_ps(cmp0, glmm_set1(1.0f));
  return _mm_sub_ps(rnd0, and0);
#endif
}

static inline __m128 glmm_trunc(__m128 v) {
#if defined(__SSE4_1__)
  return _mm_round_ps(v, _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC);
#else
  __m128 limit = glmm_setbits1(CGLM_MASK_LIMT);
  __m128 cvt0 = _mm_cvtepi32_ps(_mm_cvttps_epi32(v));
  __m128 cmp0 = _mm_cmplt_ps(glmm_abs(v), limit);
  return glmm_select(v, cvt0, cmp0);
#endif
}

static inline __m128 glmm_ceil(__m128 v) {
#if defined(__SSE4_1__)
  return _mm_ceil_ps(v);
#else
  __m128 rnd0 = glmm_round(v);
  __m128 cmp0 = _mm_cmpgt_ps(v, rnd0);
  __m128 and0 = _mm_and_ps(cmp0, glmm_set1(1.0f));
  return _mm_add_ps(rnd0, and0);
#endif
}

static inline __m128 glmm_cross3(__m128 a, __m128 b) {
  __m128 a_yzx = glmm_shuff1(a, 3, 0, 2, 1);
  __m128 b_yzx = glmm_shuff1(b, 3, 0, 2, 1);
  __m128 sub0 = _mm_sub_ps(_mm_mul_ps(a, b_yzx), _mm_mul_ps(a_yzx, b));
  return glmm_shuff1(sub0, 3, 0, 2, 1);
}
#elif defined(CGLM_SIMD_ARM)
/* Masking */
#define glmm_setbits1(x) vreinterpretq_f32_u32(vdupq_n_u32(x))
#define glmm_maskop(x) vreinterpretq_f32_u32(x)
#define glmm_maskstore(p, a) glmm_store(p, glmm_maskop(vandq_u32(vreinterpretq_u32_f32(a), vreinterpretq_u32_f32(glmm_set1(1.0f)))))

/* Abstractions */
#define glmm_casti vreinterpretq_s32_f32
#define glmm_cvtt vcvtq_s32_f32
#define glmm_setzero() glmm_set1(0.0f)
#define glmm_add vaddq_f32
#define glmm_sub vsubq_f32
#define glmm_mul vmulq_f32
#define glmm_eq(x, y) glmm_maskop(vceqq_f32((x), (y)))
#define glmm_neq(x, y) glmm_maskop(vmvnq_u32(vceqq_f32((x), (y))))
#define glmm_lt(x, y) glmm_maskop(vcltq_f32((x), (y)))
#define glmm_le(x, y) glmm_maskop(vcleq_f32((x), (y)))
#define glmm_gt(x, y) glmm_maskop(vcgtq_f32((x), (y)))
#define glmm_ge(x, y) glmm_maskop(vcgeq_f32((x), (y)))
#define glmm_and(x, y) glmm_maskop(vandq_u32(vreinterpretq_u32_f32(x), vreinterpretq_u32_f32(y)))
#define glmm_andnot(x, y) glmm_maskop(vbicq_u32(vreinterpretq_u32_f32(y), vreinterpretq_u32_f32(x)))
#define glmm_or(x, y) glmm_maskop(vorrq_u32(vreinterpretq_u32_f32(x), vreinterpretq_u32_f32(y)))

/* Integer */
#define glmm_128i int32x4_t
#define glmm_icast vreinterpretq_f32_s32
#define glmm_iset1 vdupq_n_s32
#define glmm_isetzero() glmm_iset1(0)
#define glmm_icvt vcvtq_f32_s32
#define glmm_iadd vaddq_s32
#define glmm_isub vsubq_s32
#define glmm_ieq(x, y) glmm_maskop(vceqq_s32((x), (y)))
#define glmm_iand vandq_s32
#define glmm_iandnot(x, y) vbicq_s32((y), (x))
#define glmm_isll(x, imm) vshlq_s32((x), vdupq_n_s32((imm))) /* @TODO: handle imm edgecase */
#define glmm_isrl(x, imm) vshlq_s32((x), vdupq_n_s32(-(imm)))
#define glmm_iload vld1q_s32
#define glmm_istore vst1q_s32

static inline float32x4_t glmm_load2(vec2 v) {
  return vcombine_f32(vld1_f32(v), vdup_n_f32(0.0f));
}

static inline void glmm_store2(vec2 dest, float32x4_t v) {
  vst1_f32(dest, vget_low_f32(v));
}

#define glmm_load3u glmm_load3
static inline float32x4_t glmm_load3(vec3 v) {
  float32x2_t low = vld1_f32(v);
  float32x2_t high = vld1_lane_f32(v + 2, vdup_n_f32(0.0f), 0);
  return vcombine_f32(low, high);
}

#define glmm_store3u glmm_store3
static inline void glmm_store3(vec3 dest, float32x4_t v) {
  CGLM_ALIGN(16) vec4 v4;
  vst1q_f32(v4, v);
  glm_vec3(v4, dest);
}

static inline float32x4_t glmm_setbits(uint32_t w, uint32_t z, uint32_t y, uint32_t x) {
  CGLM_ALIGN(16) uint32_t data[4] = { x, y, z, w };
  return vcvtq_f32_u32(vld1q_u32(data));
}

static inline float32x4_t glmm_select(float32x4_t a, float32x4_t b, float32x4_t mask) {
  return vbslq_f32(vreinterpretq_u32_f32(mask), b, a);
}

static inline bool glmm_all(float32x4_t v) {
  uint32x4_t eq0 = vceqq_f32(v, glmm_set1(0.0f));
#if CGLM_ARM64
  uint32x4_t min0 = vpmaxq_u32(eq0, eq0);
  uint32x4_t min1 = vpmaxq_u32(min0, min0);
  return vgetq_lane_u32(min1, 0) == 0u;
#else
  uint32x2_t min0 = vpmax_u32(vget_low_u32(eq0), vget_high_u32(eq0));
  uint32x2_t min1 = vpmax_u32(min0, min0);
  return vget_lane_u32(min1, 0) == 0u;
#endif
}

static inline bool glmm_any(float32x4_t v) {
  uint32x4_t eq0 = vceqq_f32(v, glmm_set1(0.0f));
#if CGLM_ARM64
  uint32x4_t min0 = vpminq_u32(eq0, eq0);
  uint32x4_t min1 = vpminq_u32(min0, min0);
  return vgetq_lane_u32(min1, 0) == 0u;
#else
  uint32x2_t min0 = vpmin_u32(vget_low_u32(eq0), vget_high_u32(eq0));
  uint32x2_t min1 = vpmin_u32(min0, min0);
  return vget_lane_u32(min1, 0) == 0u;
#endif
}

static inline float32x4_t glmm_isnan(float32x4_t v) {
  return vreinterpretq_f32_u32(vmvnq_u32(vceqq_f32(v, v)));
}

static inline float32x4_t glmm_isinf(float32x4_t v) {
  uint32x4_t inf_mask = vdupq_n_u32(CGLM_MASK_PINF);
  uint32x4_t abs_mask = vdupq_n_u32(CGLM_MASK_ABS); /* avoid vabsq_f32 */
  uint32x4_t abs0 = vandq_u32(vreinterpretq_u32_f32(v), abs_mask);
  return vreinterpretq_f32_u32(vceqq_f32(vreinterpretq_f32_u32(abs0), vreinterpretq_f32_u32(inf_mask)));
}

static inline float32x4_t glmm_signbit(float32x4_t v) {
  uint32x4_t sgn0 = vdupq_n_u32(CGLM_MASK_SIGN);
  return vreinterpretq_f32_u32(vandq_u32(sgn0, vreinterpretq_u32_f32(v)));
}

static inline float32x4_t glmm_sign(float32x4_t v) {
  uint32x4_t mask = vcgeq_f32(v, glmm_setzero());
  return glmm_select(glmm_set1(-1.0f), glmm_set1(1.0f), glmm_maskop(mask));
}

static inline float32x4_t glmm_negate(float32x4_t v) {
  return vnegq_f32(v);
}

static inline float32x4_t glmm_approx(float32x4_t x, float32x4_t y, float32x4_t eps) {
  return vreinterpretq_f32_u32(vcaleq_f32(vsubq_f32(x, y), eps));
}

static inline float32x4_t glmm_vdot(float32x4_t a, float32x4_t b) {
#if CGLM_ARM64
  return vdupq_n_f32(vaddvq_f32(vmulq_f32(a, b)));
#else
  float32x4_t mul0 = vmulq_f32(a, b);
  float32x2_t add0 = vadd_f32(vget_low_f32(mul0), vget_high_f32(mul0));
  float32x2_t add1 = vpadd_f32(add0, add0);
  return vcombine_f32(add1, add1);
#endif
}

static inline float32x4_t glmm_vrsqrteq(float32x4_t v) {
  float32x4_t rsqrt0 = vrsqrteq_f32(v);
  uint32x4_t inf0 = vdupq_n_u32(CGLM_MASK_PINF);
  uint32x4_t zer0 = vceqq_u32(inf0, vreinterpretq_u32_f32(rsqrt0));
  uint32x4_t and0 = vandq_u32(vmvnq_u32(zer0), vreinterpretq_u32_f32(rsqrt0));
  return vreinterpretq_f32_u32(and0);
}

static inline float32x4_t glmm_invsqrt(float32x4_t v) {
#if CGLM_ARM64
  return vdivq_f32(glmm_set1(1.0f), vsqrtq_f32(v));
#else
  float32x4_t s = glmm_vrsqrteq(v);
  s = vmulq_f32(s, vrsqrtsq_f32(vmulq_f32(v, s), s));
  s = vmulq_f32(s, vrsqrtsq_f32(vmulq_f32(v, s), s));
  return s;
#endif
}

static inline float32x4_t glmm_sqrt(float32x4_t v) {
#if CGLM_ARM64
  return vsqrtq_f32(v);
#else
  return vmulq_f32(v, glmm_invsqrt(v));
#endif
}

static inline float32x4_t glmm_pmax(float32x4_t a, float32x4_t b) {
#if defined(CGLM_USE_FAST_MINMAX)
  return vmaxq_f32(a, b);
#else
  return vbslq_f32(vcgtq_f32(a, b), a, b);
#endif
}

static inline float32x4_t glmm_pmin(float32x4_t a, float32x4_t b) {
#if defined(CGLM_USE_FAST_MINMAX)
  return vminq_f32(a, b);
#else
  return vbslq_f32(vcltq_f32(a, b), a, b);
#endif
}

static inline float32x4_t glmm_clamp(float32x4_t v, float32x4_t minVal, float32x4_t maxVal) {
#if CGLM_ARM64
  return vminnmq_f32(vmaxnmq_f32(v, minVal), maxVal);
#else
  return vminq_f32(vmaxq_f32(v, minVal), maxVal);
#endif
}

static inline float32x4_t glmm_round(float32x4_t v) {
#if CGLM_ARM64
  return vrndnq_f32(v);
#else
  uint32x4_t sign = vdupq_n_u32(CGLM_MASK_SIGN);
  uint32x4_t limit = vdupq_n_u32(CGLM_MASK_LIMT);
  uint32x4_t or0 = vorrq_u32(vandq_u32(sign, vreinterpretq_u32_f32(v)), limit);
  uint32x4_t cmp0 = vcaltq_f32(v, vreinterpretq_f32_u32(limit));
  float32x4_t add0 = vaddq_f32(v, vreinterpretq_f32_u32(or0));
  float32x4_t sub0 = vsubq_f32(add0, vreinterpretq_f32_u32(or0));
  return vbslq_f32(cmp0, sub0, v);
#endif
}

static inline float32x4_t glmm_floor(float32x4_t v) {
#if CGLM_ARM64
  return vrndmq_f32(v);
#else
  float32x4_t rnd0 = glmm_round(v);
  uint32x4_t cmp0 = vcltq_f32(v, rnd0);
  uint32x4_t and0 = vandq_u32(cmp0, vreinterpretq_u32_f32(glmm_set1(1.0f)));
  return vsubq_f32(rnd0, vreinterpretq_f32_u32(and0));
#endif
}

static inline float32x4_t glmm_trunc(float32x4_t v) {
#if CGLM_ARM64
  return vrndq_f32(v);
#else
  uint32x4_t limit = vdupq_n_u32(CGLM_MASK_LIMT);
  uint32x4_t cmp0 = vcaltq_f32(v, vreinterpretq_f32_u32(limit));
  float32x4_t cvt0 = vcvtq_f32_s32(vcvtq_s32_f32(v));
  return vbslq_f32(cmp0, cvt0, v);
#endif
}

static inline float32x4_t glmm_ceil(float32x4_t v) {
#if CGLM_ARM64
  return vrndpq_f32(v);
#else
  float32x4_t rnd0 = glmm_round(v);
  uint32x4_t cmp0 = vcgtq_f32(v, rnd0);
  uint32x4_t and0 = vandq_u32(cmp0, vreinterpretq_u32_f32(glmm_set1(1.0f)));
  return vaddq_f32(rnd0, vreinterpretq_f32_u32(and0));
#endif
}
#elif defined(CGLM_SIMD_WASM)
/* Masking */
#define glmm_setbits1(x) wasm_i32x4_splat((int)x)
#define glmm_setbits(w, z, y, x) wasm_i32x4_make((int)x, (int)y, (int)z, (int)w)
#define glmm_maskstore(p, a) glmm_store(p, wasm_v128_and(a, glmm_set1(1.0f)))

/* Abstractions */
#define glmm_casti(x) (x)
#define glmm_cvtt wasm_i32x4_trunc_sat_f32x4
#define glmm_setzero() glmm_set1(0.0f)
#define glmm_add wasm_f32x4_add
#define glmm_sub wasm_f32x4_sub
#define glmm_mul wasm_f32x4_mul
#define glmm_div wasm_f32x4_div
#define glmm_sqrt wasm_f32x4_sqrt
#define glmm_ceil wasm_f32x4_ceil
#define glmm_floor wasm_f32x4_floor
#define glmm_round wasm_f32x4_nearest
#define glmm_trunc wasm_f32x4_trunc
#define glmm_eq wasm_f32x4_eq
#define glmm_neq wasm_f32x4_ne
#define glmm_lt wasm_f32x4_lt
#define glmm_le wasm_f32x4_le
#define glmm_gt wasm_f32x4_gt
#define glmm_ge wasm_f32x4_ge
#define glmm_and wasm_v128_and
#define glmm_andnot(x, y) wasm_v128_andnot((y), (x))
#define glmm_or wasm_v128_or
#define glmm_xor wasm_v128_xor
#define glmm_pmax(x, y) wasm_f32x4_pmax((y), (x))
#define glmm_pmin(x, y) wasm_f32x4_pmin((y), (x))

/* Integer */
#define glmm_128i v128_t
#define glmm_icast(x) (x)
#define glmm_iset1 wasm_i32x4_splat
#define glmm_isetzero() wasm_i64x2_const(0, 0)
#define glmm_icvt wasm_f32x4_convert_i32x4
#define glmm_iadd wasm_i32x4_add
#define glmm_isub wasm_i32x4_sub
#define glmm_ieq wasm_i32x4_eq
#define glmm_iand wasm_v128_and
#define glmm_iandnot(x, y) wasm_v128_andnot((y), (x))
#define glmm_isll wasm_i32x4_shl /* @TODO: handle imm edgecase */
#define glmm_isrl wasm_u32x4_shr

#define glmm_load2(p) wasm_v128_load64_zero((p))
#define glmm_store2(dest, p) wasm_v128_store64_lane((dest), (p), 0);
#define glmm_load3u glmm_load3
#define glmm_store3u glmm_store3

static inline v128_t glmm_select(v128_t a, v128_t b, v128_t mask) {
  return wasm_v128_bitselect(b, a, mask);
}

static inline bool glmm_all(v128_t v) {
  return wasm_i32x4_bitmask(wasm_f32x4_ne(v, glmm_setzero())) == 0x0F;
}

static inline bool glmm_any(v128_t v) {
  v128_t cmp0 = wasm_f32x4_ne(v, glmm_setzero());
  return wasm_i32x4_bitmask(cmp0) != 0;
}

static inline bool glmm_eqv(v128_t x, v128_t y) {
  return wasm_i32x4_bitmask(wasm_f32x4_eq(x, y)) == 0x0F;
}

static inline v128_t glmm_isnan(v128_t v) {
  return wasm_f32x4_ne(v, v);
}

static inline v128_t glmm_isinf(v128_t v) {
  return wasm_f32x4_eq(glmm_abs(v), glmm_setbits1(CGLM_MASK_PINF));
}

static inline v128_t glmm_signbit(v128_t v) {
  return wasm_v128_and(v, glmm_setbits1(CGLM_MASK_SIGN));
}

static inline v128_t glmm_sign(v128_t v) { /* 1.0 : -1.0 */
  return wasm_v128_or(glmm_signbit(v), glmm_set1(1.0f));
}

static inline v128_t glmm_negate(v128_t v) {
  return wasm_v128_xor(v, glmm_float32x4_SIGNMASK_NEG);
}

static inline v128_t glmm_approx(v128_t x, v128_t y, v128_t eps) {
  v128_t sub0 = wasm_f32x4_sub(x, y);
  v128_t neg0 = wasm_f32x4_sub(glmm_setzero(), sub0);
  return wasm_f32x4_le(wasm_f32x4_pmax(neg0, sub0), eps);
}

static inline v128_t glmm_clamp(v128_t v, v128_t minVal, v128_t maxVal) {
  return wasm_f32x4_pmin(wasm_f32x4_pmax(v, minVal), maxVal);
}

static inline v128_t glmm_invsqrt(v128_t v) {
  return wasm_f32x4_div(glmm_set1(1.0f), wasm_f32x4_sqrt(v));
}
#endif
/* }================================================================== */

/*
** {==================================================================
** trig.h: Constants pulled from cephes
** ===================================================================
*/
#if defined(CGLM_SIMD)
/*!
 * @brief sincos expansion helper
 */
static inline void glmm_sincos_poly(glmm_128 x, glmm_128 *sin, glmm_128 *cos) {
  glmm_128 x2 = glmm_mul(x, x);
  glmm_128 x3 = glmm_mul(x2, x);
  glmm_128 s0 = glmm_fmadd(x2, glmm_set1(-0.0001950727f), glmm_set1(0.0083320758f));
  glmm_128 c0 = glmm_fmadd(x2, glmm_set1(-0.0013602249f), glmm_set1(0.0416566950f));
  glmm_128 s1 = glmm_fmadd(x2, s0, glmm_set1(-0.1666665247f));
  glmm_128 c1 = glmm_fmadd(x2, c0, glmm_set1(-0.4999990225f));
  *sin = glmm_fmadd(x3, s1, x);
  *cos = glmm_fmadd(x2, c1, glmm_set1(1.0f));
}

/*!
 * @brief tan expansion helper
 */
static inline glmm_128 glmm_tan_poly(glmm_128 x) {
  glmm_128 x2 = glmm_mul(x, x);
  glmm_128 x3 = glmm_mul(x2, x);
  glmm_128 c0 = glmm_fmadd(glmm_set1(9.3854018554e-3f), x2, glmm_set1(3.1199223269e-3f));
  glmm_128 c1 = glmm_fmadd(c0, x2, glmm_set1(2.4430135452e-2f));
  glmm_128 c2 = glmm_fmadd(c1, x2, glmm_set1(5.3411280700e-2f));
  glmm_128 c3 = glmm_fmadd(c2, x2, glmm_set1(1.3338799408e-1f));
  glmm_128 c4 = glmm_fmadd(c3, x2, glmm_set1(3.3333156854e-1f));
  return glmm_fmadd(x3, c4, x);
}

/*!
 * @brief atan expansion helper
 */
static inline glmm_128 glmm_atan_poly(glmm_128 x) {
  glmm_128 x2 = glmm_mul(x, x);
  glmm_128 x3 = glmm_mul(x2, x);
  glmm_128 c0 = glmm_fmadd(x2, glmm_set1(8.05374449538e-2f), glmm_set1(-1.38776856032e-1f));
  glmm_128 c1 = glmm_fmadd(x2, c0, glmm_set1(1.99777106478e-1f));
  glmm_128 c2 = glmm_fmadd(x2, c1, glmm_set1(-3.33329491539e-1f));
  return glmm_fmadd(x3, c2, x);
}

/*!
 * @brief acos expansion helper
 */
static inline glmm_128 glmm_acos_poly(glmm_128 x) {
  glmm_128 x2 = glmm_mul(x, x);
  glmm_128 x4 = glmm_mul(x2, x2);
  glmm_128 l0 = glmm_fmadd(x, glmm_set1(-0.0501743046f), glmm_set1(0.0889789874f));
  glmm_128 h0 = glmm_fmadd(x, glmm_set1(-0.0012624911f), glmm_set1(0.0066700901f));
  glmm_128 l1 = glmm_fmadd(x, l0, glmm_set1(-0.2145988016f));
  glmm_128 h1 = glmm_fmadd(x, h0, glmm_set1(-0.0170881256f));
  glmm_128 l2 = glmm_fmadd(x, l1, glmm_set1(1.5707963050f));
  glmm_128 h2 = glmm_fmadd(x, h1, glmm_set1(0.0308918810f));
  return glmm_fmadd(h2, x4, l2);
}

/*!
 * @brief tanh expansion helper
 */
static inline glmm_128 glmm_tanh_poly(glmm_128 x) {
  glmm_128 p, q;
  glmm_128 x2 = glmm_mul(x, x);
  p = glmm_fmadd(x2, glmm_set1(-2.7607684774e-16f), glmm_set1(2.0001879048e-13f));
  p = glmm_fmadd(p, x2, glmm_set1(-8.6046715221e-11f));
  p = glmm_fmadd(p, x2, glmm_set1(5.1222970903e-8f));
  p = glmm_fmadd(p, x2, glmm_set1(1.4857223571e-5f));
  p = glmm_fmadd(p, x2, glmm_set1(6.3726192887e-4f));
  p = glmm_fmadd(p, x2, glmm_set1(4.8935245589e-3f));
  q = glmm_fmadd(x2, glmm_set1(1.1982583946e-6f), glmm_set1(1.1853470568e-4f));
  q = glmm_fmadd(q, x2, glmm_set1(2.2684346324e-3f));
  q = glmm_fmadd(q, x2, glmm_set1(4.8935245589e-3f));
  return glmm_div(glmm_mul(p, x), q);
}

/*!
 * @brief atanh expansion helper: [-0.5, 0.5]
 */
static inline glmm_128 glmm_atanh_poly(glmm_128 x) {
  glmm_128 x2 = glmm_mul(x, x);
  glmm_128 z = glmm_fmadd(glmm_set1(0.1819281280f), x2, glmm_set1(8.2311116158e-2f));
  z = glmm_fmadd(x2, z, glmm_set1(0.1467213183f));
  z = glmm_fmadd(x2, z, glmm_set1(0.1997792422f));
  z = glmm_fmadd(x2, z, glmm_set1(0.3333373963f));
  z = glmm_fmadd(glmm_mul(x, x2), z, x);
  return z;
}

/*!
 * @brief exp expansion helper
 */
static inline glmm_128 glmm_exp_poly(glmm_128 x) {
  glmm_128 x2 = glmm_mul(x, x);
  glmm_128 z = glmm_set1(1.9875691500e-4f);
  z = glmm_fmadd(z, x, glmm_set1(1.3981999507e-3f));
  z = glmm_fmadd(z, x, glmm_set1(8.3334519073e-3f));
  z = glmm_fmadd(z, x, glmm_set1(4.1665795894e-2f));
  z = glmm_fmadd(z, x, glmm_set1(1.6666665459e-1f));
  z = glmm_fmadd(z, x, glmm_set1(5.0000001201e-1f));
  z = glmm_fmadd(z, x2, x);
  return glmm_add(z, glmm_set1(1.0f));
}

/*!
 * @brief log expansion helper
 */
static inline glmm_128 glmm_log_poly(glmm_128 x) {
  glmm_128 x2 = glmm_mul(x, x);
  glmm_128 z = glmm_set1(7.0376836292e-2f);
  z = glmm_fmadd(z, x, glmm_set1(-1.1514610310e-1f));
  z = glmm_fmadd(z, x, glmm_set1(1.1676998740e-1f));
  z = glmm_fmadd(z, x, glmm_set1(-1.2420140846e-1f));
  z = glmm_fmadd(z, x, glmm_set1(1.4249322787e-1f));
  z = glmm_fmadd(z, x, glmm_set1(-1.6668057665e-1f));
  z = glmm_fmadd(z, x, glmm_set1(2.0000714765e-1f));
  z = glmm_fmadd(z, x, glmm_set1(-2.4999993993e-1f));
  z = glmm_fmadd(z, x, glmm_set1(3.3333331174e-1f));
  return glmm_mul(glmm_mul(x, z), x2);
}

/* @TODO: Inf/NaN handling */
static inline glmm_128 glmm_frexp(glmm_128 v, glmm_128 *e) {
  glmm_128 num;
  glmm_128 exp_mask = glmm_setbits1(CGLM_MASK_PINF);
  glmm_128 mant_mask = glmm_setbits1(CGLM_MASK_MANT);
  glmm_128 zero_mask = glmm_eq(v, glmm_setzero());

  glmm_128i iexp = glmm_casti(glmm_and(v, exp_mask));
  iexp = glmm_isub(glmm_isrl(iexp, 23), glmm_iset1(0x7E));
  iexp = glmm_iandnot(glmm_casti(zero_mask), iexp);
  *e = glmm_icvt(iexp);

  num = glmm_or(glmm_and(v, mant_mask), glmm_set1(0.5f));
  num = glmm_or(glmm_and(v, zero_mask), glmm_andnot(zero_mask, num));
  return num;
}

static inline void glmm_sincos(glmm_128 v, glmm_128 *outSin, glmm_128 *outCos) {
  glmm_128 sinMask, cosMask;
  glmm_128 sinx, cosx, sin, cos;
  glmm_128i zero = glmm_isetzero();

  /* Range reduction & quadrant; avoiding _mm_cvtps_epi32 */
  glmm_128 qf = glmm_round(glmm_mul(v, glmm_set1(GLM_2_PIf)));
  glmm_128i quad = glmm_cvtt(qf);
  glmm_128i sinOffset = glmm_iand(quad, glmm_iset1(3));
  glmm_128i cosOffset = glmm_iand(glmm_iadd(quad, glmm_iset1(1)), glmm_iset1(3));

  /* "Extended precision modular arithmetic" */
  glmm_128 fma0 = glmm_fmadd(qf, glmm_set1(-1.5703125f), v);
  glmm_128 fma1 = glmm_fmadd(qf, glmm_set1(-4.837512969970703e-4f), fma0);
  glmm_128 x = glmm_fmadd(qf, glmm_set1(-7.549789948768648e-8f), fma1);
  glmm_sincos_poly(x, &sinx, &cosx);

  /* Select cosine when the offset is odd, sin when even */
  sinMask = glmm_ieq(glmm_iand(sinOffset, glmm_iset1(1)), zero);
  cosMask = glmm_ieq(glmm_iand(cosOffset, glmm_iset1(1)), zero);
  sin = glmm_select(cosx, sinx, sinMask);
  cos = glmm_select(cosx, sinx, cosMask);

  /* Flip sign when the offset is 1 or 2 */
  sinMask = glmm_ieq(glmm_iand(sinOffset, glmm_iset1(2)), zero);
  cosMask = glmm_ieq(glmm_iand(cosOffset, glmm_iset1(2)), zero);
  *outSin = glmm_select(glmm_negate(sin), sin, sinMask);
  *outCos = glmm_select(glmm_negate(cos), cos, cosMask);
}

static inline glmm_128 glmm_tan(glmm_128 v) {
  /* Range reduction & quadrant */
  glmm_128 qf = glmm_round(glmm_mul(v, glmm_set1(GLM_2_PIf)));
  glmm_128i tanOffset = glmm_iand(glmm_cvtt(qf), glmm_iset1(3));
  glmm_128 tanMask = glmm_ieq(glmm_iand(tanOffset, glmm_iset1(1)), glmm_isetzero());

  /* "Extended precision modular arithmetic" */
  glmm_128 fma0 = glmm_fmadd(qf, glmm_set1(-1.5703125f), v);
  glmm_128 fma1 = glmm_fmadd(qf, glmm_set1(-4.837512969970703e-4f), fma0);
  glmm_128 x = glmm_fmadd(qf, glmm_set1(-7.549789948768648e-8f), fma1);
  glmm_128 tanx = glmm_tan_poly(x);

  /* Invert when the offset is even; avoid division-by-zero */
  glmm_128 cmp0 = glmm_eq(tanx, glmm_setzero());
  glmm_128 add0 = glmm_add(tanx, glmm_and(cmp0, glmm_set1(1.0f)));
  glmm_128 inv0 = glmm_div(glmm_set1(-1.0f), add0);
  return glmm_select(inv0, tanx, tanMask);
}

static inline glmm_128 glmm_acos(glmm_128 v) {
  /* Follow glm_acos(float) and sanitize domain */
  glmm_128 x = glmm_abs(glmm_clamp(v, glmm_set1(-1.0f), glmm_set1(1.0f)));
  glmm_128 t = glmm_sqrt(glmm_sub(glmm_set1(1.0f), x));
  glmm_128 pos = glmm_mul(t, glmm_acos_poly(x));
  glmm_128 neg = glmm_sub(glmm_set1(GLM_PIf), pos);
  return glmm_select(pos, neg, glmm_lt(v, glmm_setzero()));
}

static inline glmm_128 glmm_asin(glmm_128 v) {
  return glmm_sub(glmm_set1(GLM_PI_2f), glmm_acos(v));
}

static inline glmm_128 glmm_atan(glmm_128 v) {
  glmm_128 sign_bit = glmm_signbit(v);
  glmm_128 abs = glmm_xor(v, sign_bit);

  /* Range reduction */
  glmm_128 cmp0 = glmm_gt(abs, glmm_set1(2.414213562373095f));
  glmm_128 cmp1 = glmm_gt(abs, glmm_set1(0.414213562373095f));
  glmm_128 cmp2 = glmm_andnot(cmp0, cmp1); /* low < v <= high */

  /* (-1.0 / v); avoid division-by-zero */
  glmm_128 one = glmm_set1(1.0f);
  glmm_128 denom = glmm_add(abs, glmm_and(glmm_eq(abs, glmm_setzero()), one));
  glmm_128 x0 = glmm_negate(glmm_div(one, denom));
  glmm_128 y0 = glmm_and(cmp0, glmm_set1(GLM_PI_2f));

  /* (v - 1.0) / (v + 1.0) */
  glmm_128 x1 = glmm_div(glmm_sub(abs, one), glmm_add(abs, one));
  glmm_128 y1 = glmm_and(cmp2, glmm_set1(GLM_PI_4f));

  /* Blend result */
  glmm_128 y2 = glmm_or(y0, y1);
  glmm_128 x2 = glmm_or(glmm_and(cmp0, x0), glmm_and(cmp2, x1));
  glmm_128 x3 = glmm_select(abs, x2, glmm_or(cmp0, cmp2));
  glmm_128 atanx = glmm_atan_poly(x3);
  return glmm_or(sign_bit, glmm_add(y2, atanx));
}

static inline glmm_128 glmm_atan2(glmm_128 y, glmm_128 x) {
  glmm_128 result;
  glmm_128 zero = glmm_setzero();
  glmm_128 xeq0 = glmm_eq(x, zero);
  glmm_128 xge0 = glmm_ge(x, zero);
  glmm_128 xlt0 = glmm_lt(x, zero);
  glmm_128 yeq0 = glmm_eq(y, zero);
  glmm_128 ylt0 = glmm_lt(y, zero);

  /* if (x == 0 && y == 0) || (x >= 0 & y == 0) */
  glmm_128 degenerate = glmm_and(xge0, yeq0);

  /* if (x == 0 && y != 0) return +/- GLM_PI_2 */
  glmm_128 zero_mask = glmm_andnot(yeq0, xeq0);
  glmm_128 zero_sign = glmm_xor(glmm_signbit(ylt0), glmm_set1(GLM_PI_2f));
  glmm_128 zero_case = glmm_and(zero_sign, zero_mask);

  /* if (x < 0 && y == 0) return GLM_PI */
  glmm_128 nero_case = glmm_and(glmm_and(xlt0, yeq0), glmm_set1(GLM_PIf));

  /* if (x < 0 && y < 0) sign swap */
  glmm_128 shift_mask = glmm_signbit(glmm_and(xlt0, ylt0));
  glmm_128 shift = glmm_and(xlt0, glmm_xor(shift_mask, glmm_set1(GLM_PIf)));

  /* atan(y/x); avoiding division-by-zero  */
  glmm_128 cmp0 = glmm_eq(glmm_or(xeq0, yeq0), glmm_setzero());
  glmm_128 denom = glmm_add(x, glmm_and(xeq0, glmm_set1(1.0f)));
  glmm_128 atan = glmm_atan(glmm_div(y, denom));
  atan = glmm_andnot(zero_mask, glmm_add(atan, shift));
  atan = glmm_and(cmp0, atan); /* make zero-if-zero */

  /* Blend result */
  result = glmm_andnot(degenerate, zero_case);
  result = glmm_or(result, zero_case);
  result = glmm_or(result, nero_case);
  result = glmm_or(result, atan);
  return result;
}

/*!
 * Alternatives: https://stackoverflow.com/questions/47025373/fastest-implementation-of-the-natural-exponential-function-using-sse
 */
static inline glmm_128 glmm_exp(glmm_128 x) {
  glmm_128 n, exp;
  glmm_128i pow;
  x = glmm_clamp(x, glmm_set1(-88.3762626647949f), glmm_set1(88.3762626647949f));

  /* exp(x) = exp(g + n*log(2)) */
  n = glmm_floor(glmm_fmadd(x, glmm_set1(GLM_LOG2Ef), glmm_set1(0.5f)));
  x = glmm_fmadd(n, glmm_set1(-0.693359375f), x);
  x = glmm_fmadd(n, glmm_set1(2.12194440e-4f), x);
  exp = glmm_exp_poly(x);

  /* multiply by 2^n */
  pow = glmm_cvtt(n);
  pow = glmm_iadd(pow, glmm_iset1(0x7F));
  pow = glmm_isll(pow, 23);
  return glmm_mul(exp, glmm_icast(pow));
}

static inline glmm_128 glmm_exp2(glmm_128 x) {
  return glmm_exp(glmm_mul(glmm_set1(GLM_LN2f), x));
}

static inline glmm_128 glmm_log(glmm_128 v) {
  glmm_128 shift, exp, log;
  glmm_128 one = glmm_set1(1.0f);
  glmm_128 zero = glmm_setzero();
  glmm_128 ninf = glmm_setbits1(CGLM_MASK_NINF);
  glmm_128 pinf = glmm_setbits1(CGLM_MASK_PINF);

  /* Shift input */
  glmm_128 x = glmm_frexp(v, &exp);
  shift = glmm_lt(x, glmm_set1(GLM_SQRT1_2f));
  exp = glmm_sub(exp, glmm_and(one, shift));
  x = glmm_add(glmm_sub(x, one), glmm_and(x, shift));

  /* Interpolate and add */
  log = glmm_log_poly(x);
  log = glmm_fmadd(exp, glmm_set1(-2.12194440e-4f), log);
  log = glmm_fmadd(glmm_mul(x, x), glmm_set1(-0.5f), log);
  log = glmm_fmadd(exp, glmm_set1(0.693359375f), glmm_add(x, log));

  /* Blend result */
  log = glmm_select(log, pinf, glmm_eq(v, pinf)); /* +Inf */
  log = glmm_or(log, glmm_lt(v, zero)); /* NaN */
  log = glmm_select(log, ninf, glmm_eq(v, zero)); /* -Inf */
  return log;
}

static inline glmm_128 glmm_log2(glmm_128 x) {
  return glmm_mul(glmm_set1(GLM_LOG2Ef), glmm_log(x));
}

static inline glmm_128 glmm_sinh(glmm_128 v) {
  glmm_128 half = glmm_set1(0.5f);
  return glmm_mul(half, glmm_sub(glmm_exp(v), glmm_exp(glmm_negate(v))));
}

static inline glmm_128 glmm_cosh(glmm_128 v) {
  glmm_128 half = glmm_set1(0.5f);
  return glmm_mul(half, glmm_add(glmm_exp(v), glmm_exp(glmm_negate(v))));
}

static inline glmm_128 glmm_tanh(glmm_128 v) {
  return glmm_tanh_poly(glmm_clamp(v, glmm_set1(-9.0f), glmm_set1(9.0f)));
}

static inline glmm_128 glmm_asinh(glmm_128 v) {
  glmm_128 sqrt0 = glmm_sqrt(glmm_add(glmm_mul(v, v), glmm_set1(1.0f)));
  return glmm_log(glmm_add(v, sqrt0));
}

static inline glmm_128 glmm_acosh(glmm_128 v) {
  glmm_128 one = glmm_set1(1.0f);
  glmm_128 x = glmm_pmax(v, one);
  glmm_128 sqrt0 = glmm_sqrt(glmm_sub(x, one));
  glmm_128 sqrt1 = glmm_sqrt(glmm_add(x, one));
  return glmm_log(glmm_add(x, glmm_mul(sqrt0, sqrt1)));
}

static inline glmm_128 glmm_atanh(glmm_128 v) {
  glmm_128 one = glmm_set1(1.0f);
  glmm_128 half = glmm_set1(0.5f);

  /* Follow glm_atanh(float) and sanitize domain */
  glmm_128 x = glmm_clamp(v, glmm_set1(-1.0f), one);

  /* For |v| >= 1.0 return +/-inf */
  glmm_128 sgn0 = glmm_signbit(x);
  glmm_128 inf = glmm_or(sgn0, glmm_setbits1(CGLM_MASK_PINF));

  /* For [-0.5, 0.5] use polynomial */
  glmm_128 atanh = glmm_atanh_poly(x);

  /* Otherwise use 0.5 * log((1 + x) / (1 - x)); avoiding division-by-zero */
  glmm_128 add0 = glmm_add(one, x);
  glmm_128 sub0 = glmm_sub(one, glmm_and(glmm_neq(one, x), x));
  glmm_128 mul0 = glmm_mul(half, glmm_log(glmm_div(add0, sub0)));

  /* Blend result */
  glmm_128 abs0 = glmm_xor(v, sgn0);
  glmm_128 sel0 = glmm_select(mul0, atanh, glmm_le(abs0, half));
  return glmm_select(inf, sel0, glmm_lt(abs0, one));
}
#endif
/* }================================================================== */

/*
** {==================================================================
** vec2.h
** ===================================================================
*/

/*!
 * @brief check if any component is non-zero
 */
CGLM_INLINE bool glm_vec2_any(vec2 v) {
  return v[0] != 0.f || v[1] != 0.f;
}

/*!
 * @brief check if all components are non-zero
 */
CGLM_INLINE bool glm_vec2_all(vec2 v) {
  return v[0] != 0.f && v[1] != 0.f;
}

/*!
 * @brief test if the vector components are approximately equal
 */
CGLM_INLINE void glm_vec2_approx(vec2 a, vec2 b, float eps, vec2 dest) {
  dest[0] = fabsf(a[0] - b[0]) <= eps ? 1.f : 0.f;
  dest[1] = fabsf(a[1] - b[1]) <= eps ? 1.f : 0.f;
}

/*!
 * @brief see glm_vec3_angle
 */
CGLM_INLINE float glm_vec2_angle(vec2 a, vec2 b) {
  float norm = 1.0f / (glm_vec2_norm(a) * glm_vec2_norm(b));
  return glm_acos(norm * glm_vec2_dot(a, b));
}

/* }================================================================== */

/*
** {==================================================================
** vec3.h
** ===================================================================
*/

/*!
 * @brief check if any item is non-zero
 */
CGLM_INLINE bool glm_vec3_any(vec3 v) {
  return v[0] != 0.f || v[1] != 0.f || v[2] != 0.f;
}

/*!
 * @brief check if all components are non-zero
 */
CGLM_INLINE bool glm_vec3_all(vec3 v) {
  return v[0] != 0.f && v[1] != 0.f && v[2] != 0.f;
}

/*!
 * @brief test if the vector components are approximately equal (within epsilon)
 */
CGLM_INLINE void glm_vec3_approx(vec3 a, vec3 b, float eps, vec3 dest) {
  dest[0] = fabsf(a[0] - b[0]) <= eps ? 1.f : 0.f;
  dest[1] = fabsf(a[1] - b[1]) <= eps ? 1.f : 0.f;
  dest[2] = fabsf(a[2] - b[2]) <= eps ? 1.f : 0.f;
}

/*!
 * @brief interpolate between two vectors using spherical linear interpolation
 */
CGLM_INLINE void glm_vec3_slerp(vec3 from, vec3 to, float t, vec3 dest) {
  float cosAlpha = glm_vec3_dot(from, to);
  if (fabsf(cosAlpha) >= 1.f - GLM_FLT_EPSILON)
    glm_vec3_lerp(from, to, t, dest);
  else {
    vec3 v1, v2;
    float alpha = acosf(cosAlpha);
    float sinAlpha = sinf(alpha);
    float t1 = sinf((1.f - t) * alpha) / sinAlpha;
    float t2 = sinf(t * alpha) / sinAlpha;
    glm_vec3_scale(from, t1, v1);
    glm_vec3_scale(to, t2, v2);
    glm_vec3_add(v1, v2, dest);
  }
}

/*!
 * @brief glm_vec3_cross implemented using intrinsics (RH)
 */
CGLM_INLINE void glm_vec3_cross_simd(vec3 a, vec3 b, vec3 dest) {
#if defined(CGLM_SIMD_x86)
  glmm_store3(dest, glmm_cross3(glmm_load3u(a), glmm_load3u(b)));
#else
  glm_vec3_cross(a, b, dest);
#endif
}

/* }================================================================== */

/*
** {==================================================================
** vec4.h
** ===================================================================
*/

/*!
 * @brief glm_vec4_eqv using intrinsics
 */
CGLM_INLINE bool glm_vec4_eqv_simd(vec4 a, vec4 b) {
#if defined(CGLM_SIMD_x86) || defined(CGLM_SIMD_WASM)
  return glmm_eqv(glmm_load(a), glmm_load(b));
#else
  return glm_vec4_eqv(a, b);
#endif
}

/*!
 * @brief subtract a vector from scalar (s + -v)
 */
CGLM_INLINE void glm_vec4_ssub(float s, vec4 v, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_store(dest, glmm_sub(glmm_set1(s), glmm_load(v)));
#else
  dest[0] = s - v[0];
  dest[1] = s - v[1];
  dest[2] = s - v[2];
  dest[3] = s - v[3];
#endif
}

/*!
 * @brief divide a vector from scalar
 */
CGLM_INLINE void glm_vec4_sdiv(float s, vec4 v, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_store(dest, glmm_div(glmm_set1(s), glmm_load(v)));
#else
  CGLM_ALIGN(16) vec4 v0;
  glm_vec4_broadcast(s, v0);
  glm_vec4_div(v0, v, dest);
#endif
}

/* 8.1. Angle and Trigonometry Functions */

CGLM_INLINE void glm_vec4_rad(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_store(dest, glmm_mul(glmm_load(v), glmm_set1(0.017453292519943295f)));
#else
  dest[0] = glm_rad(v[0]);
  dest[1] = glm_rad(v[1]);
  dest[2] = glm_rad(v[2]);
  dest[3] = glm_rad(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_deg(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_store(dest, glmm_mul(glmm_load(v), glmm_set1(57.29577951308232f)));
#else
  dest[0] = glm_deg(v[0]);
  dest[1] = glm_deg(v[1]);
  dest[2] = glm_deg(v[2]);
  dest[3] = glm_deg(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_sin(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_128 sin, cos;
  glmm_sincos(glmm_load(v), &sin, &cos);
  glmm_store(dest, sin);
#else
  dest[0] = sinf(v[0]);
  dest[1] = sinf(v[1]);
  dest[2] = sinf(v[2]);
  dest[3] = sinf(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_cos(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_128 sin, cos;
  glmm_sincos(glmm_load(v), &sin, &cos);
  glmm_store(dest, cos);
#else
  dest[0] = cosf(v[0]);
  dest[1] = cosf(v[1]);
  dest[2] = cosf(v[2]);
  dest[3] = cosf(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_tan(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_store(dest, glmm_tan(glmm_load(v)));
#else
  dest[0] = tanf(v[0]);
  dest[1] = tanf(v[1]);
  dest[2] = tanf(v[2]);
  dest[3] = tanf(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_asin(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_store(dest, glmm_asin(glmm_load(v)));
#else
  dest[0] = glm_asin(v[0]);
  dest[1] = glm_asin(v[1]);
  dest[2] = glm_asin(v[2]);
  dest[3] = glm_asin(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_acos(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_store(dest, glmm_acos(glmm_load(v)));
#else
  dest[0] = glm_acos(v[0]);
  dest[1] = glm_acos(v[1]);
  dest[2] = glm_acos(v[2]);
  dest[3] = glm_acos(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_atan(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_store(dest, glmm_atan(glmm_load(v)));
#else
  dest[0] = atanf(v[0]);
  dest[1] = atanf(v[1]);
  dest[2] = atanf(v[2]);
  dest[3] = atanf(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_atan2(vec4 y, vec4 x, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_store(dest, glmm_atan2(glmm_load(y), glmm_load(x)));
#else
  dest[0] = atan2f(y[0], x[0]);
  dest[1] = atan2f(y[1], x[1]);
  dest[2] = atan2f(y[2], x[2]);
  dest[3] = atan2f(y[3], x[3]);
#endif
}

CGLM_INLINE void glm_vec4_sinh(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_store(dest, glmm_sinh(glmm_load(v)));
#else
  dest[0] = sinhf(v[0]);
  dest[1] = sinhf(v[1]);
  dest[2] = sinhf(v[2]);
  dest[3] = sinhf(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_cosh(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_store(dest, glmm_cosh(glmm_load(v)));
#else
  dest[0] = coshf(v[0]);
  dest[1] = coshf(v[1]);
  dest[2] = coshf(v[2]);
  dest[3] = coshf(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_tanh(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_store(dest, glmm_tanh(glmm_load(v)));
#else
  dest[0] = tanhf(v[0]);
  dest[1] = tanhf(v[1]);
  dest[2] = tanhf(v[2]);
  dest[3] = tanhf(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_asinh(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_store(dest, glmm_asinh(glmm_load(v)));
#else
  dest[0] = asinhf(v[0]);
  dest[1] = asinhf(v[1]);
  dest[2] = asinhf(v[2]);
  dest[3] = asinhf(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_acosh(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_store(dest, glmm_acosh(glmm_load(v)));
#else
  dest[0] = glm_acosh(v[0]);
  dest[1] = glm_acosh(v[1]);
  dest[2] = glm_acosh(v[2]);
  dest[3] = glm_acosh(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_atanh(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_store(dest, glmm_atanh(glmm_load(v)));
#else
  dest[0] = glm_atanh(v[0]);
  dest[1] = glm_atanh(v[1]);
  dest[2] = glm_atanh(v[2]);
  dest[3] = glm_atanh(v[3]);
#endif
}

/* 8.2. Exponential Functions */

CGLM_INLINE void glm_vec4_pow(vec4 a, vec4 b, vec4 dest) {
  dest[0] = powf(a[0], b[0]);
  dest[1] = powf(a[1], b[1]);
  dest[2] = powf(a[2], b[2]);
  dest[3] = powf(a[3], b[3]);
}

CGLM_INLINE void glm_vec4_pows(vec4 a, float s, vec4 dest) {
  CGLM_ALIGN(16) vec4 b;
  glm_vec4_broadcast(s, b);
  glm_vec4_pow(a, b, dest);
}

CGLM_INLINE void glm_vec4_spow(float s, vec4 b, vec4 dest) {
  CGLM_ALIGN(16) vec4 a;
  glm_vec4_broadcast(s, a);
  glm_vec4_pow(a, b, dest);
}

CGLM_INLINE void glm_vec4_exp(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_store(dest, glmm_exp(glmm_load(v)));
#else
  dest[0] = expf(v[0]);
  dest[1] = expf(v[1]);
  dest[2] = expf(v[2]);
  dest[3] = expf(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_log(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_store(dest, glmm_log(glmm_load(v)));
#else
  dest[0] = logf(v[0]);
  dest[1] = logf(v[1]);
  dest[2] = logf(v[2]);
  dest[3] = logf(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_exp2(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_store(dest, glmm_exp2(glmm_load(v)));
#else
  dest[0] = exp2f(v[0]);
  dest[1] = exp2f(v[1]);
  dest[2] = exp2f(v[2]);
  dest[3] = exp2f(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_log2(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD) && defined(CGLM_USE_APPROX)
  glmm_store(dest, glmm_log2(glmm_load(v)));
#else
  dest[0] = log2f(v[0]);
  dest[1] = log2f(v[1]);
  dest[2] = log2f(v[2]);
  dest[3] = log2f(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_logB(vec4 a, vec4 b, vec4 dest) {
  dest[0] = glm_log(a[0], b[0]);
  dest[1] = glm_log(a[1], b[1]);
  dest[2] = glm_log(a[2], b[2]);
  dest[3] = glm_log(a[3], b[3]);
}

CGLM_INLINE void glm_vec4_invsqrt(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_store(dest, glmm_invsqrt(glmm_load(v)));
#else
  CGLM_ALIGN(16) vec4 sqrt, one;
  glm_vec4_sqrt(v, sqrt);
  glm_vec4_broadcast(1.f, one);
  glm_vec4_div(one, sqrt, dest);
#endif
}

/* 8.3. Common Functions */

CGLM_INLINE void glm_vec4_floor(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_store(dest, glmm_floor(glmm_load(v)));
#else
  dest[0] = floorf(v[0]);
  dest[1] = floorf(v[1]);
  dest[2] = floorf(v[2]);
  dest[3] = floorf(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_trunc(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_store(dest, glmm_trunc(glmm_load(v)));
#else
  dest[0] = truncf(v[0]);
  dest[1] = truncf(v[1]);
  dest[2] = truncf(v[2]);
  dest[3] = truncf(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_round(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_store(dest, glmm_round(glmm_load(v)));
#else
  dest[0] = glm_roundf(v[0]);
  dest[1] = glm_roundf(v[1]);
  dest[2] = glm_roundf(v[2]);
  dest[3] = glm_roundf(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_ceil(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_store(dest, glmm_ceil(glmm_load(v)));
#else
  dest[0] = ceilf(v[0]);
  dest[1] = ceilf(v[1]);
  dest[2] = ceilf(v[2]);
  dest[3] = ceilf(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_fract_simd(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_128 x = glmm_load(v);
  glmm_store(dest, glmm_sub(x, glmm_floor(x)));
#else
  glm_vec4_fract(v, dest);
#endif
}

CGLM_INLINE void glm_vec4_mod(vec4 a, vec4 b, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_128 x = glmm_load(a);
  glmm_128 y = glmm_load(b);
  glmm_128 flr0 = glmm_floor(glmm_div(x, y));
  glmm_store(dest, glmm_fnmadd(flr0, y, x));
#else
  dest[0] = fmodf(a[0], b[0]);
  dest[1] = fmodf(a[1], b[1]);
  dest[2] = fmodf(a[2], b[2]);
  dest[3] = fmodf(a[3], b[3]);
#endif
}

CGLM_INLINE void glm_vec4_mods(vec4 a, float s, vec4 dest) {
  CGLM_ALIGN(16) vec4 b;
  glm_vec4_broadcast(s, b);
  glm_vec4_mod(a, b, dest);
}

CGLM_INLINE void glm_vec4_modf(vec4 v, vec4 integral, vec4 dest) {
  dest[0] = modff(v[0], &integral[0]);
  dest[1] = modff(v[1], &integral[1]);
  dest[2] = modff(v[2], &integral[2]);
  dest[3] = modff(v[3], &integral[3]);
}

CGLM_INLINE void glm_vec4_clamp_to(vec4 v, float minVal, float maxVal, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_128 cl0 = glmm_clamp(glmm_load(v), glmm_set1(minVal), glmm_set1(maxVal));
  glmm_store(dest, cl0);
#else
  glm_vec4_copy(v, dest);
  glm_vec4_clamp(dest, minVal, maxVal);
#endif
}

CGLM_INLINE void glm_vec4_clampv(vec4 v, vec4 minVal, vec4 maxVal, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_128 cl0 = glmm_clamp(glmm_load(v), glmm_load(minVal), glmm_load(maxVal));
  glmm_store(dest, cl0);
#else
  dest[0] = glm_clamp(v[0], minVal[0], maxVal[0]);
  dest[1] = glm_clamp(v[1], minVal[1], maxVal[1]);
  dest[2] = glm_clamp(v[2], minVal[2], maxVal[2]);
  dest[3] = glm_clamp(v[3], minVal[3], maxVal[3]);
#endif
}

CGLM_INLINE void glm_vec4_step_simd(vec4 edge, vec4 x, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_maskstore(dest, glmm_le(glmm_load(edge), glmm_load(x)));
#else
  glm_vec4_step(edge, x, dest);
#endif
}

CGLM_INLINE void glm_vec4_step_uni_simd(float edge, vec4 x, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_maskstore(dest, glmm_le(glmm_set1(edge), glmm_load(x)));
#else
  glm_vec4_step_uni(edge, x, dest);
#endif
}

CGLM_INLINE void glm_vec4_smoothstep_simd(vec4 edge0, vec4 edge1, vec4 x, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_128 e0 = glmm_load(edge0);
  glmm_128 e1 = glmm_load(edge1);
  glmm_128 x0 = glmm_load(x);
  glmm_128 div0 = glmm_div(glmm_sub(x0, e0), glmm_sub(e1, e0));
  glmm_128 t = glmm_clamp(div0, glmm_set1(0.0f), glmm_set1(1.0f));
  glmm_128 mul0 = glmm_mul(glmm_set1(2.0f), t);
  glmm_128 sub0 = glmm_sub(glmm_set1(3.0f), mul0);
  glmm_store(dest, glmm_mul(glmm_mul(t, t), sub0));
#else
  glm_vec4_smoothstep(edge0, edge1, x, dest);
#endif
}

CGLM_INLINE void glm_vec4_smoothstep_uni_simd(float edge0, float edge1, vec4 x, vec4 dest) {
#if defined(CGLM_SIMD)
  CGLM_ALIGN(16) vec4 e0, e1;
  glm_vec4_broadcast(edge0, e0);
  glm_vec4_broadcast(edge1, e1);
  glm_vec4_smoothstep_simd(e0, e1, x, dest);
#else
  glm_vec4_smoothstep_uni(edge0, edge1, x, dest);
#endif
}

CGLM_INLINE void glm_vec4_isnanv(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_maskstore(dest, glmm_isnan(glmm_load(v)));
#else
  dest[0] = (float)isnan(v[0]);
  dest[1] = (float)isnan(v[1]);
  dest[2] = (float)isnan(v[2]);
  dest[3] = (float)isnan(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_isinfv(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_maskstore(dest, glmm_isinf(glmm_load(v)));
#else
  dest[0] = (float)isinf(v[0]);
  dest[1] = (float)isinf(v[1]);
  dest[2] = (float)isinf(v[2]);
  dest[3] = (float)isinf(v[3]);
#endif
}

CGLM_INLINE void glm_vec4_frexp(vec4 a, ivec4 exp, vec4 dest) {
  dest[0] = frexpf(a[0], &exp[0]);
  dest[1] = frexpf(a[1], &exp[1]);
  dest[2] = frexpf(a[2], &exp[2]);
  dest[3] = frexpf(a[3], &exp[3]);
}

CGLM_INLINE void glm_vec4_ldexp(vec4 a, ivec4 b, vec4 dest) {
  dest[0] = ldexpf(a[0], b[0]);
  dest[1] = ldexpf(a[1], b[1]);
  dest[2] = ldexpf(a[2], b[2]);
  dest[3] = ldexpf(a[3], b[3]);
}

CGLM_INLINE void glm_vec4_fma(vec4 a, vec4 b, vec4 c, vec4 dest) {
  glm_vec4_copy(c, dest);
  glm_vec4_muladd(a, b, dest);
}

/* 8.5. Geometric Functions */

CGLM_INLINE void glm_vec4_faceforward(vec4 n, vec4 i, vec4 nref, vec4 dest) {
#if defined(CGLM_SIMD_x86) || defined(CGLM_SIMD_WASM)
  glmm_128 v = glmm_load(n);
  glmm_128 dot0 = glmm_vdot(glmm_load(nref), glmm_load(i));
  glmm_128 sgn0 = glmm_sign(dot0);
  glmm_store(dest, glmm_mul(v, glmm_mul(sgn0, glmm_set1(-1.0f))));
#else
  float dot = glm_vec4_dot(nref, i);
  dest[0] = dot < 0.f ? n[0] : -n[0];
  dest[1] = dot < 0.f ? n[1] : -n[1];
  dest[2] = dot < 0.f ? n[2] : -n[2];
  dest[3] = dot < 0.f ? n[3] : -n[3];
#endif
}

CGLM_INLINE void glm_vec4_reflect(vec4 i, vec4 n, vec4 dest) {
#if defined(CGLM_SIMD_x86) || defined(CGLM_SIMD_WASM)
  glmm_128 x = glmm_load(i);
  glmm_128 y = glmm_load(n);
  glmm_128 mul0 = glmm_mul(glmm_vdot(y, x), y);
  glmm_store(dest, glmm_fnmadd(glmm_set1(2.0f), mul0, x));
#else
  float dot = glm_vec4_dot(n, i);
  glm_vec4_scale(n, 2.0f * dot, dest);
  glm_vec4_sub(i, dest, dest);
#endif
}

CGLM_INLINE void glm_vec4_refract(vec4 i, vec4 n, float eta, vec4 dest) {
#if defined(CGLM_SIMD_x86) || defined(CGLM_SIMD_WASM)
  glmm_128 x = glmm_load(i);
  glmm_128 y = glmm_load(n);
  glmm_128 e = glmm_set1(eta);
  glmm_128 one = glmm_set1(1.0f);
  glmm_128 zero = glmm_setzero();
  glmm_128 dot0 = glmm_vdot(y, x);
  glmm_128 sub0 = glmm_fnmadd(dot0, dot0, one);
  glmm_128 k = glmm_fnmadd(glmm_mul(e, e), sub0, one);
  glmm_128 mad0 = glmm_fmadd(e, dot0, glmm_sqrt(glmm_abs(k)));
  glmm_128 mad1 = glmm_fnmadd(mad0, y, glmm_mul(e, x));
  glmm_store(dest, glmm_select(zero, mad1, glmm_ge(k, zero)));
#else
  float dot = glm_vec4_dot(n, i);
  float k = 1.f - eta * eta * (1.f - dot * dot);
  if (k < 0.0f)
    glm_vec4_zero(dest);
  else {
    float s = eta * dot + sqrtf(k);
    dest[0] = eta * i[0] - s * n[0];
    dest[1] = eta * i[1] - s * n[1];
    dest[2] = eta * i[2] - s * n[2];
    dest[3] = eta * i[3] - s * n[3];
  }
#endif
}

/* 8.7. Vector Relational Functions */

CGLM_INLINE void glm_vec4_eqto(vec4 a, vec4 b, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_maskstore(dest, glmm_eq(glmm_load(a), glmm_load(b)));
#else
  dest[0] = (float)(a[0] == b[0]);
  dest[1] = (float)(a[1] == b[1]);
  dest[2] = (float)(a[2] == b[2]);
  dest[3] = (float)(a[3] == b[3]);
#endif
}

CGLM_INLINE void glm_vec4_neqto(vec4 a, vec4 b, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_maskstore(dest, glmm_neq(glmm_load(a), glmm_load(b)));
#else
  dest[0] = (float)(a[0] != b[0]);
  dest[1] = (float)(a[1] != b[1]);
  dest[2] = (float)(a[2] != b[2]);
  dest[3] = (float)(a[3] != b[3]);
#endif
}

CGLM_INLINE void glm_vec4_approx(vec4 a, vec4 b, float eps, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_maskstore(dest, glmm_approx(glmm_load(a), glmm_load(b), glmm_set1(eps)));
#else
  dest[0] = fabsf(a[0] - b[0]) <= eps ? 1.f : 0.f;
  dest[1] = fabsf(a[1] - b[1]) <= eps ? 1.f : 0.f;
  dest[2] = fabsf(a[2] - b[2]) <= eps ? 1.f : 0.f;
  dest[3] = fabsf(a[3] - b[3]) <= eps ? 1.f : 0.f;
#endif
}

CGLM_INLINE void glm_vec4_lessthan(vec4 a, vec4 b, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_maskstore(dest, glmm_lt(glmm_load(a), glmm_load(b)));
#else
  dest[0] = (float)(a[0] < b[0]);
  dest[1] = (float)(a[1] < b[1]);
  dest[2] = (float)(a[2] < b[2]);
  dest[3] = (float)(a[3] < b[3]);
#endif
}

CGLM_INLINE void glm_vec4_lessthaneq(vec4 a, vec4 b, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_maskstore(dest, glmm_le(glmm_load(a), glmm_load(b)));
#else
  dest[0] = (float)(a[0] <= b[0]);
  dest[1] = (float)(a[1] <= b[1]);
  dest[2] = (float)(a[2] <= b[2]);
  dest[3] = (float)(a[3] <= b[3]);
#endif
}

CGLM_INLINE void glm_vec4_greaterthan(vec4 a, vec4 b, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_maskstore(dest, glmm_gt(glmm_load(a), glmm_load(b)));
#else
  dest[0] = (float)(a[0] > b[0]);
  dest[1] = (float)(a[1] > b[1]);
  dest[2] = (float)(a[2] > b[2]);
  dest[3] = (float)(a[3] > b[3]);
#endif
}

CGLM_INLINE void glm_vec4_greaterthaneq(vec4 a, vec4 b, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_maskstore(dest, glmm_ge(glmm_load(a), glmm_load(b)));
#else
  dest[0] = (float)(a[0] >= b[0]);
  dest[1] = (float)(a[1] >= b[1]);
  dest[2] = (float)(a[2] >= b[2]);
  dest[3] = (float)(a[3] >= b[3]);
#endif
}

CGLM_INLINE void glm_vec4_not(vec4 v, vec4 dest) {
#if defined(CGLM_SIMD)
  glmm_maskstore(dest, glmm_eq(glmm_load(v), glmm_setzero()));
#else
  dest[0] = (float)(v[0] == 0.f);
  dest[1] = (float)(v[1] == 0.f);
  dest[2] = (float)(v[2] == 0.f);
  dest[3] = (float)(v[3] == 0.f);
#endif
}

CGLM_INLINE bool glm_vec4_any(vec4 v) {
#if defined(CGLM_SIMD_x86)
  return glmm_any(glmm_load(v));
#else
  return v[0] != 0.f || v[1] != 0.f || v[2] != 0.f || v[3] != 0.f;
#endif
}

CGLM_INLINE bool glm_vec4_all(vec4 v) {
#if defined(CGLM_SIMD_x86)
  return glmm_all(glmm_load(v));
#else
  return v[0] != 0.f && v[1] != 0.f && v[2] != 0.f && v[3] != 0.f;
#endif
}

CGLM_INLINE void glm_vec4_mixv(vec4 from, vec4 to, vec4 t, vec4 dest) {
  CGLM_ALIGN(16) vec4 v; /* from + s * (to - from) */
  glm_vec4_sub(to, from, v);
  glm_vec4_mul(t, v, v);
  glm_vec4_add(from, v, dest);
}

/* }================================================================== */

/*
** {==================================================================
** quat.h
** ===================================================================
*/

/*!
 * @brief glm_quat_from_vecs without the 'unit length' requirement
 */
CGLM_INLINE void glm_quat_from_to(vec3 a, vec3 b, versor dest) {
  vec3 axis;
  float norm_ab = sqrtf(glm_vec3_dot(a, a) * glm_vec3_dot(b, b));
  float real_part = norm_ab + glm_vec3_dot(a, b);
  if (real_part >= (GLM_FLT_EPSILON * norm_ab))
    glm_vec3_cross(a, b, axis);
  else {
    glm_vec3_ortho(a, axis); /* use arbitrary orthogonal axis */
    real_part = 0.0f;
  }
  glm_quat_init(dest, axis[0], axis[1], axis[2], real_part);
  glm_quat_normalize(dest);
}

/*!
 * @brief initialize a quaternion from three base unit vectors
 */
CGLM_INLINE void glm_quat_from_basis(vec3 x, vec3 y, vec3 z, versor dest) {
  CGLM_ALIGN_MAT mat3 m;
  glm_vec3_copy(x, m[0]);
  glm_vec3_copy(y, m[1]);
  glm_vec3_copy(z, m[2]);
  glm_mat3_quat(m, dest);
}

/*!
 * @brief convert quaternion to mat4 (@NOTE: Does not implicitly normalize 'v')
 */
CGLM_INLINE void glm_quat_mat4_simd(versor v, mat4 dest) {
#if defined(CGLM_SIMD_x86) && !defined(CGLM_SIMD_WASM)
  __m128 q = glmm_load(v);
  __m128 q2 = _mm_add_ps(q, q);
  __m128 q_yzxw = glmm_shuff1(q, 3, 0, 2, 1);
  __m128 q_zxyw = glmm_shuff1(q, 3, 1, 0, 2);
  __m128 q_wwww = glmm_shuff1(q, 3, 3, 3, 3);
  __m128 q2_yzxw = glmm_shuff1(q2, 3, 0, 2, 1);
  __m128 q2_zxyw = glmm_shuff1(q2, 3, 1, 0, 2);

  __m128 sub0 = glmm_fnmadd(q_yzxw, q2_yzxw, glmm_set1(1.0f));
  __m128 tmp0 = glmm_fnmadd(q_zxyw, q2_zxyw, sub0);
  __m128 tmp1 = glmm_fmadd(q2, q_zxyw, _mm_mul_ps(q2_yzxw, q_wwww));
  __m128 tmp2 = glmm_fnmadd(q2_zxyw, q_wwww, _mm_mul_ps(q2, q_yzxw));
  __m128 fp1 = glmm_setbits(0, 0, 0xFFFFFFFF, 0);
  __m128 fp2 = glmm_setbits(0, 0xFFFFFFFF, 0, 0);
  __m128 nfp3 = glmm_setbits(0, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF);
  tmp0 = _mm_and_ps(tmp0, nfp3); /* Ensure last row is zeroed (improve this) */
  tmp1 = _mm_and_ps(tmp1, nfp3);
  tmp2 = _mm_and_ps(tmp2, nfp3);

  #define q_select(X, Y, Z) glmm_select(glmm_select((X), (Y), fp1), (Z), fp2)
  glmm_store(dest[0], q_select(tmp0, tmp1, tmp2)); /* 1 - 2yy - 2zz, 2zw + 2xy, 2xz - 2yw, 0 */
  glmm_store(dest[1], q_select(tmp2, tmp0, tmp1)); /* 2xy - 2zw, 1 - 2zz - 2xx, 2xw + 2yz, 0 */
  glmm_store(dest[2], q_select(tmp1, tmp2, tmp0)); /* 2yw + 2zx, 2yz - 2xw, 1 - 2xx - 2yy, 0 */
  glmm_store(dest[3], _mm_set_ps(1.0f, 0.0f, 0.0f, 0.0f));
  #undef q_select
#else
  glm_quat_mat4(v, dest);
#endif
}

/*!
 * @brief conjugate of quaternion
 */
CGLM_INLINE void glm_quat_conjugate_simd(versor q, versor dest) {
#if defined(CGLM_SIMD_x86) || defined(CGLM_SIMD_WASM)
  glmm_128 mask = glmm_setbits(0, CGLM_MASK_SIGN, CGLM_MASK_SIGN, CGLM_MASK_SIGN);
  glmm_store(dest, glmm_xor(glmm_load(q), mask));
#else
  dest[0] = -q[0];
  dest[1] = -q[1];
  dest[2] = -q[2];
  dest[3] =  q[3];
#endif
}

/*!
 * @brief inverse of a non-zero quaternion
 */
CGLM_INLINE void glm_quat_inv_simd(versor q, versor dest) {
#if defined(CGLM_SIMD)
  CGLM_ALIGN(16) versor conj;
  glm_quat_conjugate_simd(q, conj);
  glm_vec4_scale(conj, 1.0f / glm_vec4_norm2(q), dest);
#else
  glm_quat_inv(q, dest);
#endif
}

/*!
 * @brief divide two quaternions
 */
CGLM_INLINE void glm_quat_div(versor p, versor q, versor dest) {
  CGLM_ALIGN(16) versor inv;
  glm_quat_inv_simd(q, inv);
  glm_quat_mul(p, inv, dest);
}

/*!
 * @brief rotate vec3 by a quaternion
 */
CGLM_INLINE void glm_quat_rotatev_simd(versor a, vec3 b, vec3 dest) {
#if defined(CGLM_SIMD_x86)
  glmm_128 q = glmm_load(a);
  glmm_128 v = glmm_load3u(b);
  glmm_128 cr0 = glmm_cross3(q, v);
  glmm_128 cr1 = glmm_cross3(q, cr0);
  glmm_128 fm0 = glmm_fmadd(cr0, glmm_splat_w(q), cr1);
  glmm_128 fm1 = glmm_fmadd(fm0, glmm_set1(2.0f), v);
  glmm_store3(dest, fm1);
#else
  glm_quat_rotatev(a, b, dest);
#endif
}

/*!
 * @brief rotate vector by a quaternion
 */
CGLM_INLINE void glm_quat_rotatev4_simd(versor q, vec4 v, vec4 dest) {
  CGLM_ALIGN(16) vec3 v3, r0;
  glm_vec3(v, v3);
  glm_vec3_zero(r0);
  glm_quat_rotatev_simd(q, v3, r0);
  glm_vec4(r0, v[3], dest);
}

/*!
 * @brief rotate a vector by using a quaternion
 */
CGLM_INLINE void glm_vec3_rotateq_simd(vec3 v, versor q, vec3 dest) {
  CGLM_ALIGN(16) versor inv;
  glm_quat_inv_simd(q, inv);
  glm_quat_rotatev_simd(inv, v, dest);
}

/*!
 * @brief rotate vector using a quaternion
 */
CGLM_INLINE void glm_vec4_rotateq_simd(vec4 v, versor q, vec4 dest) {
  CGLM_ALIGN(16) versor inv;
  glm_quat_inv_simd(q, inv);
  glm_quat_rotatev4_simd(inv, v, dest);
}

/*!
 * @brief quaternion raised to natural logarithm
 */
CGLM_INLINE void glm_quat_exp(versor q, versor dest) {
  float imlen = glm_quat_imaglen(q);
  if (imlen < GLM_FLT_EPSILON)
    glm_quat_identity(dest);
  else {
    float e = expf(q[3]);
    float s = e / imlen * sinf(imlen);
    glm_quat_init(dest, q[0] * s, q[1] * s, q[2] * s, e * cosf(imlen));
  }
}

/*!
 * @brief natural logarithm of a quaternion
 */
CGLM_INLINE void glm_quat_log(versor q, versor dest) {
  float imlen = glm_quat_imaglen(q);
  if (imlen < GLM_FLT_EPSILON) {
    if (q[3] > 0.0f)
      glm_quat_init(dest, 0.0f, 0.0f, 0.0f, logf(q[3]));
    else if (q[3] < 0.0f) /* selects an arbitrary quaternion */
      glm_quat_init(dest, GLM_PIf, 0.0f, 0.0f, logf(-q[3]));
    else
      glm_quat_identity(dest);
  }
  else {
    float e = imlen * imlen + q[3] * q[3];
    float s = atan2f(imlen, q[3]) / imlen;
    glm_quat_init(dest, q[0] * s, q[1] * s, q[2] * s, 0.5f * logf(e));
  }
}

/*!
 * @brief quaternion raised to a power
 */
CGLM_INLINE void glm_quat_pow(versor q, float s, versor dest) {
  if (glm_eq(s, 0.0f)) /* Raising to the power of zero should yield 1 */
    glm_quat_identity(dest);
  else {
    CGLM_ALIGN(16) versor log, scale;
    glm_quat_log(q, log);
    glm_vec4_scale(log, s, scale);
    glm_quat_exp(scale, dest);
  }
}

/*!
 * @brief square root of quaternion
 */
CGLM_INLINE void glm_quat_sqrt(versor q, versor dest) {
  float len = glm_quat_norm(q);
  if (glm_eq(len + q[3], 0.0f))
    glm_quat_identity(dest);
  else {
    float c = sqrtf(0.5f / (len + q[3]));
    glm_quat_init(dest, q[0] * c, q[1] * c, q[2] * c, (len + q[3]) * c);
  }
}

/*!
 * @brief inverse square root of a quaternion
 */
CGLM_INLINE void glm_quat_invsqrt(versor q, versor dest) {
  CGLM_ALIGN(16) versor sqrt;
  glm_quat_sqrt(q, sqrt);
  glm_quat_inv_simd(sqrt, dest);
}

/* }================================================================== */

/*
** {==================================================================
** ivec4.h
** ===================================================================
*/

#define glm_castS2U(i) ((unsigned int)(i))
#define glm_castU2S(i) ((int)(i))
#define glm_intop(op, v1, v2) glm_castU2S(glm_castS2U(v1) op glm_castS2U(v2))

CGLM_INLINE void glm_ivec4_broadcast(int val, ivec4 d) {
#if defined(CGLM_SIMD_x86) || defined(CGLM_SIMD_ARM)
  glmm_istore(d, glmm_iset1(val));
#else
  d[0] = d[1] = d[2] = d[3] = val;
#endif
}

CGLM_INLINE void glm_ivec4f(vec4 v, ivec4 dest) {
  dest[0] = (int)v[0];
  dest[1] = (int)v[1];
  dest[2] = (int)v[2];
  dest[3] = (int)v[3];
}

CGLM_INLINE void glm_vec4i(ivec4 v, vec4 dest) {
  dest[0] = (float)v[0];
  dest[1] = (float)v[1];
  dest[2] = (float)v[2];
  dest[3] = (float)v[3];
}

CGLM_INLINE void glm_ivec4_bnot(ivec4 v, ivec4 dest) {
  dest[0] = glm_intop(^, ~glm_castS2U(0), v[0]);
  dest[1] = glm_intop(^, ~glm_castS2U(0), v[1]);
  dest[2] = glm_intop(^, ~glm_castS2U(0), v[2]);
  dest[3] = glm_intop(^, ~glm_castS2U(0), v[3]);
}

CGLM_INLINE void glm_ivec4_band(ivec4 x, ivec4 y, ivec4 dest) {
  dest[0] = glm_intop(&, x[0], y[0]);
  dest[1] = glm_intop(&, x[1], y[1]);
  dest[2] = glm_intop(&, x[2], y[2]);
  dest[3] = glm_intop(&, x[3], y[3]);
}

CGLM_INLINE void glm_ivec4_bor(ivec4 x, ivec4 y, ivec4 dest) {
  dest[0] = glm_intop(|, x[0], y[0]);
  dest[1] = glm_intop(|, x[1], y[1]);
  dest[2] = glm_intop(|, x[2], y[2]);
  dest[3] = glm_intop(|, x[3], y[3]);
}

CGLM_INLINE void glm_ivec4_bxor(ivec4 x, ivec4 y, ivec4 dest) {
  dest[0] = glm_intop(^, x[0], y[0]);
  dest[1] = glm_intop(^, x[1], y[1]);
  dest[2] = glm_intop(^, x[2], y[2]);
  dest[3] = glm_intop(^, x[3], y[3]);
}

CGLM_INLINE void glm_ivec4_shl(ivec4 x, ivec4 y, ivec4 dest) {
  dest[0] = glm_intop(<<, x[0], y[0]);
  dest[1] = glm_intop(<<, x[1], y[1]);
  dest[2] = glm_intop(<<, x[2], y[2]);
  dest[3] = glm_intop(<<, x[3], y[3]);
}

CGLM_INLINE void glm_ivec4_shr(ivec4 x, ivec4 y, ivec4 dest) {
  dest[0] = glm_intop(>>, x[0], y[0]);
  dest[1] = glm_intop(>>, x[1], y[1]);
  dest[2] = glm_intop(>>, x[2], y[2]);
  dest[3] = glm_intop(>>, x[3], y[3]);
}

/* }================================================================== */

/*
** {==================================================================
** mat2.h
** ===================================================================
*/

/*!
 * @brief check if two matrices are identical
 */
CGLM_INLINE bool glm_mat2_eq(mat2 m1, mat2 m2) {
#if defined(CGLM_SIMD_x86) || defined(CGLM_SIMD_WASM)
  return glmm_eqv(glmm_load(m1[0]), glmm_load(m2[0]));
#else
  return glm_vec2_eqv(m1[0], m2[0]) && glm_vec2_eqv(m1[1], m2[1]);
#endif
}

/*!
 * @brief check if matrix components are identical
 */
CGLM_INLINE void glm_mat2_eqv(mat2 m1, mat2 m2, vec2 dest) {
  dest[0] = (float)glm_vec2_eqv(m1[0], m2[0]);
  dest[1] = (float)glm_vec2_eqv(m1[1], m2[1]);
}

/*!
 * @brief check if matrix components are different
 */
CGLM_INLINE void glm_mat2_neqv(mat2 m1, mat2 m2, vec2 dest) {
  dest[0] = (float)!glm_vec2_eqv(m1[0], m2[0]);
  dest[1] = (float)!glm_vec2_eqv(m1[1], m2[1]);
}

/*!
 * @brief check if two matrices are approximately equal (with epsilon)
 */
CGLM_INLINE void glm_mat2_approx(mat2 m1, mat2 m2, float eps, vec2 dest) {
  mat2 result;
  glm_vec2_approx(m1[0], m2[0], eps, result[0]);
  glm_vec2_approx(m1[1], m2[1], eps, result[1]);
  dest[0] = glm_vec2_all(result[0]);
  dest[1] = glm_vec2_all(result[1]);
}

/*!
 * @brief add a vector to each matrix component
 */
CGLM_INLINE void glm_mat2_addv(mat2 m, vec2 v, mat2 dest) {
#if defined(CGLM_SIMD_x86)
  glmm_store(dest[0], glmm_add(glmm_load(m[0]), glmm_load2h(v)));
#else
  glm_vec2_add(m[0], v, dest[0]);
  glm_vec2_add(m[1], v, dest[1]);
#endif
}

/*!
 * @brief subtract the vector from each matrix component
 */
CGLM_INLINE void glm_mat2_subv(mat2 m, vec2 v, mat2 dest) {
#if defined(CGLM_SIMD_x86)
  glmm_store(dest[0], glmm_sub(glmm_load(m[0]), glmm_load2h(v)));
#else
  glm_vec2_sub(m[0], v, dest[0]);
  glm_vec2_sub(m[1], v, dest[1]);
#endif
}

/*!
 * @brief subtract a matrix from a vector: dest = v + -m
 */
CGLM_INLINE void glm_mat2_vsub(vec2 v, mat2 m, mat2 dest) {
#if defined(CGLM_SIMD_x86)
  glmm_store(dest[0], glmm_sub(glmm_load2h(v), glmm_load(m[0])));
#else
  glm_vec2_sub(v, m[0], dest[0]);
  glm_vec2_sub(v, m[1], dest[1]);
#endif
}

/*!
 * @brief transpose multiply a vec2 with a mat2
 */
CGLM_INLINE void glm_mat2_vmul(vec2 v, mat2 m, vec2 dest) {
  vec2 res;
  res[0] = m[0][0] * v[0] + m[0][1] * v[1];
  res[1] = m[1][0] * v[0] + m[1][1] * v[1];
  glm_vec2_copy(res, dest);
}

/*!
 * @brief transpose divide a vec2 with a mat2
 */
CGLM_INLINE void glm_mat2_vdiv(vec2 v, mat2 m, vec2 dest) {
  mat2 inv;
  glm_mat2_inv(m, inv);
  glm_mat2_vmul(v, inv, dest);
}

/*!
 * @brief divide two matrices
 */
CGLM_INLINE void glm_mat2_div(mat2 m1, mat2 m2, mat2 dest) {
  mat2 inv;
  glm_mat2_inv(m2, inv);
  glm_mat2_mul(m1, inv, dest);
}

/*!
 * @brief divide mat2 with vec2 (column vector)
 */
CGLM_INLINE void glm_mat2_divv(mat2 m, vec2 v, vec2 dest) {
  mat2 inv;
  glm_mat2_inv(m, inv);
  glm_mat2_mulv(inv, v, dest);
}

/*!
 * @brief Compute the outer product of a column and row vector
 */
CGLM_INLINE void glm_mat2_outerproduct(vec2 c, vec2 r, mat2 dest) {
  glm_vec2_scale(c, r[0], dest[0]);
  glm_vec2_scale(c, r[1], dest[1]);
}

/* }================================================================== */

/*
** {==================================================================
** mat3.h
** ===================================================================
*/

/*!
 * @brief check if two matrices are identical
 */
CGLM_INLINE bool glm_mat3_eq(mat3 m1, mat3 m2) {
  return glm_vec3_eqv(m1[0], m2[0])
         && glm_vec3_eqv(m1[1], m2[1])
         && glm_vec3_eqv(m1[2], m2[2]);
}

/*!
 * @brief check if matrix components are identical
 */
CGLM_INLINE void glm_mat3_eqv(mat3 m1, mat3 m2, vec3 dest) {
  dest[0] = (float)glm_vec3_eqv(m1[0], m2[0]);
  dest[1] = (float)glm_vec3_eqv(m1[1], m2[1]);
  dest[2] = (float)glm_vec3_eqv(m1[2], m2[2]);
}

/*!
 * @brief check if matrix components are different
 */
CGLM_INLINE void glm_mat3_neqv(mat3 m1, mat3 m2, vec3 dest) {
  dest[0] = (float)!glm_vec3_eqv(m1[0], m2[0]);
  dest[1] = (float)!glm_vec3_eqv(m1[1], m2[1]);
  dest[2] = (float)!glm_vec3_eqv(m1[2], m2[2]);
}

/*!
 * @brief check if two matrices are approximately equal (with epsilon)
 */
CGLM_INLINE void glm_mat3_approx(mat3 m1, mat3 m2, float eps, vec3 dest) {
  CGLM_ALIGN_MAT mat3 result;
  glm_vec3_approx(m1[0], m2[0], eps, result[0]);
  glm_vec3_approx(m1[1], m2[1], eps, result[1]);
  glm_vec3_approx(m1[2], m2[2], eps, result[2]);
  dest[0] = glm_vec3_all(result[0]);
  dest[1] = glm_vec3_all(result[1]);
  dest[2] = glm_vec3_all(result[2]);
}

/*!
 * @brief add a vector to each matrix component
 */
CGLM_INLINE void glm_mat3_addv(mat3 m, vec3 v, mat3 dest) {
  glm_vec3_add(m[0], v, dest[0]);
  glm_vec3_add(m[1], v, dest[1]);
  glm_vec3_add(m[2], v, dest[2]);
}

/*!
 * @brief subtract the vector from each matrix component
 */
CGLM_INLINE void glm_mat3_subv(mat3 m, vec3 v, mat3 dest) {
  glm_vec3_sub(m[0], v, dest[0]);
  glm_vec3_sub(m[1], v, dest[1]);
  glm_vec3_sub(m[2], v, dest[2]);
}

/*!
 * @brief subtract a matrix from a vector: dest = v + -m
 */
CGLM_INLINE void glm_mat3_vsub(vec3 v, mat3 m, mat3 dest) {
  glm_vec3_sub(v, m[0], dest[0]);
  glm_vec3_sub(v, m[1], dest[1]);
  glm_vec3_sub(v, m[2], dest[2]);
}

/*!
 * @brief transpose multiply mat3 with vec3 (column vector)
 */
CGLM_INLINE void glm_mat3_vmul(vec3 v, mat3 m, vec3 dest) {
  vec3 res;
  res[0] = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2];
  res[1] = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2];
  res[2] = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2];
  glm_vec3_copy(res, dest);
}

/*!
 * @brief transpose divide mat3 with vec3 (column vector)
 */
CGLM_INLINE void glm_mat3_vdiv(vec3 v, mat3 m, vec3 dest) {
  CGLM_ALIGN_MAT mat3 inv;
  glm_mat3_inv(m, inv);
  glm_mat3_vmul(v, inv, dest);
}

/*!
 * @brief divide two matrices
 */
CGLM_INLINE void glm_mat3_div(mat3 m1, mat3 m2, mat3 dest) {
  CGLM_ALIGN_MAT mat3 inv;
  glm_mat3_inv(m2, inv);
  glm_mat3_mul(m1, inv, dest);
}

/*!
 * @brief divide mat3 by vec3 (column vector)
 */
CGLM_INLINE void glm_mat3_divv(mat3 m, vec3 v, vec3 dest) {
  CGLM_ALIGN_MAT mat3 inv;
  glm_mat3_inv(m, inv);
  glm_mat3_mulv(inv, v, dest);
}

/*!
 * @brief Compute the outer product of a column and row vector
 */
CGLM_INLINE void glm_mat3_outerproduct(vec3 c, vec3 r, mat3 dest) {
  glm_vec3_scale(c, r[0], dest[0]);
  glm_vec3_scale(c, r[1], dest[1]);
  glm_vec3_scale(c, r[2], dest[2]);
}

/* }================================================================== */

/*
** {==================================================================
** mat4.h
** ===================================================================
*/

/*!
 * @brief create a matrix by populating its diagonal m[X][X] with a scalar value
 *        If the value is 1.0, then the identity is created.
 */
CGLM_INLINE void glm_mat4_diag(mat4 mat, float s) {
#if defined(CGLM_SIMD_x86)
  __m128 set0 = _mm_set_ps(0.0f, 0.0f, 0.0f, s);
  glmm_store(mat[0], set0);
  glmm_store(mat[1], glmm_shuff1(set0, 1, 1, 0, 1));
  glmm_store(mat[2], glmm_shuff1(set0, 1, 0, 1, 1));
  glmm_store(mat[3], glmm_shuff1(set0, 0, 1, 1, 1));
#else
  mat4 t = { { s, 0.0f, 0.0f, 0.0f },
             { 0.0f, s, 0.0f, 0.0f },
             { 0.0f, 0.0f, s, 0.0f },
             { 0.0f, 0.0f, 0.0f, s } };
  glm_mat4_copy(t, mat);
#endif
}

/*!
 * @brief copy upper-left of mat4 to mat2
 */
CGLM_INLINE void glm_mat4_pick2(mat4 mat, mat2 dest) {
  dest[0][0] = mat[0][0];
  dest[0][1] = mat[0][1];
  dest[1][0] = mat[1][0];
  dest[1][1] = mat[1][1];
}

/*!
 * @brief copy mat2 to mat4's upper-left
 */
CGLM_INLINE void glm_mat4_ins2(mat2 mat, mat4 dest) {
  dest[0][0] = mat[0][0];
  dest[0][1] = mat[0][1];
  dest[1][0] = mat[1][0];
  dest[1][1] = mat[1][1];
}

/*!
 * @brief check if two matrices are identical
 */
CGLM_INLINE bool glm_mat4_eq(mat4 m1, mat4 m2) {
#if defined(CGLM_SIMD_x86) && defined(__AVX__)
  __m256 cmp0 = _mm256_cmp_ps(glmm_load256(m1[0]), glmm_load256(m2[0]), _CMP_EQ_OQ);
  __m256 cmp1 = _mm256_cmp_ps(glmm_load256(m1[2]), glmm_load256(m2[2]), _CMP_EQ_OQ);
  return _mm256_movemask_ps(cmp0) == 0xFF && _mm256_movemask_ps(cmp1) == 0xFF;
#else
  return glm_vec4_eqv_simd(m1[0], m2[0])
         && glm_vec4_eqv_simd(m1[1], m2[1])
         && glm_vec4_eqv_simd(m1[2], m2[2])
         && glm_vec4_eqv_simd(m1[3], m2[3]);
#endif
}

/*!
 * @brief check if matrix components are identical
 */
CGLM_INLINE void glm_mat4_eqv(mat4 m1, mat4 m2, vec4 dest) {
  dest[0] = (float)glm_vec4_eqv_simd(m1[0], m2[0]);
  dest[1] = (float)glm_vec4_eqv_simd(m1[1], m2[1]);
  dest[2] = (float)glm_vec4_eqv_simd(m1[2], m2[2]);
  dest[3] = (float)glm_vec4_eqv_simd(m1[3], m2[3]);
}

/*!
 * @brief check if matrix components are different
 */
CGLM_INLINE void glm_mat4_neqv(mat4 m1, mat4 m2, vec4 dest) {
  dest[0] = (float)!glm_vec4_eqv_simd(m1[0], m2[0]);
  dest[1] = (float)!glm_vec4_eqv_simd(m1[1], m2[1]);
  dest[2] = (float)!glm_vec4_eqv_simd(m1[2], m2[2]);
  dest[3] = (float)!glm_vec4_eqv_simd(m1[3], m2[3]);
}

/*!
 * @brief check if two matrices are approximately equal (with epsilon)
 */
CGLM_INLINE void glm_mat4_approx(mat4 m1, mat4 m2, float eps, vec4 dest) {
#if defined(CGLM_SIMD_x86) || defined(CGLM_SIMD_WASM)
  glmm_128 e = glmm_set1(eps);
  dest[0] = glmm_all(glmm_approx(glmm_load(m1[0]), glmm_load(m2[0]), e));
  dest[1] = glmm_all(glmm_approx(glmm_load(m1[1]), glmm_load(m2[1]), e));
  dest[2] = glmm_all(glmm_approx(glmm_load(m1[2]), glmm_load(m2[2]), e));
  dest[3] = glmm_all(glmm_approx(glmm_load(m1[3]), glmm_load(m2[3]), e));
#else
  mat4 result;
  glm_vec4_approx(m1[0], m2[0], eps, result[0]);
  glm_vec4_approx(m1[1], m2[1], eps, result[1]);
  glm_vec4_approx(m1[2], m2[2], eps, result[2]);
  glm_vec4_approx(m1[3], m2[3], eps, result[3]);
  dest[0] = glm_vec4_all(result[0]);
  dest[1] = glm_vec4_all(result[1]);
  dest[2] = glm_vec4_all(result[2]);
  dest[3] = glm_vec4_all(result[3]);
#endif
}

/*!
 * @brief negate columns of matrix
 */
CGLM_INLINE void glm_mat4_negate_to(mat4 mat, mat4 dest) {
#if defined(CGLM_SIMD_x86) && defined(__AVX__)
  __m256 set0 = _mm256_castsi256_ps(_mm256_set1_epi32(GLMM_NEGZEROf));
  glmm_store256(dest[0], _mm256_xor_ps(glmm_load256(mat[0]), set0));
  glmm_store256(dest[2], _mm256_xor_ps(glmm_load256(mat[2]), set0));
#else
  glm_vec4_negate_to(mat[0], dest[0]);
  glm_vec4_negate_to(mat[1], dest[1]);
  glm_vec4_negate_to(mat[2], dest[2]);
  glm_vec4_negate_to(mat[3], dest[3]);
#endif
}

/*!
 * @brief add two matrices
 */
CGLM_INLINE void glm_mat4_add(mat4 m1, mat4 m2, mat4 dest) {
#if defined(CGLM_SIMD_x86) && defined(__AVX__)
  glmm_store256(dest[0], _mm256_add_ps(glmm_load256(m1[0]), glmm_load256(m2[0])));
  glmm_store256(dest[2], _mm256_add_ps(glmm_load256(m1[2]), glmm_load256(m2[2])));
#else
  glm_vec4_add(m1[0], m2[0], dest[0]);
  glm_vec4_add(m1[1], m2[1], dest[1]);
  glm_vec4_add(m1[2], m2[2], dest[2]);
  glm_vec4_add(m1[3], m2[3], dest[3]);
#endif
}

/*!
 * @brief add a vector to each matrix component
 */
CGLM_INLINE void glm_mat4_addv(mat4 m, vec4 v, mat4 dest) {
#if defined(CGLM_SIMD_x86) && defined(__AVX__)
  __m256 set0 = _mm256_set_m128(glmm_load(v), glmm_load(v));
  glmm_store256(dest[0], _mm256_add_ps(glmm_load256(m[0]), set0));
  glmm_store256(dest[2], _mm256_add_ps(glmm_load256(m[2]), set0));
#else
  glm_vec4_add(m[0], v, dest[0]);
  glm_vec4_add(m[1], v, dest[1]);
  glm_vec4_add(m[2], v, dest[2]);
  glm_vec4_add(m[3], v, dest[3]);
#endif
}

/*!
 * @brief add scalar to a matrix
 */
CGLM_INLINE void glm_mat4_adds(mat4 m, float s, mat4 dest) {
#if defined(CGLM_SIMD_x86) && defined(__AVX__)
  __m256 set0 = _mm256_set1_ps(s);
  glmm_store256(dest[0], _mm256_add_ps(glmm_load256(m[0]), set0));
  glmm_store256(dest[2], _mm256_add_ps(glmm_load256(m[2]), set0));
#else
  glm_vec4_adds(m[0], s, dest[0]);
  glm_vec4_adds(m[1], s, dest[1]);
  glm_vec4_adds(m[2], s, dest[2]);
  glm_vec4_adds(m[3], s, dest[3]);
#endif
}

/*!
 * @brief subtract two matrices
 */
CGLM_INLINE void glm_mat4_sub(mat4 m1, mat4 m2, mat4 dest) {
#if defined(CGLM_SIMD_x86) && defined(__AVX__)
  glmm_store256(dest[0], _mm256_sub_ps(glmm_load256(m1[0]), glmm_load256(m2[0])));
  glmm_store256(dest[2], _mm256_sub_ps(glmm_load256(m1[2]), glmm_load256(m2[2])));
#else
  glm_vec4_sub(m1[0], m2[0], dest[0]);
  glm_vec4_sub(m1[1], m2[1], dest[1]);
  glm_vec4_sub(m1[2], m2[2], dest[2]);
  glm_vec4_sub(m1[3], m2[3], dest[3]);
#endif
}

/*!
 * @brief subtract the vector from each matrix component
 */
CGLM_INLINE void glm_mat4_subv(mat4 m, vec4 v, mat4 dest) {
#if defined(CGLM_SIMD_x86) && defined(__AVX__)
  __m256 set0 = _mm256_set_m128(glmm_load(v), glmm_load(v));
  glmm_store256(dest[0], _mm256_sub_ps(glmm_load256(m[0]), set0));
  glmm_store256(dest[2], _mm256_sub_ps(glmm_load256(m[2]), set0));
#else
  glm_vec4_sub(m[0], v, dest[0]);
  glm_vec4_sub(m[1], v, dest[1]);
  glm_vec4_sub(m[2], v, dest[2]);
  glm_vec4_sub(m[3], v, dest[3]);
#endif
}

/*!
 * @brief subtract a scalar from each matrix component
 */
CGLM_INLINE void glm_mat4_subs(mat4 m, float s, mat4 dest) {
  glm_mat4_adds(m, -s, dest);
}

/*!
 * @brief subtract a matrix from scalar (s + -m)
 */
CGLM_INLINE void glm_mat4_ssub(float s, mat4 m, mat4 dest) {
#if defined(CGLM_SIMD_x86) && defined(__AVX__)
  __m256 set0 = _mm256_set1_ps(s);
  glmm_store256(dest[0], _mm256_sub_ps(set0, glmm_load256(m[0])));
  glmm_store256(dest[2], _mm256_sub_ps(set0, glmm_load256(m[2])));
#else
  glm_vec4_ssub(s, m[0], dest[0]);
  glm_vec4_ssub(s, m[1], dest[1]);
  glm_vec4_ssub(s, m[2], dest[2]);
  glm_vec4_ssub(s, m[3], dest[3]);
#endif
}

/*!
 * @brief subtract a matrix from a vector: dest = v + -m
 */
CGLM_INLINE void glm_mat4_vsub(vec4 v, mat4 m, mat4 dest) {
#if defined(CGLM_SIMD_x86) && defined(__AVX__)
  __m256 set0 = _mm256_set_m128(glmm_load(v), glmm_load(v));
  glmm_store256(dest[0], _mm256_sub_ps(set0, glmm_load256(m[0])));
  glmm_store256(dest[2], _mm256_sub_ps(set0, glmm_load256(m[2])));
#else
  glm_vec4_sub(v, m[0], dest[0]);
  glm_vec4_sub(v, m[1], dest[1]);
  glm_vec4_sub(v, m[2], dest[2]);
  glm_vec4_sub(v, m[3], dest[3]);
#endif
}

/*!
 * @brief scale a matrix (multiply by scalar)
 */
CGLM_INLINE void glm_mat4_scale_to(mat4 m, float s, mat4 dest) {
  glm_mat4_copy(m, dest);
  glm_mat4_scale(dest, s);
}

/*!
 * @brief inverse scale a matrix (divide by scalar)
 */
CGLM_INLINE void glm_mat4_scale_inv(mat4 m, float s, mat4 dest) {
  if (s != 0.f)
    glm_mat4_scale_to(m, 1.0f / s, dest);
  else {
    glm_vec4_broadcast(INFINITY, dest[0]);
    glm_vec4_broadcast(INFINITY, dest[1]);
    glm_vec4_broadcast(INFINITY, dest[2]);
    glm_vec4_broadcast(INFINITY, dest[3]);
  }
}

/*!
 * @brief transpose multiply mat4 with vec4 (column vector)
 */
CGLM_INLINE void glm_mat4_vmul(vec4 v, mat4 m, vec4 dest) {
#if defined(CGLM_SIMD)
  mat4 t;
  glm_mat4_transpose_to(m, t);
  glm_mat4_mulv(t, v, dest);
#else
  CGLM_ALIGN(16) vec4 res;
  res[0] = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2] + m[0][3] * v[3];
  res[1] = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2] + m[1][3] * v[3];
  res[2] = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2] + m[2][3] * v[3];
  res[3] = m[3][0] * v[0] + m[3][1] * v[1] + m[3][2] * v[2] + m[3][3] * v[3];
  glm_vec4_copy(res, dest);
#endif
}

/*!
 * @brief transpose divide mat4 with vec4 (column vector)
 */
CGLM_INLINE void glm_mat4_vdiv(vec4 v, mat4 m, vec4 dest) {
  mat4 inv;
  glm_mat4_inv(m, inv);
  glm_mat4_vmul(v, inv, dest);
}

/*!
 * @brief divide m1 by m2
 */
CGLM_INLINE void glm_mat4_div(mat4 m1, mat4 m2, mat4 dest) {
  mat4 inv;
  glm_mat4_inv(m2, inv);
  glm_mat4_mul(m1, inv, dest);
}

/*!
 * @brief divide a mat4 by a vec4 (column vector)
 */
CGLM_INLINE void glm_mat4_divv(mat4 m, vec4 v, vec4 dest) {
  mat4 inv;
  glm_mat4_inv(m, inv);
  glm_mat4_mulv(inv, v, dest);
}

/*!
 * @brief divide a scalar by matrix
 */
CGLM_INLINE void glm_mat4_sdiv(float s, mat4 m, mat4 dest) {
#if defined(CGLM_SIMD_x86) && defined(__AVX__)
  __m256 set0 = _mm256_set1_ps(s);
  glmm_store256(dest[0], _mm256_div_ps(set0, glmm_load256(m[0])));
  glmm_store256(dest[2], _mm256_div_ps(set0, glmm_load256(m[2])));
#else
  glm_vec4_sdiv(s, m[0], dest[0]);
  glm_vec4_sdiv(s, m[1], dest[1]);
  glm_vec4_sdiv(s, m[2], dest[2]);
  glm_vec4_sdiv(s, m[3], dest[3]);
#endif
}

/*!
 * @brief multiply two matrices component-wise.
 */
CGLM_INLINE void glm_mat4_compmult(mat4 m1, mat4 m2, mat4 dest) {
#if defined(CGLM_SIMD_x86) && defined(__AVX__)
  glmm_store256(dest[0], _mm256_mul_ps(glmm_load256(m1[0]), glmm_load256(m2[0])));
  glmm_store256(dest[2], _mm256_mul_ps(glmm_load256(m1[2]), glmm_load256(m2[2])));
#else
  glm_vec4_mul(m1[0], m2[0], dest[0]);
  glm_vec4_mul(m1[1], m2[1], dest[1]);
  glm_vec4_mul(m1[2], m2[2], dest[2]);
  glm_vec4_mul(m1[3], m2[3], dest[3]);
#endif
}

/*!
 * @brief Treats the first parameter c as a column vector and the second
 *        parameter r as a row vector and does a linear algebraic matrix
 *        multiply c * r.
 */
CGLM_INLINE void glm_mat4_outerproduct(vec4 c, vec4 r, mat4 dest) {
  glm_vec4_scale(c, r[0], dest[0]);
  glm_vec4_scale(c, r[1], dest[1]);
  glm_vec4_scale(c, r[2], dest[2]);
  glm_vec4_scale(c, r[3], dest[3]);
}

/*!
 * @brief glm_rotate_make using intrinsics
 */
CGLM_INLINE void glm_rotate_make_simd(mat4 m, float angle, vec3 axis_) {
#if defined(CGLM_SIMD_x86) && !defined(CGLM_SIMD_WASM)
  float sin_a = sinf(angle);
  float cos_a = cosf(angle);
  __m128 sinA = _mm_set_ps1(sin_a);
  __m128 cosA = _mm_set_ps1(cos_a);
  __m128 ncosA = _mm_set_ps1(1.0f - cos_a);

  __m128 axis = glmm_normalize(glmm_load3u(axis_));
  __m128 axis_yzxw = glmm_shuff1(axis, 3, 0, 2, 1);
  __m128 axis_zxyw = glmm_shuff1(axis, 3, 1, 0, 2);
  __m128 v = _mm_mul_ps(_mm_mul_ps(ncosA, axis_yzxw), axis_zxyw); /* v */
  __m128 c0 = glmm_fmadd(_mm_mul_ps(ncosA, axis), axis, cosA); /* += c */
  __m128 c1 = glmm_fmadd(sinA, axis, v); /* += vs */
  __m128 c2 = glmm_fnmadd(sinA, axis, v); /* -= vs */

  __m128 d0 = _mm_and_ps(c0, glmm_setbits(0, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF));
  __m128 d1 = glmm_shuff2(c1, c2, 2, 1, 2, 0, 0, 3, 2, 1);
  __m128 d2 = glmm_shuff2(c1, c2, 0, 0, 1, 1, 2, 0, 2, 0);
  glmm_store(m[0], glmm_shuff2(d0, d1, 1, 0, 3, 0, 1, 3, 2, 0));
  glmm_store(m[1], glmm_shuff2(d0, d1, 3, 2, 3, 1, 1, 3, 0, 2));
  glmm_store(m[2], _mm_shuffle_ps(d2, d0, _MM_SHUFFLE(3, 2, 1, 0)));
  glmm_store(m[3], _mm_set_ps(1.0f, 0.0f, 0.0f, 0.0f));
#else
  glm_rotate_make(m, angle, axis_);
#endif
}

/*!
 * @brief create an affine transformation from translation, rotation, and
 *        scaling components
 */
CGLM_INLINE void glm_compose_trs(vec3 t, versor q, vec3 s, mat4 dest) {
  mat4 m;
  glm_quat_mat4(q, m);
  glm_vec4_scale(m[0], s[0], dest[0]);
  glm_vec4_scale(m[1], s[1], dest[1]);
  glm_vec4_scale(m[2], s[2], dest[2]);
  glm_vec4(t, 1.f, dest[3]);
}

/*!
 * @brief Create a spherical billboard that rotates around the specified object
 *        position (LH)
 */
CGLM_INLINE void glm_billboard_lh(vec3 obj_pos, vec3 eye, vec3 up, vec3 fwd, mat4 dest) {
  float norm;
  CGLM_ALIGN(16) vec3 f, u, s;

  glm_vec3_sub(eye, obj_pos, f);
  norm = glm_vec3_norm(f);
  if (norm == 0.0f)
    glm_vec3_negate_to(fwd, f);
  else
    glm_vec3_scale(f, 1.0f / norm, f);
  glm_vec3_crossn(up, f, s);
  glm_vec3_cross(f, s, u);

  glm_vec4(s, 0.0f, dest[0]);
  glm_vec4(u, 0.0f, dest[1]);
  glm_vec4(f, 0.0f, dest[2]);
  glm_vec4(obj_pos, 1.0f, dest[3]);
}

/*!
 * @brief Create a spherical billboard that rotates around the specified object
 *        position (RH)
 */
CGLM_INLINE void glm_billboard_rh(vec3 obj_pos, vec3 eye, vec3 up, vec3 fwd, mat4 dest) {
  float norm;
  CGLM_ALIGN(16) vec3 f, u, s;

  glm_vec3_sub(obj_pos, eye, f);
  norm = glm_vec3_norm(f);
  if (norm == 0.0f)
    glm_vec3_negate_to(fwd, f);
  else
    glm_vec3_scale(f, 1.0f / norm, f);
  glm_vec3_crossn(up, f, s);
  glm_vec3_cross(f, s, u);

  glm_vec4(s, 0.0f, dest[0]);
  glm_vec4(u, 0.0f, dest[1]);
  glm_vec4(f, 0.0f, dest[2]);
  glm_vec4(obj_pos, 1.0f, dest[3]);
}

CGLM_INLINE void glm_billboard(vec3 obj_pos, vec3 eye, vec3 up, vec3 fwd, mat4 dest) {
#if CGLM_CONFIG_CLIP_CONTROL & CGLM_CLIP_CONTROL_LH_BIT
  glm_billboard_lh(obj_pos, eye, up, fwd, dest);
#elif CGLM_CONFIG_CLIP_CONTROL & CGLM_CLIP_CONTROL_RH_BIT
  glm_billboard_rh(obj_pos, eye, up, fwd, dest);
#endif
}

/* }================================================================== */

/*
** {==================================================================
** string_cast.{h,inl}
** ===================================================================
*/

/* Formatting helpers */
#define glm_cast(x) (double)(x)
#if defined(__STDC_WANT_SECURE_LIB__) && __STDC_WANT_SECURE_LIB__
  #define glm_sprintf sprintf_s
#else
  #define glm_sprintf snprintf
#endif

/*!
 * @brief format the argument as a string
 *
 * @return The number of characters that would have been written if n had been
 *         sufficiently large, not counting the terminating null character.
 */

CGLM_INLINE int glm_vec2_str(char *buff, size_t size, vec3 v) {
  return glm_sprintf(buff, size, "vec2(%f, %f)",
    glm_cast(v[0]), glm_cast(v[1])
  );
}

CGLM_INLINE int glm_vec3_str(char *buff, size_t size, vec3 v) {
  return glm_sprintf(buff, size, "vec3(%f, %f, %f)",
    glm_cast(v[0]), glm_cast(v[1]), glm_cast(v[2])
  );
}

CGLM_INLINE int glm_vec4_str(char *buff, size_t size, vec4 v) {
  return glm_sprintf(buff, size, "vec4(%f, %f, %f, %f)",
    glm_cast(v[0]), glm_cast(v[1]), glm_cast(v[2]), glm_cast(v[3])
  );
}

CGLM_INLINE int glm_quat_str(char *buff, size_t size, versor q) {
  return glm_sprintf(buff, size, "quat(%f, {%f, %f, %f})",
    glm_cast(q[3]), glm_cast(q[0]), glm_cast(q[1]), glm_cast(q[2])
  );
}

/*!
 * @brief format the argument as a string
 *
 * @return The number of characters that would have been written if n had been
 *         sufficiently large, not counting the terminating null character.
 */

CGLM_INLINE int glm_mat2_str(char *buff, size_t size, mat2 m) {
  return glm_sprintf(buff, size, "mat2x2((%f, %f), (%f, %f))",
    glm_cast(m[0][0]), glm_cast(m[0][1]),
    glm_cast(m[1][0]), glm_cast(m[1][1])
  );
}

CGLM_INLINE int glm_mat3_str(char *buff, size_t size, mat3 m) {
  return glm_sprintf(buff, size, "mat3x3((%f, %f, %f), (%f, %f, %f), (%f, %f, %f))",
    glm_cast(m[0][0]), glm_cast(m[0][1]), glm_cast(m[0][2]),
    glm_cast(m[1][0]), glm_cast(m[1][1]), glm_cast(m[1][2]),
    glm_cast(m[2][0]), glm_cast(m[2][1]), glm_cast(m[2][2])
  );
}

CGLM_INLINE int glm_mat4_str(char *buff, size_t size, mat4 m) {
  return glm_sprintf(buff, size,
    "mat4x4((%f, %f, %f, %f), (%f, %f, %f, %f), (%f, %f, %f, %f), (%f, %f, %f, %f))",
    glm_cast(m[0][0]), glm_cast(m[0][1]), glm_cast(m[0][2]), glm_cast(m[0][3]),
    glm_cast(m[1][0]), glm_cast(m[1][1]), glm_cast(m[1][2]), glm_cast(m[1][3]),
    glm_cast(m[2][0]), glm_cast(m[2][1]), glm_cast(m[2][2]), glm_cast(m[2][3]),
    glm_cast(m[3][0]), glm_cast(m[3][1]), glm_cast(m[3][2]), glm_cast(m[3][3])
  );
}

#undef glm_cast
#undef glm_sprintf

/* }================================================================== */

/*
** {==================================================================
** hash.{h,inl}
** ===================================================================
*/

/*!
 * @brief calculates the hash of the argument
 */

CGLM_INLINE size_t glm_hashf16(float16 n) {
  return (size_t)n;
}

CGLM_INLINE size_t glm_hashf(float n) {
  union {
    float t;
    size_t a;
  } u;
  u.a = 0;
  u.t = n;
  return (n == 0.f) ? 0 : u.a;
}

CGLM_INLINE size_t glm_hash(double n) {
#if ((SIZE_MAX >> 31) >> 31) == 3
  union {
    double t;
    size_t a;
  } u;
  u.a = 0;
  u.t = n;
  return (n == 0.0) ? 0 : u.a;
#elif (SIZE_MAX >> 30) == 3
  union {
    double t;
    struct {
      size_t a;
      size_t b;
    } s;
  } u;
  u.s.a = 0;
  u.s.b = 0;
  u.t = n;
  return (n == 0.0) ? 0 : (u.s.a ^ u.s.b);
#else
  #error "Unsupported Architecture"
#endif
}

#if defined(LDBL_DIG) || defined(__LDBL_DIG__)
CGLM_INLINE size_t glm_hashl(long double n) {
#if ((SIZE_MAX >> 31) >> 31) == 3
  union {
    long double t;
    struct {
      size_t a;
      size_t b;
    } s;
  } u;
  u.s.a = 0;
  u.s.b = 0;
  u.t = n;
  return (n == 0.0L) ? 0 : (u.s.a ^ u.s.b);
#elif (SIZE_MAX >> 30) == 3
  union {
    long double t;
    struct {
      size_t a;
      size_t b;
      size_t c;
      size_t d;
    } s;
  } u;
  u.s.a = 0;
  u.s.b = 0;
  u.s.c = 0;
  u.s.d = 0;
  u.t = n;
  return (n == 0.0L) ? 0 : (u.s.a ^ u.s.b ^ u.s.c ^ u.s.d);
#else
  #error "Unsupported Architecture"
#endif
}
#endif

CGLM_INLINE size_t glm_hashstep(size_t seed, size_t value) {
  seed ^= value + 0x9E3779B9 + (seed << 6) + (seed >> 2);
  return seed;
}

/*!
 * @brief calculates the hash of the argument
 */

CGLM_INLINE size_t glm_vec2_hash(vec2 v) {
  size_t Hash = 0x9DA040E3; /* joaat("vec2") */
  Hash = glm_hashstep(Hash, glm_hashf(v[0]));
  Hash = glm_hashstep(Hash, glm_hashf(v[1]));
  return Hash;
}

CGLM_INLINE size_t glm_vec3_hash(vec3 v) {
  size_t Hash = 0xAF3DE41E;
  Hash = glm_hashstep(Hash, glm_hashf(v[0]));
  Hash = glm_hashstep(Hash, glm_hashf(v[1]));
  Hash = glm_hashstep(Hash, glm_hashf(v[2]));
  return Hash;
}

CGLM_INLINE size_t glm_vec4_hash(vec4 v) {
  size_t Hash = 0x9B2DBBFE;
  Hash = glm_hashstep(Hash, glm_hashf(v[0]));
  Hash = glm_hashstep(Hash, glm_hashf(v[1]));
  Hash = glm_hashstep(Hash, glm_hashf(v[2]));
  Hash = glm_hashstep(Hash, glm_hashf(v[3]));
  return Hash;
}

CGLM_INLINE size_t glm_quat_hash(versor v) {
  size_t Hash = 0xA3675366; /* joaat("quat") */
  Hash = glm_hashstep(Hash, glm_hashf(v[3]));
  Hash = glm_hashstep(Hash, glm_hashf(v[0]));
  Hash = glm_hashstep(Hash, glm_hashf(v[1]));
  Hash = glm_hashstep(Hash, glm_hashf(v[2]));
  return Hash;
}

CGLM_INLINE size_t glm_mat2_hash(mat2 m) {
  size_t Hash = 0x7ABED661; /* joaat("mat2") */
  Hash = glm_hashstep(Hash, glm_vec2_hash(m[0]));
  Hash = glm_hashstep(Hash, glm_vec2_hash(m[1]));
  return Hash;
}

CGLM_INLINE size_t glm_mat3_hash(mat3 m) {
  size_t Hash = 0x898473EC;
  Hash = glm_hashstep(Hash, glm_vec3_hash(m[0]));
  Hash = glm_hashstep(Hash, glm_vec3_hash(m[1]));
  Hash = glm_hashstep(Hash, glm_vec3_hash(m[2]));
  return Hash;
}

CGLM_INLINE size_t glm_mat4_hash(mat4 m) {
  size_t Hash = 0xA644AD6C;
  Hash = glm_hashstep(Hash, glm_vec4_hash(m[0]));
  Hash = glm_hashstep(Hash, glm_vec4_hash(m[1]));
  Hash = glm_hashstep(Hash, glm_vec4_hash(m[2]));
  Hash = glm_hashstep(Hash, glm_vec4_hash(m[3]));
  return Hash;
}

/* }================================================================== */

/*
** {==================================================================
** euler.h
** https://www.geometrictools.com/Documentation/EulerAngles.pdf
** ===================================================================
*/

/*!
 * @brief string to euler axis sequence
 */
CGLM_INLINE glm_euler_seq glm_parse_euler(const char *s, bool extrinsic) {
  int o1, o2, o3; /* See glm_euler_seq encoding */
  if (s[0] == 'x' && s[1] == 'z' && s[2] == 'y')
    o1 = 0, o2 = 2, o3 = 1; /* GLM_EULER_XZY */
  else if (s[0] == 'y' && s[1] == 'z' && s[2] == 'x')
    o1 = 1, o2 = 2, o3 = 0; /* GLM_EULER_YZX */
  else if (s[0] == 'y' && s[1] == 'x' && s[2] == 'z')
    o1 = 1, o2 = 0, o3 = 2; /* GLM_EULER_YXZ */
  else if (s[0] == 'z' && s[1] == 'x' && s[2] == 'y')
    o1 = 2, o2 = 0, o3 = 1; /* GLM_EULER_ZXY */
  else if (s[0] == 'z' && s[1] == 'y' && s[2] == 'x')
    o1 = 2, o2 = 1, o3 = 0; /* GLM_EULER_ZYX */
  else
    o1 = 0, o2 = 1, o3 = 2; /* GLM_EULER_XYZ */
  return extrinsic ? (glm_euler_seq)((o3 << 0) | (o2 << 2) | (o1 << 4))
                   : (glm_euler_seq)((o1 << 0) | (o2 << 2) | (o3 << 4));
}

/*!
 * @brief extract euler angles using XYZ order
 */
CGLM_INLINE void glm_euler_angles_xyz(mat4 m, vec3 dest) {
  float thetaX, thetaY, thetaZ;
  if (m[2][0] < 1.f) {
    if (m[2][0] > -1.f) {
      thetaX = atan2f(-m[2][1], m[2][2]);
      thetaY = asinf(m[2][0]);
      thetaZ = atan2f(-m[1][0], m[0][0]);
    }
    else { /* Not a unique solution */
      thetaX = -atan2f(m[0][1], m[1][1]);
      thetaY = -GLM_PI_2f;
      thetaZ = 0.0f;
    }
  }
  else { /* Not a unique solution */
    thetaX = atan2f(m[0][1], m[1][1]);
    thetaY = GLM_PI_2f;
    thetaZ = 0.0f;
  }

  dest[0] = thetaX;
  dest[1] = thetaY;
  dest[2] = thetaZ;
}

/*!
 * @brief extract euler angles using XZY order
 */
CGLM_INLINE void glm_euler_angles_xzy(mat4 m, vec3 dest) {
  float thetaX, thetaY, thetaZ;
  if (m[1][0] < 1.f) {
    if (m[1][0] > -1.f) {
      thetaX = atan2f(m[1][2], m[1][1]);
      thetaZ = asinf(-m[1][0]);
      thetaY = atan2f(m[2][0], m[0][0]);
    }
    else { /* Not a unique solution */
      thetaX = -atan2f(-m[0][2], m[2][2]);
      thetaZ = GLM_PI_2f;
      thetaY = 0.0f;
    }
  }
  else { /* Not a unique solution */
    thetaX = atan2f(-m[0][2], m[2][2]);
    thetaZ = -GLM_PI_2f;
    thetaY = 0.0f;
  }

  dest[0] = thetaX;
  dest[1] = thetaY;
  dest[2] = thetaZ;
}

/*!
 * @brief extract euler angles using YXZ order
 */
CGLM_INLINE void glm_euler_angles_yxz(mat4 m, vec3 dest) {
  float thetaX, thetaY, thetaZ;
  if (m[2][1] < 1.f) {
    if (m[2][1] > -1.f) {
      thetaY = atan2f(m[2][0], m[2][2]);
      thetaX = asinf(-m[2][1]);
      thetaZ = atan2f(m[0][1], m[1][1]);
    }
    else { /* Not a unique solution */
      thetaY = -atan2f(-m[1][0], m[0][0]);
      thetaX = GLM_PI_2f;
      thetaZ = 0.0f;
    }
  }
  else { /* Not a unique solution */
    thetaY = atan2f(-m[1][0], m[0][0]);
    thetaX = -GLM_PI_2f;
    thetaZ = 0.0f;
  }

  dest[0] = thetaX;
  dest[1] = thetaY;
  dest[2] = thetaZ;
}

/*!
 * @brief extract euler angles using YZX order
 */
CGLM_INLINE void glm_euler_angles_yzx(mat4 m, vec3 dest) {
  float thetaX, thetaY, thetaZ;
  if (m[0][1] < 1.f) {
    if (m[0][1] > -1.f) {
      thetaY = atan2f(-m[0][2], m[0][0]);
      thetaZ = asinf(m[0][1]);
      thetaX = atan2f(-m[2][1], m[1][1]);
    }
    else { /* Not a unique solution */
      thetaY = -atan2f(m[1][2], m[2][2]);
      thetaZ = -GLM_PI_2f;
      thetaX = 0.0f;
    }
  }
  else { /* Not a unique solution */
    thetaY = atan2f(m[1][2], m[2][2]);
    thetaZ = GLM_PI_2f;
    thetaX = 0.0f;
  }

  dest[0] = thetaX;
  dest[1] = thetaY;
  dest[2] = thetaZ;
}

/*!
 * @brief extract euler angles using ZXY order
 */
CGLM_INLINE void glm_euler_angles_zxy(mat4 m, vec3 dest) {
  float thetaX, thetaY, thetaZ;
  if (m[1][2] < 1.f) {
    if (m[1][2] > -1.f) {
      thetaZ = atan2f(-m[1][0], m[1][1]);
      thetaX = asinf(m[1][2]);
      thetaY = atan2f(-m[0][2], m[2][2]);
    }
    else { /* Not a unique solution */
      thetaZ = -atan2f(m[2][0], m[0][0]);
      thetaX = -GLM_PI_2f;
      thetaY = 0.0f;
    }
  }
  else { /* Not a unique solution */
    thetaZ = atan2f(m[2][0], m[0][0]);
    thetaX = GLM_PI_2f;
    thetaY = 0.0f;
  }

  dest[0] = thetaX;
  dest[1] = thetaY;
  dest[2] = thetaZ;
}

/*!
 * @brief extract euler angles using ZYX order
 */
CGLM_INLINE void glm_euler_angles_zyx(mat4 m, vec3 dest) {
  float thetaX, thetaY, thetaZ;
  if (m[0][2] < 1.f) {
    if (m[0][2] > -1.f) {
      thetaZ = atan2f(m[0][1], m[0][0]);
      thetaY = asinf(-m[0][2]);
      thetaX = atan2f(m[1][2], m[2][2]);
    }
    else { /* Not a unique solution */
      thetaZ = -atan2f(-m[2][1], m[1][1]);
      thetaY = GLM_PI_2f;
      thetaX = 0.0f;
    }
  }
  else { /* Not a unique solution */
    thetaZ = atan2f(-m[2][1], m[1][1]);
    thetaY = -GLM_PI_2f;
    thetaX = 0.0f;
  }

  dest[0] = thetaX;
  dest[1] = thetaY;
  dest[2] = thetaZ;
}

/*!
 * @brief extract euler angles
 */
CGLM_INLINE
void glm_euler_to_order(mat4 m, glm_euler_seq ord, vec3 dest) {
  switch (ord) {
    case GLM_EULER_XYZ: glm_euler_angles_xyz(m, dest); break;
    case GLM_EULER_XZY: glm_euler_angles_xzy(m, dest); break;
    case GLM_EULER_YXZ: glm_euler_angles_yxz(m, dest); break;
    case GLM_EULER_YZX: glm_euler_angles_yzx(m, dest); break;
    case GLM_EULER_ZXY: glm_euler_angles_zxy(m, dest); break;
    case GLM_EULER_ZYX: glm_euler_angles_zyx(m, dest); break;
  }
}

/*!
 * @brief build quaternion from euler angles
 */
CGLM_INLINE
void glm_euler_by_orderq(vec3 angles, glm_euler_seq ord, versor dest) {
  switch (ord) {
    case GLM_EULER_XYZ: glm_euler_xyz_quat(angles, dest); break;
    case GLM_EULER_XZY: glm_euler_xzy_quat(angles, dest); break;
    case GLM_EULER_YXZ: glm_euler_yxz_quat(angles, dest); break;
    case GLM_EULER_YZX: glm_euler_yzx_quat(angles, dest); break;
    case GLM_EULER_ZXY: glm_euler_zxy_quat(angles, dest); break;
    case GLM_EULER_ZYX: glm_euler_zyx_quat(angles, dest); break;
  }
}

/*!
 * @brief extract euler angles from a quaternion
 */
CGLM_INLINE
void glm_euler_to_orderq(versor q, glm_euler_seq ord, vec3 dest) {
  mat4 m;
  glm_quat_mat4(q, m);
  glm_euler_to_order(m, ord, dest);
}

/* }================================================================== */

#endif
