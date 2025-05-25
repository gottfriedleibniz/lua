/*
** $Id: lglm.c $
** vector, matrix, and GLSL implementations
** See Copyright Notice in lua.h
*/

#define lglm_c

#define LUA_CORE
#define CGLM_STATIC
#define CGLM_ALL_UNALIGNED
#define CGLM_USE_DEFAULT_EPSILON

#include "lprefix.h"
#include "lua.h"
#include "lauxlib.h"
#if defined(LUAGLM_HALF_TYPE)
  #undef CGLM_USE_DEFAULT_EPSILON
  #define GLM_FLT_EPSILON 0.0009765625f
#endif

#include <string.h>
#include <ctype.h>
#include <cglm/cglm.h>
#if defined(_MSC_VER)
  #include <intrin.h>
  #pragma intrinsic(_BitScanForward, _BitScanReverse)
#if defined(_M_X64) || defined(_M_ARM64)
  #pragma intrinsic(_BitScanForward64, _BitScanReverse64)
#endif
#endif

#include "lglmext.h"
#include "lglmaux.h"
#include "lglmcore.h"
#include "lstate.h"
#include "lapi.h"
#include "ldebug.h"
#include "lfunc.h"
#include "lgc.h"
#include "ltable.h"
#include "lvm.h"

/* test for a valid index (one that is not the 'nilvalue') */
#define glm_isvalid(L, o) (!ttisnil(o) || ((o) != &G(L)->nilvalue))

/* index2value copied from lapi.c */
static const TValue *glm_index2value(const lua_State *L, int idx) {
  CallInfo *ci = L->ci;
  if (idx > 0) {
    StkId o = ci->func.p + idx;
    api_check(L, idx <= ci->top.p - (ci->func.p + 1), "unacceptable index");
    return l_likely(o < L->top.p) ? s2v(o) : &G(L)->nilvalue;
  }
  else if (idx > LUA_REGISTRYINDEX) { /* negative index */
    api_check(L, idx != 0 && -idx <= L->top.p - (ci->func.p + 1),
                 "invalid index");
    return s2v(L->top.p + idx);
  }
  else if (idx == LUA_REGISTRYINDEX)
    return &G(L)->l_registry;
  else { /* upvalues */
    idx = LUA_REGISTRYINDEX - idx;
    api_check(L, idx <= MAXUPVAL + 1, "upvalue index too large");
    if (ttisCclosure(s2v(ci->func.p))) { /* C closure? */
      CClosure *func = clCvalue(s2v(ci->func.p));
      return (idx <= func->nupvalues) ? &func->upvalue[idx - 1]
                                      : &G(L)->nilvalue;
    }
    else { /* light C function or Lua function (through a hook)?) */
      api_check(L, ttislcf(s2v(ci->func.p)), "caller not a C function");
      return &G(L)->nilvalue; /* no upvalues */
    }
  }
}

/* index2value for only positive stack indexes. Intended for places where the
** compiler cannot recognize idx is always positive. */
static const TValue *glm_fasti2v(const lua_State *L, int idx) {
  const StkId o = L->ci->func.p + idx;
  api_check(L, idx > 0, "positive indices only");
  api_check(L, idx <= L->ci->top.p - (L->ci->func.p + 1), "invalid index");
  return (o < L->top.p) ? s2v(o) : &G(L)->nilvalue;
}

/* Generic tagged value to float cast */
static int glm_castvalue(const TValue *value, float *out) {
  lua_assert(out != NULL);
  switch (rawtt(value)) {
    case LUA_VNIL: *out = 0.0f; break;
    case LUA_VFALSE: *out = 0.0f; break;
    case LUA_VTRUE: *out = 1.0f; break;
    case LUA_VNUMINT: *out = cast(float, ivalue(value)); break;
    case LUA_VNUMFLT: *out = cast(float, fltvalue(value)); break;
    default: {
      return 0;
    }
  }
  return 1;
}

/*
** {==================================================================
** Object Conversion
** ===================================================================
*/

#define GLM_UNEXPECTED "unexpected arguments"

#define glm_cf(i) cast(float, i)
#define glm_fv(o) (ttisinteger(o) ? glm_cf(ivalue(o)) : glm_cf(fltvalue(o)))
#define glm_vload(o) vload(vvalue_(o))

#define glm_f4(o) check_exp(ttisvector(o), glm_vload(o))
#define glm_v2(o) check_exp(ttisvector2(o), glm_vload(o).v2)
#define glm_v3(o) check_exp(ttisvector3(o), glm_vload(o).v3)
#define glm_v4(o) check_exp(ttisvector4(o), glm_vload(o).v4)
#define glm_q(o) check_exp(ttisquat(o), glm_vload(o).q)

/*
** @MatLayout: At the moment lua_Matrix is a union of the varying matrix types.
** This stems from the previous implementation where all NxM matrix variants
** were supported and it simplified the logic when operating on a matrices with
** arbitrary dimension.
**
** Now that this runtime restricts the representation to only symmetric
** matrices, this can change. Many mat2/mat3 operations can be generalized to
** mat4 and in the worst-case glm_m2 and glm_m3 can reference inline functions
** that use glm_mat4_pick3, etc.
*/
#define glm_m2(o) m2value(o)
#define glm_m3(o) m3value(o)
#define glm_m4(o) m4value(o)

/*
** Note: cglm does not const-qualify parameters where possible (cglm/issues/83).
** Operating directly on vector and matrices requires const casting of TValues.
**
** @Independent: Operation on only vec4/mat4x4 representation. Used as an
** optimization when a function is independently applied to each element of the
** structure. This takes advantage of already existing SIMD implementations when
** possible for vec2/vec3 types (+ reduces code duplication).
*/
#define glm_vid(o) check_exp(ttisvector(o), glm_vload(o).v4)
#define glm_mid(o) check_exp(ttismatrix(o), mvalue_(o).m4)
#if defined(LUAGLM_HALF_TYPE)
  #define glm_vstore(V4, O) lua_tohalf4((V4), vvalue_(O).v4)
#else
  #define glm_vstore(V4, O) glm_vec4_copy((V4), vvalue_(O).v4)
#endif

/*
** Ensure lua_Integer arguments fit within an unsigned 32-bit integer. Used for
** bit operations and GLSL pack/unpack functions.
*/
#define LUAI_DIGITS (sizeof(lua_Unsigned) * CHAR_BIT)
#define LUAI_HAS_UNSIGNED32 ((LUA_MAXUNSIGNED >> 30) >= 3)
#define LUAI_HAS_UNSIGNED64 (((LUA_MAXUNSIGNED >> 31) >> 31) >= 3)
#if ((LUA_MAXUNSIGNED >> 30) > 3)
  #define glm_bitop_inrange(P) ((P) >= 0 && (P) <= UINT32_MAX)
#else
  #define glm_bitop_inrange(P) 1
#endif

/* check/push for number types (stubs for future built-in work) */
#define glm_checknumber(L, I) luaL_checknumber((L), (I))
#define glm_checkinteger(L, I) luaL_checkinteger((L), (I))
#define glm_checkfloat(L, I) glm_cf(glm_checknumber((L), (I)))
#define glm_checkunsigned(L, I) l_castS2U(glm_checkinteger((L), (I)))
#define glm_pushboolean(L, B) lua_pushboolean(L, B)
#define glm_pushnumber(L, N) lua_pushnumber(L, N)
#define glm_pushinteger(L, I) lua_pushinteger(L, I)
static void glm_pushnumint(lua_State *L, lua_Number d) {
  lua_Integer n;
  if (lua_numbertointeger(d, &n)) /* does 'd' fit in an integer? */
    glm_pushinteger(L, n); /* result is integer */
  else
    glm_pushnumber(L, d); /* result is float */
}

/*
** The core cglm API is functional: result vectors/matrices are passed to the
** function as trailing parameters (documented as [out] pointers). The following
** macros wrap these functions and place the results onto the Lua stack.
*/
#define glm_vec2_op(Res, Func, ...) glm_vec_op(Res, LUA_VVECTOR2, 1, Func, __VA_ARGS__)
#define glm_vec3_op(Res, Func, ...) glm_vec_op(Res, LUA_VVECTOR3, 1, Func, __VA_ARGS__)
#define glm_vec4_op(Res, Func, ...) glm_vec_op(Res, LUA_VVECTOR4, 0, Func, __VA_ARGS__)
#define glm_quat_op(Res, Func, ...) glm_vec_op(Res, LUA_VQUAT, 0, Func, __VA_ARGS__)
#define glm_vid_op(Res, Tag, Func, ...) glm_vec_op(Res, Tag, 0, Func, __VA_ARGS__)
#define glm_vec_op(Res, Tag, Zero, Func, ...) \
  LUA_MLM_BEGIN                               \
  vec4 Dest;                                  \
  TValue *io = s2v(Res);                      \
  if (Zero) glm_vec4_zero(Dest);              \
  Func(__VA_ARGS__, Dest);                    \
  glm_vstore(Dest, io);                       \
  settt_(io, Tag);                            \
  LUA_MLM_END

#define glm_pushvec2(Func, ...) glm_pushvec(LUA_VVECTOR2, 1, Func, __VA_ARGS__)
#define glm_pushvec3(Func, ...) glm_pushvec(LUA_VVECTOR3, 1, Func, __VA_ARGS__)
#define glm_pushvec4(Func, ...) glm_pushvec(LUA_VVECTOR4, 0, Func, __VA_ARGS__)
#define glm_pushquat(Func, ...) glm_pushvec(LUA_VQUAT, 0, Func, __VA_ARGS__)
#define glm_pushvid(Tag, Func, ...) glm_pushvec(Tag, 0, Func, __VA_ARGS__)
#define glm_pushvec(Tag, Zero, Func, ...)             \
  LUA_MLM_BEGIN                                       \
  lua_lock(L);                                        \
  glm_vec_op(L->top.p, Tag, Zero, Func, __VA_ARGS__); \
  api_incr_top(L);                                    \
  lua_unlock(L);                                      \
  LUA_MLM_END

#define glm_mat2_op(Res, Func, ...) glm_mat_op(Res, LUA_VMATRIX2, m2, Func, __VA_ARGS__)
#define glm_mat3_op(Res, Func, ...) glm_mat_op(Res, LUA_VMATRIX3, m3, Func, __VA_ARGS__)
#define glm_mat4_op(Res, Func, ...) glm_mat_op(Res, LUA_VMATRIX4, m4, Func, __VA_ARGS__)
#define glm_mid_op(Res, Tag, Func, ...) glm_mat_op(Res, Tag, m4, Func, __VA_ARGS__)
#define glm_mat_op(Res, Tag, Field, Func, ...) \
  LUA_MLM_BEGIN                                \
  GCMatrix *M = lmat_new(L, Tag);              \
  Func(__VA_ARGS__, M->m.Field);               \
  setmvalue(L, s2v(Res), M, Tag);              \
  luaC_checkGC(L);                             \
  LUA_MLM_END

#define glm_pushmat2(Func, ...) glm_pushmat(LUA_VMATRIX2, m2, Func, __VA_ARGS__)
#define glm_pushmat3(Func, ...) glm_pushmat(LUA_VMATRIX3, m3, Func, __VA_ARGS__)
#define glm_pushmat4(Func, ...) glm_pushmat(LUA_VMATRIX4, m4, Func, __VA_ARGS__)
#define glm_pushmat(Tag, Field, Func, ...) \
  LUA_MLM_BEGIN                            \
  GCMatrix *M;                             \
  lua_lock(L);                             \
  M = lmat_push(L, Tag);                   \
  Func(__VA_ARGS__, M->m.Field);           \
  luaC_checkGC(L);                         \
  lua_unlock(L);                           \
  LUA_MLM_END

/*
** If the object at idx is a matrix, push it to the top of the stack. Otherwise,
** create a matrix it and push it onto the stack. The resulting matrix is used
** as the destination to the given function.
*/
#define glm_mat2_recycle(Idx, Func, ...) glm_mat_recycle(Idx, LUA_VMATRIX2, m2, 1, Func, __VA_ARGS__)
#define glm_mat3_recycle(Idx, Func, ...) glm_mat_recycle(Idx, LUA_VMATRIX3, m3, 1, Func, __VA_ARGS__)
#define glm_mat4_recycle(Idx, Func, ...) glm_mat_recycle(Idx, LUA_VMATRIX4, m4, 0, Func, __VA_ARGS__)
#define glm_mid_recycle(Idx, Tag, Func, ...) glm_mat_recycle(Idx, Tag, m4, 0, Func, __VA_ARGS__)
#define glm_mat_recycle(Idx, Tag, Field, Zero, Func, ...) \
  LUA_MLM_BEGIN                                     \
  GCMatrix *M;                                      \
  TValue *O = (TValue *)glm_fasti2v(L, Idx);        \
  lua_lock(L);                                      \
  if (mvaltt(O) == Tag) { /* recycle matrix */      \
    M = gco2mat(val_(O).gc);                        \
    setobj(L, s2v(L->top.p), O);                    \
    api_incr_top(L);                                \
    if (Zero) glm_mat4_zero(M->m.m4);               \
  }                                                 \
  else { /* create a matrix */                      \
    M = lmat_push(L, Tag);                          \
  }                                                 \
  Func(__VA_ARGS__, M->m.Field);                    \
  luaC_checkGC(L);                                  \
  lua_unlock(L);                                    \
  LUA_MLM_END

l_float16 lua_tohalf(float input) {
  return glm_to_half(input);
}

float lua_fromhalf(l_float16 input) {
  return glm_from_half(input);
}

void lua_fromhalf4(const lua_hvec4 v, lua_vec4 dest) {
  glm_vec4_unpack(*(lua_hvec4 *)&v[0], dest);
}

void lua_tohalf4(const lua_vec4 v, lua_hvec4 dest) {
  glm_vec4_pack(*(lua_vec4 *)&v[0], dest);
}

#if defined(LUAGLM_HALF_TYPE)
lua_Float4 luaO_loadv(const luai_Float4 *input) {
  CGLM_ALIGN(16) lua_Float4 Result = { GLM_VEC4_ZERO_INIT };
  lua_fromhalf4(input->v4, Result.v4);
  return Result;
}

luai_Float4 luaO_storev(const lua_Float4 *input) {
  luai_Float4 Result = { GLM_IVEC4_ZERO_INIT };
  lua_tohalf4(input->v4, Result.v4);
  return Result;
}
#endif
/* }================================================================== */

/*
** {==================================================================
** Vector Object API
** ===================================================================
*/

/* Return the number of swizzled fields on success, zero otherwise */
static int swizzle(const luai_Float4 *v, int len, const char *key, luai_Float4 *out) {
  int i = 0;
  for (; i < 4 && key[i] != '\0'; ++i) {
    lu_byte n = luaO_vecindex[cast_byte(key[i])];
    if (n < len)
      out->v4[i] = v->v4[n]; /* vseti/vgeti */
    else {
      return 0;
    }
  }
  return i;
}

int lvec_rawgeti(const TValue *obj, lua_Integer n, StkId res) {
  if (l_unlikely(!lvec_fastgeti(obj, n, res))) {
    setnilvalue(s2v(res));
    return LUA_TNIL;
  }
  return LUA_TNUMBER;
}

#define SINGLE_CHAR(K) ((K) != NULL && (K)[0] != '\0' && (K)[1] == '\0')
int lvec_rawgets(const TValue *obj, const char *k, StkId res) {
  return SINGLE_CHAR(k) && vecgets(obj, k, res); /* auxgetstr handles type */
}

int lvec_rawget(const TValue *obj, TValue *key, StkId res) {
  int result;
  lua_Integer idx;
  if (tointegerns(key, &idx))
    result = lvec_fastgeti(obj, idx, res);
  else if (ttisstring(key) && tsslen(tsvalue(key)) == 1)
    result = vecgets(obj, getstr(tsvalue(key)), res);
  else
    result = 0;

  if (l_unlikely(!result)) {
    setnilvalue(s2v(res));
    return LUA_TNIL;
  }
  return LUA_TNUMBER;
}

void lvec_get(lua_State *L, const TValue *obj, TValue *key, StkId res) {
  lua_Integer idx;
  if (ttisstring(key)) {
    const size_t key_len = tsslen(tsvalue(key));
    const char *key_str = getstr(tsvalue(key));
    if (key_len >= 1 && key_len <= 4) { /* swizzle operation  */
      luai_Float4 i4 = { { 0, 0, 0, 0 } };
      switch (swizzle(&(vvalue_(obj)), ttvlen(obj), key_str, &i4)) {
        case 1: setfltvalue(s2v(res), cast_num(vgeti(i4, 0))); break;
        case 2: setvvalue(s2v(res), i4, LUA_VVECTOR2); break;
        case 3: setvvalue(s2v(res), i4, LUA_VVECTOR3); break;
        case 4: {
          lu_tag tag = ttisquat(obj) ? LUA_VQUAT : LUA_VVECTOR4;
          setvvalue(s2v(res), i4, tag);
          break;
        }
        default: {
          if (key_len == 1 && key_str[0] == 'n') {
            setivalue(s2v(res), cast(lua_Integer, ttvlen(obj))); }
          else
            goto lvec_fail;
          break;
        }
      }
      return;
    }
  }
  else if (ttisnumber(key) && tointegerns(key, &idx)) {
    if (lvec_fastgeti(obj, idx, res)) {
      return;
    }
  }
lvec_fail:
  luaV_finishget(L, obj, key, res, NULL); /* Metatable Access */
}

void lvec_len(const TValue *obj, StkId res, int api_call) {
  float result;
  TValue *o = (TValue *)obj;
  if (api_call) {
    setivalue(s2v(res), cast(lua_Integer, ttvlen(obj)));
    return;
  }

  switch (vvaltt(o)) {
    case LUA_VVECTOR2: result = glm_vec2_norm(glm_v2(o)); break;
    case LUA_VVECTOR3: result = glm_vec3_norm(glm_v3(o)); break;
    case LUA_VVECTOR4: result = glm_vec4_norm(glm_v4(o)); break;
    case LUA_VQUAT:
    default: {
      result = glm_quat_norm(glm_q(o));
      break;
    }
  }
  setfltvalue(s2v(res), cast_num(result));
}

static int lvec_equalObj(const TValue *obj1, const TValue *obj2) {
  TValue *o1 = (TValue *)obj1;
  TValue *o2 = (TValue *)obj2;
#if defined(CGLM_SIMD_x86)
  int mask = (1 << ttvlen(o1)) - 1;
  __m128 cmp0 = _mm_cmpeq_ps(glmm_load(glm_vid(o1)), glmm_load(glm_vid(o2)));
  return (_mm_movemask_ps(cmp0) & mask) == mask;
#else
  int result;
  lua_assert(tteq(o1, o2));
  switch (vvaltt(o1)) {
    case LUA_VVECTOR2: result = glm_vec2_eqv(glm_v2(o1), glm_v2(o2)); break;
    case LUA_VVECTOR3: result = glm_vec3_eqv(glm_v3(o1), glm_v3(o2)); break;
    case LUA_VVECTOR4: result = glm_vec4_eqv(glm_v4(o1), glm_v4(o2)); break;
    case LUA_VQUAT:
    default: {
      result = glm_vec4_eqv(glm_q(o1), glm_q(o2));
      break;
    }
  }
  return result;
#endif
}

int lvec_concat(const TValue *obj, const TValue *value, StkId res) {
  luai_Float4 result = vvalue(obj);
  lu_tag len = ttvlen(obj);
  if (ttisinteger(value) && len < 4)
    vseti(result, len++, glm_cf(ivalue(value)));
  else if (ttisfloat(value) && len < 4)
    vseti(result, len++, glm_cf(fltvalue(value)));
  else if (ttisboolean(value) && len < 4)
    vseti(result, len++, glm_cf(!l_isfalse(value)));
  else if (ttisvector(value)) {
    lu_tag i, other_len = ttvlen(value);
    for (i = 0; i < other_len && len < 4; ++i) {
      result.v4[len++] = vvalue_(value).v4[i]; /* vseti/vgeti */
    }
  }
  else {
    return 0;
  }
  setvvalue(s2v(res), result, vvaltag(len));
  return 1;
}

int lvec_equalkey(const TValue *k1, const Node *n2, int rtt) {
  int len = vvallen(rtt);
  CGLM_ALIGN(16) lua_Float4 key = glm_f4(k1);
  CGLM_ALIGN(16) lua_Float4 node = vload(vvalueraw(keyval(n2)));
#if defined(CGLM_SIMD_x86)
  int mask = (1 << len) - 1;
  __m128 cmp0 = _mm_cmpeq_ps(glmm_load(key.v4), glmm_load(node.v4));
  return (_mm_movemask_ps(cmp0) & mask) == mask;
#else
  int i, result = 1;
  for (i = 0; i < len; ++i)
    result &= (key.v4[i] == node.v4[i]);
  return result;
#endif
}

size_t lvec_hashkey(const TValue *obj) {
  size_t result;
  TValue *o = (TValue *)obj;
  switch (vvaltt(o)) {
    case LUA_VVECTOR2: result = glm_vec2_hash(glm_v2(o)); break;
    case LUA_VVECTOR3: result = glm_vec3_hash(glm_v3(o)); break;
    case LUA_VVECTOR4: result = glm_vec4_hash(glm_v4(o)); break;
    case LUA_VQUAT:
    default: {
      result = glm_quat_hash(glm_q(o));
      break;
    }
  }
  return result;
}

int lvec_validkey(const TValue *obj) {
  TValue *o = (TValue *)obj;
#if defined(CGLM_SIMD_x86)
  int mask = (1 << ttvlen(o)) - 1;
  return (_mm_movemask_ps(glmm_isnan(glmm_load(glm_vid(o)))) & mask) == 0;
#else
  int result;
  switch (vvaltt(o)) {
    case LUA_VVECTOR2: result = !glm_vec2_isnan(glm_v2(o)); break;
    case LUA_VVECTOR3: result = !glm_vec3_isnan(glm_v3(o)); break;
    case LUA_VVECTOR4: result = !glm_vec4_isnan(glm_v4(o)); break;
    case LUA_VQUAT:
    default: {
      result = !glm_vec4_isnan(glm_q(o));
      break;
    }
  }
  return result;
#endif
}

int lvec_next(const TValue *obj, StkId key) {
  lua_Integer idx = 0;
  TValue *o = s2v(key);
  if (l_likely(ttisnil(o) || tointegerns(o, &idx))) {
    lua_Integer i = luaL_intop(+, idx, 1); /* first empty element */
    setivalue(o, i); /* Iterator values are 1-based */
    return lvec_fastgeti(obj, i, key + 1);
  }
  return 0;
}

/* luaO_tostring/tostringbuff variant for vector types */
static int lvec_tostr(const TValue *obj, char *buff, size_t len) {
  int ncopy;
  TValue *o = (TValue *)obj;
  switch (vvaltt(o)) {
    case LUA_VVECTOR2: ncopy = glm_vec2_str(buff, len, glm_v2(o)); break;
    case LUA_VVECTOR3: ncopy = glm_vec3_str(buff, len, glm_v3(o)); break;
    case LUA_VVECTOR4: ncopy = glm_vec4_str(buff, len, glm_v4(o)); break;
    case LUA_VQUAT:
    default: {
      ncopy = glm_quat_str(buff, len, glm_q(o));
      break;
    }
  }
  lua_assert(ncopy >= 0 && cast_sizet(ncopy) < len);
  return (ncopy >= 0 && cast_sizet(ncopy) < len) ? ncopy : 0;
}

/* }================================================================== */

/*
** {==================================================================
** Matrix Object API
** ===================================================================
*/

/*
** If 'raw' (denoting rawset) is true the function will throw a runtime error
** when attempting to operate on invalid keys/fields. Otherwise this function
** attempts a metatable access.
*/
static void lmat_auxset(lua_State *L, const TValue *obj, TValue *key, TValue *val, int raw) {
  lua_Integer idx;
  lu_tag m_size = ttmlen(obj);
  if (tointegerns(key, &idx) && idx >= 1 && idx <= m_size) { /* in bounds */
    if (ttisvector(val) && ttvlen(val) == m_size) { /* valid column vector */
      CGLM_ALIGN(16) lua_Float4 f4 = glm_vload(val);
      switch (m_size) {
        case 2: glm_vec2_copy(f4.v2, glm_m2(obj)[idx - 1]); break;
        case 3: glm_vec3_copy(f4.v3, glm_m3(obj)[idx - 1]); break;
        case 4:
        default: {
          glm_vec4_copy(f4.v4, glm_m4(obj)[idx - 1]);
          break;
        }
      }
      return;
    }
  }
  raw ? luaG_runerror(L, "invalid matrix rawset")
      : luaV_finishset(L, obj, key, val, NULL);
}

static GCMatrix *lmat_push(lua_State *L, int tt) {
  GCMatrix *M = lmat_new(L, tt);
  setmvalue(L, s2v(L->top.p), M, cast(lu_tag, tt));
  api_incr_top(L);
  return M;
}

GCMatrix *lmat_new(lua_State *L, int tt) {
  GCObject *o = luaC_newobj(L, tt, sizeof(GCMatrix));
  GCMatrix *mat = gco2mat(o);
  lua_assert(tt == LUA_VMATRIX2 || tt == LUA_VMATRIX3 || tt == LUA_VMATRIX4);
  glm_mat4_zero(mat->m.m4);
  return mat;
}

int lmat_vmgeti(const TValue *obj, lua_Integer n, StkId res) {
  lu_tag m_size = ttmlen(obj);
  if (l_likely(n >= 1 && n <= cast(lua_Integer, m_size))) {
    switch (m_size) {
      case 2: glm_vec2_op(res, glm_vec2_copy, glm_m2(obj)[n - 1]); break;
      case 3: glm_vec3_op(res, glm_vec3_copy, glm_m3(obj)[n - 1]); break;
      case 4:
      default: {
        glm_vec4_op(res, glm_vec4_copy, glm_m4(obj)[n - 1]);
        break;
      }
    }
    return 1;
  }
  return 0;
}

int lmat_rawgeti(const TValue *obj, lua_Integer n, StkId res) {
  if (!lmat_fastgeti(obj, n, res)) {
    setnilvalue(s2v(res));
    return LUA_TNIL;
  }
#if defined(LUAGLM_COMPAT_5_4)
  return LUA_TTABLE; /* @TypeCompat */
#else
  return LUA_TVECTOR;
#endif
}

int lmat_rawget(const TValue *obj, TValue *key, StkId res) {
  lua_Integer idx; /* Allow float-to-int coercion */
  if (tointegerns(key, &idx) && lmat_fastgeti(obj, idx, res)) {
#if defined(LUAGLM_COMPAT_5_4)
    return LUA_TTABLE; /* @TypeCompat */
#else
    return LUA_TVECTOR;
#endif
  }
  else {
    setnilvalue(s2v(res));
    return LUA_TNIL;
  }
}

void lmat_rawset(lua_State *L, const TValue *obj, TValue *key, TValue *val) {
  lmat_auxset(L, obj, key, val, 1);
}

void lmat_get(lua_State *L, const TValue *obj, TValue *key, StkId res) {
  lua_Integer idx;
  if (!(tointegerns(key, &idx) && lmat_fastgeti(obj, idx, res))) {
    luaV_finishget(L, obj, key, res, NULL);
  }
}

void lmat_set(lua_State *L, const TValue *obj, TValue *key, TValue *val) {
  lmat_auxset(L, obj, key, val, 0);
}

void lmat_seti(lua_State *L, const TValue *obj, lua_Integer c, TValue *val) {
  TValue key;
  setivalue(&key, c);
  lmat_auxset(L, obj, &key, val, 0);
}

void lmat_len(const TValue *obj, StkId res) {
  setivalue(s2v(res), cast(lua_Integer, ttmlen(obj)));
}

int lmat_next(const TValue *obj, StkId key) {
  lua_Integer idx = 0;
  TValue *o = s2v(key);
  if (l_likely(ttisnil(o) || tointegerns(o, &idx))) {
    lua_Integer i = luaL_intop(+, idx, 1); /* first empty element */
    setivalue(o, i); /* Iterator values are 1-based */
    return lmat_fastgeti(obj, i, key + 1);
  }
  return 0;
}

static int lmat_equalObj(const TValue *obj1, const TValue *obj2) {
  int result = 0;
  TValue *o1 = (TValue *)obj1;
  TValue *o2 = (TValue *)obj2;
  lua_assert(mvaltt(o1) == mvaltt(o2));
  switch (mvaltt(o1)) {
    case LUA_VMATRIX2: result = glm_mat2_eq(glm_m2(o1), glm_m2(o2)); break;
    case LUA_VMATRIX3: result = glm_mat3_eq(glm_m3(o1), glm_m3(o2)); break;
    case LUA_VMATRIX4:
    default: {
      result = glm_mat4_eq(glm_m4(o1), glm_m4(o2));
      break;
    }
  }
  return result;
}

/* luaO_tostring/tostringbuff variant for matrix types */
static int lmat_tostr(const TValue *obj, char *buff, size_t len) {
  int ncopy;
  TValue *o = (TValue *)obj;
  switch (mvaltt(o)) {
    case LUA_VMATRIX2: ncopy = glm_mat2_str(buff, len, glm_m2(o)); break;
    case LUA_VMATRIX3: ncopy = glm_mat3_str(buff, len, glm_m3(o)); break;
    case LUA_VMATRIX4:
    default: {
      ncopy = glm_mat4_str(buff, len, glm_m4(o));
      break;
    }
  }
  lua_assert(ncopy >= 0 && cast_sizet(ncopy) < len);
  return (ncopy >= 0 && cast_sizet(ncopy) < len) ? ncopy : 0;
}

/* }================================================================== */

/*
** {==================================================================
** Library Extensions
** ===================================================================
*/

#if !defined(LUA_EXT_JOAAT)
static
#endif
lua_Unsigned luaO_jenkins(const char *str, size_t length, int ignore_case) {
  size_t i;
  uint32_t hash = 0;
  for (i = 0; i < length; ++i) {
    lu_byte by = cast_byte(str[i]); /* optimizer should unswitch */
    hash += cast(uint32_t, ignore_case ? by : tolower(by));
    hash += (hash << 10);
    hash ^= (hash >> 6);
  }

  hash += (hash << 3);
  hash ^= (hash >> 11);
  hash += (hash << 15);
  return cast(lua_Unsigned, hash);
}

lua_Unsigned luaglm_tohash(lua_State *L, int idx, int ignore_case) {
  lua_Unsigned hash = 0;
  const TValue *o = glm_index2value(L, idx);
  lua_lock(L);
  if (ttisstring(o))
    hash = luaO_jenkins(getstr(tsvalue(o)), tsslen(tsvalue(o)), ignore_case);
  else if (ttisboolean(o))
    hash = cast(lua_Unsigned, ttistrue(o));
  else if (ttisnumber(o)) {
    lua_Integer res = 0;
    hash = l_castS2U(tointegerns(o, &res) ? res : 0);
  }
  lua_unlock(L);
  return hash;
}

int lglm_equalobj(const TValue *o1, const TValue *o2) {
  if (ttisvector(o1))
    return lvec_equalObj(o1, o2);
  else if (ttismatrix(o1))
    return lmat_equalObj(o1, o2);
  return 0;
}

const char *lglm_typename(const TValue *obj) {
  switch (ttypetag(obj)) {
    case LUA_VNUMFLT: return "number";
    case LUA_VNUMINT: return "integer";
#if defined(LUAGLM_COMPAT_GLM_CPP)
    case LUA_VVECTOR2: return "vector2";
    case LUA_VVECTOR3: return "vector3";
    case LUA_VVECTOR4: return "vector4";
    case LUA_VQUAT: return "quat";
    case LUA_VMATRIX2: return "matrix2x2";
    case LUA_VMATRIX3: return "matrix3x3";
    case LUA_VMATRIX4: return "matrix4x4";
#else
    case LUA_VVECTOR2: return "vec2";
    case LUA_VVECTOR3: return "vec3";
    case LUA_VVECTOR4: return "vec4";
    case LUA_VQUAT: return "quat";
    case LUA_VMATRIX2: return "mat2x2";
    case LUA_VMATRIX3: return "mat3x3";
    case LUA_VMATRIX4: return "mat4x4";
#endif
    default: {
      return "unknown";
    }
  }
}

const char *luaglm_typename(lua_State *L, int idx) {
  const TValue *o = glm_index2value(L, idx);
  return lglm_typename(o);
}

int luaglm_format(lua_State *L, int idx, char *buff, size_t sz) {
  int len;
  const TValue *o = glm_index2value(L, idx);
  lua_lock(L);
  if (ttisnumber(o))
    len = luaO_tostringbuff(o, buff);
  else if (ttisvector(o))
    len = lvec_tostr(o, buff, sz);
  else if (ttismatrix(o))
    len = lmat_tostr(o, buff, sz);
  else {
    len = 0;
    buff[0] = '\0';
    lua_assert(0);
  }
  lua_unlock(L);
  return len;
}

const char *luaglm_pushstring(lua_State *L, int idx) {
  char buff[LUAGLM_MAXNUMBER2STR];
  int len = luaglm_format(L, idx, buff, sizeof(buff));
  return lua_pushlstring(L, buff, cast_sizet(len < 0 ? 0 : len));
}

/* }================================================================== */

/*
** {==================================================================
** grit API
** ===================================================================
*/

LUA_API int lua_isvec2(lua_State *L, int idx) { const TValue *o = glm_index2value(L, idx); return ttisvector2(o); }
LUA_API int lua_isvec3(lua_State *L, int idx) { const TValue *o = glm_index2value(L, idx); return ttisvector3(o); }
LUA_API int lua_isvec4(lua_State *L, int idx) { const TValue *o = glm_index2value(L, idx); return ttisvector4(o); }
LUA_API int lua_isquat(lua_State *L, int idx) { const TValue *o = glm_index2value(L, idx); return ttisquat(o); }
LUA_API int lua_ismat2(lua_State *L, int idx) { const TValue *o = glm_index2value(L, idx); return ttismatrix2(o); }
LUA_API int lua_ismat3(lua_State *L, int idx) { const TValue *o = glm_index2value(L, idx); return ttismatrix3(o); }
LUA_API int lua_ismat4(lua_State *L, int idx) { const TValue *o = glm_index2value(L, idx); return ttismatrix4(o); }
LUA_API int lua_isvector(lua_State *L, int idx) {
  int len = 0;
  const TValue *o = glm_index2value(L, idx);
  if (l_likely(ttisvector(o)))
    len = ttisquat(o) ? 0 : cast_int(ttvlen(o));
  else if (ttisnumber(o)) /* @ImplicitVec */
    len = 1;
  return len;
}

LUA_API int lua_ismatrix(lua_State *L, int idx) {
  int len;
  const TValue *o = glm_index2value(L, idx);
  lua_lock(L);
  len = ttismatrix(o) ? cast_int(ttmlen(o)) : 0;
  lua_unlock(L);
  return len;
}

LUA_API int lua_tovector(lua_State *L, int idx, float *v) {
  const TValue *o = glm_index2value(L, idx);
  if (ttisvector(o)) {
    int i, len = ttvlen(o);
    lua_Float4 f4 = glm_vload(o);
    for (i = 0; i < len; ++i)
      v[i] = f4.v4[i];
    return len;
  }
  return 0;
}

LUA_API int lua_tovec2(lua_State *L, int idx, lua_vec2 v2) {
  TValue *o = (TValue *)glm_index2value(L, idx);
  if (ttisvector(o) && ttvlen(o) >= 2) {
    if (v2 != NULL)
      glm_vec2_copy(glm_v2(o), v2);
    return 1;
  }
  return 0;
}

LUA_API int lua_tovec3(lua_State *L, int idx, lua_vec3 v3) {
  TValue *o = (TValue *)glm_index2value(L, idx);
  if (ttisvector(o) && ttvlen(o) >= 3) {
    if (v3 != NULL)
      glm_vec3_copy(glm_v3(o), v3);
    return 1;
  }
  return 0;
}

LUA_API int lua_tovec4(lua_State *L, int idx, lua_vec4 v4) {
  TValue *o = (TValue *)glm_index2value(L, idx);
  if (ttisvector(o) && ttvlen(o) >= 4) {
    if (v4 != NULL)
      glm_vec4_copy(glm_vid(o), v4);
    return 1;
  }
  return 0;
}

LUA_API int lua_toquat(lua_State *L, int idx, lua_versor q) {
  TValue *o = (TValue *)glm_index2value(L, idx);
  if (ttisquat(o)) {
    if (q != NULL)
      glm_quat_copy(glm_q(o), q);
    return 1;
  }
  return 0;
}

LUA_API int lua_tomat2(lua_State *L, int idx, lua_mat2 m2) {
  int result;
  TValue *o = (TValue *)glm_index2value(L, idx);
  lua_lock(L);
  if ((result = ttismatrix2(o), result) && m2 != NULL)
    glm_mat2_copy(glm_m2(o), m2);
  lua_unlock(L);
  return result;
}

LUA_API int lua_tomat3(lua_State *L, int idx, lua_mat3 m3) {
  int result;
  TValue *o = (TValue *)glm_index2value(L, idx);
  lua_lock(L);
  if ((result = ttismatrix3(o), result) && m3 != NULL)
    glm_mat3_copy(glm_m3(o), m3);
  lua_unlock(L);
  return result;
}

LUA_API int lua_tomat4(lua_State *L, int idx, lua_mat4 m4) {
  int result;
  TValue *o = (TValue *)glm_index2value(L, idx);
  lua_lock(L);
  if ((result = ttismatrix4(o), result) && m4 != NULL)
    glm_mat4_copy(glm_m4(o), m4);
  lua_unlock(L);
  return result;
}

LUA_API void lua_pushvector(lua_State *L, const float *v, int length) {
  lu_tag tag = vvaltag(length);
  lua_assert(length >= 2 && length <= 4);
  glm_pushvid(tag, glm_vec4_make, (float *)v);
}

LUA_API void lua_pushvec2(lua_State *L, const lua_vec2 v2) { glm_pushvec2(glm_vec2_copy, *(lua_vec2 *)&v2[0]); }
LUA_API void lua_pushvec3(lua_State *L, const lua_vec3 v3) { glm_pushvec3(glm_vec3_copy, *(lua_vec3 *)&v3[0]); }
LUA_API void lua_pushvec4(lua_State *L, const lua_vec4 v4) { glm_pushvec4(glm_vec4_copy, *(lua_vec4 *)&v4[0]); }
LUA_API void lua_pushquat(lua_State *L, const lua_versor q) { glm_pushquat(glm_quat_copy, *(lua_versor *)&q[0]); }
LUA_API void lua_pushmat2(lua_State *L, const lua_mat2 m2) { glm_pushmat2(glm_mat2_copy, *(lua_mat2 *)&m2[0]); }
LUA_API void lua_pushmat3(lua_State *L, const lua_mat3 m3) { glm_pushmat3(glm_mat3_copy, *(lua_mat3 *)&m3[0]); }
LUA_API void lua_pushmat4(lua_State *L, const lua_mat4 m4) { glm_pushmat4(glm_mat4_copy, *(lua_mat4 *)&m4[0]); }

#define glm_tocheck(L, I, N, T) if (!T) luaL_typeerror(L, I, N)
LUA_API void luaL_checkvec2(lua_State *L, int idx, lua_vec2 v2) { glm_tocheck(L, idx, "vec2", lua_tovec2(L, idx, v2)); }
LUA_API void luaL_checkvec3(lua_State *L, int idx, lua_vec3 v3) { glm_tocheck(L, idx, "vec3", lua_tovec3(L, idx, v3)); }
LUA_API void luaL_checkvec4(lua_State *L, int idx, lua_vec4 v4) { glm_tocheck(L, idx, "vec4", lua_tovec4(L, idx, v4)); }
LUA_API void luaL_checkquat(lua_State *L, int idx, lua_versor q) { glm_tocheck(L, idx, "quat", lua_toquat(L, idx, q)); }
LUA_API void luaL_checkmat2(lua_State *L, int idx, lua_mat2 v2) { glm_tocheck(L, idx, "mat2", lua_tomat2(L, idx, v2)); }
LUA_API void luaL_checkmat3(lua_State *L, int idx, lua_mat3 v3) { glm_tocheck(L, idx, "mat3", lua_tomat3(L, idx, v3)); }
LUA_API void luaL_checkmat4(lua_State *L, int idx, lua_mat4 v4) { glm_tocheck(L, idx, "mat4", lua_tomat4(L, idx, v4)); }
#undef glm_tocheck

/* }================================================================== */

/*
** {==================================================================
** Constructors
** ===================================================================
*/

/*
** Unpack the value at the given index, storing its contents in 'sink' (with
** length 'vlength' and starting at index 'vidx'). This function assumes at
** least one slot in sink is available.
**
** @NVC: With optimizations enabled, generates confusing/incorrect assembly.
**       Last tested on nvc 23.1-0.
*/
static int glm_populateVector(lua_State *L, int idx, vec4 sink, int vidx, int vlength) {
#if defined(__NVCOMPILER)
  volatile
#endif
  int count;
  const TValue *o = glm_fasti2v(L, idx);
  lua_assert(vlength > 0);
  switch (ttypetag(o)) {
    /* primitive: cast & store it */
    case LUA_VNIL: count = 1; sink[vidx] = 0.0f; break;
    case LUA_VFALSE: count = 1; sink[vidx] = 0.0f; break;
    case LUA_VTRUE: count = 1; sink[vidx] = 1.0f; break;
    case LUA_VNUMINT: count = 1; sink[vidx] = glm_cf(ivalue(o)); break;
    case LUA_VNUMFLT: count = 1; sink[vidx] = glm_cf(fltvalue(o)); break;
    /* vector: concatenate elements (... up to 'vlength') */
    case LUA_VVECTOR2:
    case LUA_VVECTOR3:
    case LUA_VVECTOR4:
    case LUA_VQUAT: {
      lua_Float4 f4 = glm_vload(o);
      int i, len = cast_int(ttvlen(o));
      count = GLM_MIN(len, vlength);
      for (i = 0; i < count; ++i)
        sink[vidx++] = f4.v4[i];
      break;
    }
    /* array: concatenate values (... up to 'vlength') */
    case LUA_VTABLE: {
      lua_Unsigned i, array_len;
      lua_lock(L);
      array_len = luaH_getn(hvalue(o));
      if (array_len > cast(lua_Unsigned, vlength))
        array_len = cast(lua_Unsigned, vlength);

      count = cast_int(array_len);
      for (i = 1; i <= array_len; ++i) {
        const TValue *to = luaH_getint(hvalue(o), l_castU2S(i));
        if (!glm_castvalue(to, &sink[vidx++])) { /* primitive: cast & store */
          lua_unlock(L);
          return luaL_argerror(L, idx, "unexpected table value");
        }
      }
      lua_unlock(L);
      break;
    }
    default: {
      return luaL_argerror(L, idx, "unexpected argument");
    }
  }
  return count;
}

/* Populate the matrix m by unpacking the Lua stack beginning at idx. */
static int glm_populateMatrix(lua_State *L, int idx, int desired, mat4 m) {
  int columns = desired;
  const int top = lua_gettop(L);
  if (top < idx) /* no elements */
    glm_mat4_identity(m);
  else if (top == idx) { /* one element */
    TValue *o = (TValue *)glm_fasti2v(L, idx);
    switch (ttypetag(o)) {
      /* scalar: populate diagonal of matrix */
      case LUA_VNUMINT: glm_mat4_diag(m, glm_cf(ivalue(o))); break;
      case LUA_VNUMFLT: glm_mat4_diag(m, glm_cf(fltvalue(o))); break;
      /* quaternion: cast to rotation matrix */
      case LUA_VQUAT: glm_quat_mat4(glm_q(o), m); break;
      /* vector: populate diagonal of matrix */
      case LUA_VVECTOR2:
      case LUA_VVECTOR3:
      case LUA_VVECTOR4: {
        lua_Float4 f4 = glm_vload(o);
        lu_tag len = ttvlen(o);
        m[0][0] = f4.v4[0];
        m[1][1] = f4.v4[1];
        m[2][2] = (len > 2) ? f4.v4[2] : 0.0f;
        m[3][3] = (len > 3) ? f4.v4[3] : 0.0f;
        break;
      }
      /* matrix: down/up-cast */
      case LUA_VMATRIX2: glm_mat4_ins2(glm_m2(o), m); break;
      case LUA_VMATRIX3: glm_mat4_ins3(glm_m3(o), m); break;
      case LUA_VMATRIX4: glm_mat4_copy(glm_m4(o), m); break;
      default: {
        columns = -idx;
        break;
      }
    }
  }
  else { /* Parse column vectors */
    int i;
    for (i = 0; i < columns; ++i) {
      int count = glm_populateVector(L, idx++, m[i], 0, 4);
      if (l_unlikely(count < desired)) {
        columns = -(idx - 1);
        break;
      }
    }
  }
  return columns;
}

/* Generic vector construction function */
static int glm_constructVector(lua_State *L, int desired, vec4 v) {
  int len = desired ? desired : 3;
  int top = lua_gettop(L);
  if (top == 0) /* No arguments: zero vector */
    glm_vec4_zero(v);
  else if (top == 1 && glm_castvalue(glm_index2value(L, 1), &v[0])) /* One argument: broadcast */
    glm_vec4_broadcast(v[0], v);
  else { /* Parse objects on stack up to 'desired' */
    int i, max_len = (desired ? desired : 4);
    glm_vec4_zero(v);
    for (i = 1, len = 0; i <= top && len < max_len; ++i)
      len += glm_populateVector(L, i, v, len, max_len - len);
    if ((desired && len != desired) || (!desired && len < 2))
      return luaL_error(L, GLM_UNEXPECTED);
  }
  return len;
}

/* Create a single-precision floating-point vector */
static int glm_createVector(lua_State *L, int desired) {
  vec4 v;
  int len = glm_constructVector(L, desired, v);
  glm_pushvid(vvaltag(len), glm_vec4_copy, v);
  return 1;
}

/* Create a signed integer vector (as float) */
static int glm_createVectorInt(lua_State *L, int desired) {
  vec4 v;
  int len = glm_constructVector(L, desired, v);
  glm_pushvid(vvaltag(len), glm_vec4_trunc, v);
  return 1;
}

/* Create a boolean vector (as float) */
static int glm_createVectorBool(lua_State *L, int desired) {
  vec4 v, zero = GLM_VEC4_ZERO_INIT;
  int len = glm_constructVector(L, desired, v);
  glm_pushvid(vvaltag(len), glm_vec4_neqto, v, zero);
  return 1;
}

/* Generic matrix creation function */
static int glm_createMatrix(lua_State *L, int desired) {
  CGLM_ALIGN_MAT lua_Matrix r; /* @MatLayout: Realign */
  CGLM_ALIGN_MAT lua_Matrix m = { GLM_MAT4_ZERO_INIT };
  const TValue *o1 = glm_index2value(L, 1);
  int length = glm_populateMatrix(L, 1 + ttismatrix(o1), desired, m.m4);
  switch (length) {
    case 2: glm_mat4_pick2(m.m4, r.m2); glm_mat2_copy(r.m2, m.m2); break;
    case 3: glm_mat4_pick3(m.m4, r.m3); glm_mat3_copy(r.m3, m.m3); break;
    case 4: break;
    default: {
      return length < 0 ? luaL_argerror(L, -length, "unexpected argument")
                        : luaL_error(L, GLM_UNEXPECTED);
    }
  }
  glm_mid_recycle(1, mvaltag(length), glm_mat4_copy, m.m4);
  return 1;
}

int luaglm_vec(lua_State *L) { return glm_createVector(L, 0); }
int luaglm_vec2(lua_State *L) { return glm_createVector(L, 2); }
int luaglm_vec3(lua_State *L) { return glm_createVector(L, 3); }
int luaglm_vec4(lua_State *L) { return glm_createVector(L, 4); }
int luaglm_ivec(lua_State *L) { return glm_createVectorInt(L, 0); }
int luaglm_ivec2(lua_State *L) { return glm_createVectorInt(L, 2); }
int luaglm_ivec3(lua_State *L) { return glm_createVectorInt(L, 3); }
int luaglm_ivec4(lua_State *L) { return glm_createVectorInt(L, 4); }
int luaglm_bvec(lua_State *L) { return glm_createVectorBool(L, 0); }
int luaglm_bvec2(lua_State *L) { return glm_createVectorBool(L, 2); }
int luaglm_bvec3(lua_State *L) { return glm_createVectorBool(L, 3); }
int luaglm_bvec4(lua_State *L) { return glm_createVectorBool(L, 4); }
int luaglm_mat2x2(lua_State *L) { return glm_createMatrix(L, 2); }
int luaglm_mat3x3(lua_State *L) { return glm_createMatrix(L, 3); }
int luaglm_mat4x4(lua_State *L) { return glm_createMatrix(L, 4); }
int luaglm_quat(lua_State *L) {
  versor q;
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  if (!glm_isvalid(L, o1)) /* No arguments: return the identity */
    glm_quat_identity(q);
  else if (ttisnumber(o1)) {
    TValue *o2 = (TValue *)glm_index2value(L, 2);
    if (ttisvector3(o2)) /* <angle, axis> */
#if defined(LUAGLM_COMPAT_GLM_CPP) /* degrees for grit-lua compatibility */
      glm_quatv(q, glm_rad(glm_fv(o1)), glm_v3(o2));
#else
      glm_quatv(q, glm_fv(o1), glm_v3(o2));
#endif
    else if (ttisnumber(o2)) { /* <w, x, y, z> */
      float w = glm_fv(o1);
      float x = glm_fv(o2);
      float y = glm_checkfloat(L, 3);
      float z = glm_checkfloat(L, 4);
      glm_quat_init(q, x, y, z, w);
    }
    else {
      return luaL_error(L, "{w, x, y, z} or {angle, axis} expected");
    }
  }
  else if (ttisvector3(o1)) {
    TValue *o2 = (TValue *)glm_index2value(L, 2);
    TValue *o3 = (TValue *)glm_index2value(L, 3);
    if (ttisnumber(o2)) /* <xyz, w> */
      glm_vec4(glm_v3(o1), glm_fv(o2), q);
    else if (ttisvector3(o2) && !glm_isvalid(L, o3)) /* <from, to> */
      glm_quat_from_to(glm_v3(o1), glm_v3(o2), q);
    else if (ttisvector3(o2) && ttisvector3(o3)) /* <x_axis, y_axis, z_axis> */
      glm_quat_from_basis(glm_v3(o1), glm_v3(o2), glm_v3(o3), q);
    else {
      return luaL_error(L, "{xyz, w}, {from, to} or {x, y, z} basis expected");
    }
  }
  else if (ttisvector4(o1) || ttisquat(o1)) /* quat(quat) */
    glm_vec4_copy(glm_vid(o1), q);
  else if (ttismatrix(o1)) {
    lua_lock(L);
    switch (mvaltt(o1)) {
      case LUA_VMATRIX3: glm_mat3_quat(glm_m3(o1), q); break;
      case LUA_VMATRIX4: glm_mat4_quat(glm_m4(o1), q); break;
      default: {
        lua_unlock(L);
        return luaL_typeerror(L, 1, "mat3x3 or mat4x4");
      }
    }
    lua_unlock(L);
  }
  else
    return luaL_argerror(L, 1, GLM_UNEXPECTED);
  glm_pushquat(glm_quat_copy, q);
  return 1;
}

/* }================================================================== */

/*
** {==================================================================
** gsl library
** ===================================================================
*/

/* F(Vec|Number) */
#define COMPONENT_FUNC3(VF, NF, NP)             \
  LUA_MLM_BEGIN                                 \
  TValue *O1 = (TValue *)glm_index2value(L, 1); \
  if (ttisnumber(O1))                           \
    NP(L, NF(nvalue(O1)));                      \
  else if (ttisvector(O1))                      \
    glm_pushvid(vvaltt(O1), VF, glm_vid(O1));   \
  else                                          \
    return luaL_error(L, GLM_UNEXPECTED);       \
  return 1;                                     \
  LUA_MLM_END

/* F(Vec|Quat|Number) */
#define COMPONENT_FUNC4(VF, QF, NF, NP)         \
  LUA_MLM_BEGIN                                 \
  TValue *O1 = (TValue *)glm_index2value(L, 1); \
  if (ttisnumber(O1))                           \
    NP(L, NF(nvalue(O1)));                      \
  else if (ttisvector(O1) && !ttisquat(O1))     \
    glm_pushvid(vvaltt(O1), VF, glm_vid(O1));   \
  else if (ttisquat(O1))                        \
    glm_pushquat(QF, glm_q(O1));                \
  else                                          \
    return luaL_error(L, GLM_UNEXPECTED);       \
  return 1;                                     \
  LUA_MLM_END

/* F(Vec|Float|Integer) */
#define COMPONENT_FUNC5(VF, NF, NP, IF, IP)     \
  LUA_MLM_BEGIN                                 \
  TValue *O1 = (TValue *)glm_index2value(L, 1); \
  if (ttisfloat(O1))                            \
    NP(L, NF(fltvalue(O1)));                    \
  else if (ttisinteger(O1))                     \
    IP(L, IF(ivalue(O1)));                      \
  else if (ttisvector(O1))                      \
    glm_pushvid(vvaltt(O1), VF, glm_vid(O1));   \
  else                                          \
    return luaL_error(L, GLM_UNEXPECTED);       \
  return 1;                                     \
  LUA_MLM_END

/* F(Vec|Number, Vec|Number) */
#define COMPONENT_BFUNC3(VF, NF, NP)                       \
  LUA_MLM_BEGIN                                            \
  TValue *O1 = (TValue *)glm_index2value(L, 1);            \
  TValue *O2 = (TValue *)glm_index2value(L, 2);            \
  if (ttisnumber(O1) && ttisnumber(O2))                    \
    NP(L, NF(nvalue(O1), nvalue(O2)));                     \
  else if (ttisvector(O1) && tteq(O1, O2))                 \
    glm_pushvid(vvaltt(O1), VF, glm_vid(O1), glm_vid(O2)); \
  else if (ttisvector(O1) && ttisnumber(O2)) {             \
    vec4 V2;                                               \
    glm_vec4_broadcast(glm_fv(O2), V2);                    \
    glm_pushvid(vvaltt(O1), VF, glm_vid(O1), V2);          \
  }                                                        \
  else                                                     \
    return luaL_error(L, GLM_UNEXPECTED);                  \
  return 1;                                                \
  LUA_MLM_END

/* F(Vec|Float|Integer, Vec|Float|Integer) */
#define COMPONENT_BFUNC5(VF, NF, NP, IF, IP)               \
  LUA_MLM_BEGIN                                            \
  TValue *O1 = (TValue *)glm_index2value(L, 1);            \
  TValue *O2 = (TValue *)glm_index2value(L, 2);            \
  if (ttisinteger(O1) && ttisinteger(O2))                  \
    IP(L, IF(ivalue(O1), ivalue(O2)));                     \
  else if (ttisnumber(O1) && ttisnumber(O2))               \
    NP(L, NF(nvalue(O1), nvalue(O2)));                     \
  else if (ttisvector(O1) && tteq(O1, O2))                 \
    glm_pushvid(vvaltt(O1), VF, glm_vid(O1), glm_vid(O2)); \
  else if (ttisvector(O1) && ttisnumber(O2)) {             \
    vec4 V2;                                               \
    glm_vec4_broadcast(glm_fv(O2), V2);                    \
    glm_pushvid(vvaltt(O1), VF, glm_vid(O1), V2);          \
  }                                                        \
  else                                                     \
    return luaL_error(L, GLM_UNEXPECTED);                  \
  return 1;                                                \
  LUA_MLM_END

/* GLSL implementations for lua_Number and lua_Interger */
#define l_nopI(I) I
#define l_falseI(I) 0
#define l_absI(I) (((I) < 0) ? l_castU2S(0u - l_castS2U((I))) : (I))
#define l_signI(I) (((I) > 0) - ((I) < 0))
#define l_clampN(N, L, H) ((N) <= (L) ? (L) : ((N) >= (H) ? (H) : (N)))
#define l_radN(N) ((N) * (cast_num(GLM_PI) / l_mathop(180.0)))
#define l_degN(N) ((N) * (l_mathop(180.0) / cast_num(GLM_PI)))
#define l_asinN(N) l_mathop(asin)(l_clampN((N), l_mathop(-1.0), l_mathop(1.0)))
#define l_acosN(N) l_mathop(acos)(l_clampN((N), l_mathop(-1.0), l_mathop(1.0)))
#define l_acoshN(N) ((N) <= l_mathop(1.0) ? l_mathop(0.0) : l_mathop(acosh)(N))
#define l_atanhN(N) l_mathop(atanh)(l_clampN((N), l_mathop(-1.0), l_mathop(1.0)))
#define l_rsqrtN(N) (l_mathop(1.0) / l_mathop(sqrt)(N))
#define l_signN(N) (cast_num(((N) > l_mathop(0.0)) - ((N) < l_mathop(0.0))))
#define l_roundN(N) ((N) - l_mathop(remainder)((N), l_mathop(1.0))) /* round towards even */
#define l_fractN(N) ((N) - l_mathop(floor)(N))
static LUA_INLINE lua_Number l_logN(lua_Number x, lua_Number base) {
  if (base == l_mathop(2.0)) return l_mathop(log2)(x);
  if (base == l_mathop(10.0)) return l_mathop(log10)(x);
  return l_mathop(log)(x) / l_mathop(log)(base);
}

/* 8.1. Angle and Trigonometry Functions */
int luaglm_rad(lua_State *L) { COMPONENT_FUNC3(glm_vec4_rad, l_radN, glm_pushnumber); }
int luaglm_deg(lua_State *L) { COMPONENT_FUNC3(glm_vec4_deg, l_degN, glm_pushnumber); }
int luaglm_sin(lua_State *L) { COMPONENT_FUNC3(glm_vec4_sin, l_mathop(sin), glm_pushnumber); }
int luaglm_cos(lua_State *L) { COMPONENT_FUNC3(glm_vec4_cos, l_mathop(cos), glm_pushnumber); }
int luaglm_tan(lua_State *L) { COMPONENT_FUNC3(glm_vec4_tan, l_mathop(tan), glm_pushnumber); }
int luaglm_asin(lua_State *L) { COMPONENT_FUNC3(glm_vec4_asin, l_asinN, glm_pushnumber); }
int luaglm_acos(lua_State *L) { COMPONENT_FUNC3(glm_vec4_acos, l_acosN, glm_pushnumber); }
int luaglm_sinh(lua_State *L) { COMPONENT_FUNC3(glm_vec4_sinh, l_mathop(sinh), glm_pushnumber); }
int luaglm_cosh(lua_State *L) { COMPONENT_FUNC3(glm_vec4_cosh, l_mathop(cosh), glm_pushnumber); }
int luaglm_tanh(lua_State *L) { COMPONENT_FUNC3(glm_vec4_tanh, l_mathop(tanh), glm_pushnumber); }
int luaglm_asinh(lua_State *L) { COMPONENT_FUNC3(glm_vec4_asinh, l_mathop(asinh), glm_pushnumber); }
int luaglm_acosh(lua_State *L) { COMPONENT_FUNC3(glm_vec4_acosh, l_acoshN, glm_pushnumber); }
int luaglm_atanh(lua_State *L) { COMPONENT_FUNC3(glm_vec4_atanh, l_atanhN, glm_pushnumber); }
int luaglm_atan(lua_State *L) {
  if (lua_isnoneornil(L, 2))
    COMPONENT_FUNC3(glm_vec4_atan, l_mathop(atan), glm_pushnumber);
  COMPONENT_BFUNC3(glm_vec4_atan2, l_mathop(atan2), glm_pushnumber);
}

/* 8.2. Exponential Functions */
int luaglm_exp2(lua_State *L) { COMPONENT_FUNC3(glm_vec4_exp2, l_mathop(exp2), glm_pushnumber); }
int luaglm_log2(lua_State *L) { COMPONENT_FUNC3(glm_vec4_log2, l_mathop(log2), glm_pushnumber); }
int luaglm_sqrt(lua_State *L) { COMPONENT_FUNC4(glm_vec4_sqrt, glm_quat_sqrt, l_mathop(sqrt), glm_pushnumber); }
int luaglm_invsqrt(lua_State *L) { COMPONENT_FUNC4(glm_vec4_invsqrt, glm_quat_invsqrt, l_rsqrtN, glm_pushnumber); }
int luaglm_exp(lua_State *L) { COMPONENT_FUNC4(glm_vec4_exp, glm_quat_exp, l_mathop(exp), glm_pushnumber); }
int luaglm_log(lua_State *L) {
  if (lua_isnoneornil(L, 2))
    COMPONENT_FUNC4(glm_vec4_log, glm_quat_log, l_mathop(log), glm_pushnumber);
  COMPONENT_BFUNC3(glm_vec4_logB, l_logN, glm_pushnumber);
}

int luaglm_pow(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  if (ttisquat(o1) && ttisnumber(o2)) {
    glm_pushquat(glm_quat_pow, glm_q(o1), glm_fv(o2));
    return 1;
  }
  COMPONENT_BFUNC3(glm_vec4_pow, l_mathop(pow), glm_pushnumber);
}

/* 8.3. Common Functions */
int luaglm_abs(lua_State *L) { COMPONENT_FUNC5(glm_vec4_abs, l_mathop(fabs), glm_pushnumber, l_absI, glm_pushinteger); }
int luaglm_sign(lua_State *L) { COMPONENT_FUNC5(glm_vec4_sign, l_signN, glm_pushnumber, l_signI, glm_pushinteger); }
int luaglm_floor(lua_State *L) { COMPONENT_FUNC5(glm_vec4_floor_simd, l_mathop(floor), glm_pushnumint, l_nopI, glm_pushinteger); }
int luaglm_ceil(lua_State *L) { COMPONENT_FUNC5(glm_vec4_ceil, l_mathop(ceil), glm_pushnumint, l_nopI, glm_pushinteger); }
int luaglm_trunc(lua_State *L) { COMPONENT_FUNC5(glm_vec4_trunc, l_mathop(trunc), glm_pushnumint, l_nopI, glm_pushinteger); }
int luaglm_round(lua_State *L) { COMPONENT_FUNC5(glm_vec4_round, l_roundN, glm_pushnumint, l_nopI, glm_pushinteger); }
int luaglm_fract(lua_State *L) { COMPONENT_FUNC5(glm_vec4_fract_simd, l_fractN, glm_pushnumber, l_nopI, glm_pushinteger); }
int luaglm_isnan(lua_State *L) { COMPONENT_FUNC5(glm_vec4_isnanv, isnan, glm_pushboolean, l_falseI, glm_pushboolean); }
int luaglm_isinf(lua_State *L) { COMPONENT_FUNC5(glm_vec4_isinfv, isinf, glm_pushboolean, l_falseI, glm_pushboolean); }
int luaglm_min(lua_State *L) { COMPONENT_BFUNC5(glm_vec4_minv, l_mathop(fmin), glm_pushnumber, GLM_MIN, glm_pushinteger); }
int luaglm_max(lua_State *L) { COMPONENT_BFUNC5(glm_vec4_maxv, l_mathop(fmax), glm_pushnumber, GLM_MAX, glm_pushinteger); }
int luaglm_mod(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  if (ttisinteger(o1) && ttisinteger(o2)) { /* math_fmod */
    lua_Integer d = ivalue(o2);
    luaL_argcheck(L, d != 0, 2, "zero");
    if (l_castS2U(d) + 1u <= 1u) /* special cases: -1 or 0 */
      glm_pushinteger(L, 0); /* avoid overflow with 0x80000... / -1 */
    else
      glm_pushinteger(L, ivalue(o1) % d);
    return 1;
  }
  /* Implementation different from GLSL to be consistent with lmathlib */
  COMPONENT_BFUNC3(glm_vec4_mod_simd, l_mathop(fmod), glm_pushnumber);
}

int luaglm_modf(lua_State *L) { /* math_modf compatible */
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  if (ttisvector(o1)) {
    vec4 integral, fract;
    glm_vec4_modf(glm_vid(o1), integral, fract);
    glm_pushvid(vvaltt(o1), glm_vec4_copy, integral);
    glm_pushvid(vvaltt(o1), glm_vec4_copy, fract);
  }
  else if (ttisinteger(o1)) {
    lua_settop(L, 1); /* number is its own integer part */
    glm_pushnumber(L, 0); /* no fractional part */
  }
  else {
    lua_Number n = glm_checknumber(L, 1);
    lua_Number ip = (n < 0) ? l_mathop(ceil)(n) : l_mathop(floor)(n);
    glm_pushnumint(L, ip); /* integer part (rounds toward zero) */
    glm_pushnumber(L, (n == ip) ? l_mathop(0.0) : (n - ip)); /* fractional part (test needed for inf/-inf) */
  }
  return 2;
}

int luaglm_clamp(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  TValue *o3 = (TValue *)glm_index2value(L, 3);
  int both_nil = ttisnil(o2) && ttisnil(o3); /* saturate(...) */
  if (ttisvector(o1) && tteq(o1, o2) && tteq(o2, o3))
    glm_pushvid(vvaltt(o1), glm_vec4_clampv, glm_vid(o1), glm_vid(o2), glm_vid(o3));
  else if (ttisinteger(o1) && (both_nil || (ttisinteger(o2) && ttisinteger(o3)))) {
    lua_Integer v = ivalue(o1);
    lua_Integer minVal = both_nil ? 0 : ivalue(o2);
    lua_Integer maxVal = both_nil ? 1 : ivalue(o3);
    glm_pushinteger(L, GLM_MIN(GLM_MAX(v, minVal), maxVal));
  }
  else if (both_nil || (ttisnumber(o2) && ttisnumber(o3))) {
    lua_Number minVal = both_nil ? l_mathop(0.0) : nvalue(o2);
    lua_Number maxVal = both_nil ? l_mathop(1.0) : nvalue(o3);
    if (ttisvector(o1))
      glm_pushvid(vvaltt(o1), glm_vec4_clamp_to, glm_vid(o1), glm_cf(minVal), glm_cf(maxVal));
    else if (ttisnumber(o1))
      glm_pushnumber(L, l_mathop(fmin)(l_mathop(fmax)(nvalue(o1), minVal), maxVal));
    else
      return luaL_error(L, GLM_UNEXPECTED);
  }
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 1;
}

int luaglm_mix(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  TValue *o3 = (TValue *)glm_index2value(L, 3);
  if (ttisvector(o1) && tteq(o1, o2) && tteq(o2, o3)) /* mix(vec, vec, vec) */
    glm_pushvid(vvaltt(o1), glm_vec4_mixv, glm_vid(o1), glm_vid(o2), glm_vid(o3));
  else if (ttisvector(o1) && tteq(o1, o2) && ttisnumber(o3)) /* mix(vec, vec, scalar) */
    glm_pushvid(vvaltt(o1), glm_vec4_mix, glm_vid(o1), glm_vid(o2), glm_fv(o3));
  else if (ttisnumber(o1) && ttisnumber(o2) && ttisnumber(o3)) /* mix(number, number, number) */
    glm_pushnumber(L, nvalue(o1) + nvalue(o3) * (nvalue(o2) - nvalue(o1)));
  else if (ttisboolean(o3)) /* mix(X, Y, bool) */
    lua_pushvalue(L, ttistrue(o3) ? 2 : 1);
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 1;
}

int luaglm_step(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  if (ttisvector(o1) && tteq(o1, o2))
    glm_pushvid(vvaltt(o1), glm_vec4_step_simd, glm_vid(o1), glm_vid(o2));
  else if (ttisnumber(o1) && ttisvector(o2))
    glm_pushvid(vvaltt(o2), glm_vec4_step_uni_simd, glm_fv(o1), glm_vid(o2));
  else if (ttisnumber(o1) && ttisnumber(o2))
    glm_pushnumber(L, (nvalue(o2) < nvalue(o1)) ? l_mathop(0.0) : l_mathop(1.0));
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 1;
}

int luaglm_smoothstep(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  TValue *o3 = (TValue *)glm_index2value(L, 3);
  if (ttisvector(o1) && tteq(o1, o2) && tteq(o2, o3))
    glm_pushvid(vvaltt(o1), glm_vec4_smoothstep_simd, glm_vid(o1), glm_vid(o2), glm_vid(o3));
  else if (ttisnumber(o1) && ttisnumber(o2) && ttisvector(o3))
    glm_pushvid(vvaltt(o3), glm_vec4_smoothstep_uni_simd, glm_fv(o1), glm_fv(o2), glm_vid(o3));
  else if (ttisnumber(o1) && ttisnumber(o2) && ttisnumber(o3)) {
    lua_Number e0 = nvalue(o1);
    lua_Number e1 = nvalue(o2);
    lua_Number x = nvalue(o3);
    lua_Number c = (x - e0) / (e1 - e0);
    lua_Number t = l_mathop(fmin)(l_mathop(fmax)(c, l_mathop(0.0)), l_mathop(1.0));
    glm_pushnumber(L, t * t * (l_mathop(3.0) - l_mathop(2.0) * t));
  }
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 1;
}

int luaglm_fma(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  TValue *o3 = (TValue *)glm_index2value(L, 3);
  if (ttisvector(o1) && tteq(o1, o2) && tteq(o2, o3))
    glm_pushvid(vvaltt(o1), glm_vec4_fma, glm_vid(o1), glm_vid(o2), glm_vid(o3));
  else if (ttisinteger(o1) && ttisinteger(o2) && ttisinteger(o3)) {
    lua_Integer m = luaL_intop(*, ivalue(o1), ivalue(o2));
    glm_pushinteger(L, luaL_intop(+, m, ivalue(o3)));
  }
  else if (ttisnumber(o1) && ttisnumber(o2) && ttisnumber(o3))
    glm_pushnumber(L, l_mathop(fma)(nvalue(o1), nvalue(o2), nvalue(o3)));
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 1;
}

int luaglm_frexp(lua_State *L) {
  CGLM_ALIGN(16) ivec4 exp;
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  glm_ivec4_broadcast(0, exp);
  if (ttisvector(o1)) {
    glm_pushvid(vvaltt(o1), glm_vec4_frexp, glm_vid(o1), exp);
    glm_pushvid(vvaltt(o1), glm_vec4i, exp);
  }
  else if (ttisnumber(o1)) {
    glm_pushnumber(L, l_mathop(frexp)(nvalue(o1), &exp[0]));
    glm_pushinteger(L, cast(lua_Integer, exp[0]));
  }
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 2;
}

int luaglm_ldexp(lua_State *L) {
  CGLM_ALIGN(16) ivec4 exp;
  lua_Integer r, i;
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  if (ttisvector(o1) && tteq(o1, o2)) {
    glm_ivec4f(glm_vid(o2), exp);
    glm_pushvid(vvaltt(o1), glm_vec4_ldexp, glm_vid(o1), exp);
  }
  else if ((r = tointegerns(o2, &i), r) && ttisvector(o1)) {
    glm_ivec4_broadcast(cast_int(i), exp);
    glm_pushvid(vvaltt(o1), glm_vec4_ldexp, glm_vid(o1), exp);
  }
  else if (r && ttisnumber(o1))
    glm_pushnumber(L, l_mathop(ldexp)(nvalue(o1), cast_int(i)));
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 1;
}

/* 8.4. Floating-Point Pack and Unpack Functions */
/* 8.5. Geometric Functions */
int luaglm_length(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  if (ttisinteger(o1))
    glm_pushinteger(L, 1);
  else if (ttisnumber(o1))
    glm_pushnumber(L, l_mathop(1.0));
  else {
    lua_lock(L);
    luaV_objlen(L, L->top.p, o1, 0);
    api_incr_top(L);
    lua_unlock(L);
  }
  return 1;
}

int luaglm_distance(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  if (ttisvector(o1) && tteq(o1, o2)) {
    float result;
    switch (vvaltt(o1)) {
      case LUA_VVECTOR2: result = glm_vec2_distance(glm_v2(o1), glm_v2(o2)); break;
      case LUA_VVECTOR3: result = glm_vec3_distance(glm_v3(o1), glm_v3(o2)); break;
      case LUA_VVECTOR4:
      case LUA_VQUAT:
      default: {
        result = glm_vec4_distance(glm_vid(o1), glm_vid(o2));
        break;
      }
    }
    glm_pushnumber(L, cast_num(result));
  }
  else if (ttisinteger(o1) && ttisinteger(o2)) {
    lua_Integer i = luaL_intop(-, ivalue(o1), ivalue(o2));
    glm_pushinteger(L, l_absI(i));
  }
  else if (ttisnumber(o1) && ttisnumber(o2)) /* integer coercion */
    glm_pushnumber(L, l_mathop(fabs)(nvalue(o1) - nvalue(o2)));
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 1;
}

int luaglm_dot(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  if (ttisvector(o1) && tteq(o1, o2)) {
    float dot;
    switch (vvaltt(o1)) {
      case LUA_VVECTOR2: dot = glm_vec2_dot(glm_v2(o1), glm_v2(o2)); break;
      case LUA_VVECTOR3: dot = glm_vec3_dot(glm_v3(o1), glm_v3(o2)); break;
      case LUA_VVECTOR4:
      case LUA_VQUAT:
      default: {
        dot = glm_vec4_dot(glm_vid(o1), glm_vid(o2));
        break;
      }
    }
    glm_pushnumber(L, cast_num(dot));
  }
  else if (ttisinteger(o1) && ttisinteger(o2))
    glm_pushinteger(L, ivalue(o1) * ivalue(o2));
  else if (ttisnumber(o1) && ttisnumber(o2)) /* integer coercion */
    glm_pushnumber(L, nvalue(o1) * nvalue(o2));
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 1;
}

int luaglm_cross(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  if (ttisvector3(o1) && ttisvector3(o2))
    glm_pushvec3(glm_vec3_cross_simd, glm_v3(o1), glm_v3(o2));
  else if (ttisvector3(o1) && ttisquat(o2))
    glm_pushvec3(glm_vec3_rotateq_simd, glm_v3(o1), glm_q(o2));
  else if (ttisquat(o1) && ttisquat(o2))
    glm_pushquat(glm_quat_mul, glm_q(o1), glm_q(o2));
  else if (ttisquat(o1) && ttisvector3(o2))
    glm_pushvec3(glm_quat_rotatev_simd, glm_q(o1), glm_v3(o2));
  else if (ttisvector2(o1) && ttisvector2(o2))
    glm_pushnumber(L, cast_num(glm_vec2_cross(glm_v2(o1), glm_v2(o2))));
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 1;
}

int luaglm_normalize(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  switch (vvaltt(o1)) {
    case LUA_VNUMINT: glm_pushinteger(L, 1); break;
    case LUA_VNUMFLT: glm_pushnumber(L, l_mathop(1.0)); break;
    case LUA_VVECTOR2: glm_pushvec2(glm_vec2_normalize_to, glm_v2(o1)); break;
    case LUA_VVECTOR3: glm_pushvec3(glm_vec3_normalize_to, glm_v3(o1)); break;
    case LUA_VVECTOR4: glm_pushvec4(glm_vec4_normalize_to, glm_v4(o1)); break;
    case LUA_VQUAT: glm_pushquat(glm_quat_normalize_to, glm_q(o1)); break;
    default: {
      return luaL_error(L, GLM_UNEXPECTED);
    }
  }
  return 1;
}

int luaglm_faceforward(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  TValue *o3 = (TValue *)glm_index2value(L, 3);
  if (ttisvector(o1) && tteq(o1, o2) && tteq(o2, o3)) {
    vec4 n, i, nr;
    glm_vec4_copy(glm_vid(o1), n);
    glm_vec4_copy(glm_vid(o2), i);
    glm_vec4_copy(glm_vid(o3), nr);
    switch (vvaltt(o1)) {
      case LUA_VVECTOR2: n[2] = n[3] = i[2] = i[3] = nr[2] = nr[3] = 0.0f; break;
      case LUA_VVECTOR3: n[3] = i[3] = nr[3] = 0.0f; break;
      default: {
        break;
      }
    }
    glm_pushvid(vvaltt(o1), glm_vec4_faceforward, n, i, nr);
  }
  else if (ttisnumber(o1) && ttisnumber(o2) && ttisnumber(o3)) {
    lua_Number n = nvalue(o1);
    lua_Number dot = nvalue(o3) * nvalue(o2);
    glm_pushnumber(L, dot < l_mathop(0.0) ? n : -n);
  }
  else {
    return luaL_error(L, GLM_UNEXPECTED);
  }
  return 1;
}

int luaglm_reflect(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  if (ttisvector(o1) && tteq(o1, o2)) {
    vec4 i, n;
    glm_vec4_copy(glm_vid(o1), i);
    glm_vec4_copy(glm_vid(o2), n);
    switch (vvaltt(o1)) {
      case LUA_VVECTOR2: i[2] = i[3] = n[2] = n[3] = 0.0f; break;
      case LUA_VVECTOR3: i[3] = n[3] = 0.0f; break;
      default: {
        break;
      }
    }
    glm_pushvid(vvaltt(o1), glm_vec4_reflect_simd, i, n);
  }
  else if (ttisnumber(o1) && ttisnumber(o2)) {
    lua_Number i = nvalue(o1);
    lua_Number n = nvalue(o2);
    lua_Number dot = n * i;
    glm_pushnumber(L, i - (l_mathop(2.0) * dot * n));
  }
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 1;
}

int luaglm_refract(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  TValue *o3 = (TValue *)glm_index2value(L, 3);
  if (ttisvector(o1) && ttisnumber(o3) && tteq(o1, o2)) {
    vec4 i, n;
    glm_vec4_copy(glm_vid(o1), i);
    glm_vec4_copy(glm_vid(o2), n);
    switch (vvaltt(o1)) {
      case LUA_VVECTOR2: i[2] = i[3] = n[2] = n[3] = 0.0f; break;
      case LUA_VVECTOR3: i[3] = n[3] = 0.0f; break;
      default: {
        break;
      }
    }
    glm_pushvid(vvaltt(o1), glm_vec4_refract_simd, i, n, glm_fv(o3));
  }
  else if (ttisnumber(o1) && ttisnumber(o2) && ttisnumber(o3)) {
    lua_Number i = nvalue(o1);
    lua_Number n = nvalue(o2);
    lua_Number eta = nvalue(o3);
    lua_Number dot = n * i;
    lua_Number k = l_mathop(1.0) - eta * eta * (l_mathop(1.0) - dot * dot);
    if (k >= 0)
      glm_pushnumber(L, eta * i - (eta * dot + l_mathop(sqrt)(k)) * n);
    else
      glm_pushnumber(L, l_mathop(0.0));
  }
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 1;
}

/* 8.6. Matrix Functions */
int luaglm_matrixcompmult(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  if (ttismatrix(o1) && tteq(o1, o2)) {
    glm_mid_recycle(3, mvaltt(o1), glm_mat4_compmult, glm_mid(o1), glm_mid(o2));
    return 1;
  }
  return luaL_error(L, GLM_UNEXPECTED);
}

int luaglm_outerproduct(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  if (ttisvector(o1) && tteq(o1, o2)) {
    CGLM_ALIGN_MAT lua_Matrix m = { GLM_MAT4_ZERO_INIT };
    lu_tag len = ttvlen(o1);
    switch (len) {
      case 2: glm_mat2_outerproduct(glm_v2(o1), glm_v2(o2), m.m2); break;
      case 3: glm_mat3_outerproduct(glm_v3(o1), glm_v3(o2), m.m3); break;
      case 4:
      default: {
        glm_mat4_outerproduct(glm_vid(o1), glm_vid(o2), m.m4);
        break;
      }
    }
    glm_mid_recycle(3, mvaltag(len), glm_mat4_copy, m.m4);
    return 1;
  }
  return luaL_error(L, GLM_UNEXPECTED);
}

int luaglm_transpose(lua_State *L) {
  CGLM_ALIGN_MAT lua_Matrix m = { GLM_MAT4_ZERO_INIT };
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  switch (mvaltt(o1)) {
    case LUA_VMATRIX2: glm_mat2_transpose_to(glm_m2(o1), m.m2); break;
    case LUA_VMATRIX3: glm_mat3_transpose_to(glm_m3(o1), m.m3); break;
    case LUA_VMATRIX4: glm_mat4_transpose_to(glm_m4(o1), m.m4); break;
    default: {
      return luaL_error(L, GLM_UNEXPECTED);
    }
  }
  glm_mid_recycle(2, mvaltt(o1), glm_mat4_copy, m.m4);
  return 1;
}

int luaglm_det(lua_State *L) {
  float det;
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  switch (mvaltt(o1)) {
    case LUA_VMATRIX2: det = glm_mat2_det(glm_m2(o1)); break;
    case LUA_VMATRIX3: det = glm_mat3_det(glm_m3(o1)); break;
    case LUA_VMATRIX4: det = glm_mat4_det(glm_m4(o1)); break;
    default: {
      return luaL_error(L, GLM_UNEXPECTED);
    }
  }
  glm_pushnumber(L, cast_num(det));
  return 1;
}

int luaglm_inverse(lua_State *L) {
  lua_Number n1 = l_mathop(0.0);
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  if (ttisquat(o1))
    glm_pushquat(glm_quat_inv_simd, glm_q(o1));
  else if (ttismatrix(o1)) {
    CGLM_ALIGN_MAT lua_Matrix m = { GLM_MAT4_ZERO_INIT };
    switch (mvaltt(o1)) {
      case LUA_VMATRIX2: glm_mat2_inv(glm_m2(o1), m.m2); break;
      case LUA_VMATRIX3: glm_mat3_inv(glm_m3(o1), m.m3); break;
      case LUA_VMATRIX4:
      default: {
        glm_mat4_inv(glm_m4(o1), m.m4);
        break;
      }
    }
    glm_mid_recycle(2, mvaltt(o1), glm_mat4_copy, m.m4);
  }
  else if (ttisvector(o1))
    glm_pushvid(vvaltt(o1), glm_vec4_sdiv, 1.0f, glm_vid(o1));
  else if (tonumberns(o1, n1))
    glm_pushnumber(L, l_mathop(1.0) / n1);
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 1;
}

/* 8.7. Vector Relational Functions */
int luaglm_lessThan(lua_State *L) { COMPONENT_BFUNC5(glm_vec4_lessthan, luai_numle, glm_pushboolean, luai_numle, glm_pushboolean); }
int luaglm_lessThanEqual(lua_State *L) { COMPONENT_BFUNC5(glm_vec4_lessthaneq, luai_numle, glm_pushboolean, luai_numle, glm_pushboolean); }
int luaglm_greaterThan(lua_State *L) { COMPONENT_BFUNC5(glm_vec4_greaterthan, luai_numgt, glm_pushboolean, luai_numgt, glm_pushboolean); }
int luaglm_greaterThanEqual(lua_State *L) { COMPONENT_BFUNC5(glm_vec4_greaterthaneq, luai_numge, glm_pushboolean, luai_numge, glm_pushboolean); }
int luaglm_equal(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  if (ttisvector(o1) && tteq(o1, o2)) {
    lu_tag tag = ttisquat(o1) ? LUA_VVECTOR4 : vvaltt(o1);
    glm_pushvid(tag, glm_vec4_eqto, glm_vid(o1), glm_vid(o2));
  }
  else if (ttismatrix(o1) && tteq(o1, o2)) {
    switch (mvaltt(o1)) {
      case LUA_VMATRIX2: glm_pushvec2(glm_mat2_eqv, glm_m2(o1), glm_m2(o2)); break;
      case LUA_VMATRIX3: glm_pushvec3(glm_mat3_eqv, glm_m3(o1), glm_m3(o2)); break;
      case LUA_VMATRIX4:
      default: {
        glm_pushvec4(glm_mat4_eqv, glm_m4(o1), glm_m4(o2));
        break;
      }
    }
  }
  else
    glm_pushboolean(L, lua_compare(L, 1, 2, LUA_OPEQ));
  return 1;
}

int luaglm_notEqual(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  if (ttisvector(o1) && tteq(o1, o2)) {
    lu_tag tag = ttisquat(o1) ? LUA_VVECTOR4 : vvaltt(o1);
    glm_pushvid(tag, glm_vec4_neqto, glm_vid(o1), glm_vid(o2));
  }
  else if (ttismatrix(o1) && tteq(o1, o2)) {
    switch (mvaltt(o1)) {
      case LUA_VMATRIX2: glm_pushvec2(glm_mat2_neqv, glm_m2(o1), glm_m2(o2)); break;
      case LUA_VMATRIX3: glm_pushvec3(glm_mat3_neqv, glm_m3(o1), glm_m3(o2)); break;
      case LUA_VMATRIX4:
      default: {
        glm_pushvec4(glm_mat4_neqv, glm_m4(o1), glm_m4(o2));
        break;
      }
    }
  }
  else
    glm_pushboolean(L, !lua_compare(L, 1, 2, LUA_OPEQ));
  return 1;
}

int luaglm_not(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  if (ttisboolean(o1) || ttisnil(o1)) {
    glm_pushboolean(L, l_isfalse(o1));
    return 1;
  }
#define l_nz(a) luai_numeq(a, 0)
  COMPONENT_FUNC5(glm_vec4_not, l_nz, glm_pushboolean, l_nz, glm_pushboolean);
#undef l_nz
}

int luaglm_any(lua_State *L) {
  int any;
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  switch (vvaltt(o1)) {
    case LUA_VFALSE: any = 0; break;
    case LUA_VTRUE: any = 1; break;
    case LUA_VNUMINT: any = ivalue(o1) != 0; break;
    case LUA_VNUMFLT: any = fltvalue(o1) != l_mathop(0.0); break;
    case LUA_VVECTOR2: any = glm_vec2_any(glm_v2(o1)); break;
    case LUA_VVECTOR3: any = glm_vec3_any(glm_v3(o1)); break;
    case LUA_VVECTOR4: any = glm_vec4_any(glm_v4(o1)); break;
    case LUA_VQUAT: any = glm_vec4_any(glm_q(o1)); break;
    default: {
      any = glm_isvalid(L, o1);
      break;
    }
  }
  glm_pushboolean(L, any);
  return 1;
}

int luaglm_all(lua_State *L) {
  int all;
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  switch (vvaltt(o1)) {
    case LUA_VFALSE: all = 0; break;
    case LUA_VTRUE: all = 1; break;
    case LUA_VNUMINT: all = ivalue(o1) != 0; break;
    case LUA_VNUMFLT: all = fltvalue(o1) != l_mathop(0.0); break;
    case LUA_VVECTOR2: all = glm_vec2_all(glm_v2(o1)); break;
    case LUA_VVECTOR3: all = glm_vec3_all(glm_v3(o1)); break;
    case LUA_VVECTOR4: all = glm_vec4_all(glm_v4(o1)); break;
    case LUA_VQUAT: all = glm_vec4_all(glm_q(o1)); break;
    default: {
      all = glm_isvalid(L, o1);
      break;
    }
  }
  glm_pushboolean(L, all);
  return 1;
}

/* 8.8. Integer Functions */
static lua_Unsigned l_bitCount(lua_Unsigned v) {
  lua_Unsigned c; /* https://graphics.stanford.edu/~seander/bithacks.html#CountBitsSet64 */
#define T lua_Unsigned
  v = v - ((v >> 1) & (T) ~(T)0 / 3);
  v = (v & (T) ~(T)0 / 15 * 3) + ((v >> 2) & (T) ~(T)0 / 15 * 3);
  v = (v + (v >> 4)) & (T) ~(T)0 / 255 * 15;
  c = (T)(v * ((T) ~(T)0 / 255)) >> (sizeof(T) - 1) * CHAR_BIT;
#undef T
  return c;
}

int luaglm_bitfieldExtract(lua_State *L) {
  lua_Unsigned base = glm_checkunsigned(L, 1);
  lua_Unsigned offset = glm_checkunsigned(L, 2);
  lua_Unsigned bits = glm_checkunsigned(L, 3);
#define T lua_Unsigned
  lua_Unsigned mask = bits >= cast(T, LUAI_DIGITS) ? ~(T)(0) : ((T)(1) << bits) - (T)(1);
#undef T
  base = (base >> offset) & mask;
  glm_pushinteger(L, l_castU2S(base));
  return 1;
}

int luaglm_bitfieldInsert(lua_State *L) {
  lua_Unsigned base = glm_checkunsigned(L, 1);
  lua_Unsigned insert = glm_checkunsigned(L, 2);
  lua_Unsigned offset = glm_checkunsigned(L, 3);
  lua_Unsigned bits = glm_checkunsigned(L, 4);
#define T lua_Unsigned
  lua_Unsigned mask = bits >= cast(T, LUAI_DIGITS) ? ~(T)(0) : ((T)(1) << bits) - (T)(1);
#undef T
  mask <<= offset;
  base = (base & ~mask) | ((insert << offset) & mask);
  glm_pushinteger(L, l_castU2S(base));
  return 1;
}

int luaglm_bitfieldReverse(lua_State *L) {
  lua_Unsigned v = glm_checkunsigned(L, 1);
  lua_Unsigned s = cast(lua_Unsigned, LUAI_DIGITS);
  lua_Unsigned mask = ~cast(lua_Unsigned, 0);
  while ((s >>= 1) > 0) {
    mask ^= (mask << s);
    v = ((v >> s) & mask) | ((v << s) & ~mask);
  }
  glm_pushinteger(L, l_castU2S(v));
  return 1;
}

int luaglm_bitCount(lua_State *L) {
  lua_Unsigned c = l_bitCount(glm_checkunsigned(L, 1));
  glm_pushinteger(L, l_castU2S(c));
  return 1;
}

int luaglm_findLSB(lua_State *L) {
  lua_Unsigned lsb = LUA_MAXUNSIGNED;
  lua_Unsigned x = glm_checkunsigned(L, 1);
  if (x) {
#if LUAI_HAS_UNSIGNED64 && LUA_HAS_BUILTIN(__builtin_ctzll)
    lsb = cast(lua_Unsigned, __builtin_ctzll(cast(unsigned long long, x)));
#elif LUAI_HAS_UNSIGNED32 && LUA_HAS_BUILTIN(__builtin_ctz)
    lsb = cast(lua_Unsigned, __builtin_ctz(cast(unsigned int, x)));
#elif LUAI_HAS_UNSIGNED64 && defined(_MSC_VER)
    unsigned long index;
    _BitScanForward64(&index, cast(unsigned __int64, x));
    lsb = cast(lua_Unsigned, index);
#elif LUAI_HAS_UNSIGNED32 && defined(_MSC_VER)
    unsigned long index;
    _BitScanForward(&index, cast(unsigned long, x));
    lsb = cast(lua_Unsigned, index);
#else
    lsb = l_bitCount(~x & (x - cast(lua_Unsigned, 1)));
#endif
  }
  glm_pushinteger(L, l_castU2S(lsb));
  return 1;
}

int luaglm_findMSB(lua_State *L) {
  lua_Unsigned msb = LUA_MAXUNSIGNED;
  lua_Unsigned x = glm_checkunsigned(L, 1);
  if (x) {
#if LUAI_HAS_UNSIGNED64 && LUA_HAS_BUILTIN(__builtin_clzll)
    int result = cast_int(LUAI_DIGITS - 1) - __builtin_clzll(cast(unsigned long long, x));
    msb = cast(lua_Unsigned, result);
#elif LUAI_HAS_UNSIGNED32 && LUA_HAS_BUILTIN(__builtin_clz)
    int result = cast_int(LUAI_DIGITS - 1) - __builtin_clz(cast(unsigned int, x));
    msb = cast(lua_Unsigned, result);
#elif LUAI_HAS_UNSIGNED64 && defined(_MSC_VER)
    unsigned long index;
    _BitScanReverse64(&index, cast(unsigned __int64, x));
    msb = cast(lua_Unsigned, index);
#elif LUAI_HAS_UNSIGNED32 && defined(_MSC_VER)
    unsigned long index;
    _BitScanReverse(&index, cast(unsigned long, x));
    msb = cast(lua_Unsigned, index);
#else
    msb = 0;
    while (x >>= 1)
      msb++;
#endif
  }
  glm_pushinteger(L, l_castU2S(msb));
  return 1;
}

/* 8.9. Texture Functions */
/* 8.10. Atomic-Counter Functions */
/* 8.11. Image Functions */
/* 8.12. Fragment Processing Functions */
/* 8.13. Geometry Shader Functions */
/* 8.14. Fragment Processing Functions */
/* 8.15. Noise Functions */

/* 8.X. Extensions */
int luaglm_approx(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  lua_Number eps = luaL_optnumber(L, 3, cast_num(GLM_FLT_EPSILON));
  if (ttisvector(o1) && tteq(o1, o2)) {
    lu_tag tag = ttisquat(o1) ? LUA_VVECTOR4 : vvaltt(o1);
    glm_pushvid(tag, glm_vec4_approx, glm_vid(o1), glm_vid(o2), glm_cf(eps));
  }
  else if (ttismatrix(o1) && tteq(o1, o2)) {
    CGLM_ALIGN(16) lua_Float4 f4 = { GLM_VEC4_ZERO_INIT };
    switch (mvaltt(o1)) {
      case LUA_VMATRIX2: glm_mat2_approx(glm_m2(o1), glm_m2(o2), glm_cf(eps), f4.v2); break;
      case LUA_VMATRIX3: glm_mat3_approx(glm_m3(o1), glm_m3(o2), glm_cf(eps), f4.v3); break;
      case LUA_VMATRIX4:
      default: {
        glm_mat4_approx(glm_m4(o1), glm_m4(o2), glm_cf(eps), f4.v4);
        break;
      }
    }
    glm_pushvid(vvaltag(ttmlen(o1)), glm_vec4_copy, f4.v4);
  }
  else if (ttisnumber(o1) && ttisnumber(o2)) {
    TValue *o3 = (TValue *)glm_index2value(L, 3);
    if (ttisinteger(o1) && ttisinteger(o2) && ttisinteger(o3)) {
      lua_Integer i = luaL_intop(-, ivalue(o1), ivalue(o2));
      glm_pushboolean(L, l_absI(i) <= ivalue(o3));
    }
    else
      glm_pushboolean(L, l_mathop(fabs)(nvalue(o1) - nvalue(o2)) <= eps);
  }
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 1;
}

int luaglm_angle(lua_State *L) {
  float angle;
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  if (ttisvector3(o1) && ttisvector3(o2))
    angle = glm_vec3_angle(glm_v3(o1), glm_v3(o2));
  else if (ttisvector2(o1) && ttisvector2(o2))
    angle = glm_vec2_angle(glm_v2(o1), glm_v2(o2));
  else if (ttisquat(o1))
    angle = glm_quat_angle(glm_q(o1));
  else
    return luaL_error(L, GLM_UNEXPECTED);
  glm_pushnumber(L, cast_num(angle));
  return 1;
}

int luaglm_quat_axis(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  if (ttisquat(o1)) {
    glm_pushvec3(glm_quat_axis, glm_q(o1));
    return 1;
  }
  return luaL_error(L, GLM_UNEXPECTED);
}

int luaglm_quat_for(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  if (ttisvector3(o1) && ttisvector3(o2)) {
    glm_pushquat(glm_quat_for, glm_v3(o1), glm_v3(o2));
    return 1;
  }
  return luaL_error(L, GLM_UNEXPECTED);
}

int luaglm_slerp(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  TValue *o3 = (TValue *)glm_index2value(L, 3);
  if (ttisquat(o1) && ttisquat(o2) && ttisnumber(o3))
    glm_pushquat(glm_quat_slerp, glm_q(o1), glm_q(o2), glm_fv(o3));
  else if (ttisvector3(o1) && ttisvector3(o2) && ttisnumber(o3))
    glm_pushvec3(glm_vec3_slerp, glm_v3(o1), glm_v3(o2), glm_fv(o3));
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 1;
}

int luaglm_translate(lua_State *L) {
  CGLM_ALIGN_MAT lua_Matrix m;
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  if (ttismatrix4(o1) && ttisvector3(o2))
    glm_translate_to(glm_m4(o1), glm_v3(o2), m.m4);
  else if (ttismatrix3(o1) && ttisvector2(o2))
    glm_mat4_zero(m.m4), glm_translate2d_to(glm_m3(o1), glm_v2(o2), m.m3);
  else
    return luaL_error(L, GLM_UNEXPECTED);
  glm_mid_recycle(3, mvaltt(o1), glm_mat4_copy, m.m4);
  return 1;
}

int luaglm_scale(lua_State *L) {
  CGLM_ALIGN_MAT lua_Matrix m;
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  if (ttismatrix4(o1) && ttisvector3(o2))
    glm_scale_to(glm_m4(o1), glm_v3(o2), m.m4);
  else if (ttismatrix3(o1) && ttisvector2(o2))
    glm_mat4_zero(m.m4), glm_scale2d_to(glm_m3(o1), glm_v2(o2), m.m3);
  else
    return luaL_error(L, GLM_UNEXPECTED);
  glm_mid_recycle(3, mvaltt(o1), glm_mat4_copy, m.m4);
  return 1;
}

int luaglm_rotate(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  TValue *o3 = (TValue *)glm_index2value(L, 3);
  if (ttismatrix4(o1) && ttisnumber(o2) && ttisvector3(o3)) {
    mat4 rot;
    glm_rotate_make_simd(rot, glm_fv(o2), glm_v3(o3));
    glm_mat4_recycle(4, glm_mul_rot, glm_m4(o1), rot);
  }
  else if (ttisquat(o1) && ttisnumber(o2) && ttisvector3(o3)) {
    versor q2;
    glm_quatv(q2, glm_fv(o2), glm_v3(o3));
    glm_pushquat(glm_quat_mul, glm_q(o1), q2);
  }
  else if (ttismatrix3(o1) && ttisnumber(o2))
    glm_mat3_recycle(3, glm_rotate2d_to, glm_m3(o1), glm_fv(o2));
  else if (ttisvector2(o1) && ttisnumber(o2))
    glm_pushvec2(glm_vec2_rotate, glm_v2(o1), glm_fv(o2));
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 1;
}

int luaglm_frustum(lua_State *L) {
  float left = glm_checkfloat(L, 1);
  float right = glm_checkfloat(L, 2);
  float bottom = glm_checkfloat(L, 3);
  float top = glm_checkfloat(L, 4);
  float nearZ = glm_checkfloat(L, 5);
  float farZ = glm_checkfloat(L, 6);
  if (!glm_eq(nearZ, farZ)) {
    glm_mat4_recycle(7, glm_frustum, left, right, bottom, top, nearZ, farZ);
    return 1;
  }
  return luaL_error(L, "invalid plane");
}

int luaglm_ortho(lua_State *L) {
  float left = glm_checkfloat(L, 1);
  float right = glm_checkfloat(L, 2);
  float bottom = glm_checkfloat(L, 3);
  float top = glm_checkfloat(L, 4);
  float nearZ = glm_checkfloat(L, 5);
  float farZ = glm_checkfloat(L, 6);
  if (!glm_eq(nearZ, farZ)) {
    glm_mat4_recycle(7, glm_ortho, left, right, bottom, top, nearZ, farZ);
    return 1;
  }
  return luaL_error(L, "invalid plane");
}

int luaglm_perspective(lua_State *L) {
  float fovy = glm_checkfloat(L, 1);
  float aspect = glm_checkfloat(L, 2);
  float nearZ = glm_checkfloat(L, 3);
  float farZ = glm_checkfloat(L, 4);
  if (!glm_eq(nearZ, farZ)) {
    glm_mat4_recycle(5, glm_perspective, fovy, aspect, nearZ, farZ);
    return 1;
  }
  return luaL_error(L, "invalid plane");
}

int luaglm_project(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  TValue *o3 = (TValue *)glm_index2value(L, 3);
  if (ttisvector3(o1) && ttismatrix4(o2) && ttisvector4(o3))
    glm_pushvec3(glm_project, glm_v3(o1), glm_m4(o2), glm_v4(o3));
  else if (ttisvector3(o1) && ttisvector3(o2))
    glm_pushvec3(glm_vec3_proj, glm_v3(o1), glm_v3(o2));
  else
    return luaL_error(L, GLM_UNEXPECTED);
  return 1;
}

int luaglm_unproject(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  TValue *o3 = (TValue *)glm_index2value(L, 3);
  if (ttisvector3(o1) && ttismatrix4(o2) && ttisvector4(o3)) {
    glm_pushvec3(glm_unproject, glm_v3(o1), glm_m4(o2), glm_v4(o3));
    return 1;
  }
  return luaL_error(L, GLM_UNEXPECTED);
}

int luaglm_lookat(lua_State *L) {
  int idx;
  mat4 m;
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  TValue *o3 = (TValue *)glm_index2value(L, 3);
  if (ttisvector3(o1) && ttisvector3(o2) && ttisvector3(o3))
    idx = 4, glm_lookat(glm_v3(o1), glm_v3(o2), glm_v3(o3), m);
  else if (ttisvector3(o1) && ttisquat(o2))
    idx = 3, glm_quat_look(glm_v3(o1), glm_q(o2), m);
  else
    return luaL_error(L, GLM_UNEXPECTED);
  glm_mat4_recycle(idx, glm_mat4_copy, m);
  return 1;
}

int luaglm_billboard(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  TValue *o3 = (TValue *)glm_index2value(L, 3);
  TValue *o4 = (TValue *)glm_index2value(L, 4);
  if (ttisvector3(o1) && ttisvector3(o2) && ttisvector3(o3) && ttisvector3(o4)) {
    glm_mat4_recycle(5, glm_billboard, glm_v3(o1), glm_v3(o2), glm_v3(o3), glm_v3(o4));
    return 1;
  }
  return luaL_error(L, GLM_UNEXPECTED);
}

int luaglm_compose(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  TValue *o3 = (TValue *)glm_index2value(L, 3);
  if (ttisvector3(o1) && ttisquat(o2) && ttisvector3(o3)) {
    glm_mat4_recycle(4, glm_compose_trs, glm_v3(o1), glm_q(o2), glm_v3(o3));
    return 1;
  }
  return luaL_error(L, GLM_UNEXPECTED);
}

int luaglm_decompose(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  if (ttismatrix4(o1)) {
    mat4 rotation;
    vec4 translation;
    CGLM_ALIGN(16) vec3 scale;
    glm_decompose(glm_m4(o1), translation, rotation, scale);
    glm_pushvec3(glm_vec3, translation);
    glm_pushquat(glm_mat4_quat, rotation);
    glm_pushvec3(glm_vec3_copy, scale);
    return 3;
  }
  return luaL_error(L, GLM_UNEXPECTED);
}

int luaglm_from_euler(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  TValue *o3 = (TValue *)glm_index2value(L, 3);
  if (ttisvector3(o1) && ttisstring(o2) && tsslen(tsvalue(o2)) == 3) {
    glm_euler_seq order = glm_parse_euler(getstr(tsvalue(o2)), ttistrue(o3));
    glm_pushquat(glm_euler_by_orderq, glm_v3(o1), order);
    return 1;
  }
  return luaL_argerror(L, 2, "invalid axis order");
}

int luaglm_to_euler(lua_State *L) {
  TValue *o1 = (TValue *)glm_index2value(L, 1);
  TValue *o2 = (TValue *)glm_index2value(L, 2);
  TValue *o3 = (TValue *)glm_index2value(L, 3);
  if (ttisstring(o2) && tsslen(tsvalue(o2)) == 3) {
    mat4 m;
    glm_euler_seq order = glm_parse_euler(getstr(tsvalue(o2)), ttistrue(o3));
    if (ttisquat(o1))
      glm_quat_mat4(glm_q(o1), m);
    else if (ttismatrix4(o1))
      glm_mat4_copy(glm_m4(o1), m);
    else if (ttismatrix3(o1))
      glm_mat4_identity(m), glm_mat4_ins3(m3value(o1), m);
    else
      return luaL_argerror(L, 1, GLM_UNEXPECTED);
    glm_pushvec3(glm_euler_to_order, m, order);
    return 1;
  }
  return luaL_argerror(L, 2, "invalid axis order");
}

/* }================================================================== */

/*
** {==================================================================
** arithmetic/bitwise/comparison opcode implementations.
** ===================================================================
*/

/*
** OP_UNM for vector, quaternion, and matrix types:
**   A B R[A] := -R[B]
*/
int lglm_UNM(lua_State *L, StkId ra, TValue *rb) {
  switch (ttype(rb)) {
    case LUA_TVECTOR: /* -vec */
      glm_vid_op(ra, vvaltt(rb), glm_vec4_negate_to, glm_vid(rb));
      break;
    case LUA_TMATRIX: /* -mat */
      glm_mid_op(ra, mvaltt(rb), glm_mat4_negate_to, glm_mid(rb));
      break;
    default: {
      lua_assert(0);
      return 0;
    }
  }
  return 1;
}

/*
** OP_ADD for vector, quaternion, and matrix types:
**   A B C   R[A] := R[B] + R[C]
*/
int lglm_ADD(lua_State *L, StkId ra, TValue *rb, TValue *rc) {
  switch (ttype(rb)) {
    case LUA_TNUMBER: {
      if (ttisvector(rc)) /* scalar + vec */
        glm_vid_op(ra, vvaltt(rc), glm_vec4_adds, glm_vid(rc), glm_fv(rb));
      else if (ttismatrix(rc)) /* scalar + mat */
        glm_mid_op(ra, mvaltt(rc), glm_mat4_adds, glm_mid(rc), glm_fv(rb));
      else
        return 0;
      break;
    }
    case LUA_TVECTOR: {
      if (tteq(rb, rc)) /* vec + vec */
        glm_vid_op(ra, vvaltt(rb), glm_vec4_add, glm_vid(rb), glm_vid(rc));
      else if (ttisnumber(rc)) /* vec + scalar */
        glm_vid_op(ra, vvaltt(rb), glm_vec4_adds, glm_vid(rb), glm_fv(rc));
      else if (ttismatrix(rc) && ttvareq(rb, rc)) { /* vec + mat */
        switch (mvaltt(rc)) {
          case LUA_VMATRIX2: glm_mat2_op(ra, glm_mat2_addv, glm_m2(rc), glm_v2(rb)); break;
          case LUA_VMATRIX3: glm_mat3_op(ra, glm_mat3_addv, glm_m3(rc), glm_v3(rb)); break;
          case LUA_VMATRIX4:
          default: {
            glm_mat4_op(ra, glm_mat4_addv, glm_m4(rc), glm_v4(rb));
            break;
          }
        }
      }
      else
        return 0;
      break;
    }
    case LUA_TMATRIX: {
      if (tteq(rb, rc)) /* mat + mat */
        glm_mid_op(ra, mvaltt(rb), glm_mat4_add, glm_mid(rb), glm_mid(rc));
      else if (ttisnumber(rc)) /* mat + scalar */
        glm_mid_op(ra, mvaltt(rb), glm_mat4_adds, glm_mid(rb), glm_fv(rc));
      else if (ttisvector(rc) && ttvareq(rb, rc)) { /* mat + vec */
        switch (mvaltt(rb)) {
          case LUA_VMATRIX2: glm_mat2_op(ra, glm_mat2_addv, glm_m2(rb), glm_v2(rc)); break;
          case LUA_VMATRIX3: glm_mat3_op(ra, glm_mat3_addv, glm_m3(rb), glm_v3(rc)); break;
          case LUA_VMATRIX4:
          default: {
            glm_mat4_op(ra, glm_mat4_addv, glm_m4(rb), glm_v4(rc));
            break;
          }
        }
      }
      else
        return 0;
      break;
    }
    default: {
      lua_assert(0);
      return 0;
    }
  }
  return 1;
}

/*
** OP_ADDI for vector, quaternion, and matrix types.
**   A B sC  R[A] := R[B] + sC
*/
int lglm_ADDI(lua_State *L, StkId ra, TValue *rb, int imm) {
  switch (ttype(rb)) {
    case LUA_TVECTOR: /* vec + imm */
      glm_vid_op(ra, vvaltt(rb), glm_vec4_adds, glm_vid(rb), glm_cf(imm));
      break;
    case LUA_TMATRIX: /* mat + imm */
      glm_mid_op(ra, mvaltt(rb), glm_mat4_adds, glm_mid(rb), glm_cf(imm));
      break;
    default: {
      lua_assert(0);
      return 0;
    }
  }
  return 1;
}

/*
** OP_SUB for vector, quaternion, and matrix types:
**   A B C   R[A] := R[B] - R[C]
*/
int lglm_SUB(lua_State *L, StkId ra, TValue *rb, TValue *rc) {
  switch (ttype(rb)) {
    case LUA_TNUMBER: {
      if (ttisvector(rc)) /* scalar - vec */
        glm_vid_op(ra, vvaltt(rc), glm_vec4_ssub, glm_fv(rb), glm_vid(rc));
      else if (ttismatrix(rc)) /* scalar - mat */
        glm_mid_op(ra, mvaltt(rc), glm_mat4_ssub, glm_fv(rb), glm_mid(rc));
      else
        return 0;
      break;
    }
    case LUA_TVECTOR: {
      if (tteq(rb, rc)) /* vec - vec */
        glm_vid_op(ra, vvaltt(rb), glm_vec4_sub, glm_vid(rb), glm_vid(rc));
      else if (ttisnumber(rc)) /* vec - scalar */
        glm_vid_op(ra, vvaltt(rb), glm_vec4_subs, glm_vid(rb), glm_fv(rc));
      else if (ttismatrix(rc) && ttvareq(rb, rc)) { /* vec - mat */
        switch (mvaltt(rc)) {
          case LUA_VMATRIX2: glm_mat2_op(ra, glm_mat2_vsub, glm_v2(rb), glm_m2(rc)); break;
          case LUA_VMATRIX3: glm_mat3_op(ra, glm_mat3_vsub, glm_v3(rb), glm_m3(rc)); break;
          case LUA_VMATRIX4:
          default: {
            glm_mat4_op(ra, glm_mat4_vsub, glm_v4(rb), glm_m4(rc));
            break;
          }
        }
      }
      else
        return 0;
      break;
    }
    case LUA_TMATRIX: {
      if (tteq(rb, rc)) /* mat - mat */
        glm_mid_op(ra, mvaltt(rb), glm_mat4_sub, glm_mid(rb), glm_mid(rc));
      else if (ttisnumber(rc)) /* mat - scalar */
        glm_mid_op(ra, mvaltt(rb), glm_mat4_subs, glm_mid(rb), glm_fv(rc));
      else if (ttisvector(rc) && ttvareq(rb, rc)) { /* mat - vec */
        switch (mvaltt(rb)) {
          case LUA_VMATRIX2: glm_mat2_op(ra, glm_mat2_subv, glm_m2(rb), glm_v2(rc)); break;
          case LUA_VMATRIX3: glm_mat3_op(ra, glm_mat3_subv, glm_m3(rb), glm_v3(rc)); break;
          case LUA_VMATRIX4:
          default: {
            glm_mat4_op(ra, glm_mat4_subv, glm_m4(rb), glm_v4(rc));
            break;
          }
        }
      }
      else
        return 0;
      break;
    }
    default: {
      lua_assert(0);
      return 0;
    }
  }
  return 1;
}

/* R[A] := Number * R[C] */
static int lglm_NUMMUL(lua_State *L, StkId ra, TValue *rb, TValue *rc) {
  switch (ttype(rc)) {
    case LUA_TVECTOR: /* scalar * vec */
      glm_vid_op(ra, vvaltt(rc), glm_vec4_scale, glm_vid(rc), glm_fv(rb));
      break;
    case LUA_TMATRIX: /* scalar * mat */
      glm_mid_op(ra, mvaltt(rc), glm_mat4_scale_to, glm_mid(rc), glm_fv(rb));
      break;
    default: {
      lua_assert(0);
      return 0;
    }
  }
  return 1;
}

/* R[A] := Vec * R[C] */
static int lglm_VECMUL(lua_State *L, StkId ra, TValue *rb, TValue *rc) {
  if (tteq(rb, rc)) /* vec * vec */
    glm_vid_op(ra, vvaltt(rb), glm_vec4_mul, glm_vid(rb), glm_vid(rc));
  else if (ttisnumber(rc)) /* vec * scalar */
    glm_vid_op(ra, vvaltt(rb), glm_vec4_scale, glm_vid(rb), glm_fv(rc));
  else if (ttisvector3(rb) && ttisquat(rc)) /* vec3 * quat */
    glm_vec3_op(ra, glm_vec3_rotateq_simd, glm_v3(rb), glm_q(rc));
  else if (ttisvector4(rb) && ttisquat(rc)) /* vec4 * quat */
    glm_vec4_op(ra, glm_vec4_rotateq_simd, glm_v4(rb), glm_q(rc));
  else if (ttismatrix(rc) && ttvareq(rb, rc)) { /* vecN * matNxN */
    switch (mvaltt(rc)) {
      case LUA_VMATRIX2: glm_vec2_op(ra, glm_mat2_vmul, glm_v2(rb), glm_m2(rc)); break;
      case LUA_VMATRIX3: glm_vec3_op(ra, glm_mat3_vmul, glm_v3(rb), glm_m3(rc)); break;
      case LUA_VMATRIX4:
      default: {
        glm_vec4_op(ra, glm_mat4_vmul, glm_v4(rb), glm_m4(rc));
        break;
      }
    }
  }
  else {
    UNUSED(L);
    return 0;
  }
  return 1;
}

/* R[A] := Quat * R[C] */
static int lglm_QUATMUL(lua_State *L, StkId ra, TValue *rb, TValue *rc) {
  switch (vvaltt(rc)) {
    case LUA_VNUMINT: glm_quat_op(ra, glm_vec4_scale, glm_q(rb), glm_cf(ivalue(rc))); break;
    case LUA_VNUMFLT: glm_quat_op(ra, glm_vec4_scale, glm_q(rb), glm_cf(fltvalue(rc))); break;
    case LUA_VVECTOR3: glm_vec3_op(ra, glm_quat_rotatev_simd, glm_q(rb), glm_v3(rc)); break;
    case LUA_VVECTOR4: glm_vec4_op(ra, glm_quat_rotatev4_simd, glm_q(rb), glm_v4(rc)); break;
    case LUA_VQUAT: glm_quat_op(ra, glm_quat_mul, glm_q(rb), glm_q(rc)); break;
    default: {
      UNUSED(L);
      return 0;
    }
  }
  return 1;
}

/* R[A] := Mat * R[C] */
static int lglm_MATMUL(lua_State *L, StkId ra, TValue *rb, TValue *rc) {
  if (ttisvector(rc) && ttvareq(rb, rc)) { /* matNxN * vecN */
    switch (mvaltt(rb)) {
      case LUA_VMATRIX2: glm_vec2_op(ra, glm_mat2_mulv, glm_m2(rb), glm_v2(rc)); break;
      case LUA_VMATRIX3: glm_vec3_op(ra, glm_mat3_mulv, glm_m3(rb), glm_v3(rc)); break;
      case LUA_VMATRIX4:
      default: {
        glm_vec4_op(ra, glm_mat4_mulv, glm_m4(rb), glm_v4(rc));
        break;
      }
    }
  }
  else if (tteq(rb, rc)) { /* mat * mat */
    switch (mvaltt(rb)) {
      case LUA_VMATRIX2: glm_mat2_op(ra, glm_mat2_mul, glm_m2(rb), glm_m2(rc)); break;
      case LUA_VMATRIX3: glm_mat3_op(ra, glm_mat3_mul, glm_m3(rb), glm_m3(rc)); break;
      case LUA_VMATRIX4:
      default: {
        glm_mat4_op(ra, glm_mat4_mul, glm_m4(rb), glm_m4(rc));
        break;
      }
    }
  }
  else if (ttismatrix4(rb) && ttisvector3(rc)) /* mat4x4 * vec3 */
    glm_vec3_op(ra, glm_mat4_mulv3, glm_m4(rb), glm_v3(rc), 1.0f);
  else if (ttismatrix4(rb) && ttisquat(rc)) /* mat4x4 * quat */
    glm_mat4_op(ra, glm_quat_rotate, glm_m4(rb), glm_q(rc));
  else if (ttisnumber(rc)) /* mat * scalar */
    glm_mid_op(ra, mvaltt(rb), glm_mat4_scale_to, glm_mid(rb), glm_fv(rc));
  else
    return 0;
  return 1;
}

/*
** OP_MUL for vector, quaternion, and matrix types:
**   A B C   R[A] := R[B] * R[C]
*/
int lglm_MUL(lua_State *L, StkId ra, TValue *rb, TValue *rc) {
  switch (ttype(rb)) {
    case LUA_TNUMBER: return lglm_NUMMUL(L, ra, rb, rc);
    case LUA_TMATRIX: return lglm_MATMUL(L, ra, rb, rc);
    case LUA_TVECTOR:
      return !ttisquat(rb) ? lglm_VECMUL(L, ra, rb, rc)
                           : lglm_QUATMUL(L, ra, rb, rc);
    default: {
      lua_assert(0);
      return 0;
    }
  }
}

/* R[A] := Number / R[C] */
static int lglm_NUMDIV(lua_State *L, StkId ra, TValue *rb, TValue *rc) {
  switch (ttype(rc)) {
    case LUA_TVECTOR: /* scalar / vec */
      glm_vid_op(ra, vvaltt(rc), glm_vec4_sdiv, glm_fv(rb), glm_vid(rc));
      break;
    case LUA_TMATRIX: /* scalar / mat */
      glm_mid_op(ra, mvaltt(rc), glm_mat4_sdiv, glm_fv(rb), glm_mid(rc));
      break;
    default: {
      lua_assert(0);
      return 0;
    }
  }
  return 1;
}

/* R[A] := Vec / R[C] */
static int lglm_VECDIV(lua_State *L, StkId ra, TValue *rb, TValue *rc) {
  if (tteq(rb, rc)) /* vec / vec */
    glm_vid_op(ra, vvaltt(rb), glm_vec4_div, glm_vid(rb), glm_vid(rc));
  else if (ttisnumber(rc)) /* vec / scalar */
    glm_vid_op(ra, vvaltt(rb), glm_vec4_divs, glm_vid(rb), glm_fv(rc));
  else if (ttismatrix(rc) && ttvareq(rb, rc)) { /* vec / mat */
    switch (vvaltt(rb)) {
      case LUA_VVECTOR2: glm_vec2_op(ra, glm_mat2_vdiv, glm_v2(rb), glm_m2(rc)); break;
      case LUA_VVECTOR3: glm_vec3_op(ra, glm_mat3_vdiv, glm_v3(rb), glm_m3(rc)); break;
      case LUA_VVECTOR4:
      default: {
        glm_vec4_op(ra, glm_mat4_vdiv, glm_v4(rb), glm_m4(rc));
        break;
      }
    }
  }
  else if (ttisquat(rc)) { /* vec / quat */
    versor q;
    glm_quat_inv_simd(glm_q(rc), q);
    switch (vvaltt(rb)) {
      case LUA_VVECTOR3: glm_vec3_op(ra, glm_quat_rotatev_simd, q, glm_v3(rb)); break;
      case LUA_VVECTOR4: glm_vec4_op(ra, glm_quat_rotatev4_simd, q, glm_v4(rb)); break;
      default: {
        return 0;
      }
    }
  }
  else {
    UNUSED(L);
    return 0;
  }
  return 1;
}

/* R[A] := Quat / R[C] */
static int lglm_QUATDIV(lua_State *L, StkId ra, TValue *rb, TValue *rc) {
  if (ttisquat(rc)) /* quat / quat */
    glm_quat_op(ra, glm_quat_div, glm_q(rb), glm_q(rc));
  else if (ttisnumber(rc)) { /* quat / scalar */
    float fc = glm_fv(rc);
    if (!glm_eq(fc, 0.0f))
      glm_quat_op(ra, glm_vec4_divs, glm_q(rb), fc);
    else { /* Dividing by approx-zero; set to identity */
      versor q;
      glm_quat_identity(q);
      glm_quat_op(ra, glm_quat_copy, q);
    }
  }
  else {
    UNUSED(L);
    return 0;
  }
  return 1;
}

/* R[A] := Mat / R[C] */
static int lglm_MATDIV(lua_State *L, StkId ra, TValue *rb, TValue *rc) {
  if (ttisvector(rc) && ttvareq(rb, rc)) { /* matrix / vector */
    switch (mvaltt(rb)) {
      case LUA_VMATRIX2: glm_vec2_op(ra, glm_mat2_divv, glm_m2(rb), glm_v2(rc)); break;
      case LUA_VMATRIX3: glm_vec3_op(ra, glm_mat3_divv, glm_m3(rb), glm_v3(rc)); break;
      case LUA_VMATRIX4:
      default: {
        glm_vec4_op(ra, glm_mat4_divv, glm_m4(rb), glm_v4(rc));
        break;
      }
    }
  }
  else if (tteq(rb, rc)) { /* matrix / matrix */
    switch (mvaltt(rb)) {
      case LUA_VMATRIX2: glm_mat2_op(ra, glm_mat2_div, glm_m2(rb), glm_m2(rc)); break;
      case LUA_VMATRIX3: glm_mat3_op(ra, glm_mat3_div, glm_m3(rb), glm_m3(rc)); break;
      case LUA_VMATRIX4:
      default: {
        glm_mat4_op(ra, glm_mat4_div, glm_m4(rb), glm_m4(rc));
        break;
      }
    }
  }
  else if (ttismatrix4(rb) && ttisquat(rc)) { /* matrix / quat */
    versor q;
    glm_quat_inv_simd(glm_q(rc), q);
    glm_mat4_op(ra, glm_quat_rotate, glm_m4(rb), q);
  }
  else if (ttisnumber(rc)) /* matrix / scalar */
    glm_mid_op(ra, mvaltt(rb), glm_mat4_scale_inv, glm_mid(rb), glm_fv(rc));
  else
    return 0;
  return 1;
}

/*
** OP_DIV for vector, quaternion, and matrix types:
**   A B C   R[A] := R[B] / R[C]
*/
int lglm_DIV(lua_State *L, StkId ra, TValue *rb, TValue *rc) {
  switch (ttype(rb)) {
    case LUA_TNUMBER: return lglm_NUMDIV(L, ra, rb, rc);
    case LUA_TMATRIX: return lglm_MATDIV(L, ra, rb, rc);
    case LUA_TVECTOR:
      return !ttisquat(rb) ? lglm_VECDIV(L, ra, rb, rc)
                           : lglm_QUATDIV(L, ra, rb, rc);
    default: {
      lua_assert(0);
      return 0;
    }
  }
}

/*
** OP_IDIV for vector, quaternion, and matrix types:
**   A B C   R[A] := R[B] // R[C]
*/
int lglm_IDIV(lua_State *L, StkId ra, TValue *rb, TValue *rc) {
  if (ttisvector(rb)) {
    vec4 dest;
    if (tteq(rb, rc)) /* vec // vec */
      glm_vec4_div(glm_vid(rb), glm_vid(rc), dest);
    else if (ttisnumber(rc)) /* vec // scalar */
      glm_vec4_divs(glm_vid(rb), glm_fv(rc), dest);
    else {
      return 0;
    }
    glm_vid_op(ra, vvaltt(rb), glm_vec4_floor_simd, dest);
    return 1;
  }
  UNUSED(L);
  return 0;
}

/*
** OP_MOD for vector, quaternion, and matrix types. Using fmod for the same
** reasons described in llimits.h:
**   A B C   R[A] := R[B] % R[C]
*/
int lglm_MOD(lua_State *L, StkId ra, TValue *rb, TValue *rc) {
  if (ttisvector(rb)) {
    if (tteq(rb, rc)) /* vec % vec */
      glm_vid_op(ra, vvaltt(rb), glm_vec4_mod_simd, glm_vid(rb), glm_vid(rc));
    else if (ttisnumber(rc)) /* vec % scalar */
      glm_vid_op(ra, vvaltt(rb), glm_vec4_mods_simd, glm_vid(rb), glm_fv(rc));
    else
      return 0;
  }
  else {
    UNUSED(L);
    return 0;
  }
  return 1;
}

/*
** OP_POW for vector, quaternion, and matrix types:
**   A B C   R[A] := R[B] ^ R[C]
*/
int lglm_POW(lua_State *L, StkId ra, TValue *rb, TValue *rc) {
  if (ttisvector(rb) && !ttisquat(rb)) {
    if (tteq(rb, rc)) /* vec ^ vec */
      glm_vid_op(ra, vvaltt(rb), glm_vec4_pow, glm_vid(rb), glm_vid(rc));
    else if (ttisnumber(rc)) /* vec ^ scalar */
      glm_vid_op(ra, vvaltt(rb), glm_vec4_pows, glm_vid(rb), glm_fv(rc));
    else {
      return 0;
    }
  }
  else if (ttisquat(rb) && ttisnumber(rc)) /* quat ^ scalar */
    glm_quat_op(ra, glm_quat_pow, glm_q(rb), glm_fv(rc));
  else if (ttisnumber(rb) && ttisvector(rc) && !ttisquat(rc)) /* scalar ^ vec */
    glm_vid_op(ra, vvaltt(rc), glm_vec4_spow, glm_fv(rb), glm_vid(rc));
  else {
    UNUSED(L);
    return 0;
  }
  return 1;
}

/*
** Bit operations. As bitwise operators only apply to int vectors this
** implementation will int-cast.
*/
#define lglm_BitOp(L, A, B, C, F)                                         \
  LUA_MLM_BEGIN                                                           \
  lua_Integer i;                                                          \
  CGLM_ALIGN(16) ivec4 LHS, RHS, DEST;                                    \
  if (ttisvector(B) && tteq(B, C)) {                                      \
    glm_ivec4f(glm_vid(B), LHS);                                          \
    glm_ivec4f(glm_vid(C), RHS);                                          \
  }                                                                       \
  else if (ttisvector(B) && tointegerns(C, &i) && glm_bitop_inrange(i)) { \
    glm_ivec4f(glm_vid(B), LHS);                                          \
    glm_ivec4_broadcast(cast_int(i), RHS);                                \
  }                                                                       \
  else if (ttisvector(C) && tointegerns(B, &i) && glm_bitop_inrange(i)) { \
    glm_ivec4_broadcast(cast_int(i), LHS);                                \
    glm_ivec4f(glm_vid(C), RHS);                                          \
  }                                                                       \
  else {                                                                  \
    UNUSED(L);                                                            \
    return 0;                                                             \
  }                                                                       \
  F(LHS, RHS, DEST);                                                      \
  glm_vid_op(A, vvaltt(B), glm_vec4i, DEST);                              \
  return 1;                                                               \
  LUA_MLM_END

/*
** OP_BNOT for vector, quaternion, and matrix types:
**   A B R[A] := ~R[B]
*/
int lglm_BNOT(lua_State *L, StkId ra, TValue *rb) {
  if (ttisvector(rb) && !ttisquat(rb)) {
    CGLM_ALIGN(16) ivec4 v, dest;
    glm_ivec4f(glm_vid(rb), v);
    glm_ivec4_bnot(v, dest);
    glm_vid_op(ra, vvaltt(rb), glm_vec4i, dest);
    return 1;
  }
  UNUSED(L);
  return 0;
}

/*
** OP_B{AND, OR, XOR, SHL, SHR} for vector, quaternion, and matrix types:
**  A B C   R[A] := R[B] & R[C]
**  A B C   R[A] := R[B] | R[C]
**  A B C   R[A] := R[B] ~ R[C]
**  A B C   R[A] := R[B] << R[C]
**  A B C   R[A] := R[B] >> R[C]
*/
int lglm_BAND(lua_State *L, StkId ra, TValue *rb, TValue *rc) { lglm_BitOp(L, ra, rb, rc, glm_ivec4_band); }
int lglm_BOR(lua_State *L, StkId ra, TValue *rb, TValue *rc) { lglm_BitOp(L, ra, rb, rc, glm_ivec4_bor); }
int lglm_BXOR(lua_State *L, StkId ra, TValue *rb, TValue *rc) { lglm_BitOp(L, ra, rb, rc, glm_ivec4_bxor); }
int lglm_BSHL(lua_State *L, StkId ra, TValue *rb, TValue *rc) { lglm_BitOp(L, ra, rb, rc, glm_ivec4_shl); }
int lglm_BSHR(lua_State *L, StkId ra, TValue *rb, TValue *rc) { lglm_BitOp(L, ra, rb, rc, glm_ivec4_shr); }

/*
** OP_SHRI for vector types.
**   A B sC  R[A] := R[B] >> sC
**   A B sC  R[A] := R[B] << sC
*/
int lglm_SHRI(lua_State *L, StkId ra, TValue *rb, int imm) {
  const int nbits = cast_int(sizeof(unsigned int) * CHAR_BIT);
  if (ttisvector(rb) && !ttisquat(rb)) {
    CGLM_ALIGN(16) ivec4 lhs, rhs, dest;
    glm_ivec4f(glm_vid(rb), lhs);
    if (imm < 0 && imm > -nbits) { /* shift right */
      glm_ivec4_broadcast(-imm, rhs);
      glm_ivec4_shr(lhs, rhs, dest);
    }
    else if (imm >= 0 && imm < nbits) { /* shift left */
      glm_ivec4_broadcast(imm, rhs);
      glm_ivec4_shl(lhs, rhs, dest);
    }
    else {
      glm_ivec4_copy(lhs, dest);
    }
    glm_vid_op(ra, vvaltt(rb), glm_vec4i, dest);
    return 1;
  }
  UNUSED(L);
  return 0;
}

/*
** OP_SHLI for vector types.
**   A B sC  R[A] := sC << R[B]
**   A B sC  R[A] := sC >> R[B]
*/
int lglm_SHLI(lua_State *L, StkId ra, TValue *rb, int imm) {
  const int nbits = cast_int(sizeof(int) * CHAR_BIT);
  if (ttisvector(rb) && !ttisquat(rb)) {
    CGLM_ALIGN(16) ivec4 lhs, rhs, dest;
    glm_ivec4f(glm_vid(rb), rhs);
    if (imm < 0 && imm > -nbits) { /* shift right */
      glm_ivec4_broadcast(-imm, lhs);
      glm_ivec4_shr(lhs, rhs, dest);
    }
    else if (imm >= 0 && imm < nbits) { /* shift left */
      glm_ivec4_broadcast(-imm, lhs);
      glm_ivec4_shl(lhs, rhs, dest);
    }
    else {
      glm_ivec4_copy(rhs, dest);
    }
    glm_vid_op(ra, vvaltt(rb), glm_vec4i, dest);
    return 1;
  }
  UNUSED(L);
  return 0;
}

#undef lglm_BitOp
/* }================================================================== */
