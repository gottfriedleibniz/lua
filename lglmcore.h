/*
** $Id: lglmcore.h $
** Internal definitions for GLM objects
** See Copyright Notice in lua.h
*/

#ifndef lglmcore_h
#define lglmcore_h

#include "luaconf.h"
#include "lua.h"
#include "lobject.h"

/*
** {==================================================================
** Internal vector definitions
** ===================================================================
*/

/* 'fast track' equivalent macros for vectors; see luaV_fastget */
#define lvec_fastgeti(T, I, S) vecgeti((T), (I), (S))
#define lvec_fastgets(T, K, S) \
  ((tsslen((K)) == 1) && vecgets((T), getstr((K)), (S)))

/* Helper function for integer access */
static LUA_INLINE int vecgeti (const TValue *obj, lua_Integer n, StkId res) {
  const lua_Integer D = cast(lua_Integer, ttvlen(obj));
  if (luai_likely(n >= 1 && n <= D)) {  /* Accessing vectors is 0-based */
    setfltvalue(s2v(res), cast_num(vgeti(vvalue_(obj), n - 1)));
    return 1;
  }
  return 0;
}

/* Helper function for character access */
static LUA_INLINE int vecgets (const TValue *obj, const char *k, StkId res) {
  lu_byte _n = luaO_vecindex[cast_uchar(*k)];
  if (luai_likely(_n < ttvlen(obj))) {
    setfltvalue(s2v(res), cast_num(vgeti(vvalue_(obj), _n)));
    return 1;
  }
  else if (*k == 'n') {  /* Dimension field has precedence over TM_INDEX */
    setivalue(s2v(res), cast(lua_Integer, ttvlen(obj)));
    return 1;
  }
  return 0;
}

LUAI_FUNC int lvec_rawgeti (const TValue *obj, lua_Integer n, StkId res);
LUAI_FUNC int lvec_rawgets (const TValue *obj, const char *k, StkId res);
LUAI_FUNC int lvec_rawget (const TValue *obj, TValue *key, StkId res);
LUAI_FUNC void lvec_get (lua_State *L, const TValue *obj, TValue *key, StkId res);
LUAI_FUNC void lvec_len (const TValue *obj, StkId res, int api_call);
LUAI_FUNC int lvec_next (const TValue *obj, StkId key);
LUAI_FUNC int lvec_concat (const TValue *obj, const TValue *value, StkId res);
LUAI_FUNC int lvec_equalkey (const TValue *k1, const Node *n2, int rtt);
LUAI_FUNC int lvec_validkey (const TValue *obj);
LUAI_FUNC size_t lvec_hashkey (const TValue *obj);

/* }================================================================== */

/*
** {==================================================================
** Internal matrix definitions
** ===================================================================
*/

/* 'fast track' equivalent macros for matrices; see luaV_fastget */
#define lmat_fastgeti(T, I, S) lmat_vmgeti((T), (I), (S))

LUAI_FUNC GCMatrix *lmat_new (lua_State *L, int tt);

/* lmat_rawgeti that does not set 'res' to nil on invalid access. */
LUAI_FUNC int lmat_vmgeti (const TValue *obj, lua_Integer n, StkId res);
LUAI_FUNC int lmat_rawgeti (const TValue *obj, lua_Integer n, StkId res);
LUAI_FUNC int lmat_rawget (const TValue *obj, TValue *key, StkId res);
LUAI_FUNC void lmat_rawset (lua_State *L, const TValue *obj, TValue *key, TValue *val);
LUAI_FUNC void lmat_get (lua_State *L, const TValue *obj, TValue *key, StkId res);
LUAI_FUNC void lmat_set (lua_State *L, const TValue *obj, TValue *key, TValue *val);
LUAI_FUNC void lmat_seti (lua_State *L, const TValue *obj, lua_Integer key, TValue *val);
LUAI_FUNC void lmat_len (const TValue *obj, StkId res);
LUAI_FUNC int lmat_next (const TValue *obj, StkId key);

/* }================================================================== */

/*
** {==================================================================
** VM Operations & Miscellaneous
** ===================================================================
*/

/* luaV_equalobj variant for vector/matrix types. */
LUAI_FUNC int lglm_equalobj (const TValue *o1, const TValue *o2);

/* Typename for given vector/matrix object */
LUAI_FUNC const char *lglm_typename (const TValue *obj);

/* Unary Operators */
LUAI_FUNC int lglm_BNOT (lua_State *L, StkId ra, TValue *rb);
LUAI_FUNC int lglm_UNM (lua_State *L, StkId ra, TValue *rb);

/* Binary Operators */
LUAI_FUNC int lglm_ADD (lua_State *L, StkId ra, TValue *rb, TValue *rc);
LUAI_FUNC int lglm_SUB (lua_State *L, StkId ra, TValue *rb, TValue *rc);
LUAI_FUNC int lglm_MUL (lua_State *L, StkId ra, TValue *rb, TValue *rc);
LUAI_FUNC int lglm_DIV (lua_State *L, StkId ra, TValue *rb, TValue *rc);
LUAI_FUNC int lglm_IDIV (lua_State *L, StkId ra, TValue *rb, TValue *rc);
LUAI_FUNC int lglm_MOD (lua_State *L, StkId ra, TValue *rb, TValue *rc);
LUAI_FUNC int lglm_POW (lua_State *L, StkId ra, TValue *rb, TValue *rc);
LUAI_FUNC int lglm_BAND (lua_State *L, StkId ra, TValue *rb, TValue *rc);
LUAI_FUNC int lglm_BOR (lua_State *L, StkId ra, TValue *rb, TValue *rc);
LUAI_FUNC int lglm_BXOR (lua_State *L, StkId ra, TValue *rb, TValue *rc);
LUAI_FUNC int lglm_BSHL (lua_State *L, StkId ra, TValue *rb, TValue *rc);
LUAI_FUNC int lglm_BSHR (lua_State *L, StkId ra, TValue *rb, TValue *rc);

/* Binary Immediate Operators */
LUAI_FUNC int lglm_ADDI (lua_State *L, StkId ra, TValue *rb, int imm);
LUAI_FUNC int lglm_SHRI (lua_State *L, StkId ra, TValue *rb, int imm);
LUAI_FUNC int lglm_SHLI (lua_State *L, StkId ra, TValue *rb, int imm);

/* }================================================================== */

#endif
