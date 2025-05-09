/*
** $Id: ltable.h $
** Lua tables (hash)
** See Copyright Notice in lua.h
*/

#ifndef ltable_h
#define ltable_h

#include "lobject.h"


#define gnode(t,i)	(&(t)->node[i])
#define gval(n)		(&(n)->i_val)
#define gnext(n)	((n)->u.next)


/*
** Clear all bits of fast-access metamethods, which means that the table
** may have any of these metamethods. (First access that fails after the
** clearing will set the bit again.)
*/
#define invalidateTMcache(t)	((t)->flags &= ~maskflags)


/* true when 't' is using 'dummynode' as its hash part */
#define isdummy(t)		((t)->lastfree == NULL)


/* allocated size for hash nodes */
#define allocsizenode(t)	(isdummy(t) ? 0 : sizenode(t))


/* returns the Node, given the value of a table entry */
#define nodefromval(v)	cast(Node *, (v))


LUAI_FUNC const TValue *luaH_getint (Table *t, lua_Integer key);
LUAI_FUNC void luaH_setint (lua_State *L, Table *t, lua_Integer key,
                                                    TValue *value);
LUAI_FUNC const TValue *luaH_getshortstr (Table *t, TString *key);
LUAI_FUNC const TValue *luaH_getstr (Table *t, TString *key);
LUAI_FUNC const TValue *luaH_get (Table *t, const TValue *key);
LUAI_FUNC void luaH_set (lua_State *L, Table *t, const TValue *key,
                                                 TValue *value);
LUAI_FUNC void luaH_finishset (lua_State *L, Table *t, const TValue *key,
                                       const TValue *slot, TValue *value);
LUAI_FUNC Table *luaH_new (lua_State *L);
LUAI_FUNC void luaH_resize (lua_State *L, Table *t, unsigned int nasize,
                                                    unsigned int nhsize);
LUAI_FUNC void luaH_resizearray (lua_State *L, Table *t, unsigned int nasize);
LUAI_FUNC void luaH_free (lua_State *L, Table *t);
LUAI_FUNC int luaH_next (lua_State *L, Table *t, StkId key);
#if defined(LUA_EXT_ITERATION)
LUAI_FUNC unsigned int luaH_rawnext (lua_State *L, Table *t, StkId v, unsigned int idx);
#endif
LUAI_FUNC lua_Unsigned luaH_getn (Table *t);
LUAI_FUNC unsigned int luaH_realasize (const Table *t);
#if defined(LUA_EXT_READONLY)
LUAI_FUNC void luaH_setreadonly (Table *t, int readonly);
#endif
#if defined(LUA_EXT_API)
LUAI_FUNC int luaH_type (const Table *t);
LUAI_FUNC void luaH_clear (lua_State *L, Table *t);
LUAI_FUNC void luaH_rehash (lua_State *L, Table *t);
LUAI_FUNC void luaH_compact (lua_State *L, Table *t);
LUAI_FUNC void luaH_clone (lua_State *L, const Table *t, Table *t2);
#endif


#if defined(LUA_DEBUG)
LUAI_FUNC Node *luaH_mainposition (const Table *t, const TValue *key);
#endif


#endif
