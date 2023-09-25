/*
** $Id: lobject.h $
** Type definitions for Lua objects
** See Copyright Notice in lua.h
*/


#ifndef lobject_h
#define lobject_h


#include <stdarg.h>


#include "llimits.h"
#include "lua.h"


/*
** Extra types for collectable non-values
*/
#define LUA_TUPVAL	LUA_NUMTYPES  /* upvalues */
#define LUA_TPROTO	(LUA_NUMTYPES+1)  /* function prototypes */
#define LUA_TDEADKEY	(LUA_NUMTYPES+2)  /* removed keys in tables */
#if defined(LUA_EXT_ITERATION)
#define LUA_TITER	(LUA_NUMTYPES+3)  /* iterator marker */
#endif



/*
** number of all possible types (including LUA_TNONE but excluding DEADKEY)
*/
#define LUA_TOTALTYPES		(LUA_TPROTO + 2)


/*
** tags for Tagged Values have the following use of bits:
** bits 0-3: actual tag (a LUA_T* constant)
** bits 4-5: variant bits
** bit 6: whether value is collectable
*/

#if defined(LUA_EXT_FAT_TYPES)	/* { */

#if LUAI_IS32INT
typedef int lu_tag;
#else
typedef long lu_tag;
#endif
#define BITS_TAG		0x000FF
#define BITS_VARIANT		0x0FF00
#define BIT_ISCOLLECTABLE	0x10000
#define VARIANT_OFFSET		8

#else				/* }{ */

typedef lu_byte lu_tag;
#define BITS_TAG		0x0F
#define BITS_VARIANT		0x30
#define BIT_ISCOLLECTABLE	0x40
#define VARIANT_OFFSET		4

#endif				/* } */

/* add variant bits to a type */
#define makevariant(t,v)	((t) | ((v) << VARIANT_OFFSET))

/* Internal vector representation */
typedef union lua_Float4 {
  lua_vec4 v4;
  lua_vec3 v3;
  lua_vec2 v2;
  lua_versor q;
} lua_Float4;

#if defined(LUAGLM_HALF_TYPE)
typedef l_float16 luai_vecf;
typedef union lua_hvec {
  lua_hvec4 v4;
  lua_hvec3 v3;
  lua_hvec2 v2;
  lua_hversor q;
} luai_Float4;
#else
typedef float luai_vecf;
typedef lua_Float4 luai_Float4;
#endif

/*
** Union of all Lua values
*/
typedef union Value {
  struct GCObject *gc;    /* collectable objects */
  void *p;         /* light userdata */
  luai_Float4 f4;  /* vector and quaternion stub */
  lua_CFunction f; /* light C functions */
  lua_Integer i;   /* integer numbers */
  lua_Number n;    /* float numbers */
#if defined(LUA_EXT_ITERATION)
  unsigned int it; /* iterator index */
#endif
  /* not used, but may avoid warnings for uninitialized value */
  lu_byte ub;
} Value;


/*
** Tagged Values. This is the basic representation of values in Lua:
** an actual value plus a tag with its type.
*/

#define TValuefields	Value value_; lu_tag tt_

typedef struct TValue {
  TValuefields;
} TValue;


#define val_(o)		((o)->value_)
#define valraw(o)	(val_(o))


/* raw type tag of a TValue */
#define rawtt(o)	((o)->tt_)

/* tag with no variants */
#define novariant(t)	((t) & BITS_TAG)

/* type tag of a TValue */
#define withvariant(t)	((t) & (BITS_TAG | BITS_VARIANT))
#define ttypetag(o)	withvariant(rawtt(o))

/* type of a TValue */
#define ttype(o)	(novariant(rawtt(o)))


/* Macros to test type */
#define checktag(o,t)		(rawtt(o) == (t))
#define checktype(o,t)		(ttype(o) == (t))


/* Macros for internal tests */

/* collectable object has the same tag as the original value */
#define righttt(obj)		(ttypetag(obj) == gcvalue(obj)->tt)

/*
** Any value being manipulated by the program either is non
** collectable, or the collectable object has the right tag
** and it is not dead. The option 'L == NULL' allows other
** macros using this one to be used where L is not available.
*/
#define checkliveness(L,obj) \
	((void)L, lua_longassert(!iscollectable(obj) || \
		(righttt(obj) && (L == NULL || !isdead(G(L),gcvalue(obj))))))


/* Macros to set values */

/* set a value's tag */
#define settt_(o,t)	((o)->tt_=(t))

/* @LuaGLM: 'union Value' assignment operators */
#define setval(io1, io2) (io1)->value_ = (io2)->value_
#define setkeyval(n, io) (n).key_val = (io)->value_
#define getkeyval(io, n) (io)->value_ = (n).key_val

/* main macro to copy values (from 'obj2' to 'obj1') */
#define setobj(L,obj1,obj2) \
	{ TValue *io1=(obj1); const TValue *io2=(obj2); \
          setval(io1, io2); settt_(io1, io2->tt_); \
	  checkliveness(L,io1); lua_assert(!isnonstrictnil(io1)); }

/*
** Different types of assignments, according to source and destination.
** (They are mostly equal now, but may be different in the future.)
*/

/* from stack to stack */
#define setobjs2s(L,o1,o2)	setobj(L,s2v(o1),s2v(o2))
/* to stack (not from same stack) */
#define setobj2s(L,o1,o2)	setobj(L,s2v(o1),o2)
/* from table to same table */
#define setobjt2t	setobj
/* to new object */
#define setobj2n	setobj
/* to table */
#define setobj2t	setobj


/*
** Entries in a Lua stack. Field 'tbclist' forms a list of all
** to-be-closed variables active in this stack. Dummy entries are
** used when the distance between two tbc variables does not fit
** in an unsigned short. They are represented by delta==0, and
** their real delta is always the maximum value that fits in
** that field.
*/
typedef union StackValue {
  TValue val;
  struct {
    TValuefields;
#if defined(LUA_EXT_DEFER)
    lu_byte deferred;
#endif
    unsigned short delta;
  } tbclist;
} StackValue;


/* index to stack elements */
typedef StackValue *StkId;


/*
** When reallocating the stack, change all pointers to the stack into
** proper offsets.
*/
typedef union {
  StkId p;  /* actual pointer */
  ptrdiff_t offset;  /* used while the stack is being reallocated */
} StkIdRel;


/* convert a 'StackValue' to a 'TValue' */
#define s2v(o)	(&(o)->val)



/*
** {==================================================================
** Nil
** ===================================================================
*/

/* Standard nil */
#define LUA_VNIL	makevariant(LUA_TNIL, 0)

/* Empty slot (which might be different from a slot containing nil) */
#define LUA_VEMPTY	makevariant(LUA_TNIL, 1)

/* Value returned for a key not found in a table (absent key) */
#define LUA_VABSTKEY	makevariant(LUA_TNIL, 2)


/* macro to test for (any kind of) nil */
#define ttisnil(v)		checktype((v), LUA_TNIL)


/* macro to test for a standard nil */
#define ttisstrictnil(o)	checktag((o), LUA_VNIL)


#define setnilvalue(obj) settt_(obj, LUA_VNIL)


#define isabstkey(v)		checktag((v), LUA_VABSTKEY)


/*
** macro to detect non-standard nils (used only in assertions)
*/
#define isnonstrictnil(v)	(ttisnil(v) && !ttisstrictnil(v))


/*
** By default, entries with any kind of nil are considered empty.
** (In any definition, values associated with absent keys must also
** be accepted as empty.)
*/
#define isempty(v)		ttisnil(v)


/* macro defining a value corresponding to an absent key */
#define ABSTKEYCONSTANT		{NULL}, LUA_VABSTKEY


/* mark an entry as empty */
#define setempty(v)		settt_(v, LUA_VEMPTY)



/* }================================================================== */


/*
** {==================================================================
** Booleans
** ===================================================================
*/


#define LUA_VFALSE	makevariant(LUA_TBOOLEAN, 0)
#define LUA_VTRUE	makevariant(LUA_TBOOLEAN, 1)

#define ttisboolean(o)		checktype((o), LUA_TBOOLEAN)
#define ttisfalse(o)		checktag((o), LUA_VFALSE)
#define ttistrue(o)		checktag((o), LUA_VTRUE)


#define l_isfalse(o)	(ttisfalse(o) || ttisnil(o))


#define setbfvalue(obj)		settt_(obj, LUA_VFALSE)
#define setbtvalue(obj)		settt_(obj, LUA_VTRUE)

/* }================================================================== */


/*
** {==================================================================
** Threads
** ===================================================================
*/

#define LUA_VTHREAD		makevariant(LUA_TTHREAD, 0)

#define ttisthread(o)		checktag((o), ctb(LUA_VTHREAD))

#define thvalue(o)	check_exp(ttisthread(o), gco2th(val_(o).gc))

#define setthvalue(L,obj,x) \
  { TValue *io = (obj); lua_State *x_ = (x); \
    val_(io).gc = obj2gco(x_); settt_(io, ctb(LUA_VTHREAD)); \
    checkliveness(L,io); }

#define setthvalue2s(L,o,t)	setthvalue(L,s2v(o),t)

/* }================================================================== */


/*
** {==================================================================
** Collectable Objects
** ===================================================================
*/

/*
** Common Header for all collectable objects (in macro form, to be
** included in other objects)
*/
#define CommonHeader	struct GCObject *next; lu_tag tt; lu_byte marked


/* Common type for all collectable objects */
typedef struct GCObject {
  CommonHeader;
} GCObject;

#define iscollectable(o)	(rawtt(o) & BIT_ISCOLLECTABLE)

/* mark a tag as collectable */
#define ctb(t)			((t) | BIT_ISCOLLECTABLE)

#define gcvalue(o)	check_exp(iscollectable(o), val_(o).gc)

#define gcvalueraw(v)	((v).gc)

#define setgcovalue(L,obj,x) \
  { TValue *io = (obj); GCObject *i_g=(x); \
    val_(io).gc = i_g; settt_(io, ctb(i_g->tt)); }

/* }================================================================== */


/*
** {==================================================================
** Numbers
** ===================================================================
*/

/* Variant tags for numbers */
#define LUA_VNUMINT	makevariant(LUA_TNUMBER, 0)  /* integer numbers */
#define LUA_VNUMFLT	makevariant(LUA_TNUMBER, 1)  /* float numbers */

#define ttisnumber(o)		checktype((o), LUA_TNUMBER)
#define ttisfloat(o)		checktag((o), LUA_VNUMFLT)
#define ttisinteger(o)		checktag((o), LUA_VNUMINT)

#define nvalue(o)	check_exp(ttisnumber(o), \
	(ttisinteger(o) ? cast_num(ivalue(o)) : fltvalue(o)))
#define fltvalue(o)	check_exp(ttisfloat(o), val_(o).n)
#define ivalue(o)	check_exp(ttisinteger(o), val_(o).i)

#define fltvalueraw(v)	((v).n)
#define ivalueraw(v)	((v).i)

#define setfltvalue(obj,x) \
  { TValue *io=(obj); val_(io).n=(x); settt_(io, LUA_VNUMFLT); }

#define chgfltvalue(obj,x) \
  { TValue *io=(obj); lua_assert(ttisfloat(io)); val_(io).n=(x); }

#define setivalue(obj,x) \
  { TValue *io=(obj); val_(io).i=(x); settt_(io, LUA_VNUMINT); }

#define chgivalue(obj,x) \
  { TValue *io=(obj); lua_assert(ttisinteger(io)); val_(io).i=(x); }

/* }================================================================== */


/*
** {==================================================================
** Strings
** ===================================================================
*/

/* Variant tags for strings */
#define LUA_VSHRSTR	makevariant(LUA_TSTRING, 0)  /* short strings */
#define LUA_VLNGSTR	makevariant(LUA_TSTRING, 1)  /* long strings */

#define ttisstring(o)		checktype((o), LUA_TSTRING)
#define ttisshrstring(o)	checktag((o), ctb(LUA_VSHRSTR))
#define ttislngstring(o)	checktag((o), ctb(LUA_VLNGSTR))

#define tsvalueraw(v)	(gco2ts((v).gc))

#define tsvalue(o)	check_exp(ttisstring(o), gco2ts(val_(o).gc))

#define setsvalue(L,obj,x) \
  { TValue *io = (obj); TString *x_ = (x); \
    val_(io).gc = obj2gco(x_); settt_(io, ctb(x_->tt)); \
    checkliveness(L,io); }

/* set a string to the stack */
#define setsvalue2s(L,o,s)	setsvalue(L,s2v(o),s)

/* set a string to a new object */
#define setsvalue2n	setsvalue


/*
** Header for a string value.
*/
typedef struct TString {
  CommonHeader;
  lu_byte extra;  /* reserved words for short strings; "has hash" for longs */
  lu_byte shrlen;  /* length for short strings, 0xFF for long strings */
  unsigned int hash;
  union {
    size_t lnglen;  /* length for long strings */
    struct TString *hnext;  /* linked list for hash table */
  } u;
  char contents[1];
} TString;



/*
** Get the actual string (array of bytes) from a 'TString'. (Generic
** version and specialized versions for long and short strings.)
*/
#define getstr(ts)	((ts)->contents)
#define getlngstr(ts)	check_exp((ts)->shrlen == 0xFF, (ts)->contents)
#define getshrstr(ts)	check_exp((ts)->shrlen != 0xFF, (ts)->contents)


/* get string length from 'TString *s' */
#define tsslen(s)  \
	((s)->shrlen != 0xFF ? (s)->shrlen : (s)->u.lnglen)

/* }================================================================== */


/*
** {==================================================================
** Userdata
** ===================================================================
*/


/*
** Light userdata should be a variant of userdata, but for compatibility
** reasons they are also different types.
*/
#define LUA_VLIGHTUSERDATA	makevariant(LUA_TLIGHTUSERDATA, 0)

#define LUA_VUSERDATA		makevariant(LUA_TUSERDATA, 0)

#define ttislightuserdata(o)	checktag((o), LUA_VLIGHTUSERDATA)
#define ttisfulluserdata(o)	checktag((o), ctb(LUA_VUSERDATA))

#define pvalue(o)	check_exp(ttislightuserdata(o), val_(o).p)
#define uvalue(o)	check_exp(ttisfulluserdata(o), gco2u(val_(o).gc))

#define pvalueraw(v)	((v).p)

#define setpvalue(obj,x) \
  { TValue *io=(obj); val_(io).p=(x); settt_(io, LUA_VLIGHTUSERDATA); }

#define setuvalue(L,obj,x) \
  { TValue *io = (obj); Udata *x_ = (x); \
    val_(io).gc = obj2gco(x_); settt_(io, ctb(LUA_VUSERDATA)); \
    checkliveness(L,io); }


/* Ensures that addresses after this type are always fully aligned. */
typedef union UValue {
  TValue uv;
  LUAI_MAXALIGN;  /* ensures maximum alignment for udata bytes */
} UValue;


/*
** Header for userdata with user values;
** memory area follows the end of this structure.
*/
typedef struct Udata {
  CommonHeader;
#if defined(LUA_EXT_USERTAG)
  lu_byte nuvalue;  /* number of user values */
  unsigned short tag;
#else
  unsigned short nuvalue;  /* number of user values */
#endif
  size_t len;  /* number of bytes */
  struct Table *metatable;
  GCObject *gclist;
  UValue uv[1];  /* user values */
} Udata;


/*
** Header for userdata with no user values. These userdata do not need
** to be gray during GC, and therefore do not need a 'gclist' field.
** To simplify, the code always use 'Udata' for both kinds of userdata,
** making sure it never accesses 'gclist' on userdata with no user values.
** This structure here is used only to compute the correct size for
** this representation. (The 'bindata' field in its end ensures correct
** alignment for binary data following this header.)
*/
typedef struct Udata0 {
  CommonHeader;
#if defined(LUA_EXT_USERTAG)
  lu_byte nuvalue;  /* number of user values */
  unsigned short tag;
#else
  unsigned short nuvalue;  /* number of user values */
#endif
  size_t len;  /* number of bytes */
  struct Table *metatable;
  union {LUAI_MAXALIGN;} bindata;
} Udata0;


/* compute the offset of the memory area of a userdata */
#define udatamemoffset(nuv) \
	((nuv) == 0 ? offsetof(Udata0, bindata)  \
                    : offsetof(Udata, uv) + (sizeof(UValue) * (nuv)))

/* get the address of the memory block inside 'Udata' */
#define getudatamem(u)	(cast_charp(u) + udatamemoffset((u)->nuvalue))

/* compute the size of a userdata */
#define sizeudata(nuv,nb)	(udatamemoffset(nuv) + (nb))

/* }================================================================== */


/*
** {==================================================================
** Prototypes
** ===================================================================
*/

#define LUA_VPROTO	makevariant(LUA_TPROTO, 0)


/*
** Description of an upvalue for function prototypes
*/
typedef struct Upvaldesc {
  TString *name;  /* upvalue name (for debug information) */
  lu_byte instack;  /* whether it is in stack (register) */
  lu_byte idx;  /* index of upvalue (in stack or in outer function's list) */
  lu_byte kind;  /* kind of corresponding variable */
} Upvaldesc;


/*
** Description of a local variable for function prototypes
** (used for debug information)
*/
typedef struct LocVar {
  TString *varname;
  int startpc;  /* first point where variable is active */
  int endpc;    /* first point where variable is dead */
} LocVar;


/*
** Associates the absolute line source for a given instruction ('pc').
** The array 'lineinfo' gives, for each instruction, the difference in
** lines from the previous instruction. When that difference does not
** fit into a byte, Lua saves the absolute line for that instruction.
** (Lua also saves the absolute line periodically, to speed up the
** computation of a line number: we can use binary search in the
** absolute-line array, but we must traverse the 'lineinfo' array
** linearly to compute a line.)
*/
typedef struct AbsLineInfo {
  int pc;
  int line;
} AbsLineInfo;

/*
** Function Prototypes
*/
typedef struct Proto {
  CommonHeader;
  lu_byte numparams;  /* number of fixed (named) parameters */
  lu_byte is_vararg;
  lu_byte maxstacksize;  /* number of registers needed by this function */
  int sizeupvalues;  /* size of 'upvalues' */
  int sizek;  /* size of 'k' */
  int sizecode;
  int sizelineinfo;
  int sizep;  /* size of 'p' */
  int sizelocvars;
  int sizeabslineinfo;  /* size of 'abslineinfo' */
  int linedefined;  /* debug information  */
  int lastlinedefined;  /* debug information  */
  TValue *k;  /* constants used by the function */
  Instruction *code;  /* opcodes */
  struct Proto **p;  /* functions defined inside the function */
  Upvaldesc *upvalues;  /* upvalue information */
  ls_byte *lineinfo;  /* information about source lines (debug information) */
  AbsLineInfo *abslineinfo;  /* idem */
  LocVar *locvars;  /* information about local variables (debug information) */
  TString  *source;  /* used for debug information */
  GCObject *gclist;
} Proto;

/* }================================================================== */


/*
** {==================================================================
** Functions
** ===================================================================
*/

#define LUA_VUPVAL	makevariant(LUA_TUPVAL, 0)


/* Variant tags for functions */
#define LUA_VLCL	makevariant(LUA_TFUNCTION, 0)  /* Lua closure */
#define LUA_VLCF	makevariant(LUA_TFUNCTION, 1)  /* light C function */
#define LUA_VCCL	makevariant(LUA_TFUNCTION, 2)  /* C closure */

#define ttisfunction(o)		checktype(o, LUA_TFUNCTION)
#define ttisLclosure(o)		checktag((o), ctb(LUA_VLCL))
#define ttislcf(o)		checktag((o), LUA_VLCF)
#define ttisCclosure(o)		checktag((o), ctb(LUA_VCCL))
#define ttisclosure(o)         (ttisLclosure(o) || ttisCclosure(o))


#define isLfunction(o)	ttisLclosure(o)

#define clvalue(o)	check_exp(ttisclosure(o), gco2cl(val_(o).gc))
#define clLvalue(o)	check_exp(ttisLclosure(o), gco2lcl(val_(o).gc))
#define fvalue(o)	check_exp(ttislcf(o), val_(o).f)
#define clCvalue(o)	check_exp(ttisCclosure(o), gco2ccl(val_(o).gc))

#define fvalueraw(v)	((v).f)

#define setclLvalue(L,obj,x) \
  { TValue *io = (obj); LClosure *x_ = (x); \
    val_(io).gc = obj2gco(x_); settt_(io, ctb(LUA_VLCL)); \
    checkliveness(L,io); }

#define setclLvalue2s(L,o,cl)	setclLvalue(L,s2v(o),cl)

#define setfvalue(obj,x) \
  { TValue *io=(obj); val_(io).f=(x); settt_(io, LUA_VLCF); }

#define setclCvalue(L,obj,x) \
  { TValue *io = (obj); CClosure *x_ = (x); \
    val_(io).gc = obj2gco(x_); settt_(io, ctb(LUA_VCCL)); \
    checkliveness(L,io); }


/*
** Upvalues for Lua closures
*/
typedef struct UpVal {
  CommonHeader;
  union {
    TValue *p;  /* points to stack or to its own value */
    ptrdiff_t offset;  /* used while the stack is being reallocated */
  } v;
  union {
    struct {  /* (when open) */
      struct UpVal *next;  /* linked list */
      struct UpVal **previous;
    } open;
    TValue value;  /* the value (when closed) */
  } u;
} UpVal;



#define ClosureHeader \
	CommonHeader; lu_byte nupvalues; GCObject *gclist

typedef struct CClosure {
  ClosureHeader;
  lua_CFunction f;
  TValue upvalue[1];  /* list of upvalues */
} CClosure;


typedef struct LClosure {
  ClosureHeader;
  struct Proto *p;
  UpVal *upvals[1];  /* list of upvalues */
} LClosure;


typedef union Closure {
  CClosure c;
  LClosure l;
} Closure;


#define getproto(o)	(clLvalue(o)->p)

/* }================================================================== */


/*
** {==================================================================
** Tables
** ===================================================================
*/

#define LUA_VTABLE	makevariant(LUA_TTABLE, 0)

#define ttistable(o)		checktag((o), ctb(LUA_VTABLE))

#define hvalue(o)	check_exp(ttistable(o), gco2t(val_(o).gc))

#define sethvalue(L,obj,x) \
  { TValue *io = (obj); Table *x_ = (x); \
    val_(io).gc = obj2gco(x_); settt_(io, ctb(LUA_VTABLE)); \
    checkliveness(L,io); }

#define sethvalue2s(L,o,h)	sethvalue(L,s2v(o),h)


/*
** Nodes for Hash tables: A pack of two TValue's (key-value pairs)
** plus a 'next' field to link colliding entries. The distribution
** of the key's fields ('key_tt' and 'key_val') not forming a proper
** 'TValue' allows for a smaller size for 'Node' both in 4-byte
** and 8-byte alignments.
*/
typedef union Node {
  struct NodeKey {
    TValuefields;  /* fields for value */
    lu_tag key_tt;  /* key type */
    int next;  /* for chaining */
    Value key_val;  /* key value */
  } u;
  TValue i_val;  /* direct access to node's value as a proper 'TValue' */
} Node;


/* copy a value into a key */
#define setnodekey(L,node,obj) \
	{ Node *n_=(node); const TValue *io_=(obj); \
	  setkeyval(n_->u, io_); n_->u.key_tt = io_->tt_; \
	  checkliveness(L,io_); }


/* copy a value from a key */
#define getnodekey(L,obj,node) \
	{ TValue *io_=(obj); const Node *n_=(node); \
	  getkeyval(io_, n_->u); io_->tt_ = n_->u.key_tt; \
	  checkliveness(L,io_); }


/*
** About 'alimit': if 'isrealasize(t)' is true, then 'alimit' is the
** real size of 'array'. Otherwise, the real size of 'array' is the
** smallest power of two not smaller than 'alimit' (or zero iff 'alimit'
** is zero); 'alimit' is then used as a hint for #t.
*/

#define BITRAS		(1 << 7)
#define isrealasize(t)		(!((t)->flags & BITRAS))
#define setrealasize(t)		((t)->flags &= cast_byte(~BITRAS))
#define setnorealasize(t)	((t)->flags |= BITRAS)

/* Use the remaining 'flags' bit for the extended read-only property. */
#if defined(LUA_EXT_READONLY)
#define BITRONLY		(1 << 6)
#define isreadonly(t)		((t)->flags & BITRONLY)
#define setreadonly(t)		((t)->flags |= BITRONLY)
#define setnotreadonly(t)	((t)->flags &= cast_byte(~BITRONLY))
#endif

typedef struct Table {
  CommonHeader;
  lu_byte flags;  /* 1<<p means tagmethod(p) is not present */
  lu_byte lsizenode;  /* log2 of size of 'node' array */
  unsigned int alimit;  /* "limit" of 'array' array */
  TValue *array;  /* array part */
  Node *node;
  Node *lastfree;  /* any free position is before this position */
  struct Table *metatable;
  GCObject *gclist;
} Table;


/*
** Macros to manipulate keys inserted in nodes
*/
#define keytt(node)		((node)->u.key_tt)
#define keyval(node)		((node)->u.key_val)

#define keyisnil(node)		(keytt(node) == LUA_VNIL) /* @LuaExt: LUA_TNIL */
#define keyisinteger(node)	(keytt(node) == LUA_VNUMINT)
#define keyival(node)		(keyval(node).i)
#define keyisshrstr(node)	(keytt(node) == ctb(LUA_VSHRSTR))
#define keystrval(node)		(gco2ts(keyval(node).gc))

#define setnilkey(node)		(keytt(node) = LUA_VNIL) /* @LuaExt: LUA_TNIL */

#define keyiscollectable(n)	(keytt(n) & BIT_ISCOLLECTABLE)

#define gckey(n)	(keyval(n).gc)
#define gckeyN(n)	(keyiscollectable(n) ? gckey(n) : NULL)


/*
** Dead keys in tables have the tag DEADKEY but keep their original
** gcvalue. This distinguishes them from regular keys but allows them to
** be found when searched in a special way. ('next' needs that to find
** keys removed from a table during a traversal.)
*/
#define setdeadkey(node)	(keytt(node) = LUA_TDEADKEY)
#define keyisdead(node)		(keytt(node) == LUA_TDEADKEY)

/* }================================================================== */

#if defined(LUA_EXT_ITERATION)
#define LUA_VPAIRS makevariant(LUA_TITER, 0)
#define LUA_VIPAIRS makevariant(LUA_TITER, 1)

#define ttisitern(o) checktype((o), LUA_TITER)
#define ttispairs(o) checktag((o), LUA_VPAIRS)
#define ttisipairs(o) checktag((o), LUA_VIPAIRS)

#define parisv(o) check_exp(ttisitern(o), val_(o).it)
#define chgpairsv(obj, x) val_(obj).it = (x)
#define setpairsv(obj, x) setintern(obj, x, LUA_VPAIRS)
#define setipairsv(obj, x) setintern(obj, x, LUA_VIPAIRS)
#define setintern(obj, x, tag) \
  LUA_MLM_BEGIN                \
  TValue *io = (obj);          \
  val_(io).it = (x);           \
  settt_(io, tag);             \
  LUA_MLM_END
#endif

/* }================================================================== */

/*
** {==================================================================
** Vector Object API
** @ImplicitVec: single component vectors are represented by LUA_TNUMBER
** ===================================================================
*/

#define LUA_VVECTOR2 makevariant(LUA_TVECTOR, 0)
#define LUA_VVECTOR3 makevariant(LUA_TVECTOR, 1)
#define LUA_VVECTOR4 makevariant(LUA_TVECTOR, 2)
#define LUA_VQUAT makevariant(LUA_TVECTOR, 3)

#define ttisvector(o) checktype((o), LUA_TVECTOR)
#define ttisvector2(o) checktag((o), LUA_VVECTOR2)
#define ttisvector3(o) checktag((o), LUA_VVECTOR3)
#define ttisvector4(o) checktag((o), LUA_VVECTOR4)
#define ttisquat(o) checktag((o), LUA_VQUAT)

#define vvaltt(o) rawtt(o)
#define vvalueraw(o) ((o).f4)
#define vvalue_(o) vvalueraw(val_((o)))
#define vvalue(o) check_exp(ttisvector(o), vvalue_(o))
#define setvvalue(obj, x, o) \
  LUA_MLM_BEGIN              \
  TValue *io = (obj);        \
  val_(io).f4 = (x);         \
  settt_(io, (o));           \
  LUA_MLM_END

/* Conversion between internal/external storage types */
#if defined(LUAGLM_HALF_TYPE)
  #define vloadf(F) lua_fromhalf((F))
  #define vstoref(F) lua_tohalf((F))
  #define vload(F) luaO_loadv(&(F))
  #define vstore(F) luaO_storev(&(F))
  LUAI_FUNC lua_Float4 (luaO_loadv)(const luai_Float4 *input);
  LUAI_FUNC luai_Float4 (luaO_storev)(const lua_Float4 *input);
#else
  #define vloadf(F) (F)
  #define vstoref(F) (F)
  #define vload(F) (F)
  #define vstore(F) (F)
#endif

/* get/set wrappers for accessing luai_Float4 by index */
#define vgeti(v, i) vloadf((v).v4[i])
#define vseti(v, i, f) ((v).v4[i] = vstoref(f))

/* Compute the vector tag associated with component size L. Does not sanitize
** and assumes equals 2, 3, or 4 */
#define vvaltag(L) cast(lu_tag, makevariant(LUA_TVECTOR, ((L) - 2) /* & 0x3 */))

/* Compute the vector length associated with vector tag V. Does not sanitize
** and assumes a valid tag */
#define vvallen(V) (((V) == LUA_VQUAT) ? 4 : (2 + (((V) & BITS_VARIANT) >> VARIANT_OFFSET)))
#define ttvlen(o) vvallen(vvaltt(o))

/* Compare raw types (used to verify mat+mat or vec+vec operations) */
#define tteq(o1, o2) (rawtt(o1) == rawtt(o2))

/* Compare variant bits (used to verify dimensions to vec+mat operations) */
#define ttvareq(o1, o2) ((rawtt(o1) & BITS_VARIANT) == (rawtt(o2) & BITS_VARIANT))

/* Table for accessing (and swizzling) vectors. */
LUAI_DDEC(const lu_byte luaO_vecindex[UCHAR_MAX + 1]);

/* }================================================================== */

/*
** {==================================================================
** Matrix Object API
** ===================================================================
*/

typedef union lua_Matrix {
  lua_mat4 m4;  /* X-by-4 matrix */
  lua_mat3 m3;  /* X-by-3 matrix */
  lua_mat2 m2;  /* X-by-2 matrix */
} lua_Matrix;

typedef struct GCMatrix {
  CommonHeader;
  lua_Matrix m;
} GCMatrix;

#define LUA_VMATRIX2 makevariant(LUA_TMATRIX, 0)
#define LUA_VMATRIX3 makevariant(LUA_TMATRIX, 1)
#define LUA_VMATRIX4 makevariant(LUA_TMATRIX, 2)

#define ttismatrix(o) checktype((o), LUA_TMATRIX)
#define ttismatrix2(o) checktag((o), ctb(LUA_VMATRIX2))
#define ttismatrix3(o) checktag((o), ctb(LUA_VMATRIX3))
#define ttismatrix4(o) checktag((o), ctb(LUA_VMATRIX4))
#define ttisglm(o) (ttisvector(o) || ttismatrix(o))

#define mvaltt(o) ttypetag(o)
#define mvalue_(o) gco2mat(val_(o).gc)->m

#define mvalue(o) check_exp(ttismatrix(o), mvalue_(o))
#define m2value(o) check_exp(ttismatrix2(o), mvalue_(o).m2)
#define m3value(o) check_exp(ttismatrix3(o), mvalue_(o).m3)
#define m4value(o) check_exp(ttismatrix4(o), mvalue_(o).m4)
#define setmvalue(L, o, x, t) \
  LUA_MLM_BEGIN               \
  TValue *io = (o);           \
  GCMatrix *x_ = (x);         \
  val_(io).gc = obj2gco(x_);  \
  settt_(io, ctb(t));         \
  checkliveness(L, io);       \
  LUA_MLM_END

#define mvaltag(L) cast(lu_tag, makevariant(LUA_TMATRIX, ((L) - 2) /* & 0x3 */))
#define mvallen(V) (2 + (((V) & BITS_VARIANT) >> VARIANT_OFFSET))
#define ttmlen(o) mvallen(mvaltt(o))

/* }================================================================== */

/*
** 'module' operation for hashing (size is always a power of 2)
*/
#define lmod(s,size) \
	(check_exp((size&(size-1))==0, (cast_int((s) & ((size)-1)))))


#define twoto(x)	(1<<(x))
#define sizenode(t)	(twoto((t)->lsizenode))
#if defined(_MSC_VER) && _MSC_VER >= 1920 && defined(__cplusplus)
  /* @LuaExt: suppress http://lua-users.org/lists/lua-l/2010-10/msg00703.html */
  #define sizenode_t(t) _Pragma("warning(suppress:4334)") cast_sizet(sizenode(t))
#else
  #define sizenode_t(t) cast_sizet(sizenode(t))
#endif


/* size of buffer for 'luaO_utf8esc' function */
#define UTF8BUFFSZ	8

LUAI_FUNC int luaO_utf8esc (char *buff, unsigned long x);
LUAI_FUNC int luaO_ceillog2 (unsigned int x);
LUAI_FUNC int luaO_rawarith (lua_State *L, int op, const TValue *p1,
                             const TValue *p2, TValue *res);
LUAI_FUNC void luaO_arith (lua_State *L, int op, const TValue *p1,
                           const TValue *p2, StkId res);
LUAI_FUNC size_t luaO_str2num (const char *s, TValue *o);
LUAI_FUNC int luaO_hexavalue (int c);
LUAI_FUNC void luaO_tostring (lua_State *L, TValue *obj);
LUAI_FUNC const char *luaO_pushvfstring (lua_State *L, const char *fmt,
                                                       va_list argp);
LUAI_FUNC const char *luaO_pushfstring (lua_State *L, const char *fmt, ...);
LUAI_FUNC void luaO_chunkid (char *out, const char *source, size_t srclen);

/* @LuaExt: avoid luaO_pushvfstring */
LUAI_FUNC int luaO_tostringbuff (const TValue *obj, char *buff);

#if defined(LUA_EXT_JOAAT)
LUAI_FUNC lua_Unsigned luaO_jenkins (const char *string, size_t length, int ignore_case);
#endif

#endif

