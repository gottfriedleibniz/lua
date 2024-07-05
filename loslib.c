/*
** $Id: loslib.c $
** Standard Operating System library
** See Copyright Notice in lua.h
*/

#define loslib_c
#define LUA_LIB

#include "lprefix.h"


#include <errno.h>
#include <locale.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "lua.h"

#include "lauxlib.h"
#include "lualib.h"


/*
** {==================================================================
** List of valid conversion specifiers for the 'strftime' function;
** options are grouped by length; group of length 2 start with '||'.
** ===================================================================
*/
#if !defined(LUA_STRFTIMEOPTIONS)	/* { */

#if defined(LUA_USE_WINDOWS)
#define LUA_STRFTIMEOPTIONS  "aAbBcdHIjmMpSUwWxXyYzZ%" \
    "||" "#c#x#d#H#I#j#m#M#S#U#w#W#y#Y"  /* two-char options */
#elif defined(LUA_USE_C89)  /* ANSI C 89 (only 1-char options) */
#define LUA_STRFTIMEOPTIONS  "aAbBcdHIjmMpSUwWxXyYZ%"
#else  /* C99 specification */
#define LUA_STRFTIMEOPTIONS  "aAbBcCdDeFgGhHIjmMnprRStTuUVwWxXyYzZ%" \
    "||" "EcECExEXEyEY" "OdOeOHOIOmOMOSOuOUOVOwOWOy"  /* two-char options */
#endif

#endif					/* } */
/* }================================================================== */


/*
** {==================================================================
** Configuration for time-related stuff
** ===================================================================
*/

/*
** type to represent time_t in Lua
*/
#if !defined(LUA_NUMTIME)	/* { */

#define l_timet			lua_Integer
#define l_pushtime(L,t)		lua_pushinteger(L,(lua_Integer)(t))
#define l_gettime(L,arg)	luaL_checkinteger(L, arg)

#else				/* }{ */

#define l_timet			lua_Number
#define l_pushtime(L,t)		lua_pushnumber(L,(lua_Number)(t))
#define l_gettime(L,arg)	luaL_checknumber(L, arg)

#endif				/* } */


#if !defined(l_gmtime)		/* { */
/*
** By default, Lua uses gmtime/localtime, except when POSIX is available,
** where it uses gmtime_r/localtime_r
*/

#if defined(LUA_USE_POSIX)	/* { */

#define l_gmtime(t,r)		gmtime_r(t,r)
#define l_localtime(t,r)	localtime_r(t,r)

#else				/* }{ */

/* ISO C definitions */
#define l_gmtime(t,r)		((void)(r)->tm_sec, gmtime(t))
#define l_localtime(t,r)	((void)(r)->tm_sec, localtime(t))

#endif				/* } */

#endif				/* } */

/* }================================================================== */


/*
** {==================================================================
** Configuration for 'tmpnam':
** By default, Lua uses tmpnam except when POSIX is available, where
** it uses mkstemp.
** ===================================================================
*/
#if !defined(lua_tmpnam)	/* { */

/* @LuaExt: tmpnam is not defined on WASI */
#if defined(LUA_USE_POSIX) || defined(__wasi__)	/* { */

#include <unistd.h>

#define LUA_TMPNAMBUFSIZE	32

#if !defined(LUA_TMPNAMTEMPLATE)
#define LUA_TMPNAMTEMPLATE	"/tmp/lua_XXXXXX"
#endif

#define lua_tmpnam(b,e) { \
        strcpy(b, LUA_TMPNAMTEMPLATE); \
        e = mkstemp(b); \
        if (e != -1) close(e); \
        e = (e == -1); }

#else				/* }{ */

/* ISO C definitions */
#define LUA_TMPNAMBUFSIZE	L_tmpnam
#define lua_tmpnam(b,e)		{ e = (tmpnam(b) == NULL); }

#endif				/* } */

#endif				/* } */
/* }================================================================== */

/*
** {==================================================================
** High-Resolution Timers
** ===================================================================
*/
#if defined(LUA_EXT_CHRONO)
#include "llimits.h"

#define LUA_SYS_CLOCK /* high resolution timers enabled */
#if defined(_WIN32)
  #define WIN32_LEAN_AND_MEAN
  #include <windows.h>
  static LARGE_INTEGER freq;
  static void InitPerformanceCounter(void) {
    static int init = 1; /* @NOTE: potential race condition */
    if (init) {
      init = 0;
      QueryPerformanceFrequency(&freq);
    }
  }
#elif defined(__APPLE__)
  #include <sys/time.h>
#elif defined(_POSIX_TIMERS) && _POSIX_TIMERS > 0
#ifdef _POSIX_MONOTONIC_CLOCK
  #define HAVE_CLOCK_GETTIME
  #include <time.h>
#else
  #include <sys/time.h>
  #warning "A nanosecond resolution clock is not available; falling back to gettimeofday()"
#endif
#else
  #undef LUA_SYS_CLOCK
  #warning "A nanosecond resolution clock is not available"
#endif

#if defined(LUA_SYS_CLOCK)
#include <stdint.h>
static int os_microtime(lua_State *L) {
#if defined(_WIN32)
  LARGE_INTEGER now;
  QueryPerformanceCounter(&now);
  l_pushtime(L, ((1000000L * now.QuadPart) / freq.QuadPart));
#elif defined(HAVE_CLOCK_GETTIME)
  struct timespec tp;
  if (clock_gettime(CLOCK_MONOTONIC, &tp) != 0)
    return luaL_error(L, "clock_gettime(CLOCK_MONOTONIC) failed:%s", strerror(errno));
  l_pushtime(L, (tp.tv_sec * 1000000L) + (tp.tv_nsec / 1000L));
#else
  struct timeval tp;
  if (gettimeofday(&tp, NULL) < 0)
    return luaL_error(L, "gettimeofday() failed!:%s", strerror(errno));
  l_pushtime(L, (tp.tv_sec * 1000000L) + tp.tv_usec);
#endif
  return 1;
}

#if LUA_32BITS == 0
static int os_nanotime(lua_State *L) {
#if defined(_WIN32)
  LARGE_INTEGER now;
  QueryPerformanceCounter(&now);
  l_pushtime(L, ((1000000000L * now.QuadPart) / freq.QuadPart));
#elif defined(HAVE_CLOCK_GETTIME)
  struct timespec tp;
  if (clock_gettime(CLOCK_MONOTONIC, &tp) != 0)
    return luaL_error(L, "clock_gettime(CLOCK_MONOTONIC) failed:%s", strerror(errno));
  l_pushtime(L, (tp.tv_sec * 1000000000L) + tp.tv_nsec);
#else
  struct timeval tp;
  if (gettimeofday(&tp, NULL) < 0)
    return luaL_error(L, "gettimeofday() failed!:%s", strerror(errno));
  l_pushtime(L, (tp.tv_sec * 1000000000L) + (tp.tv_usec * 1000L));
#endif
  return 1;
}
#endif
#endif

/*
@@ LUA_SYS_RDTSC Sample the rdtsc instruction and return the processor timestamp
*/
#if LUA_INT_DEFAULT == LUA_INT_LONGLONG && defined(UINTMAX_MAX)
#if defined(_MSC_VER) && defined(_M_X64)
  #include <intrin.h>
  #define LUA_SYS_RDTSC
#elif defined(__GNUC__) && defined(__x86_64__)
  #if defined(__has_include) && __has_include(<x86intrin.h>)
    #include <x86intrin.h>
    #define LUA_SYS_RDTSC
  #endif
#elif defined(__GNUC__) && defined(__aarch64__)
  #define LUA_SYS_RDTSC
#endif
#endif

#if defined(LUA_SYS_RDTSC)
static int os_rdtsc(lua_State *L) {
#if defined(__aarch64__)
  uint64_t cval;
  __asm__ volatile("mrs %0, cntvct_el0" : "=r" (cval));
  l_pushtime(L, cval);
#else
  unsigned int aux;
  l_pushtime(L, __rdtscp(&aux));
#endif
  return 1;
}
#endif

#endif
/* }================================================================== */

#if !defined(l_system)
#if defined(LUA_USE_IOS)
/* Despite claiming to be ISO C, iOS does not implement 'system'. */
#define l_system(cmd) ((cmd) == NULL ? 0 : -1)
#else
#define l_system(cmd)	system(cmd)  /* default definition */
#endif
#endif


#if !defined(LUA_SANDBOX_LIBS)
static int os_execute (lua_State *L) {
  const char *cmd = luaL_optstring(L, 1, NULL);
  int stat;
  errno = 0;
  stat = l_system(cmd);
  if (cmd != NULL)
    return luaL_execresult(L, stat);
  else {
    lua_pushboolean(L, stat);  /* true if there is a shell */
    return 1;
  }
}


static int os_remove (lua_State *L) {
  const char *filename = luaL_checkstring(L, 1);
  errno = 0;
  return luaL_fileresult(L, remove(filename) == 0, filename);
}


static int os_rename (lua_State *L) {
  const char *fromname = luaL_checkstring(L, 1);
  const char *toname = luaL_checkstring(L, 2);
  errno = 0;
  return luaL_fileresult(L, rename(fromname, toname) == 0, NULL);
}


static int os_tmpname (lua_State *L) {
  char buff[LUA_TMPNAMBUFSIZE];
  int err;
  lua_tmpnam(buff, err);
  if (l_unlikely(err))
    return luaL_error(L, "unable to generate a unique filename");
  lua_pushstring(L, buff);
  return 1;
}


static int os_getenv (lua_State *L) {
  lua_pushstring(L, getenv(luaL_checkstring(L, 1)));  /* if NULL push nil */
  return 1;
}
#endif


static int os_clock (lua_State *L) {
  lua_pushnumber(L, ((lua_Number)clock())/(lua_Number)CLOCKS_PER_SEC);
  return 1;
}


/*
** {======================================================
** Time/Date operations
** { year=%Y, month=%m, day=%d, hour=%H, min=%M, sec=%S,
**   wday=%w+1, yday=%j, isdst=? }
** =======================================================
*/

/*
** About the overflow check: an overflow cannot occur when time
** is represented by a lua_Integer, because either lua_Integer is
** large enough to represent all int fields or it is not large enough
** to represent a time that cause a field to overflow.  However, if
** times are represented as doubles and lua_Integer is int, then the
** time 0x1.e1853b0d184f6p+55 would cause an overflow when adding 1900
** to compute the year.
*/
static void setfield (lua_State *L, const char *key, int value, int delta) {
  #if (defined(LUA_NUMTIME) && LUA_MAXINTEGER <= INT_MAX)
    if (l_unlikely(value > LUA_MAXINTEGER - delta))
      luaL_error(L, "field '%s' is out-of-bound", key);
  #endif
  lua_pushinteger(L, (lua_Integer)value + delta);
  lua_setfield(L, -2, key);
}


static void setboolfield (lua_State *L, const char *key, int value) {
  if (value < 0)  /* undefined? */
    return;  /* does not set field */
  lua_pushboolean(L, value);
  lua_setfield(L, -2, key);
}


/*
** Set all fields from structure 'tm' in the table on top of the stack
*/
static void setallfields (lua_State *L, struct tm *stm) {
  setfield(L, "year", stm->tm_year, 1900);
  setfield(L, "month", stm->tm_mon, 1);
  setfield(L, "day", stm->tm_mday, 0);
  setfield(L, "hour", stm->tm_hour, 0);
  setfield(L, "min", stm->tm_min, 0);
  setfield(L, "sec", stm->tm_sec, 0);
  setfield(L, "yday", stm->tm_yday, 1);
  setfield(L, "wday", stm->tm_wday, 1);
  setboolfield(L, "isdst", stm->tm_isdst);
}


static int getboolfield (lua_State *L, const char *key) {
  int res;
  res = (lua_getfield(L, -1, key) == LUA_TNIL) ? -1 : lua_toboolean(L, -1);
  lua_pop(L, 1);
  return res;
}


static int getfield (lua_State *L, const char *key, int d, int delta) {
  int isnum;
  int t = lua_getfield(L, -1, key);  /* get field and its type */
  lua_Integer res = lua_tointegerx(L, -1, &isnum);
  if (!isnum) {  /* field is not an integer? */
    if (l_unlikely(t != LUA_TNIL))  /* some other value? */
      return luaL_error(L, "field '%s' is not an integer", key);
    else if (l_unlikely(d < 0))  /* absent field; no default? */
      return luaL_error(L, "field '%s' missing in date table", key);
    res = d;
  }
  else {
    if (!(res >= 0 ? res - delta <= INT_MAX : INT_MIN + delta <= res))
      return luaL_error(L, "field '%s' is out-of-bound", key);
    res -= delta;
  }
  lua_pop(L, 1);
  return (int)res;
}


static const char *checkoption (lua_State *L, const char *conv,
                                ptrdiff_t convlen, char *buff) {
  const char *option = LUA_STRFTIMEOPTIONS;
  int oplen = 1;  /* length of options being checked */
  for (; *option != '\0' && oplen <= convlen; option += oplen) {
    if (*option == '|')  /* next block? */
      oplen++;  /* will check options with next length (+1) */
    else if (memcmp(conv, option, oplen) == 0) {  /* match? */
      memcpy(buff, conv, oplen);  /* copy valid option to buffer */
      buff[oplen] = '\0';
      return conv + oplen;  /* return next item */
    }
  }
  luaL_argerror(L, 1,
    lua_pushfstring(L, "invalid conversion specifier '%%%s'", conv));
  return conv;  /* to avoid warnings */
}


static time_t l_checktime (lua_State *L, int arg) {
  l_timet t = l_gettime(L, arg);
  luaL_argcheck(L, (time_t)t == t, arg, "time out-of-bounds");
  return (time_t)t;
}


/* maximum size for an individual 'strftime' item */
#define SIZETIMEFMT	250


static int os_date (lua_State *L) {
  size_t slen;
  const char *s = luaL_optlstring(L, 1, "%c", &slen);
  time_t t = luaL_opt(L, l_checktime, 2, time(NULL));
  const char *se = s + slen;  /* 's' end */
  struct tm tmr, *stm;
  if (*s == '!') {  /* UTC? */
    stm = l_gmtime(&t, &tmr);
    s++;  /* skip '!' */
  }
  else
    stm = l_localtime(&t, &tmr);
  if (stm == NULL)  /* invalid date? */
    return luaL_error(L,
                 "date result cannot be represented in this installation");
  if (strcmp(s, "*t") == 0) {
    lua_createtable(L, 0, 9);  /* 9 = number of fields */
    setallfields(L, stm);
  }
  else {
    char cc[4];  /* buffer for individual conversion specifiers */
    luaL_Buffer b;
    cc[0] = '%';
    luaL_buffinit(L, &b);
    while (s < se) {
      if (*s != '%')  /* not a conversion specifier? */
        luaL_addchar(&b, *s++);
      else {
        size_t reslen;
        char *buff = luaL_prepbuffsize(&b, SIZETIMEFMT);
        s++;  /* skip '%' */
        s = checkoption(L, s, se - s, cc + 1);  /* copy specifier to 'cc' */
        reslen = strftime(buff, SIZETIMEFMT, cc, stm);
        luaL_addsize(&b, reslen);
      }
    }
    luaL_pushresult(&b);
  }
  return 1;
}


static int os_time (lua_State *L) {
  time_t t;
  if (lua_isnoneornil(L, 1))  /* called without args? */
    t = time(NULL);  /* get current time */
  else {
    struct tm ts;
    luaL_checktype(L, 1, LUA_TTABLE);
    lua_settop(L, 1);  /* make sure table is at the top */
    ts.tm_year = getfield(L, "year", -1, 1900);
    ts.tm_mon = getfield(L, "month", -1, 1);
    ts.tm_mday = getfield(L, "day", -1, 0);
    ts.tm_hour = getfield(L, "hour", 12, 0);
    ts.tm_min = getfield(L, "min", 0, 0);
    ts.tm_sec = getfield(L, "sec", 0, 0);
    ts.tm_isdst = getboolfield(L, "isdst");
    t = mktime(&ts);
    setallfields(L, &ts);  /* update fields with normalized values */
  }
  if (t != (time_t)(l_timet)t || t == (time_t)(-1))
    return luaL_error(L,
                  "time result cannot be represented in this installation");
  l_pushtime(L, t);
  return 1;
}


/*
** Use a built-in implementation of difftime to ensure consistent behavior
** across C libraries. For example, glibc will account for time_t overflows
** while ucrt will return 0 and set errno to EINVAL.
*/
#if defined(LUA_EXT_CHRONO) && defined(UINTMAX_MAX)
static lua_Number t_subtract(time_t end, time_t start) {
  if (((time_t)-1 >= 0))  /* unsigned */
    return cast_num(end - start);
  else {
    uintmax_t dt = cast(uintmax_t, end) - cast(uintmax_t, start);
    lua_Number result = cast_num(dt);
    if (UINTMAX_MAX / 2 < INTMAX_MAX) {
      uintmax_t hdt = dt / 2;
      time_t dht = (end / 2) - (start / 2);
      if (2 < dht - hdt + 1) {
        result = cast_num(dt + 2.0L * (UINTMAX_MAX - UINTMAX_MAX / 2));
      }
    }
    return result;
  }
}

static int os_difftime (lua_State *L) {
  time_t t1 = l_checktime(L, 1);
  time_t t2 = l_checktime(L, 2);
  lua_pushnumber(L, t1 < t2 ? -t_subtract(t2, t1) : t_subtract(t1, t2));
  return 1;
}
#else
static int os_difftime (lua_State *L) {
  time_t t1 = l_checktime(L, 1);
  time_t t2 = l_checktime(L, 2);
  lua_pushnumber(L, (lua_Number)difftime(t1, t2));
  return 1;
}
#endif

/* }====================================================== */


#if !defined(LUA_SANDBOX_LIBS)
static int os_setlocale (lua_State *L) {
#if defined(__EMSCRIPTEN__) || defined(__wasi__)
  /* musl: all locale names are valid: invalid or unavailable definitions become
  ** aliases for C.UTF-8 */
  size_t l;
  const char *s = luaL_optlstring(L, 1, NULL, &l);
  if (l == 0)  /* empty string or nil */
    lua_pushliteral(L, "C");
  else if (strcmp(s, "C") == 0 || strcmp(s, "C.UTF-8") == 0)
    lua_pushvalue(L, 1);
  else
    lua_pushnil(L);
  return 1;
#else
  static const int cat[] = {LC_ALL, LC_COLLATE, LC_CTYPE, LC_MONETARY,
                      LC_NUMERIC, LC_TIME};
  static const char *const catnames[] = {"all", "collate", "ctype", "monetary",
     "numeric", "time", NULL};
  const char *l = luaL_optstring(L, 1, NULL);
  int op = luaL_checkoption(L, 2, "all", catnames);
  lua_pushstring(L, setlocale(cat[op], l));
  return 1;
#endif
}


static int os_exit (lua_State *L) {
  int status;
  if (lua_isboolean(L, 1))
    status = (lua_toboolean(L, 1) ? EXIT_SUCCESS : EXIT_FAILURE);
  else
    status = (int)luaL_optinteger(L, 1, EXIT_SUCCESS);
  if (lua_toboolean(L, 2))
    lua_close(L);
  if (L) exit(status);  /* 'if' to avoid warnings for unreachable 'return' */
  return 0;
}
#endif


static const luaL_Reg syslib[] = {
  {"clock",     os_clock},
  {"date",      os_date},
  {"difftime",  os_difftime},
#if !defined(LUA_SANDBOX_LIBS)
  {"execute",   os_execute},
  {"exit",      os_exit},
  {"getenv",    os_getenv},
  {"remove",    os_remove},
  {"rename",    os_rename},
  {"setlocale", os_setlocale},
#endif
  {"time",      os_time},
#if !defined(LUA_SANDBOX_LIBS)
  {"tmpname",   os_tmpname},
#endif
#if defined(LUA_SYS_CLOCK)
  {"usec",      os_microtime},
  #if LUA_32BITS == 0
  {"nsec",      os_nanotime},
  #endif
#endif
#if defined(LUA_SYS_RDTSC)
  {"rdtsc",     os_rdtsc},
#endif
#if defined(LUA_EXT_API)
  {"platform",  NULL},
#endif
  {NULL, NULL}
};

/* }====================================================== */

#if defined(LUA_EXT_API) && !defined(LUA_PLATFORM)
#define LUA_PLATFORM "unknown"
#endif


LUAMOD_API int luaopen_os (lua_State *L) {
#if defined(LUA_SYS_CLOCK) && defined(_WIN32)
  InitPerformanceCounter();
#endif
  luaL_newlib(L, syslib);
#if defined(LUA_EXT_API)
  lua_pushliteral(L, LUA_PLATFORM);
  lua_setfield(L, -2, "platform");
#endif
  return 1;
}

