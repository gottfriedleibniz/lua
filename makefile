# Makefile for building Lua
# Based on https://www.lua.org/download.html

# == CHANGE THE SETTINGS BELOW TO SUIT YOUR ENVIRONMENT =======================

# Your platform. See 'make help' for possible values.
PLAT= guess

# Lua configuration: see README & luaconf.h
LUA_CONFIG=

# Environment variables
AR?= ar
RANLIB?= ranlib
RM?= rm -f
LD= $(CC)

# Compiler configuration
DEFAULT_CFLAGS= -Wall -Wextra -std=c99 -O3
DEFAULT_LDFLAGS= -s
DEFAULT_LIBS= -lm
DEFAULT_ARFLAGS= rc

# Platform detection
UNAME?= $(shell uname -s 2>/dev/null | tr '[:upper:]' '[:lower:]' || echo posix)
UNAME:= $(patsubst cygwin%,cygwin,$(UNAME))
UNAME:= $(patsubst mingw%,mingw,$(UNAME))
UNAME:= $(patsubst msys%,msys,$(UNAME))
COMPILER_VERSION:= $(shell $(CC) --version)
ifneq '' '$(findstring nvc,$(COMPILER_VERSION))'
	DEFAULT_CFLAGS+= -Kieee
else ifneq '' '$(findstring Intel,$(COMPILER_VERSION))'
	DEFAULT_CFLAGS+= -fp-model=precise -fno-math-errno
else ifneq '' '$(findstring icc,$(COMPILER_VERSION))'
	DEFAULT_CFLAGS+= -diag-disable=279,10441,11074,11076 -fno-math-errno
	DEFAULT_LDFLAGS+= -diag-disable=10441
else
	DEFAULT_CFLAGS+= -fno-math-errno
endif

# Build options
ifdef DEBUG
EXTRA_OBJS+= ltests.o
DEFAULT_CFLAGS:=$(patsubst -O%,-O0,$(DEFAULT_CFLAGS)) -g -DLUA_USER_H='"ltests.h"'
DEFAULT_LDFLAGS:=$(patsubst -s,,$(DEFAULT_LDFLAGS))
endif

ifdef UNITY
CORE_O= onelua.o
LIB_O=
EXTRA_OBJS=
LUA_CONFIG+= -DMAKE_LIB
endif

ifdef EXTRA_TESTS
DEFAULT_CFLAGS+= -g -DLUAI_ASSERT -DHARDSTACKTESTS -DHARDMEMTESTS -DEMERGENCYGCTESTS -DEXTERNMEMCHECK
DEFAULT_LDFLAGS:= $(patsubst -s,,$(DEFAULT_LDFLAGS))
endif

ifdef UBSAN
UBSAN_FLAGS= -fsanitize=null,undefined,alignment,signed-integer-overflow
DEFAULT_CFLAGS+= $(UBSAN_FLAGS) -g -fno-omit-frame-pointer
DEFAULT_LDFLAGS:= $(patsubst -s,,$(DEFAULT_LDFLAGS)) $(UBSAN_FLAGS)
endif

ifdef ASAN
ASAN_FLAGS= -fsanitize=address
DEFAULT_CFLAGS+= $(ASAN_FLAGS) -g -fno-omit-frame-pointer
DEFAULT_LDFLAGS:= $(patsubst -s,,$(DEFAULT_LDFLAGS)) $(ASAN_FLAGS)
endif

# == Verbose ==========================
ifdef VERBOSE
CWARNSCPP= \
	-Wfatal-errors \
	-Wshadow \
	-Wsign-compare \
	-Wundef \
	-Wwrite-strings \
	-Wredundant-decls \
	-Wdisabled-optimization \
	-Wdouble-promotion \
	-Wmissing-declarations \
	# the next warnings might be useful sometimes,
	# but usually they generate too much noise
	# -Werror \
	# -pedantic   # warns if we use jump tables \
	# -Wconversion  \
	# -Wsign-conversion \
	# -Wstrict-overflow=2 \
	# -Wformat=2 \
	# -Wcast-qual \

# Warnings for gcc, not valid for clang
CWARNGCC= \
	-Wlogical-op \
	-Wno-aggressive-loop-optimizations \
	-Wno-inline \

# The next warnings are neither valid nor needed for C++
CWARNSC= -Wdeclaration-after-statement \
	-Wmissing-prototypes \
	-Wnested-externs \
	-Wstrict-prototypes \
	-Wc++-compat \
	-Wold-style-definition

CWARNEVERYTHING= -Weverything \
	-Wno-padded \
	-Wno-extra-semi-stmt \
	-Wno-gnu-label-as-value \
	-Wno-sign-conversion \
	-Wno-cast-qual \
	-Wno-documentation-pedantic \
	-Wno-float-equal \
	-Wno-bad-function-cast

DEFAULT_CFLAGS+= $(CWARNSCPP) $(CWARNSC)
ifeq ($(CC),gcc)
	DEFAULT_CFLAGS+= $(CWARNGCC)
endif
endif

# == END OF USER SETTINGS -- NO NEED TO CHANGE ANYTHING BELOW THIS LINE =======

PLATS= guess aix bsd c89 freebsd netbsd openbsd linux linux-noreadline darwin msys cygwin mingw posix sunos windows emscripten wasi

LUA_T?= lua
LUA_O?= lua.o

LUA_A?= liblua.a
CORE_O?= lapi.o lcode.o lctype.o ldebug.o ldo.o ldump.o lfunc.o lgc.o llex.o lmem.o lobject.o lopcodes.o lparser.o lstate.o lstring.o ltable.o ltm.o lundump.o lvm.o lzio.o
LIB_O?= lauxlib.o lbaselib.o lcorolib.o ldblib.o liolib.o lmathlib.o loadlib.o loslib.o lstrlib.o ltablib.o lutf8lib.o linit.o

BASE_O= $(CORE_O) $(LIB_O) $(EXTRA_OBJS)
ALL_O= $(BASE_O) $(LUA_O)
ALL_T= $(LUA_A) $(LUA_T)

# PLAT dependent configuration
SYSCFLAGS=
SYSLDFLAGS=
SYSLIBS=

CFLAGS= $(DEFAULT_CFLAGS) $(EXTRA_CFLAGS) $(SYSCFLAGS) $(LUA_CONFIG)
LDFLAGS= $(DEFAULT_LDFLAGS) $(EXTRA_LDFLAGS) $(SYSLDFLAGS)
LDLIBS= $(DEFAULT_LIBS) $(EXTRA_LIBS) $(SYSLIBS)
ARFLAGS= $(DEFAULT_ARFLAGS)

# Targets
build: $(PLAT)

clean:
	$(RM) $(ALL_T) $(ALL_O) ltests.o onelua.o liblua.a lua lua.exe lua.js lua.pdb lua.wasm lua54.dylib lua54.dll lua54.exp lua54.lib $(EXTRA_OBJS)

depend:
	@$(CC) $(CFLAGS) -MM l*.c

echo:
	@echo "PLAT= $(PLAT)"
	@echo "CC= $(CC)"
	@echo "CFLAGS= $(CFLAGS)"
	@echo "LD= $(LD)"
	@echo "LDFLAGS= $(LDFLAGS)"
	@echo "LDLIBS= $(LDLIBS)"
	@echo "AR= $(AR)"
	@echo "RANLIB= $(RANLIB)"
	@echo "RM= $(RM)"
	@echo "UNAME= $(UNAME)"

help:
	@echo "Do 'make PLATFORM' where PLATFORM is one of these:"
	@echo "   $(PLATS)"
	@echo "See README.md for complete instructions."

# Targets that do not create files (not all makes understand .PHONY).
.PHONY: build clean depend echo help $(PLATS)

$(ALL_O): makefile ltests.h

$(LUA_A): $(BASE_O)

$(LUA_T): $(LUA_O) $(LUA_A)
	$(LD) -o $(LUA_T) $(LDFLAGS) $(LUA_O) $(LUA_A) $(LDLIBS)

%.a:
	$(AR) $(ARFLAGS) $@ $^
	$(RANLIB) $@

%.so:
	$(CC) -shared -o $@ $^

%.dll:
	$(LD) -shared -o $@ $^
	$(STRIP) --strip-unneeded $@

# Convenience targets for popular platforms.
guess:
	@echo Guessing $(UNAME)
	@$(MAKE) $(UNAME)

aix: CC= xlc
aix: SYSCFLAGS= -DLUA_USE_POSIX -DLUA_USE_DLOPEN
aix: SYSLIBS= -ldl
aix: SYSLDFLAGS= -brtl -bexpall
aix: $(ALL_T)

bsd: SYSCFLAGS= -DLUA_USE_POSIX -DLUA_USE_DLOPEN
bsd: SYSLIBS= -Wl,-E
bsd: $(ALL_T)

c89: DEFAULT_CFLAGS:=$(patsubst -std=%,-std=gnu89,$(DEFAULT_CFLAGS)) -Wno-unused-function
c89: SYSCFLAGS= -DLUA_USE_C89
c89: $(ALL_T)
	@echo ''
	@echo '*** C89 does not guarantee 64-bit integers for Lua.'
	@echo '*** Make sure to compile all external Lua libraries'
	@echo '*** with LUA_USE_C89 to ensure consistency'
	@echo ''

freebsd netbsd openbsd: SYSCFLAGS= -DLUA_USE_LINUX -DLUA_USE_READLINE -I/usr/include/edit
freebsd netbsd openbsd: SYSLIBS= -Wl,-E -ledit
freebsd netbsd openbsd: $(ALL_T)

darwin: DEFAULT_LDFLAGS:=$(patsubst -s,,$(DEFAULT_LDFLAGS))
darwin: SYSCFLAGS= -DLUA_USE_MACOSX -DLUA_USE_READLINE
darwin: SYSLIBS= -lreadline
darwin: $(ALL_T)

linux: SYSCFLAGS= -DLUA_USE_LINUX -DLUA_USE_READLINE
linux: SYSLIBS= -Wl,-E -ldl -lreadline
linux: $(ALL_T)

linux-noreadline: SYSCFLAGS= -DLUA_USE_LINUX
linux-noreadline: SYSLIBS= -Wl,-E -ldl
linux-noreadline: $(ALL_T)

posix: SYSCFLAGS= -DLUA_USE_POSIX
posix: $(ALL_T)

sunos: SYSCFLAGS= -DLUA_USE_POSIX -DLUA_USE_DLOPEN -D_REENTRANT
sunos: SYSLIBS= -ldl
sunos: $(ALL_T)

# Windows
lua54.dll: $(BASE_O)

msys cygwin: STRIP?= strip
msys cygwin: LUA_T= lua.exe
msys cygwin: SYSCFLAGS= -D_WIN32 -DLUA_USE_POSIX -DLUA_BUILD_AS_DLL
msys cygwin: lua54.dll $(LUA_O)
	$(LD) -o $(LUA_T) $(LDFLAGS) $(LUA_O) lua54.dll $(LDLIBS)

mingw: STRIP?= strip
mingw: UNAME= win32
mingw: LUA_T= lua.exe
mingw: SYSCFLAGS= -DLUA_BUILD_AS_DLL -DLUA_USE_BUILTIN_JMP
mingw: lua54.dll $(LUA_O)
	$(LD) -o $(LUA_T) $(LDFLAGS) $(LUA_O) lua54.dll $(LDLIBS)

# WSL
windows: CC= clang.exe
windows: AR= llvm-ar.exe
windows: RANLIB= llvm-ranlib.exe
windows: STRIP= llvm-strip.exe
windows: UNAME= win32
windows: LUA_T= lua.exe
windows: SYSCFLAGS= -DLUA_BUILD_AS_DLL -DLUA_NO_UNWIND
ifdef DEBUG
windows: SYSLDFLAGS= -Xlinker /STACK:2097152
endif
windows: DEFAULT_LIBS:=$(patsubst -lm,,$(DEFAULT_LIBS))
windows: DEFAULT_LDFLAGS:=$(patsubst -s,,$(DEFAULT_LDFLAGS))
windows: lua54.dll $(LUA_O)
	$(LD) -o $(LUA_T) $(LDFLAGS) $(LUA_O) lua54.lib $(LDLIBS)

# WASM
emscripten: CC= emcc
emscripten: AR= emar
emscripten: RANLIB= :
emscripten: UNAME= wasm
emscripten: LUA_T= lua.js
emscripten: DEFAULT_CFLAGS+= -fPIC -sSTRICT=1 -sSUPPORT_LONGJMP=wasm -msimd128
emscripten: DEFAULT_LDFLAGS+= -sMAIN_MODULE=1 -sNODERAWFS=1 -sSUPPORT_LONGJMP=wasm -sALLOW_TABLE_GROWTH=1 -sALLOW_MEMORY_GROWTH=1 -sMALLOC=emmalloc
ifdef DEBUG
emscripten: DEFAULT_LDFLAGS+= -sTOTAL_STACK=2MB -sEXIT_RUNTIME=1 -sASSERTIONS=1 -sSAFE_HEAP=1 -sSTACK_OVERFLOW_CHECK=2
else
emscripten: DEFAULT_LDFLAGS+= -sTOTAL_STACK=1MB
endif
emscripten: SYSCFLAGS= -DLUA_USE_POSIX -DLUA_USE_DLOPEN
emscripten: SYSLIBS= -Wl,-E
emscripten: $(ALL_T)

# WASI(X)
wasi: CC= $(WASI_SDK)/bin/clang --sysroot=$(WASI_SYSROOT) --target=$(WASI_TARGET)
wasi: AR= $(WASI_SDK)/bin/ar
wasi: RANLIB= $(WASI_SDK)/bin/ranlib
wasi: STRIP= $(WASI_SDK)/bin/strip
wasi: UNAME= wasm
wasi: LUA_T= lua.wasm
wasi: SYSCFLAGS= -matomics -mbulk-memory -mmutable-globals -pthread -mthread-model posix -ftls-model=local-exec -fno-trapping-math
wasi: SYSCFLAGS+= -D_WASI_EMULATED_SIGNAL -D_WASI_EMULATED_PROCESS_CLOCKS
wasi: SYSLIBS= -lwasi-emulated-process-clocks
wasi: SYSLIBS+= -Wl,--shared-memory -Wl,--max-memory=4294967296
wasi: SYSLIBS+= -Wl,--import-memory
wasi: SYSLIBS+= -Wl,--export-dynamic
wasi: SYSLIBS+= -Wl,--export=__heap_base
wasi: SYSLIBS+= -Wl,--export=__stack_pointer
wasi: SYSLIBS+= -Wl,--export=__data_end
wasi: SYSLIBS+= -Wl,--export=__wasm_init_tls
wasi: SYSLIBS+= -Wl,--export=__wasm_signal
wasi: SYSLIBS+= -Wl,--export=__tls_size
wasi: SYSLIBS+= -Wl,--export=__tls_align
wasi: SYSLIBS+= -Wl,--export=__tls_base
wasi: $(ALL_T)
	$(WASM_OPT) -O2 --asyncify --enable-bulk-memory --enable-threads --enable-simd $(LUA_T) -o $(LUA_T)

# == DO NOT DELETE ===========================================================
lapi.o: lapi.c lprefix.h lua.h luaconf.h lapi.h llimits.h lstate.h \
 lobject.h ltm.h lzio.h lmem.h ldebug.h ldo.h lfunc.h lgc.h lstring.h \
 ltable.h lundump.h lvm.h
lauxlib.o: lauxlib.c lprefix.h lua.h luaconf.h lauxlib.h
lbaselib.o: lbaselib.c lprefix.h lua.h luaconf.h lauxlib.h lualib.h
lcode.o: lcode.c lprefix.h lua.h luaconf.h lcode.h llex.h lobject.h \
 llimits.h lzio.h lmem.h lopcodes.h lparser.h ldebug.h lstate.h ltm.h \
 ldo.h lgc.h lstring.h ltable.h lvm.h
lcorolib.o: lcorolib.c lprefix.h lua.h luaconf.h lauxlib.h lualib.h
lctype.o: lctype.c lprefix.h lctype.h lua.h luaconf.h llimits.h
ldblib.o: ldblib.c lprefix.h lua.h luaconf.h lauxlib.h lualib.h
ldebug.o: ldebug.c lprefix.h lua.h luaconf.h lapi.h llimits.h lstate.h \
 lobject.h ltm.h lzio.h lmem.h lcode.h llex.h lopcodes.h lparser.h \
 ldebug.h ldo.h lfunc.h lstring.h lgc.h ltable.h lvm.h
ldo.o: ldo.c lprefix.h lua.h luaconf.h lapi.h llimits.h lstate.h \
 lobject.h ltm.h lzio.h lmem.h ldebug.h ldo.h lfunc.h lgc.h lopcodes.h \
 lparser.h lstring.h ltable.h lundump.h lvm.h
ldump.o: ldump.c lprefix.h lua.h luaconf.h lobject.h llimits.h lstate.h \
 ltm.h lzio.h lmem.h lundump.h
lfunc.o: lfunc.c lprefix.h lua.h luaconf.h ldebug.h lstate.h lobject.h \
 llimits.h ltm.h lzio.h lmem.h ldo.h lfunc.h lgc.h
lgc.o: lgc.c lprefix.h lua.h luaconf.h ldebug.h lstate.h lobject.h \
 llimits.h ltm.h lzio.h lmem.h ldo.h lfunc.h lgc.h lstring.h ltable.h
linit.o: linit.c lprefix.h lua.h luaconf.h lualib.h lauxlib.h
liolib.o: liolib.c lprefix.h lua.h luaconf.h lauxlib.h lualib.h
llex.o: llex.c lprefix.h lua.h luaconf.h lctype.h llimits.h ldebug.h \
 lstate.h lobject.h ltm.h lzio.h lmem.h ldo.h lgc.h llex.h lparser.h \
 lstring.h ltable.h
lmathlib.o: lmathlib.c lprefix.h lua.h luaconf.h lauxlib.h lualib.h
lmem.o: lmem.c lprefix.h lua.h luaconf.h ldebug.h lstate.h lobject.h \
 llimits.h ltm.h lzio.h lmem.h ldo.h lgc.h
loadlib.o: loadlib.c lprefix.h lua.h luaconf.h lauxlib.h lualib.h
lobject.o: lobject.c lprefix.h lua.h luaconf.h lctype.h llimits.h \
 ldebug.h lstate.h lobject.h ltm.h lzio.h lmem.h ldo.h lstring.h lgc.h \
 lvm.h
lopcodes.o: lopcodes.c lprefix.h lopcodes.h llimits.h lua.h luaconf.h
loslib.o: loslib.c lprefix.h lua.h luaconf.h lauxlib.h lualib.h llimits.h
lparser.o: lparser.c lprefix.h lua.h luaconf.h lcode.h llex.h lobject.h \
 llimits.h lzio.h lmem.h lopcodes.h lparser.h ldebug.h lstate.h ltm.h \
 ldo.h lfunc.h lstring.h lgc.h ltable.h
lstate.o: lstate.c lprefix.h lua.h luaconf.h lapi.h llimits.h lstate.h \
 lobject.h ltm.h lzio.h lmem.h ldebug.h ldo.h lfunc.h lgc.h llex.h \
 lstring.h ltable.h
lstring.o: lstring.c lprefix.h lua.h luaconf.h ldebug.h lstate.h \
 lobject.h llimits.h ltm.h lzio.h lmem.h ldo.h lstring.h lgc.h
lstrlib.o: lstrlib.c lprefix.h lua.h luaconf.h lauxlib.h lualib.h
ltable.o: ltable.c lprefix.h lua.h luaconf.h ldebug.h lstate.h lobject.h \
 llimits.h ltm.h lzio.h lmem.h ldo.h lgc.h lstring.h ltable.h lvm.h
ltablib.o: ltablib.c lprefix.h lua.h luaconf.h lauxlib.h lualib.h
ltests.o: ltests.c lprefix.h lua.h luaconf.h lapi.h llimits.h lstate.h \
 lobject.h ltm.h lzio.h lmem.h lauxlib.h lcode.h llex.h lopcodes.h \
 lparser.h lctype.h ldebug.h ldo.h lfunc.h lstring.h lgc.h ltable.h \
 lualib.h
ltm.o: ltm.c lprefix.h lua.h luaconf.h ldebug.h lstate.h lobject.h \
 llimits.h ltm.h lzio.h lmem.h ldo.h lgc.h lstring.h ltable.h lvm.h
lua.o: lua.c lprefix.h lua.h luaconf.h lauxlib.h lualib.h
lundump.o: lundump.c lprefix.h lua.h luaconf.h ldebug.h lstate.h \
 lobject.h llimits.h ltm.h lzio.h lmem.h ldo.h lfunc.h lstring.h lgc.h \
 lundump.h
lutf8lib.o: lutf8lib.c lprefix.h lua.h luaconf.h lauxlib.h lualib.h
lvm.o: lvm.c lprefix.h lua.h luaconf.h ldebug.h lstate.h lobject.h \
 llimits.h ltm.h lzio.h lmem.h ldo.h lfunc.h lgc.h lopcodes.h lstring.h \
 ltable.h lvm.h
	$(CC) $(CFLAGS) -fomit-frame-pointer -c -o $@ lvm.c
lzio.o: lzio.c lprefix.h lua.h luaconf.h llimits.h lmem.h lstate.h \
 lobject.h ltm.h lzio.h
onelua.o: onelua.c lprefix.h luaconf.h lzio.c lua.h llimits.h lmem.h \
 lstate.h lobject.h ltm.h lzio.h lctype.c lctype.h lopcodes.c lopcodes.h \
 lmem.c ldebug.h ldo.h lgc.h lundump.c lfunc.h lstring.h lundump.h \
 ldump.c lstate.c lapi.h llex.h ltable.h lgc.c llex.c lparser.h lcode.c \
 lcode.h lvm.h lparser.c ldebug.c lfunc.c lobject.c ltm.c lstring.c \
 ltable.c ldo.c lvm.c lauxlib.h lapi.c \
 lauxlib.c lbaselib.c lualib.h lcorolib.c ldblib.c liolib.c \
 lmathlib.c loadlib.c loslib.c lstrlib.c ltablib.c lutf8lib.c linit.c \
 lua.c
	$(CC) $(CFLAGS) -c -o $@ onelua.c

# Compiler modules
llex.o:
	$(CC) $(CFLAGS) $(CMCFLAGS) -c -o $@ llex.c
lparser.o:
	$(CC) $(CFLAGS) $(CMCFLAGS) -c -o $@ lparser.c
lcode.o:
	$(CC) $(CFLAGS) $(CMCFLAGS) -c -o $@ lcode.c

# (end of Makefile)
