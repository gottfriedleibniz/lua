# change this variable to point to the directory with Lua headers
# of the version being tested
LUA_DIR= ../../

# compilation should generate Dynamic-Link Libraries
CFLAGS= -Wall -std=gnu99 -O2 -I$(LUA_DIR) -shared
LDFLAGS=

# libraries used by the tests
TESTS= lib1 lib11 lib2 lib21 lib2-v2

# Platform detection
UNAME?= $(shell uname -s 2>/dev/null || echo posix)
UNAME:= $(patsubst CYGWIN%,CYGWIN,$(UNAME))
UNAME:= $(patsubst MINGW%,MINGW,$(UNAME))
UNAME:= $(patsubst MSYS%,MSYS,$(UNAME))
UNAME:= $(shell echo $(UNAME) | tr '[:upper:]' '[:lower:]')

# Targets (match targets in root makefile)
build:
	@echo Guessing $(UNAME)
	@$(MAKE) $(UNAME)

clean:
	rm -f $(TESTS:=.so) $(TESTS:=.dll) $(TESTS:=.lib) $(TESTS:=.exp)

.PHONY: build clean bsd freebsd netbsd openbsd linux posix sunos msys cygwin mingw windows emscripten

bsd freebsd netbsd openbsd linux posix: CFLAGS+= -fPIC
bsd freebsd netbsd openbsd linux posix: $(TESTS:=.so)

msys cygwin: SYSCFLAGS= -D_WIN32 -DLUA_USE_POSIX -DLUA_BUILD_AS_DLL
msys cygwin: LDFLAGS+= -L. -L$(LUA_DIR) -llua54
msys cygwin: $(TESTS:=.dll)

mingw: SYSCFLAGS= -DLUA_BUILD_AS_DLL
mingw: LDFLAGS+= -L. -L$(LUA_DIR) -llua54
mingw: $(TESTS:=.dll)

windows: CC= clang.exe
windows: CFLAGS+= -DLUA_BUILD_AS_DLL
windows: LDFLAGS+= -L. -L$(LUA_DIR) -llua54
windows: $(TESTS:=.dll)

emscripten: CC= emcc
emscripten: LDFLAGS+= -sSIDE_MODULE=2
emscripten: $(TESTS:=.so)

%.so:
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)
%.dll:
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

lib1.so lib1.dll: lib1.c
lib2.so lib2.dll: lib2.c
lib11.so lib11.dll: lib11.c
lib21.so lib21.dll: lib21.c
lib2-v2.so lib2-v2.dll: lib22.c

lib11.dll: LDFLAGS+= -llib1
lib21.dll: LDFLAGS+= -llib2
