# Lua

Tail Calling Interpreter - Work-in-Progress

https://blog.reverberate.org/2021/04/21/musttail-efficient-interpreters.html

https://github.com/llvm/llvm-project/pull/76868

## Building

Uses a modified (GNU make required) version of the makefile
[bundled](https://www.lua.org/download.html) with releases of Lua.

```bash
# Rules: Default rule is "guess"
#   clean - Clean build directory
#   echo - Output build flags
#   guess - Build (guesses platform)
#   help - List platforms
#
# Options:
#   UNITY - Unity/Jumbo build (onelua)
#   DEBUG - Enable internal testing flags (and objects)
#   UBSAN - Compile with and link UBSan
#   ASAN - Compile with and link ASan
#
# Flags:
#   EXTRA_CFLAGS - Additional C flags (e.g., -march=native)
#   EXTRA_LDFLAGS - Additional Linker flags (e.g., -fuse-ld=mold)

# Default Build (Add UNITY=1 for unity builds)
# "cc" is aliased to an upstream compilation of clang
> make

# Run Lua
> ./lua ...

# WSL/Windows: clang.exe
> make windows
> ./lua.exe ...

# Emscripten
> make emscripten

# Note: older node versions may require --experimental-wasm-eh
> node lua.js ...

# Development: debug build
# Ensure ./testes/libs is compiled for the correct platform
> make DEBUG=1 ...

# Run only the portable tests (lua.exe for WSL)
(cd testes && ../lua -e"_port=true" all.lua)
```
