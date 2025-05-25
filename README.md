# LuaGLM

A Lua 5.4.8 runtime providing vector, quaternion, and matrix basic types with an
assortment of GLSL built-in functions.

## Vectors and Quaternions

Vectors are a new basic type and are viewed as **immutable** tables of floats
accessible by keys `1, 2, 3, 4`, `x, y, z, w`, and `r, g, b, a`
(case-insensitive).

```lua
-- Generic constructor
> v = vec(math.pi, math.exp(1), (1 + math.sqrt(5)) / 2)

-- Constructor with explicit length
> v3 = vec3(math.pi, math.exp(1), (1 + math.sqrt(5)) / 2)

-- Accessing vector fields
> v[3] + v.z
3.2360680103302

-- Vector swizzling
> v.xyzx
vec4(3.141593, 2.718282, 1.618034, 3.141593)

-- Test vectors for equality
> v == v3
true

-- Arithmetic operations on vector types
> ((v + v3) * v) - v3
vec3(16.597618, 12.059830, 3.618034)

-- OP_LEN returns the magnitude of a vector. The C API, e.g., lua_len, returns
-- the component count
> #v
4.4583287239075

-- Component count
> v.n
3

-- Iterate over each value
> for k,v in pairs(v3) do print(k,v) end
1       3.1415927410126
2       2.7182817459106
3       1.6180340051651

-- As table keys
> t = { }
> t[v // 1.0] = "Hello, World!"
> t[v // 1.0]
Hello, World!
```

Quaternions are a variant of vector:

```lua
-- Create a quaternion with {w,x,y,z} components
> quat(1, 0, 0, 0)
quat(1.000000, {0.000000, 0.000000, 0.000000})

-- Create a quaternion by rotating an axis by 35 degrees
> quat(radians(35.0), vec(1,0,0))
quat(0.953717, {0.300706, 0.000000, 0.000000})

-- Multiply a direction vector by the quaternion
> quat(radians(35.0), vec(1,0,0)) * norm(vec(1))
vec3(0.577350, 0.141783, 0.804092)

-- Vectors and quaternions have explicit an type string even though they are
-- represented internally by the same LUA_TVECTOR tag
> print(type(vec3()), type(quat()))
vec3 quat
```

When a vector or quaternion value is accessed by another field, some additional
rules exist prior to a `__index` metamethod lookup:

1. If a string key has less-than five characters it is passed through a
   swizzling filter. Returning a vector if all characters are valid fields,
   e.g., `v.zyx == vec3(v.z, v.y, v.x)`.
1. The component count of a vector can be accessed by the `n` field (similar to
   `table.pack`). The length operator returns the magnitude of the vector.

### Vector API

Vector and quaternion values are represented by the `LUA_TVECTOR` tag and are
internally represented using an array of floats (or half-precision floats, see
**Configuration**). On an API level they are effectively tables and accessing
their values can be done using the same C API functions:

- [`lua_next`](https://www.lua.org/manual/5.4/manual.html#lua_next)
- [`lua_geti`](https://www.lua.org/manual/5.4/manual.html#lua_geti),
  [`lua_getfield`](https://www.lua.org/manual/5.4/manual.html#lua_getfield)
- [`lua_rawget`](https://www.lua.org/manual/5.4/manual.html#lua_rawget),
  [`lua_rawgeti`](https://www.lua.org/manual/5.4/manual.html#lua_rawgeti)
- [`lua_rawlen`](https://www.lua.org/manual/5.4/manual.html#lua_rawlen),
  [`lua_len`](https://www.lua.org/manual/5.4/manual.html#lua_len): Returns the
  component count of the vec/quat.

For backwards compatibility the `LUAGLM_COMPAT_5_4` build option can be used to
map `LUA_TVECTOR` and `LUA_TMATRIX` types to `LUA_TTABLE` when interfacing with
the C API. The vector specific API can also be referenced in [lua.h](lua.h).

### Vector Methods

Vector and quaternion values do not maintain an explicit metatable. The Lua
functions `getmetatable` and `debug.setmetatable` and C API functions
`lua_setmetatable` and `lua_getmetatable` can be used to define explicit
metatables for the `LUA_TVECTOR` tag.

For performance, all arithmetic and bitwise operations have internal
implementations that take precedence over any script-defined metamethod. All
other metamethods (e.g., `__concat`, `_call`, etc.) are allowed custom
implementations.

## Matrices

Matrices are another basic type and represent **mutable** collections of
**column**-major vectors that are accessible by integer keys `1, 2, 3, 4`. They
are **collectible** objects and beholden to the garbage collector.

```lua
-- Create a matrix
> m = mat(vec(1.0, 0.0, 0.0), vec(0.0, 0.819152, 0.573576), vec(0.0, -0.573576, 0.819152))

-- Like vectors, matrices have an explicit type string even though they are
-- internally represented by the same LUA_TMATRIX tag
> type(m)
mat3x3

-- tostring for matrix and vector types
> tostring(m)
mat3x3((1.000000, 0.000000, 0.000000), (0.000000, 0.819152, 0.573576), (0.000000, -0.573576, 0.819152))

-- The length operator corresponds to the number of column vectors
> #m
3

-- Access a column component
> m[2]
vec3(0.000000, 0.819152, 0.573576)

-- Iterate over each column
> for k,v in pairs(m) do print(k, v) end
1       vec3(1.000000, 0.000000, 0.000000)
2       vec3(0.000000, 0.819152, 0.573576)
3       vec3(0.000000, -0.573576, 0.819152)

-- Multiply a vector by the given matrix
> m * vec(0,1,0)
vec3(0.000000, 0.819152, 0.573576)
```

### Matrix API

Matrix objects are represented by the `LUA_TMATRIX` type. On an API level they
are effectively tables (arrays) and accessing/modifying their components can be
done using the same C API functions:

- [`lua_next`](https://www.lua.org/manual/5.4/manual.html#lua_next)
- [`lua_gettable`](https://www.lua.org/manual/5.4/manual.html#lua_gettable),
  [`lua_settable`](https://www.lua.org/manual/5.4/manual.html#lua_settable)
- [`lua_geti`](https://www.lua.org/manual/5.4/manual.html#lua_geti),
  [`lua_seti`](https://www.lua.org/manual/5.4/manual.html#lua_seti)
- [`lua_rawget`](https://www.lua.org/manual/5.4/manual.html#lua_rawget),
  [`lua_rawset`](https://www.lua.org/manual/5.4/manual.html#lua_rawset)
- [`lua_rawgeti`](https://www.lua.org/manual/5.4/manual.html#lua_rawgeti),
  [`lua_rawseti`](https://www.lua.org/manual/5.4/manual.html#lua_rawseti).
- [`lua_rawlen`](https://www.lua.org/manual/5.4/manual.html#lua_rawlen),
  [`lua_len`](https://www.lua.org/manual/5.4/manual.html#lua_len)

The matrix specific C API can be referenced in [lua.h](lua.h).

### Matrix Methods

Like vectors and quaternions, matrix objects do not maintain an explicit
metatable. See the `Methods` section for vectors and quaternions.

## Library Functions

The runtime mirrors and extends most built-in functions specified in the
[OpenGL Shading Language](https://registry.khronos.org/OpenGL/specs/gl/GLSLangSpec.4.60.pdf).
Documentation for these functions is located in [GLSL.md](GLSL.md).

## Power Patches

The runtime [imports](http://lua-users.org/wiki/LuaPowerPatches) and extends
many small/useful changes to the Lua parser, API, and runtime, all bound to
preprocessor flags:

### Compound Operators

Add `"+=", "-=", "*=", "/=", "//=", "%=", "<<=", ">>=", "&=", "|=", "^=", and "..="`
to the language. The increment and decrement operators (`++, --`) have not been
implemented due to one of those operators being reserved.

### Extended Literals

Allow binary numerals, underscores, and f/F/.f/.F suffixes in literals.

```lua
-- Separators
x = 1_234_456

-- Floating-point literal suffixes
x = 1f

-- Binary Numerals
verify_bf16(1.0, 0b0_01111111_000_0000)
```

### Float16

Support for
[half-precision](https://en.wikipedia.org/wiki/Half-precision_floating-point_format)
floating-points in `string.pack` and `string.unpack` using `'e'` as the
[format](https://www.lua.org/manual/5.4/manual.html#6.4.2) specifier.

```lua
> string.unpack("e", string.pack("e", math.pi))
3.140625        3
```

### Safe Navigation

An indexing operation that suppresses errors on accesses into undefined values
(similar to the safe-navigation operators in C#, Kotlin, etc.), e.g.,

```lua
-- Indexing
t?.x?.y == nil
t.x?[1] == nil

-- Expression chain short circuiting
knees = head?.shoulders.knees
x,y,z = sigma?.tau.foot:gun("flower_power")

-- Functions
local x,y,z = call.exists?("Hello,", "World!", math.abs(-42))
if call.fix_bug?() then
    error("Impossible")
end

-- Self syntax
local e,n,i,a,c = t.y?:z()
local linc,pdp = t.y:z?()
if t?:x(0xC00010FF, 0xFEEDC0DE) then
    print(0xB105F00D)
end
```

### If-Expressions (Ternary Operator)

The [idiomatic](http://lua-users.org/wiki/TernaryOperator) approach to
approximate ternary operators in Lua is syntax of the form:

```lua
local x = a and b or c
```

However, if `a` evaluates to true while `b` evaluates to false/nil, then the
expression will not behave exactly like a ternary operator. Instead, this patch
supports ternary-like operators using `if COND then EXPR else EXPR` syntax:

```lua
local x = if a then b else c
local x = if a then b elseif c then d else e
print(if a then b else c)
```

### Set Constructors

Syntactic sugar to improve the syntax for specifying sets, i.e.,

```lua
t = { .a, .b }
```

is functionally equivalent to:

```lua
t = { a = true, b = true }
```

### Compile Time Jenkins' Hashes

String literals wrapped in back-ticks are Jenkins' one-at-a-time hashed when
parsed.

```lua
> `Hello, World!`
1395890823
```

For runtime hashing, the `joaat` function is included in the base library:

```lua
-- joaat(input[, ignore_case]): Compute the Jenkins hash of the input string.
-- If 'ignore_case' is true, the byte data is hashed as is. Otherwise, each
-- character is tolower'd prior to hashing.
> joaat("Hello, World!")
1395890823
```

```lua
> joaat("CPed")
2491553369
```

### Lambda Expressions (Short Function Notation)

Syntactic sugar for writing concise anonymous functions of the form `|a, b, ...| expr`.
Where `expr` is any expression equivalent to `function(a, b, ...) return expr end`.
For example,

```lua
> f = |x| x^2 - 1 -- function(x) return x^2 - 1 end

> f(2)
3.0

> f(vec3(1, 2, 3))
vec3(0.000000, 3.000000, 8.000000)

-- 'hexadump' from lua-MessagePack.lua
> hexadump = |s| s:gsub('.', |c| string.format('%02X ', c:byte()))

> hexadump("\221\255\255\255\255Z")
DD FF FF FF FF 5A
```

### Defer

Imports the defer statement from
[Ravi](https://github.com/dibyendumajumdar/ravi/tree/master/patches) into the
runtime. In addition, `func2close` from ltests.h has been imported into the base
library.

```lua
-- LUA_EXT_DEFER:
defer
    numopen = numopen - 1
end

-- LUA_EXT_DEFER_API: closing function. Can also be used to supply a
-- to-be-closed variable to a generic for loop.
local _ <close> = defer(function()
    numopen = numopen - 1
end)
```

### Optimized Iteration

A generic 'for' loop starts by evaluating its explist to produce four values: an
iterator function, a state, an initial value for the control variable, and a
closing (to-be-closed) value. However, the \_\_pairs metamethod does not support
the optional closing value.

This extension introduces optimizations to pairs/ipairs in for-loops by using
the unused to-be-closed slot to cache an index variable (or marker) and is based
on the loop optimization patch described in
[lua-l](http://lua-users.org/lists/lua-l/2022-06/msg00031.html).

### Extended API

Expose `lua_createtable` and API functions common to other custom Lua runtimes.

```lua
-- Creates a new empty table.
-- narr: a hint for how many elements the table will have as a sequence.
-- nrec: a hint for how many other elements the table will have.
t = table.create(narr[, nrec])
table.new = table.create -- Deprecated alias

-- Create a new array filled w/ some value.
-- narr: number of elements to fill (i.e., size of array)
-- value: value to fill the table with
t = table.fill(narr[, value])

-- Restore the table to its initial value (removing its contents) while
-- retaining its internal pointer.
t = table.clear(t)
t = table.wipe(t) -- Deprecated alias

-- Request the removal of unused capacity in the given table (shrink_to_fit).
t = table.compact(t)

-- An efficient (implemented using memcpy) table shallow-copy implementations.
t2 = table.clone(t)

-- Debug: force an explicit rehash of the table.
t = table.rehash(t)

-- Return the type of table being used. Note, this function only measures the
-- size of the "array part" of a Lua table and the "root" node of its
-- "hash part". Once an "array" becomes "mixed", or if a table has all of
-- values nil'd out, the table.type will remain "mixed" or "hash".
label = table.type(t) -- "empty", "array", "hash", or "mixed"

-- Trim characters off the beginning and end of a string.
str = string.trim(input [, chars])
```

### Readonly

Introduce the ability to make a table read-only and prohibit any modifications.

```lua
-- Mark a table as readonly.
--
-- This behavior is 'shallow', i.e., non-frozen tables stored within 't' are
-- still mutable.
--
-- Frozen tables respect the '__newindex' metamethod, however, any attempt to
-- modify the table by that function (e.g., __newindex = rawset) will lead to an
-- error being thrown.
--
-- Tables with 'protected' metatables, i.e., a '__metatable' field, cannot be
-- frozen.
t = table.freeze(t)

-- Return true if the provided table is configured as readonly; false otherwise.
bool = table.isfrozen(t)
```

This extension changes a few C API guarantees:

1. lua\_setmetatable: `@apii{1,0,-}` to `@apii{1,0,v}`
1. lua\_rawset: `@apii{2,0,m}` to `@apii{2,0,v}`
1. lua\_rawsetp: `@apii{1,0,m}` to `@apii{1,0,v}`
1. lua\_rawseti: `@apii{1,0,m}` to `@apii{1,0,v}`

With the error corresponding to a readonly violation (or out-of-memory).

### Usertags (Tagged Userdata)

An alternative to
[luaL\_checkudata](https://www.lua.org/manual/5.4/manual.html#luaL_checkudata)
that associates an integer tag with a userdata **type** (`tname`).

```c
/* Create a tagged user data with zero user values */
#define lua_newusertag(L, s, t) lua_newusertaguv(L, s, t, 0)

/* Create and push on the stack a new full userdata with tag 'tag' */
void *lua_newusertaguv(lua_State *L, size_t size, int tag, int nuvalue);

/*
 * If the value at the given index is a userdata with tag 'tag', return its
 * memory-block address (a pointer). Otherwise, NULL is returned.
 */
void *lua_tousertag(lua_State *L, int idx, int tag);

/*
 * Check whether the value at the given index is a userdata with tag 'tag',
 * returning the userdatas memory-block. Otherwise, an error is thrown.
 */
void *luaL_checkusertag(lua_State *L, int idx, int tag);

/*
 * Associate a label with a usertag. If the tag is registered to another name,
 * an error is thrown.
 *
 * This function exists to weakly enforce (or sanitize) tag conflicts between
 * libraries. The current API does not require a tag to be named before use.
 */
int luaL_nameusertag(lua_State *L, int tag, const char *name);
```

### Readline History

Keep a persistent list of commands that have been run on the Lua interpreter.
Uses the `LUA_HISTORY` environment variable to declare location history.

## Building

The Lua can be compiled as C or as C++. All functions required to integrate
[cglm](https://github.com/recp/cglm) into the runtime are defined in
[lglmcore.h](lglmcore.h) while [lglmaux.h](lglmaux.h) maintains the functions to
integrate cglm into the base Lua libraries.

### Configuration

Defined in [luaconf.h](luaconf.h):

- **LuaGLM Options**
  - **LUAGLM_COMPAT_5_4**: Lua 5.4 C-API compatibility (e.g., map `LUA_T{VECTOR,MATRIX}` to `LUA_TTABLE`).
  - **LUAGLM_HALF_TYPE**: Use `Float16` as the vector storage type.
- **Power Patches**: See Lua Power Patches section.
  - **LUA_EXT_COMPOUND**: Enable 'Compound Operators'.
  - **LUA_EXT_DEFER**: Enable 'Defer'.
  - **LUA_EXT_IFEXPR**: Enable 'If-Expressions'.
  - **LUA_EXT_JOAAT**: Enable 'Compile Time Jenkins' Hashes'.
  - **LUA_EXT_LAMBDA**: Enable 'Lambda Expressions'.
  - **LUA_EXT_LITERAL**: Enable 'Extended Literals'.
  - **LUA_EXT_SAFENAV**: Enable 'Safe Navigation'.
  - **LUA_EXT_SETINIT**: Enable 'Set Constructors'.
  - **LUA_EXT_ITERATION**: Enable 'Optimized Iteration'.
  - **LUA_EXT_API**: Enable 'Extended API'.
  - **LUA_EXT_CHRONO**: Enable nanosecond resolution timers and rdtsc sampling (`os.{usec,nsec,rdtsc}`).
  - **LUA_EXT_HALF**: Enable 'Float16'.
  - **LUA_EXT_READONLY**: Enable 'Readonly'.
  - **LUA_EXT_USERTAG**: Enable 'Usertags'.
  - **LUA_EXT_READLINE_HISTORY**: Enable 'Readline History'.

### Make

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

# Ensure cglm is initialized
└> git submodule update --init

# Default Build (Add UNITY=1 for unity builds)
└> make

# Run Lua
└> ./lua ...

# WSL/Windows: clang.exe
└> make windows
└> ./lua.exe ...

# Emscripten
└> make emscripten

# Note: older node versions may require --experimental-wasm-eh
└> node lua.js ...

# Development: debug build
# Ensure ./testes/libs is compiled for the correct platform
└> make DEBUG=1 ...
```

## Sources & Acknowledgments

1. [grit-lua](https://github.com/grit-engine/grit-lua): Original implementation and inspiration.
1. [cglm](https://github.com/recp/cglm): Vector and Matrix functionality.
1. [OpenGL Shading Language](https://registry.khronos.org/OpenGL/specs/gl/GLSLangSpec.4.60.pdf)

## Developer Notes

1. Improve SIGFPE testing (and more defensive programming in [lglm.c](lglm.c) where possible).
1. Cleanup test scripts/environment and publish.
1. Parser/opcodes/builtins for default libraries.
1. String parsing/formatting optimizations.
1. PyBufferProtocol-esque/memory API.
1. Experiment with table comprehensions (sugar).
1. Replace quicksort with introsort.
1. Sandboxing improvements.
1. Deterministic glsl lib.
1. Dual quaternion and GA helpers.
1. Expand the complex number API.

## License

Lua and cglm are distributed under the terms of the
[MIT license](https://opensource.org/licenses/mit-license.html).
See the Copyright Notice in [lua.h](lua.h).
