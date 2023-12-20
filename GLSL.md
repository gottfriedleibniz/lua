# Chapter 8. Built-In Functions

Imported [OpenGL Shading
Language](https://registry.khronos.org/OpenGL/specs/gl/GLSLangSpec.4.60.pdf)
functions. All functions are declared in the base library (`_G`).

## Constructors

### vec

```lua
-- Create a zero-initialized vector of specified type and length, e.g., vec3():
v = [i,b]vec[2,3,4]()

-- Create a vector of specified type and length with the given scalar for each
-- component, e.g., vec3(0):
v = [i,b]vec[2,3,4](scalar)

-- Create a vector of specified type and length and value, e.g., vec3(1,2,3):
v = [i,b]vec[2,3,4](scalar, ..., scalar)

-- Generic vector constructor; infers length based on arguments. Rules:
--   1. A primitive value (number, integer, boolean) will be stored at v[X];
--   2. A vector, quaternion, or array (of N dimensions) will have its contents
--      unpacked and stored at v[X], v[X + 1], ..., v[X + N] following x, y, z,
--      w ordering;
v = [i,b]vec(...)
```

#### vec Examples

```lua
> vec(math.pi, math.pi, math.pi)
vec3(3.141593, 3.141593, 3.141593)

> vec3(math.pi)
vec(3.141593, 3.141593, 3.141593)

-- Parse array
> vec(1, {2, 3, 4})
vec4(1.000000, 2.000000, 3.000000, 4.000000)
```

### quat

```lua
-- Return the identity quaternion.
q = quat()

-- Generic quaternion constructor.
q = quat(w --[[ number ]], x --[[ number ]], y --[[ number ]], z --[[ number ]])
q = quat(xyz --[[ vec3 ]], w --[[ number ]])
q = quat(xyzw --[[ vec4 ]])

-- Build a quaternion from an angle (in radians) and axis of rotation.
q = quat(angle --[[ number ]], axis --[[ vec3 ]])

-- Create the shortest arc quaternion that rotates 'source' to coincide with
-- 'target'.
q = quat(source --[[ vec3 ]], target --[[ vec3 ]])

-- Create a quaternion from three unit vectors.
q = quat(x --[[ vec3 ]], y --[[ vec3 ]], z --[[ vec3 ]])

-- Matrix to quaternion.
q = quat(m --[[ mat3x3 ]])
q = quat(m --[[ mat4x4 ]])
```

### mat

```lua
-- All matrix constructors accept an optional preallocated matrix object that is
-- used as the return value.

-- Create a identity matrix of specified dimension (C x R).
m = matC[R]([matC[R] sink])

-- Create a diagonal matrix with the given scalar.
m = matC[R]([matC[R] sink,] scalar --[[ number ]])

-- Create a diagonal matrix with the given vector. Zero filling if dimensions
-- are inconsistent (i.e., N ~= C)
m = matC[R]([matC[R] sink,] diag --[[ vecN ]])

-- Create a rotation matrix from a quaternion.
m = matC[R]([matC[R] sink,] rot --[[ quat ]])

-- Copy/Cast a matrix (sink required).
m = matC[R](matC[R] sink, m --[[ matrix ]])
```

#### mat Examples

```lua
> e = math.exp(1)
> mat3x3(vec3(e, e, e), vec3(math.pi, math.pi, math.pi), vec3(1,1,1))
mat3x3((2.718282, 2.718282, 2.718282), (3.141593, 3.141593, 3.141593), (1.000000, 1.000000, 1.000000))

-- Infer matrix dimensions based on arguments
> mat(vec3(e, e, e), vec3(math.pi, math.pi, math.pi), vec3(1,1,1))
mat3x3((2.718282, 2.718282, 2.718282), (3.141593, 3.141593, 3.141593), (1.000000, 1.000000, 1.000000))

> mat4x4(math.pi)
mat4x4((3.141593, 0.000000, 0.000000, 0.000000), (0.000000, 3.141593, 0.000000, 0.000000), (0.000000, 0.000000, 3.141593, 0.000000), (0.000000, 0.000000, 0.000000, 3.141593))

> mat3x3(quat(35.0, vec3(0,0,1)))
mat3x3((0.819152, 0.573576, 0.000000), (-0.573576, 0.819152, 0.000000), (0.000000, 0.000000, 1.000000))

-- Recycling constructor
> m = mat3x3()
> mat3x3(m, quat(35.0, vec3(0,0,1))) == m
true
```

## 8.1. Angle and Trigonometry Functions

### radians

Converts degrees to radians.

```lua
vec|number = radians(vec|number degrees)
```

### degrees

Converts radians to degrees.

```lua
vec|number = degrees(vec|number radians)
```

### sin

The standard trigonometric sine function.

```lua
vec|number = sin(vec|number angle)
```

### cos

The standard trigonometric cosine function.

```lua
vec|number = cos(vec|number angle)
```

### tan

The standard trigonometric tangent.

```lua
vec|number = tan(vec|number angle)
```

### asin

Arc sine. Returns an angle whose sine is x.

```lua
vec|number = asin(vec|number x)
```

### acos

Arc cosine. Returns an angle whose cosine is x.

```lua
vec|number = acos(vec|number x)
```

### atan

Arc tangent. Returns an angle whose tangent is y / x. The signs of x and y are
used to determine what quadrant the angle is in.

```lua
vec = atan(vec y, vec x)
vec = atan(vec y, number x)
number = atan(number y, number x)
vec|number = atan(vec|number y_over_x)
```

### sinh

Returns the hyperbolic sine function.

```lua
vec|number = sinh(vec|number x)
```

### cosh

Returns the hyperbolic cosine function.

```lua
vec|number = cosh(vec|number x)
```

### tanh

Returns the hyperbolic tangent function.

```lua
vec|number = tanh(vec|number x)
```

### asinh

Arc hyperbolic sine; returns the inverse of sinh.

```lua
vec|number = asinh(vec|number x)
```

### acosh

Arc hyperbolic cosine; returns the non-negative inverse of cosh.

```lua
vec|number = acosh(vec|number x)
```

### atanh

Arc hyperbolic tangent; returns the inverse of tanh.

```lua
vec|number = atanh(vec|number x)
```

## 8.2. Exponential Functions

### pow

Returns x raised to the y power, i.e., `x^y`.

```lua
vec = pow(vec x, vec y)
vec|quat|number = pow(vec|quat|number x, number y)
```

### exp

Returns the natural exponentiation of x, i.e., `e^x`.

```lua
vec|quat|number = exp(vec|quat|number x)
```

### log

Returns the natural logarithm of x.

```lua
vec|number = log(vec|number x[, vec|number base])
quat = log(quat x)
```

### exp2

Returns 2 raised to the x power.

```lua
vec|number = exp2(vec|number x)
```

### log2

Returns the base 2 logarithm of x.

```lua
vec|number = log2(vec|number x)
```

### sqrt

Returns sqrt(x).

```lua
vec|quat|number = sqrt(vec|quat|number x)
```

### inversesqrt

Returns `1 / sqrt(x)`.

```lua
vec|quat|number = inversesqrt(vec|quat|number x)
```

## 8.3. Common Functions

### abs

Returns x if x ≥ 0; otherwise it returns -x.

```lua
vec|number = abs(vec|number x)
```

### sign

Returns 1.0 if x > 0, 0.0 if x = 0, or -1.0 if x < 0.

```lua
vec|number = sign(vec|number x)
```

### floor

Returns a value equal to the nearest integer that is less than or equal to x.

```lua
vec|number = floor(vec|number x)
```

### trunc

Returns a value equal to the nearest integer to x whose absolute value is not
larger than the absolute value of x.

```lua
vec|number = trunc(vec|number x)
```

### round

Returns a value equal to the nearest integer to x.

```lua
vec|number = round(vec|number x)
```

### roundEven

Returns a value equal to the nearest integer to x. A fractional part of 0.5 will
round toward the nearest even integer.

```lua
vec|number = roundEven(vec|number x)
```

### ceil

Returns a value equal to the nearest integer that is greater than or equal to x.

```lua
vec|number = ceil(vec|number x)
```

### fract

Returns x - floor(x).

```lua
vec|number = fract(vec|number x)
```

### mod

Modulus. Returns x - y \* floor(x / y).

```lua
vec = mod(vec x, vec y)
vec = mod(vec x, number y)
number = mod(number x, number y)
```

### modf

Returns the integral and fractional parts of x. The second result is always a
float (return values consistent with `math.modf`).

```lua
vec,vec = modf(vec x)
integer,number = modf(integer x)
integer,number = modf(number x)
```

### min

Returns y if y < x; otherwise it returns x.

```lua
vec = min(vec x, vec y)
vec = min(vec x, number y)
number = min(number x, number y)
integer = min(integer x, integer y)
```

### max

Returns y if x < y; otherwise it returns x.

```lua
vec = max(vec x, vec y)
vec = max(vec x, number y)
number = max(number x, number y)
integer = max(integer x, integer y)
```

### clamp

Returns min(max(x, minVal), maxVal).

```lua

vec = clamp(vec x, vec minVal, vec maxVal)
vec = clamp(vec x, number minVal, number maxVal)
number = clamp(number x, number minVal, number maxVal)
integer = clamp(integer x, integer minVal, integer maxVal)

-- Saturate: clamp(X, 0, 1)
vec|number = clamp(vec|number x)
```

### mix

Returns the linear blend of x and y, i.e., `x * (1 - a) + y * a`.

```lua
vec = mix(vec x, vec y, vec|number a)
number = mix(number x, number y, number a)

-- If a is true, returns y. Otherwise x is returned.
T = mix(T x, T y, bool a)
```

### step

Returns 0.0 if x < edge; otherwise it returns 1.0.

```lua
vec|number = step(vec|number edge, vec|number x)
vec|number = step(number edge, vec|number x)
```

### smoothstep

Returns 0.0 if `x <= edge0` and 1.0 if `x >= edge1`, and performs smooth Hermite
interpolation between 0 and 1 when `edge0 < x < edge1`.

```lua
vec|number = smoothstep(vec|number edge0, vec|number edge1, vec|number x)
vec|number = smoothstep(number edge0, number edge1, vec|number x)
```

### isnan

Returns true if x holds a NaN. Returns false otherwise.

```lua
vec|bool = isnan(vec|number x)
```

### isinf

Returns true if x holds a positive infinity or negative infinity.

```lua
vec|bool = isinf(vec|number x)
```

### floatBitsToInt
### floatBitsToUint
### intBitsToFloat
### uintBitsToFloat

Unimplemented. See [Format Strings for Pack and Unpack](https://www.lua.org/manual/5.4/manual.html#6.4.2).

### fma

Computes and returns a \* b + c

```lua
vec|number = fma(vec|number a, vec|number b, vec|number c)
```

### frexp

Splits x into a floating-point significand in the range `[0.5, 1.0]`, and an
integral exponent of two.

```lua
vec|number,vec|number = frexp(vec|number x)
```

### ldexp

Builds a floating-point number from x and the corresponding integral exponent of
two in exp, returning: `significand * 2^exponent`

```lua
vec|number = frexp(vec|number x, vec|number exp)
```

## 8.4. Floating-Point Pack and Unpack Functions

Unimplemented. See [Format Strings for Pack and Unpack](https://www.lua.org/manual/5.4/manual.html#6.4.2).

## 8.5. Geometric Functions

### length

Returns the length of vector x.

```lua
vec = length(vec)
_ = length(_) -- Generic length operation
```

### distance

Returns the distance between p0 and p1, i.e., `length(p0 - p1)`.

```lua
number = distance(vec|number p0, vec|number p1)
```

### dot

Returns the dot product of x and y.

```lua
vec|number = dot(vec|number p0, vec|number p1)
```

### cross

Returns the cross product of x and y

```lua
vec2 = cross(vec2 x, vec2 y)
vec3 = cross(vec3 x, vec3 y)
vec3 = cross(vec3 x, quat y)
quat = cross(quat x, quat y)
quat = cross(quat x, vec3 y)
```

### normalize

Returns a vector in the same direction as x but with a length of 1.

```lua
vec|quat|number = normalize(vec|quat|number x)
```

### faceforward

If dot(Nref, I) < 0 return N, otherwise return -N.

```lua
vec|number = faceforward(vec N, vec I, vec nRef)
```

### reflect

For the incident vector I and surface orientation N, returns the reflection
direction: `I - 2 * dot(N, I) * N`.

```lua
vec|number = reflect(vec|number I, vec|number N)
```

### refract

For the incident vector I and surface normal N, and the ratio of indices of
refraction eta, return the refraction vector.

```lua
vec|number = refract(vec|number I, vec|number N, number eta)
```

## 8.6. Matrix Functions

### matrixCompMult

Multiply matrix x by matrix y component-wise, i.e., `result[i][j]` is the scalar
product of `x[i][j]` and `y[i][j]`.

```lua
mat = transpose(mat x, mat y[, mat recycle])
```

### outerProduct

Treats the first parameter c as a column vector and the second parameter r as a
row vector and does a linear algebraic matrix multiply `c * r`.

```lua
mat = transpose(vec c, vec r[, mat recycle])
```

### transpose

Returns a matrix that is the transpose of m.

```lua
mat = transpose(mat m[, mat recycle])
```

### determinant

Returns the determinant of m.

```lua
number = determinant(mat m)
```

### inverse

Returns a value that is the inverse of x.

```lua
mat = inverse(mat x[, mat recycle])
quat|vec|number = inverse(quat|vec|number x)
```

## 8.7. Vector Relational Functions

### lessThan

Returns the component-wise compare of x < y.

```lua
vec = lessThan(vec x, vec y)
bool = lessThan(number x, number y)
bool = lessThan(integer x, integer y)
```

### lessThanEqual

Returns the component-wise compare of x <= y.

```lua
vec = lessThanEqual(vec x, vec y)
bool = lessThanEqual(number x, number y)
bool = lessThanEqual(integer x, integer y)
```

### greaterThan

Returns the component-wise compare of x > y.

```lua
vec = greaterThan(vec x, vec y)
bool = greaterThan(number x, number y)
bool = greaterThan(integer x, integer y)
```

### greaterThanEqual

Returns the component-wise compare of x >= y.

```lua
vec = greaterThanEqual(vec x, vec y)
bool = greaterThanEqual(number x, number y)
bool = greaterThanEqual(integer x, integer y)
```

### equal

Returns the component-wise compare of x == y.

```lua
vec = equal(mat x, mat y)
vec = equal(vec x, vec|number y)
bool = equal(number x, number y)
bool = equal(integer x, integer y)
```

### notEqual

Returns the component-wise compare of x != y.

```lua
vec = notEqual(mat x, mat y)
vec = notEqual(vec x, vec y)
bool = notEqual(number x, number y)
bool = notEqual(integer x, integer y)
```

### any

Returns true if any component of x is true (non-zero).

```lua
bool = any(vec|number|bool x)
```

### all

Returns true only if all components of x are true (non-zero).

```lua
bool = all(vec|number|bool x)
```

### not\_

Returns the component-wise logical complement of x.

```lua
vec|bool|bool = not_(vec|number|bool x)
```

## 8.8. Integer Functions

### uaddCarry

Unimplemented

### usubBorrow

Unimplemented

### imulExtended/umulExtended

Unimplemented

### bitfieldExtract

Extracts bits `[offset, offset + bits - 1]` from value, returning them in the
least significant bits of the result.

```lua
integer = bitfieldExtract(integer value, integer offset, integer bits)
```

### bitfieldInsert

Inserts the bits least significant bits of insert into base.

```lua
integer = bitfieldInsert(integer base, integer insert, integer offset, integer bits)
```

### bitfieldReverse

Reverses the bits of value.

```lua
integer = bitfieldReverse(integer value)
```

### bitCount

Returns the number of one bits in the binary representation of value.

```lua
integer = bitCount(integer value)
```

### findLSB

Returns the bit number of the least significant one bit in the binary
representation of value. If value is zero, -1 will be returned. Unimplemented
for vector types.

```lua
integer = findLSB(integer value)
```

### findMSB

Returns the bit number of the most significant bit in the binary representation
of value. Unimplemented for vector types.

```lua
integer = findMSB(integer value)
```

## 8.X. Extensions

### lerp

Alias to mix

### inv

Alias to inverse

### norm

Alias to normalize

### approx

Returns the component-wise comparison of `|x - y| <= epsilon`.

```lua
vec = approx(vec x, vec y[, number epsilon])
vec = approx(mat x, mat y[, number epsilon])
bool = approx(number x, number y[, number epsilon])
bool = approx(integer x, integer y[, integer epsilon])
```

### axis

Return the axis of rotation.

```lua
vec3 = axis(quat q)
```

### angle

Return the angle of a quaternion, matrix, or between two vectors.

```lua
number = angle(quat r)
number = angle(vec2 x, vec2 y)
number = angle(vec3 x, vec3 y)
```

### quat\_for

Create a quaternion from a forward (direction) and up vector (quatLookAt).

```lua
quat = quat_for(vec3 dir, vec3 up)
```

### slerp

Returns the spherical blend of x and y.

```lua
quat = slerp(quat x, quat y, number t)
vec3 = slerp(vec3 x, vec3 y, number t)
```

### translate

Translate a transformation matrix.

```lua
mat4 = translate(mat4 m, vec3 translate[, mat4 recycle])
mat3 = translate(mat3 m, vec2 translate[, mat3 recycle])
```

### scale

Scale a transformation matrix.

```lua
mat4 = scale(mat4 m, vec3 scale[, mat4 recycle])
mat3 = scale(mat3 m, vec2 scale[, mat3 recycle])
```

### rotate

Rotate a transformation matrix.

```lua
quat = rotate(quat q, number angle, vec3 axis)
mat4 = rotate(mat4 m, number angle, vec3 axis[, mat4 recycle])
mat3 = rotate(mat3 m, number angle[, mat3 recycle])
vec2 = rotate(vec2 m, number angle)
```

### frustum

Create a perspective projection matrix. Depends upon compilation flags
`CGLM_FORCE_DEPTH_ZERO_TO_ONE` and `CGLM_FORCE_LEFT_HANDED`.

```lua
mat4 = frustum(number left, number right, number bottom, number top, number nearZ, number farZ[, mat4 recycle])
```

### ortho

Create an orthographic projection matrix. Depends upon compilation flags
`CGLM_FORCE_DEPTH_ZERO_TO_ONE` and `CGLM_FORCE_LEFT_HANDED`.

```lua
mat4 = ortho(number left, number right, number bottom, number top, number nearZ, number farZ[, mat4 recycle])
```

### perspective

Create a perspective projection matrix. Depends upon compilation flags
`CGLM_FORCE_DEPTH_ZERO_TO_ONE` and `CGLM_FORCE_LEFT_HANDED`.

```lua
mat4 = perspective(number fovy, number aspect, number nearZ, number farZ[, mat4 recycle])
```

### project

Map object coordinates to window coordinates.

```lua
vec3 = project(vec3 pos, mat4 modelView, vec4 viewport)
vec3 = project(vec3 x, vec3 y) -- Project X onto Y
```

### unproject

Map the specified viewport coordinates into the specified space:

1. Projection to View.
1. View to World.
1. Model to Object.

```lua
vec3 = unproject(vec3 pos, mat4 m, vec4 viewport)
```

### lookat

Create a view matrix from camera coordinates and direction.

```lua
mat4 = lookat(vec3 eye, vec3 center, vec3 up[, mat4 recycle])
mat4 = lookat(vec3 eye, quat rotation[, mat4 recycle])
```

### billboard

Create a spherical billboard matrix that rotates around the specified object
position.

```lua
mat4 = billboard(vec3 obj, vec3 eye, vec3 up, vec3 fwd[, mat4 recycle])
```

### compose

Create an affine transformation matrix from translation, rotation, and scaling
components.

```lua
mat4 = compose(vec3 translation, quat rotation, vec3 scale[, mat4 recycle])
```

### decompose

Decompose an affine transformation matrix into translation, rotation, and
scaling components.

```lua
vec3,quat,vec3 = decompose(mat4 m)
```

### from\_euler

Build a quaternion from Euler angles. The axis-ordering is specified by
character strings: `'xyz', 'xzy', 'yxz', 'yzx', 'zxy', 'zyx'`.

```lua
quat = from_euler(vec3 euler, string order[, bool extrinsic])
```

### to\_euler

Extract the Euler angles from a transformation matrix or quaternion using the
specified rotation sequence.

```lua
vec3 = to_euler(quat q, string order[, bool extrinsic])
vec3 = to_euler(mat4 m, string order[, bool extrinsic])
vec3 = to_euler(mat3 m, string order[, bool extrinsic])
```

## 8.Z. Constants

### epsilon

EPSILON value for vector types
