-- ================================================================
-- These tests are mostly adapted from Lua 5.4 tests for TBC variables
-- These functions test bytecode generation, and also provide
-- helper routines that we use later on in other test cases

-- testing opcodes
function check (f, ...)
  if not T then
    return true
  end
  local arg = {...}
  local c = T.listcode(f)
  for i=1, #arg do
    assert(string.find(c[i], arg[i]))
  end
  assert(c[#arg+2] == nil)
end

-- Test defer statement
do
    local y = 0
    local function x()
        defer y = y + 1 end
        defer y = y + 1 end
    end
    check(x, 'DEFER', 'CLOSURE', 'DEFER', 'CLOSURE', 'RETURN')
    x()
    assert(y == 2)
    print 'Test 1 OK'
end

-- Test defer statement
do
    local y = 0
    local function x()
        defer y = y + 1 end
        error('raise error')
        defer y = y + 2 end -- will not be called
    end
    pcall(x)
    assert(y == 1)
    print 'Test 2 OK'
end

-- Test defer statement
do
    local y = 0
    local function x()
        defer y = y + 1 end
        defer y = y + 2; error('err') end
        defer y = y + 3 end
    end
    pcall(x)
    assert(y == 6)
    print 'Test 3 OK'
end

-- Test defer statement in tailcalls
do
    local y = 0
    local function x (n)
        defer y = y + 1 end
        if n > 0 then return x(n - 1) end
    end
    pcall(x, 3)
    assert(y == 4)
    print 'Test 4 OK'
end

-- Simulate a test of resource closure with defer
do
    local y = 0
    local z = { count = 0 }
    z.__index = z;
    function z:new()
        local object = {}
        setmetatable(object, z)
        return object
    end
    function z:open(arg)
        if (arg) then
            z.count = z.count + 1
            return
        end
        y = 1
        error('error opening')
    end
    function z.close()
        z.count = z.count - 1
    end
    local function x(arg)
        local f = z:new()
        f:open(arg)
        assert(z.count == 1)
        defer f:close() end
    end
    x('filename')
    assert(y == 0)
    assert(z.count == 0)
    pcall(x, false)
    assert(z.count == 0)
    assert(y == 1)
    print 'Test 5 OK'
end

--- Test stack reallocation in defer statement
do
    local function x(a) if a <= 0 then return else x(a-1) end end
    local y = 100
    local function z(...)
        -- recursive call to make stack
        defer x(y) end
        return ...
    end
    do
        local a,b,c = z(1,2,3)
        assert(a == 1 and b == 2 and c == 3)
        a,b,c = z(3,2,1)
        assert(a == 3 and b == 2 and c == 1)
    end
    print 'Test 6 OK'
end

-- Adapted from Lua 5.4
local function stack(n) n = ((n == 0) or stack(n - 1)) end

local function func2close (f, x, y)
    local obj = setmetatable({}, {__close = f})
    if x then
        return x, obj, y
    else
        return obj
    end
end

do
    local function t()
        local a = {}
        do
            local b = false   -- not to be closed
            -- x is <close>
            local x = setmetatable({"x"}, {__close = function (self)
                                                    a[#a + 1] = self[1] end})
            defer getmetatable(x).__close(x) end
            -- y is <close>
            local w, y, z = func2close(function (self, err)
                                    assert(err == nil); a[#a + 1] = "y"
                                end, 10, 20)
            defer getmetatable(y).__close(y) end
            local c = nil  -- not to be closed
            a[#a + 1] = "in"
            assert(w == 10 and z == 20)
        end
        a[#a + 1] = "out"
        assert(a[1] == "in" and a[2] == "y" and a[3] == "x" and a[4] == "out")
    end
    t()
    print 'Test 7 OK'
end

do
    local function t()
    local X = false

    local x, closescope = func2close(function () stack(10); X = true end, 100)
    assert(x == 100);  x = 101;   -- 'x' is not read-only

    -- closing functions do not corrupt returning values
    local function foo (x)
        local _ = closescope
        defer getmetatable(_).__close(_) end
        return x, X, 23
    end

    local a, b, c = foo(1.5)
    assert(a == 1.5 and b == false and c == 23 and X == true)

    X = false
    foo = function (x)
        local _ = closescope
        defer getmetatable(_).__close(_) end
        local y = 15
        return y
    end

    assert(foo() == 15 and X == true)

    X = false
    foo = function ()
        local x = closescope
        defer getmetatable(x).__close(x) end
        return x
    end

    assert(foo() == closescope and X == true)
    end
    t()
    print 'Test 8 OK'
end

do
    local function t()
        -- calls cannot be tail in the scope of to-be-closed variables
        local X, Y
        local function foo ()
            local _ = func2close(function () Y = 10 end)
            defer getmetatable(_).__close(_) end
            assert(X == true and Y == nil)    -- 'X' not closed yet
            return 1,2,3
        end

        local function bar ()
            local _ = func2close(function () X = false end)
            defer getmetatable(_).__close(_) end
            X = true
            do
                return foo()    -- not a tail call!
            end
        end

        local a, b, c, d = bar()
        assert(a == 1 and b == 2 and c == 3 and X == false and Y == 10 and d == nil)
        return foo, bar
    end
    local f,b = t()
    print 'Test 9 OK'
end

do
    local function t()
        -- an error in a wrapped coroutine closes variables
        local x = false
        local y = false
        local co = coroutine.wrap(function ()
            local xv = func2close(function () x = true end)
            defer getmetatable(xv).__close(xv) end
            do
                local yv = func2close(function () y = true end)
                defer getmetatable(yv).__close(yv) end
                coroutine.yield(100)   -- yield doesn't close variable
            end
            coroutine.yield(200)   -- yield doesn't close variable
            error(23)              -- error does
        end)

        local b = co()
        assert(b == 100 and not x and not y)
        b = co()
        assert(b == 200 and not x and y)
        local a, b = pcall(co)
        assert(not a and b == 23 and x and y)
    end
    t()
    print 'Test 10 OK'
end

-- a suspended coroutine should not close its variables when collected
do
    function t()
        local co
        co = coroutine.wrap(function()
            -- should not run
            local x = func2close(function () os.exit(false) end)
            defer getmetatable(x).__close(x) end
            co = nil
            coroutine.yield()
        end)
        co()                 -- start coroutine
        assert(co == nil)    -- eventually it will be collected
        collectgarbage()
    end
    t()
    print 'Test 11 OK'
end

do
    local function t()
        -- error in a wrapped coroutine raising errors when closing a variable
        local x = 0
        local co = coroutine.wrap(function ()
            local xx = func2close(function () x = x + 1; error("@YYY") end)
            defer getmetatable(xx).__close(xx) end
            local xv = func2close(function () x = x + 1; error("@XXX") end)
            defer getmetatable(xv).__close(xv) end
            coroutine.yield(100)
            error(200)
        end)
        assert(co() == 100); assert(x == 0)
        local st, msg = pcall(co); assert(x == 2)
        assert(not st and string.find(msg, "@YYY"))   -- should get first error raised

        local x = 0
        local y = 0
        co = coroutine.wrap(function ()
            local xx = func2close(function () y = y + 1; error("YYY") end)
            defer getmetatable(xx).__close(xx) end
            local xv = func2close(function () x = x + 1; error("XXX") end)
            defer getmetatable(xv).__close(xv) end
            coroutine.yield(100)
            return 200
        end)
        assert(co() == 100); assert(x == 0)
        local st, msg = pcall(co)
        assert(x == 1 and y == 1)
        -- should get first error raised
        assert(not st and string.find(msg, "%w+%.%w+:%d+: YYY"))
    end
    t()
    print 'Test 12 OK'
end

print 'OK'
