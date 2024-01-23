vmcase(OP_MOVE) {
  vmbegin(OP_MOVE)
  StkId ra = RA(i);
  setobjs2s(L, ra, RB(i));
  vmbreak;
  vmend(OP_MOVE)
}
vmcase(OP_LOADI) {
  vmbegin(OP_LOADI)
  StkId ra = RA(i);
  lua_Integer b = GETARG_sBx(i);
  setivalue(s2v(ra), b);
  vmbreak;
  vmend(OP_LOADI)
}
vmcase(OP_LOADF) {
  vmbegin(OP_LOADF)
  StkId ra = RA(i);
  int b = GETARG_sBx(i);
  setfltvalue(s2v(ra), cast_num(b));
  vmbreak;
  vmend(OP_LOADF)
}
vmcase(OP_LOADK) {
  vmbegin(OP_LOADK)
  StkId ra = RA(i);
  TValue *rb = k + GETARG_Bx(i);
  setobj2s(L, ra, rb);
  vmbreak;
  vmend(OP_LOADK)
}
vmcase(OP_LOADKX) {
  vmbegin(OP_LOADKX)
  StkId ra = RA(i);
  TValue *rb;
  rb = k + GETARG_Ax(*pc); pc++;
  setobj2s(L, ra, rb);
  vmbreak;
  vmend(OP_LOADKX)
}
vmcase(OP_LOADFALSE) {
  vmbegin(OP_LOADFALSE)
  StkId ra = RA(i);
  setbfvalue(s2v(ra));
  vmbreak;
  vmend(OP_LOADFALSE)
}
vmcase(OP_LFALSESKIP) {
  vmbegin(OP_LFALSESKIP)
  StkId ra = RA(i);
  setbfvalue(s2v(ra));
  pc++;  /* skip next instruction */
  vmbreak;
  vmend(OP_LFALSESKIP)
}
vmcase(OP_LOADTRUE) {
  vmbegin(OP_LOADTRUE)
  StkId ra = RA(i);
  setbtvalue(s2v(ra));
  vmbreak;
  vmend(OP_LOADTRUE)
}
vmcase(OP_LOADNIL) {
  vmbegin(OP_LOADNIL)
  StkId ra = RA(i);
  int b = GETARG_B(i);
  do {
    setnilvalue(s2v(ra++));
  } while (b--);
  vmbreak;
  vmend(OP_LOADNIL)
}
vmcase(OP_GETUPVAL) {
  vmbegin(OP_GETUPVAL)
  StkId ra = RA(i);
  int b = GETARG_B(i);
  setobj2s(L, ra, cl->upvals[b]->v.p);
  vmbreak;
  vmend(OP_GETUPVAL)
}
vmcase(OP_SETUPVAL) {
  vmbegin(OP_SETUPVAL)
  StkId ra = RA(i);
  UpVal *uv = cl->upvals[GETARG_B(i)];
  setobj(L, uv->v.p, s2v(ra));
  luaC_barrier(L, uv, s2v(ra));
  vmbreak;
  vmend(OP_SETUPVAL)
}
vmcase(OP_GETTABUP) {
  vmbegin(OP_GETTABUP)
  StkId ra = RA(i);
  const TValue *slot;
  TValue *upval = cl->upvals[GETARG_B(i)]->v.p;
  TValue *rc = KC(i);
  TString *key = tsvalue(rc);  /* key must be a short string */
  if (luaV_fastget(L, upval, key, slot, luaH_getshortstr)) {
    setobj2s(L, ra, slot);
  }
  else
    Protect(luaV_finishget(L, upval, rc, ra, slot));
  vmbreak;
  vmend(OP_GETTABUP)
}
vmcase(OP_GETTABLE) {
  vmbegin(OP_GETTABLE)
  StkId ra = RA(i);
  const TValue *slot;
  TValue *rb = vRB(i);
  TValue *rc = vRC(i);
  lua_Unsigned n;
  if (ttisinteger(rc)  /* fast track for integers? */
      ? (cast_void(n = ivalue(rc)), luaV_fastgeti(L, rb, n, slot))
      : luaV_fastget(L, rb, rc, slot, luaH_get)) {
    setobj2s(L, ra, slot);
  }
  else
    Protect(luaV_finishget(L, rb, rc, ra, slot));
  vmbreak;
  vmend(OP_GETTABLE)
}
vmcase(OP_GETI) {
  vmbegin(OP_GETI)
  StkId ra = RA(i);
  const TValue *slot;
  TValue *rb = vRB(i);
  int c = GETARG_C(i);
  if (luaV_fastgeti(L, rb, c, slot)) {
    setobj2s(L, ra, slot);
  }
  else {
    TValue key;
    setivalue(&key, c);
    Protect(luaV_finishget(L, rb, &key, ra, slot));
  }
  vmbreak;
  vmend(OP_GETI)
}
vmcase(OP_GETFIELD) {
  vmbegin(OP_GETFIELD)
  StkId ra = RA(i);
  const TValue *slot;
  TValue *rb = vRB(i);
  TValue *rc = KC(i);
  TString *key = tsvalue(rc);  /* key must be a short string */
  if (luaV_fastget(L, rb, key, slot, luaH_getshortstr)) {
    setobj2s(L, ra, slot);
  }
  else
    Protect(luaV_finishget(L, rb, rc, ra, slot));
  vmbreak;
  vmend(OP_GETFIELD)
}
vmcase(OP_SETTABUP) {
  vmbegin(OP_SETTABUP)
  const TValue *slot;
  TValue *upval = cl->upvals[GETARG_A(i)]->v.p;
  TValue *rb = KB(i);
  TValue *rc = RKC(i);
  TString *key = tsvalue(rb);  /* key must be a short string */
  if (luaV_fastget(L, upval, key, slot, luaH_getshortstr)) {
    luaV_finishfastset(L, upval, slot, rc);
  }
  else
    Protect(luaV_finishset(L, upval, rb, rc, slot));
  vmbreak;
  vmend(OP_SETTABUP)
}
vmcase(OP_SETTABLE) {
  vmbegin(OP_SETTABLE)
  StkId ra = RA(i);
  const TValue *slot;
  TValue *rb = vRB(i);  /* key (table is in 'ra') */
  TValue *rc = RKC(i);  /* value */
  lua_Unsigned n;
  if (ttisinteger(rb)  /* fast track for integers? */
      ? (cast_void(n = ivalue(rb)), luaV_fastgeti(L, s2v(ra), n, slot))
      : luaV_fastget(L, s2v(ra), rb, slot, luaH_get)) {
    luaV_finishfastset(L, s2v(ra), slot, rc);
  }
  else
    Protect(luaV_finishset(L, s2v(ra), rb, rc, slot));
  vmbreak;
  vmend(OP_SETTABLE)
}
vmcase(OP_SETI) {
  vmbegin(OP_SETI)
  StkId ra = RA(i);
  const TValue *slot;
  int c = GETARG_B(i);
  TValue *rc = RKC(i);
  if (luaV_fastgeti(L, s2v(ra), c, slot)) {
    luaV_finishfastset(L, s2v(ra), slot, rc);
  }
  else {
    TValue key;
    setivalue(&key, c);
    Protect(luaV_finishset(L, s2v(ra), &key, rc, slot));
  }
  vmbreak;
  vmend(OP_SETI)
}
vmcase(OP_SETFIELD) {
  vmbegin(OP_SETFIELD)
  StkId ra = RA(i);
  const TValue *slot;
  TValue *rb = KB(i);
  TValue *rc = RKC(i);
  TString *key = tsvalue(rb);  /* key must be a short string */
  if (luaV_fastget(L, s2v(ra), key, slot, luaH_getshortstr)) {
    luaV_finishfastset(L, s2v(ra), slot, rc);
  }
  else
    Protect(luaV_finishset(L, s2v(ra), rb, rc, slot));
  vmbreak;
  vmend(OP_SETFIELD)
}
vmcase(OP_NEWTABLE) {
  vmbegin(OP_NEWTABLE)
  StkId ra = RA(i);
  int b = GETARG_B(i);  /* log2(hash size) + 1 */
  int c = GETARG_C(i);  /* array size */
  Table *t;
  if (b > 0)
    b = 1 << (b - 1);  /* size is 2^(b - 1) */
  lua_assert((!TESTARG_k(i)) == (GETARG_Ax(*pc) == 0));
  if (TESTARG_k(i))  /* non-zero extra argument? */
    c += GETARG_Ax(*pc) * (MAXARG_C + 1);  /* add it to size */
  pc++;  /* skip extra argument */
  L->top.p = ra + 1;  /* correct top in case of emergency GC */
  t = luaH_new(L);  /* memory allocation */
  sethvalue2s(L, ra, t);
  if (b != 0 || c != 0)
    luaH_resize(L, t, c, b);  /* idem */
  checkGC(L, ra + 1);
  vmbreak;
  vmend(OP_NEWTABLE)
}
vmcase(OP_SELF) {
  vmbegin(OP_SELF)
  StkId ra = RA(i);
  const TValue *slot;
  TValue *rb = vRB(i);
  TValue *rc = RKC(i);
  TString *key = tsvalue(rc);  /* key must be a string */
  setobj2s(L, ra + 1, rb);
  if (luaV_fastget(L, rb, key, slot, luaH_getstr)) {
    setobj2s(L, ra, slot);
  }
  else
    Protect(luaV_finishget(L, rb, rc, ra, slot));
  vmbreak;
  vmend(OP_SELF)
}
vmcase(OP_ADDI) {
  vmbegin(OP_ADDI)
  op_arithI(L, l_addi, luai_numadd);
  vmbreak;
  vmend(OP_ADDI)
}
vmcase(OP_ADDK) {
  vmbegin(OP_ADDK)
  op_arithK(L, l_addi, luai_numadd);
  vmbreak;
  vmend(OP_ADDK)
}
vmcase(OP_SUBK) {
  vmbegin(OP_SUBK)
  op_arithK(L, l_subi, luai_numsub);
  vmbreak;
  vmend(OP_SUBK)
}
vmcase(OP_MULK) {
  vmbegin(OP_MULK)
  op_arithK(L, l_muli, luai_nummul);
  vmbreak;
  vmend(OP_MULK)
}
vmcase(OP_MODK) {
  vmbegin(OP_MODK)
  savestate(L, ci);  /* in case of division by 0 */
  op_arithK(L, luaV_mod, luaV_modf);
  vmbreak;
  vmend(OP_MODK)
}
vmcase(OP_POWK) {
  vmbegin(OP_POWK)
  op_arithfK(L, luai_numpow);
  vmbreak;
  vmend(OP_POWK)
}
vmcase(OP_DIVK) {
  vmbegin(OP_DIVK)
  op_arithfK(L, luai_numdiv);
  vmbreak;
  vmend(OP_DIVK)
}
vmcase(OP_IDIVK) {
  vmbegin(OP_IDIVK)
  savestate(L, ci);  /* in case of division by 0 */
  op_arithK(L, luaV_idiv, luai_numidiv);
  vmbreak;
  vmend(OP_IDIVK)
}
vmcase(OP_BANDK) {
  vmbegin(OP_BANDK)
  op_bitwiseK(L, l_band);
  vmbreak;
  vmend(OP_BANDK)
}
vmcase(OP_BORK) {
  vmbegin(OP_BORK)
  op_bitwiseK(L, l_bor);
  vmbreak;
  vmend(OP_BORK)
}
vmcase(OP_BXORK) {
  vmbegin(OP_BXORK)
  op_bitwiseK(L, l_bxor);
  vmbreak;
  vmend(OP_BXORK)
}
vmcase(OP_SHRI) {
  vmbegin(OP_SHRI)
  StkId ra = RA(i);
  TValue *rb = vRB(i);
  int ic = GETARG_sC(i);
  lua_Integer ib;
  if (tointegerns(rb, &ib)) {
    pc++; setivalue(s2v(ra), luaV_shiftl(ib, -ic));
  }
  vmbreak;
  vmend(OP_SHRI)
}
vmcase(OP_SHLI) {
  vmbegin(OP_SHLI)
  StkId ra = RA(i);
  TValue *rb = vRB(i);
  int ic = GETARG_sC(i);
  lua_Integer ib;
  if (tointegerns(rb, &ib)) {
    pc++; setivalue(s2v(ra), luaV_shiftl(ic, ib));
  }
  vmbreak;
  vmend(OP_SHLI)
}
vmcase(OP_ADD) {
  vmbegin(OP_ADD)
  op_arith(L, l_addi, luai_numadd);
  vmbreak;
  vmend(OP_ADD)
}
vmcase(OP_SUB) {
  vmbegin(OP_SUB)
  op_arith(L, l_subi, luai_numsub);
  vmbreak;
  vmend(OP_SUB)
}
vmcase(OP_MUL) {
  vmbegin(OP_MUL)
  op_arith(L, l_muli, luai_nummul);
  vmbreak;
  vmend(OP_MUL)
}
vmcase(OP_MOD) {
  vmbegin(OP_MOD)
  savestate(L, ci);  /* in case of division by 0 */
  op_arith(L, luaV_mod, luaV_modf);
  vmbreak;
  vmend(OP_MOD)
}
vmcase(OP_POW) {
  vmbegin(OP_POW)
  op_arithf(L, luai_numpow);
  vmbreak;
  vmend(OP_POW)
}
vmcase(OP_DIV) {  /* float division (always with floats) */
  vmbegin(OP_DIV)
  op_arithf(L, luai_numdiv);
  vmbreak;
  vmend(OP_DIV)
}
vmcase(OP_IDIV) {  /* floor division */
  vmbegin(OP_IDIV)
  savestate(L, ci);  /* in case of division by 0 */
  op_arith(L, luaV_idiv, luai_numidiv);
  vmbreak;
  vmend(OP_IDIV)
}
vmcase(OP_BAND) {
  vmbegin(OP_BAND)
  op_bitwise(L, l_band);
  vmbreak;
  vmend(OP_BAND)
}
vmcase(OP_BOR) {
  vmbegin(OP_BOR)
  op_bitwise(L, l_bor);
  vmbreak;
  vmend(OP_BOR)
}
vmcase(OP_BXOR) {
  vmbegin(OP_BXOR)
  op_bitwise(L, l_bxor);
  vmbreak;
  vmend(OP_BXOR)
}
vmcase(OP_SHL) {  /* @LuaExt: fix incorrect ordering */
  vmbegin(OP_SHL)
  op_bitwise(L, luaV_shiftl);
  vmbreak;
  vmend(OP_SHL)
}
vmcase(OP_SHR) {
  vmbegin(OP_SHR)
  op_bitwise(L, luaV_shiftr);
  vmbreak;
  vmend(OP_SHR)
}
vmcase(OP_MMBIN) {
  vmbegin(OP_MMBIN)
  StkId ra = RA(i);
  Instruction pi = *(pc - 2);  /* original arith. expression */
  TValue *rb = vRB(i);
  TMS tm = (TMS)GETARG_C(i);
  StkId result = RA(pi);
  lua_assert(OP_ADD <= GET_OPCODE(pi) && GET_OPCODE(pi) <= OP_SHR);
  Protect(luaT_trybinTM(L, s2v(ra), rb, result, tm));
  vmbreak;
  vmend(OP_MMBIN)
}
vmcase(OP_MMBINI) {
  vmbegin(OP_MMBINI)
  StkId ra = RA(i);
  Instruction pi = *(pc - 2);  /* original arith. expression */
  int imm = GETARG_sB(i);
  TMS tm = (TMS)GETARG_C(i);
  int flip = GETARG_k(i);
  StkId result = RA(pi);
  Protect(luaT_trybiniTM(L, s2v(ra), imm, flip, result, tm));
  vmbreak;
  vmend(OP_MMBINI)
}
vmcase(OP_MMBINK) {
  vmbegin(OP_MMBINK)
  StkId ra = RA(i);
  Instruction pi = *(pc - 2);  /* original arith. expression */
  TValue *imm = KB(i);
  TMS tm = (TMS)GETARG_C(i);
  int flip = GETARG_k(i);
  StkId result = RA(pi);
  Protect(luaT_trybinassocTM(L, s2v(ra), imm, flip, result, tm));
  vmbreak;
  vmend(OP_MMBINK)
}
vmcase(OP_UNM) {
  vmbegin(OP_UNM)
  StkId ra = RA(i);
  TValue *rb = vRB(i);
  if (ttisinteger(rb)) {
    lua_Integer ib = ivalue(rb);
    setivalue(s2v(ra), intop(-, 0, ib));
  }
  else if (ttisfloat(rb)) {  /* @LuaExt: tonumberns */
    lua_Number nb = fltvalue(rb);
    setfltvalue(s2v(ra), luai_numunm(L, nb));
  }
  else
    Protect(luaT_trybinTM(L, rb, rb, ra, TM_UNM));
  vmbreak;
  vmend(OP_UNM)
}
vmcase(OP_BNOT) {
  vmbegin(OP_BNOT)
  StkId ra = RA(i);
  TValue *rb = vRB(i);
  lua_Integer ib;
  if (tointegerns(rb, &ib)) {
    setivalue(s2v(ra), intop(^, ~l_castS2U(0), ib));
  }
  else
    Protect(luaT_trybinTM(L, rb, rb, ra, TM_BNOT));
  vmbreak;
  vmend(OP_BNOT)
}
vmcase(OP_NOT) {
  vmbegin(OP_NOT)
  StkId ra = RA(i);
  TValue *rb = vRB(i);
  if (l_isfalse(rb))
    setbtvalue(s2v(ra));
  else
    setbfvalue(s2v(ra));
  vmbreak;
  vmend(OP_NOT)
}
vmcase(OP_LEN) {
  vmbegin(OP_LEN)
  StkId ra = RA(i);
  Protect(luaV_objlen(L, ra, vRB(i)));
  vmbreak;
  vmend(OP_LEN)
}
vmcase(OP_CONCAT) {
  vmbegin(OP_CONCAT)
  StkId ra = RA(i);
  int n = GETARG_B(i);  /* number of elements to concatenate */
  L->top.p = ra + n;  /* mark the end of concat operands */
  ProtectNT(luaV_concat(L, n));
  checkGC(L, L->top.p); /* 'luaV_concat' ensures correct top */
  vmbreak;
  vmend(OP_CONCAT)
}
vmcase(OP_CLOSE) {
  vmbegin(OP_CLOSE)
  StkId ra = RA(i);
  Protect(luaF_close(L, ra, LUA_OK, 1));
  vmbreak;
  vmend(OP_CLOSE)
}
vmcase(OP_TBC) {
  vmbegin(OP_TBC)
  StkId ra = RA(i);
  /* create new to-be-closed upvalue */
  halfProtect(luaF_newtbcupval(L, ra));
  vmbreak;
  vmend(OP_TBC)
}
vmcase(OP_JMP) {
  vmbegin(OP_JMP)
  dojump(ci, i, 0);
  vmbreak;
  vmend(OP_JMP)
}
vmcase(OP_EQ) {
  vmbegin(OP_EQ)
  StkId ra = RA(i);
  int cond;
  TValue *rb = vRB(i);
  Protect(cond = luaV_equalobj(L, s2v(ra), rb));
  docondjump();
  vmbreak;
  vmend(OP_EQ)
}
vmcase(OP_LT) {
  vmbegin(OP_LT)
  op_order(L, l_lti, LTnum, lessthanothers);
  vmbreak;
  vmend(OP_LT)
}
vmcase(OP_LE) {
  vmbegin(OP_LE)
  op_order(L, l_lei, LEnum, lessequalothers);
  vmbreak;
  vmend(OP_LE)
}
vmcase(OP_EQK) {
  vmbegin(OP_EQK)
  StkId ra = RA(i);
  TValue *rb = KB(i);
  /* basic types do not use '__eq'; we can use raw equality */
  int cond = luaV_rawequalobj(s2v(ra), rb);
  docondjump();
  vmbreak;
  vmend(OP_EQK)
}
vmcase(OP_EQI) {
  vmbegin(OP_EQI)
  StkId ra = RA(i);
  int cond;
  int im = GETARG_sB(i);
  if (ttisinteger(s2v(ra)))
    cond = (ivalue(s2v(ra)) == im);
  else if (ttisfloat(s2v(ra)))
    cond = luai_numeq(fltvalue(s2v(ra)), cast_num(im));
  else
    cond = 0;  /* other types cannot be equal to a number */
  docondjump();
  vmbreak;
  vmend(OP_EQI)
}
vmcase(OP_LTI) {
  vmbegin(OP_LTI)
  op_orderI(L, l_lti, luai_numlt, 0, TM_LT);
  vmbreak;
  vmend(OP_LTI)
}
vmcase(OP_LEI) {
  vmbegin(OP_LEI)
  op_orderI(L, l_lei, luai_numle, 0, TM_LE);
  vmbreak;
  vmend(OP_LEI)
}
vmcase(OP_GTI) {
  vmbegin(OP_GTI)
  op_orderI(L, l_gti, luai_numgt, 1, TM_LT);
  vmbreak;
  vmend(OP_GTI)
}
vmcase(OP_GEI) {
  vmbegin(OP_GEI)
  op_orderI(L, l_gei, luai_numge, 1, TM_LE);
  vmbreak;
  vmend(OP_GEI)
}
vmcase(OP_TEST) {
  vmbegin(OP_TEST)
  StkId ra = RA(i);
  int cond = !l_isfalse(s2v(ra));
  docondjump();
  vmbreak;
  vmend(OP_TEST)
}
vmcase(OP_TESTSET) {
  vmbegin(OP_TESTSET)
  StkId ra = RA(i);
  TValue *rb = vRB(i);
#if 1 /* @LuaExt: compiler output experiment */
  Instruction ni = *(pc++);
  if (l_isfalse(rb) != GETARG_k(i)) {
    pc += GETARG_sJ(ni);
    setobj2s(L, ra, rb);
    updatetrap(ci);
  }
#else
  if (l_isfalse(rb) == GETARG_k(i))
    pc++;
  else {
    setobj2s(L, ra, rb);
    donextjump(ci);
  }
#endif
  vmbreak;
  vmend(OP_TESTSET)
}
vmcase(OP_CALL) {
  vmbegin(OP_CALL)
  StkId ra = RA(i);
  CallInfo *newci;
  int b = GETARG_B(i);
  int nresults = GETARG_C(i) - 1;
  if (b != 0)  /* fixed number of arguments? */
    L->top.p = ra + b;  /* top signals number of arguments */
  /* else previous instruction set top */
  savepc(L);  /* in case of errors */
  if ((newci = luaD_precall(L, ra, nresults)) == NULL)
    updatetrap(ci);  /* C call; nothing else to be done */
  else {  /* Lua call: run function in this same C frame */
    vmframe_start(L, newci);
  }
  vmbreak;
  vmend(OP_CALL)
}
vmcase(OP_TAILCALL) {
  vmbegin(OP_TAILCALL)
  StkId ra = RA(i);
  int b = GETARG_B(i);  /* number of arguments + 1 (function) */
  int n;  /* number of results when calling a C function */
  int nparams1 = GETARG_C(i);
  /* delta is virtual 'func' - real 'func' (vararg functions) */
  int delta = (nparams1) ? ci->u.l.nextraargs + nparams1 : 0;
  if (b != 0)
    L->top.p = ra + b;
  else  /* previous instruction set top */
    b = cast_int(L->top.p - ra);
  savepc(ci);  /* several calls here can raise errors */
  if (TESTARG_k(i)) {
    luaF_closeupval(L, base);  /* close upvalues from current call */
    lua_assert(L->tbclist.p < base);  /* no pending tbc variables */
    lua_assert(base == ci->func.p + 1);
  }
  if ((n = luaD_pretailcall(L, ci, ra, b, delta)) < 0) {  /* Lua function? */
    vmframe_start(L, ci);
  }
  else {  /* C function? */
    ci->func.p -= delta;  /* restore 'func' (if vararg) */
    luaD_poscall(L, ci, n);  /* finish caller */
    updatetrap(ci);  /* 'luaD_poscall' can change hooks */
    doreturn();  /* caller returns after the tail call */
  }
  vmend(OP_TAILCALL)
}
vmcase(OP_RETURN) {
  vmbegin(OP_RETURN)
  StkId ra = RA(i);
  int n = GETARG_B(i) - 1;  /* number of results */
  int nparams1 = GETARG_C(i);
  if (n < 0)  /* not fixed? */
    n = cast_int(L->top.p - ra);  /* get what is available */
  savepc(ci);
  if (TESTARG_k(i)) {  /* may there be open upvalues? */
    ci->u2.nres = n;  /* save number of returns */
    if (L->top.p < ci->top.p)
      L->top.p = ci->top.p;
    luaF_close(L, base, CLOSEKTOP, 1);
    updatetrap(ci);
    updatestack(ci);
  }
  if (nparams1)  /* vararg function? */
    ci->func.p -= ci->u.l.nextraargs + nparams1;
  L->top.p = ra + n;  /* set call for 'luaD_poscall' */
  luaD_poscall(L, ci, n);
  updatetrap(ci);  /* 'luaD_poscall' can change hooks */
  doreturn();
  vmend(OP_RETURN)
}
vmcase(OP_RETURN0) {
  vmbegin(OP_RETURN0)
  if (l_unlikely(L->hookmask)) {
    StkId ra = RA(i);
    L->top.p = ra;
    savepc(ci);
    luaD_poscall(L, ci, 0);  /* no hurry... */
    trap = 1;
  }
  else {  /* do the 'poscall' here */
    int nres;
    L->ci = ci->previous;  /* back to caller */
    L->top.p = base - 1;
    for (nres = ci->nresults; l_unlikely(nres > 0); nres--)
      setnilvalue(s2v(L->top.p++));  /* all results are nil */
  }
  doreturn();
  vmend(OP_RETURN0)
}
vmcase(OP_RETURN1) {
  vmbegin(OP_RETURN1)
  if (l_unlikely(L->hookmask)) {
    StkId ra = RA(i);
    L->top.p = ra + 1;
    savepc(ci);
    luaD_poscall(L, ci, 1);  /* no hurry... */
    trap = 1;
  }
  else {  /* do the 'poscall' here */
    int nres = ci->nresults;
    L->ci = ci->previous;  /* back to caller */
    if (nres == 0)
      L->top.p = base - 1;  /* asked for no results */
    else {
      StkId ra = RA(i);
      setobjs2s(L, base - 1, ra);  /* at least this result */
      L->top.p = base;
      for (; l_unlikely(nres > 1); nres--)
        setnilvalue(s2v(L->top.p++));  /* complete missing results */
    }
  }
  doreturn();
  vmend(OP_RETURN1)
}
vmcase(OP_FORLOOP) {
  vmbegin(OP_FORLOOP)
  StkId ra = RA(i);
  if (ttisinteger(s2v(ra + 2))) {  /* integer loop? */
    lua_Unsigned count = l_castS2U(ivalue(s2v(ra + 1)));
    if (count > 0) {  /* still more iterations? */
      lua_Integer step = ivalue(s2v(ra + 2));
      lua_Integer idx = ivalue(s2v(ra));  /* internal index */
      chgivalue(s2v(ra + 1), count - 1);  /* update counter */
      idx = intop(+, idx, step);  /* add step to index */
      chgivalue(s2v(ra), idx);  /* update internal index */
      setivalue(s2v(ra + 3), idx);  /* and control variable */
      pc -= GETARG_Bx(i);  /* jump back */
    }
  }
  else if (floatforloop(ra))  /* float loop */
    pc -= GETARG_Bx(i);  /* jump back */
  updatetrap(ci);  /* allows a signal to break the loop */
  vmbreak;
  vmend(OP_FORLOOP)
}
vmcase(OP_FORPREP) {
  vmbegin(OP_FORPREP)
  StkId ra = RA(i);
  savestate(L, ci);  /* in case of errors */
  if (forprep(L, ra))
    pc += GETARG_Bx(i) + 1;  /* skip the loop */
  vmbreak;
  vmend(OP_FORPREP)
}
vmcase(OP_TFORLOOP) { OP_TFORLOOP: {
  vmbegin(OP_TFORLOOP)
  StkId ra = RA(i);
  if (!ttisnil(s2v(ra + 4))) {  /* continue loop? */
    setobjs2s(L, ra + 2, ra + 4);  /* save control variable */
    pc -= GETARG_Bx(i);  /* jump back */
  }
  vmbreak;
  vmend(OP_TFORLOOP)
}}
vmcase(OP_TFORCALL) { OP_TFORCALL: {
  vmbegin(OP_TFORCALL)
    StkId ra = RA(i);
  /* 'ra' has the iterator function, 'ra + 1' has the state,
      'ra + 2' has the control variable, and 'ra + 3' has the
      to-be-closed variable. The call will use the stack after
      these values (starting at 'ra + 4')
  */
  /* push function, state, and control variable */
  memcpy(ra + 4, ra, 3 * sizeof(*ra));
  L->top.p = ra + 4 + 3;
  ProtectNT(luaD_call(L, ra + 4, GETARG_C(i)));  /* do the call */
  updatestack(ci);  /* stack may have changed */
  i = *(pc++);  /* go to next instruction */
  lua_assert(GET_OPCODE(i) == OP_TFORLOOP && ra == RA(i));
  vmgoto(OP_TFORLOOP);
  vmend(OP_TFORCALL)
}}
vmcase(OP_TFORPREP) {
  vmbegin(OP_TFORPREP)
  StkId ra = RA(i);
  /* create to-be-closed upvalue (if needed) */
  halfProtect(luaF_newtbcupval(L, ra + 3));
  pc += GETARG_Bx(i);
  i = *(pc++);  /* go to next instruction */
  lua_assert(GET_OPCODE(i) == OP_TFORCALL && ra == RA(i));
  vmgoto(OP_TFORCALL);
  vmend(OP_TFORPREP)
}
vmcase(OP_SETLIST) {
  vmbegin(OP_SETLIST)
  StkId ra = RA(i);
  int n = GETARG_B(i);
  unsigned int last = GETARG_C(i);
  Table *h = hvalue(s2v(ra));
  if (n == 0)
    n = cast_int(L->top.p - ra) - 1;  /* get up to the top */
  else
    L->top.p = ci->top.p;  /* correct top in case of emergency GC */
  last += n;
  if (TESTARG_k(i)) {
    last += GETARG_Ax(*pc) * (MAXARG_C + 1);
    pc++;
  }
  if (last > luaH_realasize(h))  /* needs more space? */
    luaH_resizearray(L, h, last);  /* preallocate it at once */
  for (; n > 0; n--) {
    TValue *val = s2v(ra + n);
    setobj2t(L, &h->array[last - 1], val);
    last--;
    luaC_barrierback(L, obj2gco(h), val);
  }
  vmbreak;
  vmend(OP_SETLIST)
}
vmcase(OP_CLOSURE) {
  vmbegin(OP_CLOSURE)
  StkId ra = RA(i);
  Proto *p = cl->p->p[GETARG_Bx(i)];
  halfProtect(pushclosure(L, p, cl->upvals, base, ra));
  checkGC(L, ra + 1);
  vmbreak;
  vmend(OP_CLOSURE)
}
vmcase(OP_VARARG) {
  vmbegin(OP_VARARG)
  StkId ra = RA(i);
  int n = GETARG_C(i) - 1;  /* required results */
  Protect(luaT_getvarargs(L, ci, ra, n));
  vmbreak;
  vmend(OP_VARARG)
}
vmcase(OP_VARARGPREP) {
  vmbegin(OP_VARARGPREP)
  ProtectNT(luaT_adjustvarargs(L, GETARG_A(i), ci, cl->p));
  if (l_unlikely(trap)) {  /* previous "Protect" updated trap */
    luaD_hookcall(L, ci);
    L->oldpc = 1;  /* next opcode will be seen as a "new" line */
  }
  updatebase(ci);  /* function has new base after adjustment */
  vmbreak;
  vmend(OP_VARARGPREP)
}
vmcase(OP_EXTRAARG) {
  vmbegin(OP_EXTRAARG)
  lua_assert(0);
  vmbreak;
  vmend(OP_EXTRAARG)
}
