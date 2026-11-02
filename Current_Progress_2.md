# Kites RVV — Changes Since the Progress Checkpoint

**Date:** 1 November 2026
**Baseline:** the state described in `Current_Progress.txt` (`vsetvli` with immediate 192, `vadd.vv`, vector register GUI section, undo case 3).
**Result:** `vsetvli`, `vmv.v.i`, `vmv.v.x` and `vadd.vv` assemble to the correct RVV encodings and run correctly. Verified end to end with the program in section 8.

---

## 1. Summary of files touched

| File | Change |
|---|---|
| `src/assembler/code_generator.cpp` | Fixed `vadd.vv` operand order; return type to `uint32_t`; removed illegal-vtype throw from `vsetvli`; added `vmv.v.i` / `vmv.v.x` encoders and dispatch |
| `src/assembler/parse_formats/vector_formats.cpp` | `vsetvli` vtype legality check at parse time; `vadd.vv` operands stored in text order; new `parse_O_VEC_C_I`, `parse_O_VEC_C_GPR` |
| `src/common/instructions.h` | New `SyntaxType::O_VEC_C_I`, `O_VEC_C_GPR` |
| `src/common/instructions.cpp` | Syntax map entries, `getExpectedSyntaxes` strings, **`valid_instructions` list** gets `vmv.v.i`, `vmv.v.x` |
| `src/assembler/parser.h` / `parser.cpp` | Declarations and `case` labels for the two new syntaxes |
| `src/processor/rvss/rvss_processor.cpp` | `ExecuteVsetvli()` rewritten (full AVL rules, undo records); `ExecuteVector()` rewritten (single write, `vmv` forms) |
| UI files (see section 7) | Recommended changes; status not confirmed |

---

## 2. Corrections to earlier notes

1. **Immediate 192 is `e8`, not `e32`.** `192 = 0xC0` decodes as vlmul=000, vsew=000 (e8), vta=1, vma=1, so VLMAX = 16. **`e32, m1, ta, ma` is 208 (`0xD0`)**, VLMAX = 4. Use `vsetvli t0, zero, 208`.
2. **`vadd.vv` had `vs1` and `vs2` swapped** in the encoding. `vadd.vv v3, v2, v1` is now `0x022081d7` (was `0x021101d7`). Addition is commutative so results looked right, but it would break `vsub.vv` and any disassembler comparison.
3. The `vmv.v.i v2, 7` word is `0x5e03b157`. (A different value was quoted in chat by mistake; the simulator's value is correct.)

---

## 3. Assembler: `code_generator.cpp`

### 3.1 `vadd.vv` operand order and return type

The ICUnit holds operands in **text order** (`rd` = 1st, `rs1` = 2nd, `rs2` = 3rd). Assembly is `vadd.vv vd, vs2, vs1`, so the 2nd text operand is `vs2`.

```cpp
uint32_t generateVAddVvMachineCode(const ICUnit &block)      // was int32_t
{
    const uint32_t vd  = VectorRegisterIndex(block.getRd());
    const uint32_t vs2 = VectorRegisterIndex(block.getRs1());  // 2nd text operand
    const uint32_t vs1 = VectorRegisterIndex(block.getRs2());  // 3rd text operand
    return (1u << 25) | (vs2 << 20) | (vs1 << 15) | (0b000u << 12) | (vd << 7) | 0b1010111u;
}
```

The parser (section 4.2) must store operands in the same text order, otherwise the two swaps cancel and the old wrong word comes back.

### 3.2 `vsetvli`: removed the throw

Deleted from `generateVSetVliMachineCode`:

```cpp
if (!vec::isLegalVtype(zimm))
    throw std::runtime_error("Illegal vtype immediate");
```

An illegal vtype is now reported by the parser with a line number (section 4.1). The `zimm > 0x7FF` check stays.

### 3.3 New encoders for `vmv.v.i` and `vmv.v.x`

Both use `funct6 = 010111`, `vm = 1`, `vs2 = 0`.

```cpp
uint32_t generateVmvVIMachineCode(const ICUnit &block)
{
    const uint32_t vd    = VectorRegisterIndex(block.getRd());
    const int      imm   = std::stoi(block.getImm());
    const uint32_t simm5 = static_cast<uint32_t>(imm) & 0x1F;
    return (0b010111u << 26) | (1u << 25) | (0u << 20) | (simm5 << 15) |
           (0b011u << 12) | (vd << 7) | 0b1010111u;
}

uint32_t generateVmvVXMachineCode(const ICUnit &block)
{
    const uint32_t vd  = VectorRegisterIndex(block.getRd());
    const uint32_t rs1 = static_cast<uint32_t>(std::stoi(block.getRs1().substr(1)));
    return (0b010111u << 26) | (1u << 25) | (0u << 20) | (rs1 << 15) |
           (0b100u << 12) | (vd << 7) | 0b1010111u;
}
```

Dispatch in `generateMachineCode()`, next to `vadd.vv`:

```cpp
else if (block.getOpcode() == "vmv.v.i")
    code = generateVmvVIMachineCode(block);
else if (block.getOpcode() == "vmv.v.x")
    code = generateVmvVXMachineCode(block);
```

Reference encodings: `vmv.v.i v1, 5` = `0x5e02b0d7`, `vmv.v.x v1, t0` = `0x5e02c0d7`.

---

## 4. Assembler: `vector_formats.cpp`

### 4.1 `vsetvli` legality check (parse time)

Placed **only in the `vsetvli` function**, after the check that token 5 is a `NUM`. (Putting it in the `vadd.vv` function made `std::stoll("v1")` throw. That was the "stoll error" seen during testing.) Requires `#include "utils/utils.h"` and `#include "common/vector_defs.h"`.

```cpp
const long long imm = std::stoll(peekToken(5).value);   // or reuse the existing variable

if (imm < 0 || imm > 0x7FF || !vec::isLegalVtype(static_cast<uint64_t>(imm)))
{
    recordError(ParseError(peekToken(5).line_number, "Illegal vtype immediate"));
    errors_.all_errors.emplace_back(errors::ImmediateOutOfRangeError(
        "Illegal vtype immediate",
        "Reserved LMUL/SEW encoding, or SEW > LMUL*ELEN (e32,m1,ta,ma is 208)",
        filename_, peekToken(5).line_number, peekToken(5).column_number,
        GetLineFromFile(filename_, peekToken(5).line_number)));
    skipCurrentLine();
    return true;
}
```

`vsetvli t0, zero, 12` (reserved LMUL) is now rejected with an editor error instead of throwing.

### 4.2 `vadd.vv` stores operands in text order

```cpp
block.setRd(peekToken(1).value);    // vd  (1st operand)
block.setRs1(peekToken(3).value);   // vs2 (2nd operand)
block.setRs2(peekToken(5).value);   // vs1 (3rd operand)
```

### 4.3 New `vmv.v.i vd, simm5` (`parse_O_VEC_C_I`)

Accepts `VEC_REGISTER , NUM` on one line. Range check `-16 <= imm <= 15` with `ImmediateOutOfRangeError`. Stores `rd` and `imm`.

### 4.4 New `vmv.v.x vd, rs1` (`parse_O_VEC_C_GPR`)

Accepts `VEC_REGISTER , GP_REGISTER`. The scalar register goes through `reg_alias_to_name` (`t0` becomes `x5`), like every other `parse_O_*` function. Skipping that step was the cause of the very first wrong-`rd` bug.

---

## 5. Instruction tables and parser dispatch

**`instructions.h`** (after `O_VEC_C_VEC_C_VEC`):

```cpp
    O_VEC_C_I,     ///< vector-register , immediate
    O_VEC_C_GPR,   ///< vector-register , general-register
```

**`instructions.cpp`:**

```cpp
    {"vmv.v.i", {SyntaxType::O_VEC_C_I}},
    {"vmv.v.x", {SyntaxType::O_VEC_C_GPR}},
```

`getExpectedSyntaxes`: `"<vec-reg>, <imm>"` and `"<vec-reg>, <gp-reg>"`.

**`valid_instructions` (line ~379) must contain the new mnemonics.** This was the cause of the "failed to parse vmv" error: the lexer only produces an OPCODE token for words in this list.

```cpp
    "vadd.vv", "vsetvli", "vmv.v.i", "vmv.v.x",
```

**`parser.h`:** `bool parse_O_VEC_C_I();` and `bool parse_O_VEC_C_GPR();`
**`parser.cpp`:** `case SyntaxType::O_VEC_C_I:` and `case SyntaxType::O_VEC_C_GPR:` calling them.

Consistency check for any new mnemonic:

```
grep -rn '"vmv.v.i"' src | wc -l      # should equal the count for "vadd.vv" (3)
```

---

## 6. Processor: `rvss_processor.cpp`

### 6.1 `ExecuteVsetvli()` rewritten

Full spec behavior, with undo records for `vtype`, `vl` and `rd` on **every** path, including the illegal one.

- Illegal vtype: `vtype = vill`, `vl = 0`, `rd = 0`.
- `rs1 != x0`: `vl = min(x[rs1], VLMAX)`.
- `rs1 = x0`, `rd != x0`: `vl = VLMAX`.
- `rs1 = x0`, `rd = x0`: keep `vl`; if the new VLMAX differs, set `vill`.

```cpp
void RVSSProcessor::ExecuteVsetvli()
{
    const uint8_t  rd   = (current_instruction_ >> 7)  & 0x1F;
    const uint8_t  rs1  = (current_instruction_ >> 15) & 0x1F;
    const uint64_t zimm = (current_instruction_ >> 20) & 0x7FF;

    const uint64_t old_vtype = registers_.ReadCsr(vconst::kCsrVtype);
    const uint64_t old_vl    = registers_.ReadCsr(vconst::kCsrVl);
    const uint64_t old_rd    = registers_.ReadGpr(rd);

    uint64_t new_vtype = zimm;
    uint64_t new_vl    = 0;

    if (!vec::isLegalVtype(zimm))
    {
        new_vtype = vconst::kVtypeVill;
        new_vl    = 0;
    }
    else if (rs1 != 0)
    {
        new_vl = std::min<uint64_t>(registers_.ReadGpr(rs1), vec::vlmax(zimm));
    }
    else if (rd != 0)
    {
        new_vl = vec::vlmax(zimm);
    }
    else if (vec::isLegalVtype(old_vtype) && vec::vlmax(old_vtype) == vec::vlmax(zimm))
    {
        new_vl = old_vl;
    }
    else
    {
        new_vtype = vconst::kVtypeVill;
        new_vl    = 0;
    }

    registers_.WriteCsr(vconst::kCsrVtype, new_vtype);
    registers_.WriteCsr(vconst::kCsrVl, new_vl);

    if (old_vtype != new_vtype)
        current_delta_.register_changes.push_back({vconst::kCsrVtype, 1, old_vtype, new_vtype});
    if (old_vl != new_vl)
        current_delta_.register_changes.push_back({vconst::kCsrVl, 1, old_vl, new_vl});

    if (rd != 0)
    {
        registers_.WriteGpr(rd, new_vl);
        if (old_rd != new_vl)
            current_delta_.register_changes.push_back({rd, 0, old_rd, new_vl});
    }
}
```

CSR undo records use the CSR **address** as the index, matching `Undo()` case 1 and the existing `ExecuteCsr()`.

### 6.2 `ExecuteVector()` rewritten

Handles `vadd.vv`, `vmv.v.x`, `vmv.v.i`. Key points:

- `isLegalVtype` on the whole `vtype` value (this also rejects `vill`). A fresh reset rejects vector ops until a `vsetvli` has run.
- Result is built in a local copy of the destination and written **once** with `WriteVreg` (one UI refresh, not up to 16).
- `old_vector` is captured **before** the write, so undo works.
- Elements at or beyond `vl` are left unchanged.
- Unsupported: masked (`vm = 0`, which for `funct6 = 010111` would be `vmerge`) and LMUL != 1. Both return without changing state.
- `vmv.v.x` splats `x[rs1]`, truncated to SEW. `vmv.v.i` splats the sign-extended 5-bit immediate.

```cpp
void RVSSProcessor::ExecuteVector()
{
    const uint32_t inst   = current_instruction_;
    const uint8_t  opcode = inst & 0x7F;
    const uint8_t  vd     = (inst >> 7)  & 0x1F;
    const uint8_t  funct3 = (inst >> 12) & 0x7;
    const uint8_t  vs1    = (inst >> 15) & 0x1F;   // also rs1 (OPIVX) or simm5 (OPIVI)
    const uint8_t  vs2    = (inst >> 20) & 0x1F;
    const uint8_t  vm     = (inst >> 25) & 0x1;
    const uint8_t  funct6 = (inst >> 26) & 0x3F;

    enum class Op { VaddVV, VmvVX, VmvVI, Unsupported } op = Op::Unsupported;
    if (opcode == 0x57 && funct6 == 0b000000 && funct3 == 0b000)                  op = Op::VaddVV;
    else if (opcode == 0x57 && funct6 == 0b010111 && vs2 == 0 && funct3 == 0b100) op = Op::VmvVX;
    else if (opcode == 0x57 && funct6 == 0b010111 && vs2 == 0 && funct3 == 0b011) op = Op::VmvVI;

    if (op == Op::Unsupported)
    { std::cerr << "Unsupported vector instruction\n"; return; }
    if (vm != 1)
    { std::cerr << "Masked vector instructions are not supported yet\n"; return; }

    const uint64_t raw_vtype = registers_.ReadCsr(vconst::kCsrVtype);
    if (!vec::isLegalVtype(raw_vtype))
    { std::cerr << "Illegal instruction: vtype is invalid (vill)\n"; return; }
    const vec::VType vtype = vec::decodeVtype(raw_vtype);
    if (vtype.vlmul != 0)
    { std::cerr << "Only LMUL=1 is supported yet\n"; return; }

    const size_t   sew_bytes = vec::sewBits(vtype.vsew) / 8;
    const uint64_t vl        = std::min<uint64_t>(registers_.ReadCsr(vconst::kCsrVl),
                                                  vec::vlmax(raw_vtype));

    uint64_t splat = 0;
    if (op == Op::VmvVX)
        splat = registers_.ReadGpr(vs1);
    else if (op == Op::VmvVI)
        splat = static_cast<uint64_t>(static_cast<int64_t>(vs1 ^ 0x10) - 0x10);

    const auto old_vd = registers_.ReadVreg(vd);
    auto       new_vd = old_vd;

    for (uint64_t i = 0; i < vl; ++i)
    {
        const uint64_t value = (op == Op::VaddVV)
            ? registers_.ReadVectorElement(vs2, sew_bytes, i) +
              registers_.ReadVectorElement(vs1, sew_bytes, i)
            : splat;
        for (size_t k = 0; k < sew_bytes; ++k)
            new_vd[i * sew_bytes + k] = static_cast<uint8_t>((value >> (8 * k)) & 0xFF);
    }

    if (old_vd != new_vd)
    {
        registers_.WriteVreg(vd, new_vd);
        current_delta_.register_changes.push_back({vd, 3, 0, 0, old_vd, new_vd});
    }
}
```

The dispatch in `Execute()` (`funct3 != 0b111` goes to `ExecuteVector`) already covers `vmv.v.i` (funct3 = 011) and `vmv.v.x` (funct3 = 100).

There is no "illegal instruction" status in the simulator (existing statuses are `VM_EXIT`, `VM_PROGRAM_END`, etc.), so error cases print to `std::cerr` and leave state unchanged.

---

## 7. UI and control unit: recommended, status unknown

These were suggested but their state in your tree was not confirmed. Tick off what applies.

- [ ] **`registermodel.cpp` `flags()`:** rows 64 and above must not be editable. Editing a `v` row reaches `ModifyRegister("v3")`, where `reg_alias_to_name.at(...)` throws.
  `if (index.row() >= 64) return Qt::ItemIsSelectable | Qt::ItemIsEnabled;` (before the line that sets `Qt::ItemIsEditable`).
- [ ] **`registermodel.cpp`:** remove the duplicate `connect` for `updateVRegister` in the constructor.
- [ ] **`registermodel.cpp` `changeRegisterFile()`:** connect `updateVRegister` there, and disconnect the old register file's signal.
- [ ] **`rvss_control_unit.cpp`:** for opcode `0b1010111`, `reg_write_`, `mem_read_` and `mem_write_` must be false. `Execute()` writes registers itself, so WriteBack must not overwrite `rd` with a stale `execution_result_`. Confirmed fine for `vsetvli` (`t0` showed `0x4`), but worth a look.
- [ ] **`registercontainer.cpp`:** the vector Value column is truncated (`0x0000000500000005000...`). Widen it or elide in the middle so the low lanes stay visible.

---

## 8. Verification

**Test program** (run in the simulator):

```asm
vsetvli t0, zero, 208
vmv.v.i v1, 5
vmv.v.i v2, 7
vadd.vv v3, v2, v1
```

| Check | Expected | Observed |
|---|---|---|
| `vsetvli` word | `0d0072d7` | `0d0072d7` |
| `vmv.v.i v1, 5` word | `5e02b0d7` | `5e02b0d7` |
| `vmv.v.i v2, 7` word | `5e03b157` | `5e03b157` |
| `vadd.vv v3, v2, v1` word | `022081d7` | `022081d7` |
| `t0` after `vsetvli` | `0x4` | `0x4` |
| `v1` lanes | `0x00000005` x4 | `0x00000005...` |
| `v2` lanes | `0x00000007` x4 | `0x00000007...` |
| `v3` lanes | `0x0000000C` x4 | `0x0000000C...` |

**Standalone logic tests** (plain `g++` against the real `RegisterFile`, all passing): register-group element addressing; vtype parsing/legality/VLMAX; `vsetvli` AVL rules, illegal path and undo; `vadd.vv` including e8 wraparound, tail preservation and vill rejection; `vmv.v.i` sign extension (-16..15); `vmv.v.x` truncation to SEW; undo data capture.

**Still to verify in the GUI:**

1. **Undo/redo:** after the program above, press Undo repeatedly. `v3` should clear first, then `v2`, `v1`, then `t0` returns to 0. Redo should restore each in order.
2. **`vmv.v.x`:** `addi t1, zero, 300` / `vsetvli t0, zero, 208` / `vmv.v.x v4, t1` gives `0x0000012C` per lane. With `vsetvli t0, zero, 192` (e8) it gives `0x2C` per byte.
3. **Error line:** `vsetvli t0, zero, 12` shows an editor error and does not crash.

---

## 9. Getting an exact git diff

This file describes the changes; git has the exact ones. Replace `<checkpoint>` with the commit you consider the baseline (find it with `git log --oneline`).

```bash
git log --oneline -n 10
git diff --stat <checkpoint>
git diff <checkpoint> > vector_changes_since_checkpoint.patch
```

If your baseline is simply "uncommitted work":

```bash
git diff --stat
git diff > vector_changes_since_checkpoint.patch
```

Commit when the GUI checks above pass:

```bash
git add -A
git commit -m "vector: vadd.vv encoding fix, vsetvli AVL rules, vmv.v.i/x, single-write ExecuteVector"
```

---

## 10. Known limitations (unchanged or still open)

- Only `vsetvli`, `vadd.vv`, `vmv.v.i`, `vmv.v.x` exist.
- Unmasked only. `v0.t` syntax is not parsed.
- LMUL = 1 only. `vstart` is read by nothing yet (always 0).
- Tail and mask policies: tail elements are always left undisturbed (a legal implementation of both `ta` and `tu`).
- Named vtype syntax (`e32, m1, ta, ma`) is not supported in `vsetvli`; the immediate form is. `vec::parseVtypeTokens()` already exists in `vector_defs.h` for when you add it. The lexer tags every name after a comma as `LABEL_REF`, so the parser can accept those tokens directly.
- No vector loads/stores yet, so the only way to put data in a vector register is `vmv.v.i` / `vmv.v.x`.

## 11. Suggested next steps

1. More `.vv` arithmetic (`vsub.vv`, `vmul.vv`, `vand.vv`, `vor.vv`, `vxor.vv`), table-driven: they differ only in `funct6` and the operation.
2. Reductions: `vredsum.vs vd, vs2, vs1` (sum of `vs2[0..vl-1]` plus `vs1[0]` into `vd[0]`).
3. Vector loads and stores: `vle32.v`, `vse32.v`.
4. Named vtype syntax, then `vstart`, masking, LMUL > 1.
