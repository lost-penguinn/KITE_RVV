
/**
 * @file vector_defs.h
 * @brief Header-only helpers for the RISC-V Vector extension (RVV 1.0).
 *
 * Provides:
 *   - vtype encoding and decoding
 *   - SEW and LMUL calculations
 *   - vtype legality checks
 *   - VLMAX calculation
 *   - Parsing assembler vtype tokens
 *
 * VLEN = 128 bits, ELEN = 64 bits.
 *
 * vtype layout (XLEN = 64):
 *   bit 63    : vill
 *   bit 7     : vma
 *   bit 6     : vta
 *   bits 5:3  : vsew
 *   bits 2:0  : vlmul
 */

#ifndef VECTOR_DEFS_H
#define VECTOR_DEFS_H

#include "processor/registers.h"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace Kites
{
namespace vec
{

struct VType
{
    uint8_t vlmul = 0;
    uint8_t vsew = 0;
    bool vta = false;
    bool vma = false;
};

/**
 * Decode the low 8 bits of a vtype value.
 */
inline VType decodeVtype(uint64_t raw)
{
    VType t;

    t.vlmul = static_cast<uint8_t>(raw & 0x7);
    t.vsew = static_cast<uint8_t>((raw >> 3) & 0x7);
    t.vta = ((raw >> 6) & 1) != 0;
    t.vma = ((raw >> 7) & 1) != 0;

    return t;
}

/**
 * Encode VType fields into the low 8 bits.
 */
inline uint32_t encodeVtype(const VType &t)
{
    return static_cast<uint32_t>(t.vlmul & 0x7)
         | (static_cast<uint32_t>(t.vsew & 0x7) << 3)
         | (static_cast<uint32_t>(t.vta) << 6)
         | (static_cast<uint32_t>(t.vma) << 7);
}

/**
 * Return element width in bits.
 * Returns 0 for reserved vsew encodings.
 */
inline unsigned sewBits(uint8_t vsew)
{
    return (vsew <= 3) ? (8u << vsew) : 0u;
}

/**
 * Convert LMUL encoding into a fraction.
 *
 * Returns false for the reserved encoding 0b100.
 */
inline bool lmulFraction(uint8_t vlmul, unsigned &num, unsigned &den)
{
    switch (vlmul)
    {
        case 0b000:
            num = 1;
            den = 1;
            return true;

        case 0b001:
            num = 2;
            den = 1;
            return true;

        case 0b010:
            num = 4;
            den = 1;
            return true;

        case 0b011:
            num = 8;
            den = 1;
            return true;

        case 0b101:
            num = 1;
            den = 8;
            return true;

        case 0b110:
            num = 1;
            den = 4;
            return true;

        case 0b111:
            num = 1;
            den = 2;
            return true;

        default:
            return false;
    }
}

/**
 * Check whether a vtype value is legal.
 *
 * Illegal conditions:
 *   - vill or reserved upper bits are set
 *   - vsew is reserved
 *   - vlmul is reserved
 *   - SEW > LMUL * ELEN
 */
inline bool isLegalVtype(uint64_t raw)
{
    // Bits above vma must be zero.
    if ((raw >> 8) != 0)
        return false;

    VType t = decodeVtype(raw);

    unsigned sew = sewBits(t.vsew);

    unsigned num = 0;
    unsigned den = 0;

    if (sew == 0)
        return false;

    if (!lmulFraction(t.vlmul, num, den))
        return false;

    return sew * den <= vconst::kElenBits * num;
}

/**
 * Calculate VLMAX:
 *
 * VLMAX = VLEN * LMUL / SEW
 *
 * Returns 0 for an illegal vtype.
 */
inline uint64_t vlmax(uint64_t raw)
{
    if (!isLegalVtype(raw))
        return 0;

    VType t = decodeVtype(raw);

    unsigned num = 1;
    unsigned den = 1;

    if (!lmulFraction(t.vlmul, num, den))
        return 0;

    return
        (static_cast<uint64_t>(vconst::kVlenBits) * num) /
        (static_cast<uint64_t>(den) * sewBits(t.vsew));
}

struct VtypeParseResult
{
    bool ok = false;
    uint32_t zimm = 0;
    std::string error;
};

/**
 * Parse vtype tokens:
 *
 * Examples:
 *   {"e32", "m1"}
 *   {"e32", "m1", "ta", "ma"}
 *   {"e16", "m2", "tu", "mu"}
 *
 * Omitted policies default to:
 *   tu = tail undisturbed
 *   mu = mask undisturbed
 */
inline VtypeParseResult parseVtypeTokens(
    const std::vector<std::string> &tokens)
{
    VtypeParseResult result;

    auto fail = [&result](std::string message)
    {
        result.ok = false;
        result.error = std::move(message);
        return result;
    };

    if (tokens.size() < 2 || tokens.size() > 4)
    {
        return fail(
            "expected e<sew>, m<lmul>[, ta|tu][, ma|mu]");
    }

    VType t;

    // Parse SEW.
    const std::string &sew = tokens[0];

    if (sew == "e8")
        t.vsew = 0;
    else if (sew == "e16")
        t.vsew = 1;
    else if (sew == "e32")
        t.vsew = 2;
    else if (sew == "e64")
        t.vsew = 3;
    else
    {
        return fail(
            "invalid element width: " + sew);
    }

    // Parse LMUL.
    const std::string &lm = tokens[1];

    if (lm == "m1")
        t.vlmul = 0b000;
    else if (lm == "m2")
        t.vlmul = 0b001;
    else if (lm == "m4")
        t.vlmul = 0b010;
    else if (lm == "m8")
        t.vlmul = 0b011;
    else if (lm == "mf8")
        t.vlmul = 0b101;
    else if (lm == "mf4")
        t.vlmul = 0b110;
    else if (lm == "mf2")
        t.vlmul = 0b111;
    else
    {
        return fail(
            "invalid LMUL: " + lm);
    }

    // Parse tail and mask policies.
    size_t i = 2;

    if (i < tokens.size() &&
        (tokens[i] == "ta" || tokens[i] == "tu"))
    {
        t.vta = (tokens[i++] == "ta");
    }

    if (i < tokens.size() &&
        (tokens[i] == "ma" || tokens[i] == "mu"))
    {
        t.vma = (tokens[i++] == "ma");
    }

    if (i != tokens.size())
    {
        return fail(
            "unexpected policy token: " + tokens[i]);
    }

    result.zimm = encodeVtype(t);

    if (!isLegalVtype(result.zimm))
    {
        return fail(
            "illegal combination: SEW is larger than LMUL * ELEN");
    }

    result.ok = true;
    return result;
}

/**
 * Convert a legal vtype into readable text.
 *
 * Example:
 *   e32, m1, ta, ma
 */
inline std::string vtypeToString(uint64_t raw)
{
    if (!isLegalVtype(raw))
        return "illegal";

    VType t = decodeVtype(raw);

    static const char *lmulNames[8] =
    {
        "m1", "m2", "m4", "m8",
        "?", "mf8", "mf4", "mf2"
    };

    return
        "e" + std::to_string(sewBits(t.vsew)) +
        ", " + lmulNames[t.vlmul] +
        ", " + (t.vta ? "ta" : "tu") +
        ", " + (t.vma ? "ma" : "mu");
}

} // namespace vec
} // namespace Kites

#endif // VECTOR_DEFS_H
