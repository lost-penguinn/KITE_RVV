/**
 * @file registers.h
 * @brief Contains the definition of the RegisterFile class for managing registers in the simulator.
 * @author Vishank Singh, https://github.com/VishankSingh
 */

#ifndef REGISTERS_H
#define REGISTERS_H

#include <QObject>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Kites
{
/**
 * @brief Vector extension constants (RVV 1.0).
 *
 * vstart / vl / vtype / vlenb live in the normal CSR array at their architectural addresses,
 * so the existing CSR undo/redo path (RegisterChange reg_type 1) covers them for free.
 */
namespace vconst
{
constexpr size_t kVlenBits = 128;           ///< VLEN: bits per vector register.
constexpr size_t kVlenBytes = kVlenBits / 8; ///< Bytes per vector register.
constexpr size_t kNumVectorRegs = 32;       ///< v0..v31.
constexpr size_t kElenBits = 64;            ///< Widest supported element (SEW <= 64).

constexpr size_t kCsrVstart = 0x008;
constexpr size_t kCsrVl = 0xC20;
constexpr size_t kCsrVtype = 0xC21;
constexpr size_t kCsrVlenb = 0xC22;

constexpr uint64_t kVtypeVill = 1ULL << 63; ///< vill bit of vtype (XLEN = 64).
} // namespace vconst

/**
 * @brief Represents a register file containing integer, floating-point, and vector registers.
 */
class RegisterFile : public QObject
{
    Q_OBJECT
  private:
    static constexpr size_t NUM_GPR = 32; ///< Number of General-Purpose Registers (GPR).
    static constexpr size_t NUM_FPR = 32; ///< Number of Floating-Point Registers (FPR).

    std::array<uint64_t, NUM_GPR> gpr_ = {}; ///< Array for storing GPR values.
    std::array<uint64_t, NUM_FPR> fpr_ = {}; ///< Array for storing FPR values.

    static constexpr size_t NUM_CSR = 4096; ///< Number of Control and Status Registers (CSR).

    std::array<uint64_t, NUM_CSR> csr_ = {}; ///< Array for storing CSR values.

    /// Vector register file: 32 registers of VLEN/8 bytes, element 0 at byte 0 (little-endian).
    /// Fixed-size storage on purpose: no dynamic allocation anywhere in the vector state.
    std::array<std::array<uint8_t, vconst::kVlenBytes>, vconst::kNumVectorRegs> vpr_ = {};

  public:
    /**
     * @brief Enum representing the type of a register.
     */
    enum class RegisterType
    {
        INVALID,        ///< Invalid register type.
        INTEGER,        ///< General-purpose integer register.
        FLOATING_POINT, ///< Floating-point register.
        VECTOR,         ///< Vector register.
        CSR             ///< Control and Status Register (CSR).
    };

    explicit RegisterFile(QObject *parent = nullptr);
    virtual ~RegisterFile() = default;

    void Reset();

    /**
     * @brief Reads the value of a General-Purpose Register (GPR).
     * @param reg The index of the GPR to read.
     * @return The value of the GPR at the specified index.
     */
    [[nodiscard]] uint64_t ReadGpr(size_t reg) const;

    /**
     * @brief Writes a value to a General-Purpose Register (GPR).
     * @param reg The index of the GPR to write.
     * @param value The value to write.
     */
    void WriteGpr(size_t reg, uint64_t value);

    /**
     * @brief Reads the value of a Floating-Point Register (FPR).
     * @param reg The index of the FPR to read.
     * @return The value of the FPR at the specified index.
     */
    [[nodiscard]] uint64_t ReadFpr(size_t reg) const;

    /**
     * @brief Writes a value to a Floating-Point Register (FPR).
     * @param reg The index of the FPR to write.
     * @param value The value to write.
     */
    void WriteFpr(size_t reg, uint64_t value);

    [[nodiscard]] uint64_t ReadCsr(size_t reg) const;

    void WriteCsr(size_t reg, uint64_t value);

    /**
     * @brief Retrieves the values of all General-Purpose Registers (GPR).
     * @return A vector containing the values of all GPRs.
     */
    [[nodiscard]] std::vector<uint64_t> GetGprValues() const;

    /**
     * @brief Retrieves the values of all Floating-Point Registers (FPR).
     * @return A vector containing the values of all FPRs.
     */
    [[nodiscard]] std::vector<uint64_t> GetFprValues() const;

    void ModifyRegister(const std::string &reg_name, uint64_t value);

    // ---------------- Vector registers ----------------

    /// Returns a copy of the raw bytes of vector register `reg`.
    [[nodiscard]] std::array<uint8_t, vconst::kVlenBytes> ReadVreg(size_t reg) const;

    /// Overwrites vector register `reg` with `bytes`.
    void WriteVreg(size_t reg, const std::array<uint8_t, vconst::kVlenBytes> &bytes);

    /**
     * @brief Reads element `index` of width `sew_bytes` (1, 2, 4 or 8) from the register group
     *        starting at `base_reg`. Elements past the end of one register continue into the
     *        next register, which is exactly how LMUL > 1 register groups are laid out.
     * @throws std::out_of_range if the element falls outside v0..v31.
     */
    [[nodiscard]] uint64_t ReadVectorElement(size_t base_reg, size_t sew_bytes,
                                             size_t index) const;

    /// Writes the low `sew_bytes` bytes of `value` as element `index` (see ReadVectorElement).
    void WriteVectorElement(size_t base_reg, size_t sew_bytes, size_t index, uint64_t value);

  signals:
    void updateRegister(size_t regIndex, uint64_t value);
    void updateFRegister(size_t regIndex, uint64_t value);
    void updateVRegister(size_t regIndex); ///< Emitted when any byte of vector reg changes.
    void registerResetSignal();
};

extern const std::unordered_set<std::string> valid_general_purpose_registers;

extern const std::unordered_set<std::string> valid_floating_point_registers;

extern const std::unordered_set<std::string> valid_csr_registers;

extern const std::unordered_map<std::string, int> csr_to_address;

/**
 * @brief Map of register aliases to their actual names.
 */
extern const std::unordered_map<std::string, std::string> reg_alias_to_name;

bool IsValidGeneralPurposeRegister(const std::string &reg);

bool IsValidFloatingPointRegister(const std::string &reg);

bool IsValidCsr(const std::string &reg);

/// True for "v0" .. "v31" (no leading zeros, no aliases).
bool IsValidVectorRegister(const std::string &reg);

/// Returns the register number of a valid vector register name; throws std::invalid_argument.
size_t VectorRegisterIndex(const std::string &reg);
}//namespace Kites
#endif // REGISTERS_H