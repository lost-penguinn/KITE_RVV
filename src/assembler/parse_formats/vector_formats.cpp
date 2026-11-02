#include "assembler/parser.h"
#include "processor/registers.h"
#include "common/vector_defs.h"
#include "utils/utils.h"
namespace Kites
{

bool Parser::parse_O_VEC_C_VEC_C_VEC()
{
    const int line = currentToken().line_number;

    if (peekToken(1).line_number != line ||
        peekToken(1).type != TokenType::VEC_REGISTER ||
        peekToken(2).type != TokenType::COMMA ||
        peekToken(3).line_number != line ||
        peekToken(3).type != TokenType::VEC_REGISTER ||
        peekToken(4).type != TokenType::COMMA ||
        peekToken(5).line_number != line ||
        peekToken(5).type != TokenType::VEC_REGISTER ||
        (peekToken(6).type != TokenType::EOF_ &&
         peekToken(6).line_number == line))
    {
        return false;
    }
    
    ICUnit block;
    block.setOpcode(currentToken().value);
    block.setLineNumber(line);
    block.setInstructionIndex(instruction_index_);

    block.setRd(peekToken(1).value);    // vd  (1st operand)
    block.setRs1(peekToken(3).value);   // vs2 (2nd operand)
    block.setRs2(peekToken(5).value);   // vs1 (3rd operand)

    skipCurrentLine();

    intermediate_code_.emplace_back(block, true);
    instruction_number_line_number_mapping_[instruction_index_] = line;
    instruction_index_++;

    return true;
}
// vmv.v.i vd, simm5
bool Parser::parse_O_VEC_C_I()
{
    const unsigned int line = currentToken().line_number;
    if (peekToken(1).line_number == line && peekToken(1).type == TokenType::VEC_REGISTER &&
        peekToken(2).line_number == line && peekToken(2).type == TokenType::COMMA &&
        peekToken(3).line_number == line && peekToken(3).type == TokenType::NUM &&
        (peekToken(4).type == TokenType::EOF_ || peekToken(4).line_number != line))
    {
        long long imm = 0;
        try { imm = std::stoll(peekToken(3).value); } catch (...) { imm = 1000; }

        if (imm < -16 || imm > 15)
        {
            recordError(ParseError(peekToken(3).line_number, "Immediate value out of range"));
            errors_.all_errors.emplace_back(errors::ImmediateOutOfRangeError(
                "Immediate value out of range", "Expected: -16 <= imm <= 15", filename_,
                peekToken(3).line_number, peekToken(3).column_number,
                GetLineFromFile(filename_, peekToken(3).line_number)));
            skipCurrentLine();
            return true;
        }

        ICUnit block;
        block.setOpcode(currentToken().value);
        block.setLineNumber(line);
        block.setInstructionIndex(instruction_index_);
        block.setRd(peekToken(1).value);
        block.setImm(std::to_string(imm));

        skipCurrentLine();
        intermediate_code_.emplace_back(block, true);
        instruction_number_line_number_mapping_[instruction_index_] = block.getLineNumber();
        instruction_index_++;
        return true;
    }
    return false;
}

// vmv.v.x vd, rs1
bool Parser::parse_O_VEC_C_GPR()
{
    const unsigned int line = currentToken().line_number;
    if (peekToken(1).line_number == line && peekToken(1).type == TokenType::VEC_REGISTER &&
        peekToken(2).line_number == line && peekToken(2).type == TokenType::COMMA &&
        peekToken(3).line_number == line && peekToken(3).type == TokenType::GP_REGISTER &&
        (peekToken(4).type == TokenType::EOF_ || peekToken(4).line_number != line))
    {
        ICUnit block;
        block.setOpcode(currentToken().value);
        block.setLineNumber(line);
        block.setInstructionIndex(instruction_index_);
        block.setRd(peekToken(1).value);
        block.setRs1(reg_alias_to_name.at(peekToken(3).value));   // "t0" -> "x5", like every other parser

        skipCurrentLine();
        intermediate_code_.emplace_back(block, true);
        instruction_number_line_number_mapping_[instruction_index_] = block.getLineNumber();
        instruction_index_++;
        return true;
    }
    return false;
}

// vle32.v / vse32.v vd, (rs1)
bool Parser::parse_O_VEC_C_LP_GPR_RP()
{
    const unsigned int line = currentToken().line_number;

    if (peekToken(1).line_number != line ||
        peekToken(1).type != TokenType::VEC_REGISTER ||
        peekToken(2).line_number != line ||
        peekToken(2).type != TokenType::COMMA ||
        peekToken(3).line_number != line ||
        peekToken(3).type != TokenType::LPAREN ||
        peekToken(4).line_number != line ||
        peekToken(4).type != TokenType::GP_REGISTER ||
        peekToken(5).line_number != line ||
        peekToken(5).type != TokenType::RPAREN ||
        (peekToken(6).type != TokenType::EOF_ &&
         peekToken(6).line_number == line))
    {
        return false;
    }

    ICUnit block;
    block.setOpcode(currentToken().value);
    block.setLineNumber(line);
    block.setInstructionIndex(instruction_index_);

    block.setRd(peekToken(1).value);
    block.setRs1(reg_alias_to_name.at(peekToken(4).value));

    skipCurrentLine();

    intermediate_code_.emplace_back(block, true);
    instruction_number_line_number_mapping_[instruction_index_] = line;
    instruction_index_++;

    return true;
}
// vd, vs2, rs1 — vector-scalar arithmetic
bool Parser::parse_O_VEC_C_VEC_C_GPR()
{
    const int line = currentToken().line_number;

    if (peekToken(1).line_number != line ||
        peekToken(1).type != TokenType::VEC_REGISTER ||
        peekToken(2).type != TokenType::COMMA ||
        peekToken(3).line_number != line ||
        peekToken(3).type != TokenType::VEC_REGISTER ||
        peekToken(4).type != TokenType::COMMA ||
        peekToken(5).line_number != line ||
        peekToken(5).type != TokenType::GP_REGISTER ||
        (peekToken(6).type != TokenType::EOF_ &&
         peekToken(6).line_number == line))
    {
        return false;
    }

    ICUnit block;
    block.setOpcode(currentToken().value);
    block.setLineNumber(line);
    block.setInstructionIndex(instruction_index_);

    block.setRd(peekToken(1).value);    // vd
    block.setRs1(peekToken(3).value);   // vs2
    block.setRs2(reg_alias_to_name.at(peekToken(5).value)); // scalar GPR

    skipCurrentLine();

    intermediate_code_.emplace_back(block, true);
    instruction_number_line_number_mapping_[instruction_index_] = line;
    instruction_index_++;

    return true;
}
} // namespace Kites
