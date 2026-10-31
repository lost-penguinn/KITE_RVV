#include "assembler/parser.h"
#include "processor/registers.h"

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

    block.setRd(peekToken(1).value);
    block.setRs2(peekToken(3).value);
    block.setRs1(peekToken(5).value);

    skipCurrentLine();

    intermediate_code_.emplace_back(block, true);
    instruction_number_line_number_mapping_[instruction_index_] = line;
    instruction_index_++;

    return true;
}

} // namespace Kites
