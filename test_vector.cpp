#include "common/vector_defs.h"
#include <iostream>

int main()
{
    using namespace Kites::vec;

    auto result = parseVtypeTokens({"e32", "m1", "ta", "ma"});

    std::cout << "Valid: " << result.ok << '\n';
    std::cout << "Zimm: 0x" << std::hex << result.zimm << '\n';
    std::cout << "VLMAX: " << std::dec << vlmax(result.zimm) << '\n';
    std::cout << "Vtype: " << vtypeToString(result.zimm) << '\n';

    std::cout << "Illegal raw 0x0C: "
              << isLegalVtype(0x0C) << '\n';

    return 0;
}
