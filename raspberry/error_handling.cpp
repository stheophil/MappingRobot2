#include <iostream>
#include "error_handling.h"

bool Verify(bool bResult, const char* szMessage, const char* szFile, int nLine) {
	if(!bResult) {
		std::cout << "VERIFY: " << szMessage << " (" << szFile << ":" << nLine << ")\n";
		std::terminate();
	}
	return bResult;
}
