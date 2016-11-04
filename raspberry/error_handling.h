#pragma once

bool Verify(bool bResult, const char* szMessage, const char* szFile, int nLine);

#define VERIFY(expr) Verify(expr, #expr, __FILE__, __LINE__)
#define VERIFYEQUAL(expr, result) Verify(expr==result, #expr, __FILE__, __LINE__)
#define ASSERT(expr) (assert(expr))

// #define ENABLE_LOG

#ifdef ENABLE_LOG
#define LOG(x) std::cout << (x) << std::endl;
#else
#define LOG(x)
#endif
