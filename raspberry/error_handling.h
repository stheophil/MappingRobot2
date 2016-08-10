#ifndef _error_handling_h
#define _error_handling_h

bool Verify(bool bResult, const char* szMessage, const char* szFile, int nLine);

#define VERIFY(expr) Verify(expr, #expr, __FILE__, __LINE__)
#define VERIFYEQUAL(expr, result) Verify(expr==result, #expr, __FILE__, __LINE__)
#define ASSERT(expr) (assert(expr))

#define LOG(x)

#endif