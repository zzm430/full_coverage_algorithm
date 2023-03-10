#ifndef MO_MACRO_DEFINE_H_INCLUDED
#define MO_MACRO_DEFINE_H_INCLUDED

#include <stdio.h>

/**< flag define area */
#ifdef __cplusplus
#ifdef _MSC_VER
#ifdef DLL_EXPORT_API
#define DLL_API_C_DECL extern "C" __declspec(dllexport)
#else
#define DLL_API_C_DECL extern "C" __declspec(dllimport)
#endif  // DLL_EXPORT_API
#else
#define DLL_API_C_DECL extern "C" __attribute__((visibility("default")))
#endif  // _MSC_VER
#else
#ifdef _MSC_VER
#ifdef DLL_EXPORT_API
#define DLL_API_C_DECL __declspec(dllexport)
#else
#define DLL_API_C_DECL __declspec(dllimport)
#endif  // DLL_EXPORT_API
#else
#define DLL_API_C_DECL __attribute__((visibility("default")))
#endif  // _MSC_VER
#endif

#ifndef IN
#define IN
#endif  // IN

#ifndef OUT
#define OUT
#endif  // OUT

/**< data define area */
#define MO_INVALID_HANDLE 0UL

/**< type define area */
#define MO_CAMERA_HANDLE uint64_t

/**< fuction define area */
#ifndef FREE
#define FREE(pvoid)                      \
    if (NULL != (pvoid)) {               \
        void** pvTmp = (void**)&(pvoid); \
        free(*pvTmp);                    \
        *pvTmp = NULL;                   \
    }
#endif  // FREE

#ifndef DELETE
#define DELETE(pvoid, type)              \
    if (NULL != (pvoid)) {               \
        type** pvTmp = (type**)&(pvoid); \
        delete *pvTmp;                   \
        *pvTmp = NULL;                   \
    }
#endif  // FREE

#ifndef BREAK_INFO_PRINT
#if __cplusplus < 201103L
#define BREAK_INFO_PRINT(result, func, ...)    \
    if (0 > (result)) {                        \
        printf(#func                           \
               " fails, return %d "__VA_ARGS__ \
               "\n",                           \
               result);                        \
        break;                                 \
    } else {                                   \
        printf(#func " succeed!\n");           \
    }
#else
#define BREAK_INFO_PRINT(result, func, ...)                          \
    if (0 > (result)) {                                              \
        printf(#func " fails, return %d " __VA_ARGS__ "\n", result); \
        break;                                                       \
    } else {                                                         \
        printf(#func " succeed!\n");                                 \
    }
#endif  // __cplusplus < 201103L
#endif  // BREAK_INFO_PRINT

#ifndef RESULT_INFO_PRINT
#if __cplusplus < 201103L
#define RESULT_INFO_PRINT(result, func, ...)   \
    if (0 > (result)) {                        \
        printf(#func                           \
               " fails, return %d "__VA_ARGS__ \
               "\n",                           \
               result);                        \
    } else {                                   \
        printf(#func " succeed!\n");           \
    }
#else
#define RESULT_INFO_PRINT(result, func, ...)                         \
    if (0 > (result)) {                                              \
        printf(#func " fails, return %d " __VA_ARGS__ "\n", result); \
    } else {                                                         \
        printf(#func " succeed!\n");                                 \
    }
#endif  // __cplusplus < 201103L
#endif  // RESULT_INFO_PRINT

#ifdef WANNA_OUTPUT_DEBUG_INFO
#ifndef DEBUG_INFO_PRINT
#if __cplusplus < 201103L
#define DEBUG_INFO_PRINT(format, ...) \
    printf("\nFile: %s : Line: %d ->Function: %s\n" format "\n", __BASE_FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__);
#else
#define DEBUG_INFO_PRINT(format, ...) \
    printf("\nFile: %s : Line: %d ->Function: %s\n" format "\n", __BASE_FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__);
#endif  // __cplusplus < 201103L
#endif  // DEBUG_INFO_PRINT(format,...)
#else
#ifndef DEBUG_INFO_PRINT
#define DEBUG_INFO_PRINT(format, ...)
#endif  // DEBUG_INFO_PRINT(format,...)
#endif  // WANNA_OUTPUT_DEBUG_INFO

#endif  // MO_MACRO_DEFINE_H_INCLUDED
