#ifndef SP_CROSSPLAT_FUNC_H
#define SP_CROSSPLAT_FUNC_H

//Header files should be included in the file where the functions are used.

#if defined(_MSC_VER) && (defined(WIN32) || defined(WIN64))

#define SP_memcpy(dst, dstSize, src, cpysize) memcpy_s((dst), (dstSize), (src), (cpysize))

#define SP_sprintf(dst, dstSize, ...) sprintf_s((dst), (dstSize), __VA_ARGS__)

#define SP_sscanf(src, ...) sscanf_s((src), __VA_ARGS__)

#define SP_fscanf(srcFile, ...) fscanf_s((srcFile), __VA_ARGS__)

#define SP_strncat(dest, destsz, src, count) strncat_s((dest), (destsz), (src), (count))

/// in MSC implementation, use _TRUNCATE for third argument, max number of characters to store
#define SP_snprintf(dst, dstSize, ...)  _snprintf_s((dst), (dstSize), _TRUNCATE, __VA_ARGS__)

/// in MSC implementation, use _TRUNCATE for third argument, max number of characters to store
#define SP_vsnprintf(dst, dstSize, sFormat, argList)  vsnprintf_s((dst), (dstSize), _TRUNCATE, (sFormat), (argList))

/// SP_fopen returns true if the file could be opened and false otherwise.
#define SP_fopen(pFile, fName, mode) (0 == fopen_s((pFile), (fName), (mode)))

#define SP_copy(srcFirst, srcLast, dstFirst, numEleCopy) std::copy((srcFirst), (srcLast), \
    stdext::make_checked_array_iterator((dstFirst), (numEleCopy)))

/// if local time is obtained successfully, SP_localTime returns a pointer to the time struct.
/// Otherwise, it returns a nullptr.
/// <param name="pTimeInfo"> address of a valid object of type tm, required for MSC implementation.
/// Note: result is to be stored in the returned pointer, and may not be updated in pTimeInfo.
/// </param>
#define SP_localtime(pTimeInfo, curTime) \
    ((0 == localtime_s((pTimeInfo), (curTime))) ? pTimeInfo : nullptr)

#define SP_alignDepthToIntrinsics(extrinsicsTranslation, inputIntrinsics, pInputDepth, \
	outputIntrinsics, pOutputDepth) ScenePerception::AlignDepthToIntrinsics(extrinsicsTranslation, \
	inputIntrinsics, pInputDepth, outputIntrinsics, pOutputDepth, false)

#else //defined(__ANDROID__)

#ifdef __cplusplus
extern "C" {
#endif

#if defined (__ANDROID__)
#include "safe_lib.h"
#else
#include <cstring>
#endif

#ifdef __cplusplus
}
#endif

#if defined (__ANDROID__)
	#define SP_memcpy(dst, dstSize, src, cpysize) memcpy_s(dst, dstSize, src, cpysize)
#else
	#define SP_memcpy(dst, dstSize, src, cpysize) std::memcpy(dst, src, ((dstSize > cpysize)? cpysize : dstSize))
#endif

#define SP_sprintf(dst, dstSize, ...) std::sprintf((dst), __VA_ARGS__)

#define SP_snprintf(dst, dstSize, ...) std::snprintf((dst), (dstSize), __VA_ARGS__)

#define SP_vsnprintf(dst, dstSize, sFormat, argList)  std::vsnprintf((dst), (dstSize), (sFormat), (argList))

#define SP_sscanf(src, ...) std::sscanf((src), __VA_ARGS__)

#define SP_fscanf(srcFile, ...) std::fscanf((srcFile), __VA_ARGS__)

#define SP_strncat(dest, destsz, src, count) strncat((dest), (src), ((destsz > count) ? count : destsz))

#define SP_fopen(_FILE, _Filename, _Mode) ((nullptr != (_FILE)) && (nullptr != (*(_FILE) = fopen((_Filename), (_Mode)))))

#define SP_copy(srcFirst, srcLast, dstFirst, numEleCopy) std::copy((srcFirst), (srcLast), (dstFirst))

/// if local time is obtained successfully, SP_localTime returns a pointer to the time struct.
/// Otherwise, it returns a nullptr.
/// <param name="pTimeInfo"> address of a valid object of type tm, required for MSC implementation.
/// Note: result is to be stored in the returned pointer, and may not be updated in pTimeInfo.
/// </param>
#define SP_localtime(pTimeInfo, curTime) std::localtime((curTime))

#define SP_alignDepthToIntrinsics(extrinsicsTranslation, inputIntrinsics, pInputDepth, \
	outputIntrinsics, pOutputDepth) ScenePerception::AlignDepthToIntrinsics(extrinsicsTranslation, \
	inputIntrinsics, pInputDepth, outputIntrinsics, pOutputDepth, false)

#endif //defined(_MSC_VER) && (defined(WIN32) || defined(WIN64))

#endif //SP_CROSSPLAT_FUNC_H
