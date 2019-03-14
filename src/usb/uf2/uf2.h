/**

Microsoft UF2

The MIT License (MIT)

Copyright (c) Microsoft Corporation

All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
#ifndef UF2FORMAT_H
#define UF2FORMAT_H 1

#include "uf2cfg.h"

#include <stdint.h>
#include <stdbool.h>

#include "app_util.h"

#include "dfu_types.h"

#define SD_MAGIC_NUMBER 0x51b1e5db
#define SD_MAGIC_OK() (*((uint32_t*)(SOFTDEVICE_INFO_STRUCT_ADDRESS+4)) == 0x51b1e5db)
extern bool sdRunning;

// All entries are little endian.

#define UF2_MAGIC_START0 0x0A324655UL // "UF2\n"
#define UF2_MAGIC_START1 0x9E5D5157UL // Randomly selected
#define UF2_MAGIC_END 0x0AB16F30UL    // Ditto

// If set, the block is "comment" and should not be flashed to the device
#define UF2_FLAG_NOFLASH 0x00000001
#define UF2_FLAG_FAMILYID 0x00002000

#define MAX_BLOCKS (FLASH_SIZE / 256 + 100)
typedef struct {
    uint32_t numBlocks;
    uint32_t numWritten;
    uint8_t writtenMask[MAX_BLOCKS / 8 + 1];
} WriteState;

typedef struct {
    // 32 byte header
    uint32_t magicStart0;
    uint32_t magicStart1;
    uint32_t flags;
    uint32_t targetAddr;
    uint32_t payloadSize;
    uint32_t blockNo;
    uint32_t numBlocks;
    uint32_t familyID;

    // raw data;
    uint8_t data[476];

    // store magic also at the end to limit damage from partial block reads
    uint32_t magicEnd;
} UF2_Block;

typedef struct {
    uint8_t version;
    uint8_t ep_in;
    uint8_t ep_out;
    uint8_t reserved0;
    uint32_t cbw_tag;
    uint32_t blocks_remaining;
    uint8_t *buffer;
} UF2_HandoverArgs;

typedef void (*UF2_MSC_Handover_Handler)(UF2_HandoverArgs *handover);
typedef void (*UF2_HID_Handover_Handler)(int ep);

// this is required to be exactly 16 bytes long by the linker script
typedef struct {
    void *reserved0;
    UF2_HID_Handover_Handler handoverHID;
    UF2_MSC_Handover_Handler handoverMSC;
    const char *info_uf2;
} UF2_BInfo;

#define UF2_BINFO ((UF2_BInfo *)(APP_START_ADDRESS - sizeof(UF2_BInfo)))

static inline bool is_uf2_block(void *data) {
    UF2_Block *bl = (UF2_Block *)data;
    return bl->magicStart0 == UF2_MAGIC_START0 && bl->magicStart1 == UF2_MAGIC_START1 &&
           bl->magicEnd == UF2_MAGIC_END;
}

static inline bool in_uf2_bootloader_space(const void *addr) {
    return USER_FLASH_END <= (uint32_t)addr && (uint32_t)addr < FLASH_SIZE;
}


#ifdef UF2_DEFINE_HANDOVER
static inline const char *uf2_info(void) {
    if (in_uf2_bootloader_space(UF2_BINFO->info_uf2))
        return UF2_BINFO->info_uf2;
    return "N/A";
}

static inline void hf2_handover(uint8_t ep) {
    const char *board_info = UF2_BINFO->info_uf2;
    UF2_HID_Handover_Handler fn = UF2_BINFO->handoverHID;

    if (in_uf2_bootloader_space(board_info) && in_uf2_bootloader_space((const void *)fn) &&
        ((uint32_t)fn & 1)) {
        // Pass control to bootloader; never returns
        fn(ep & 0xf);
    }
}

// the ep_in/ep_out are without the 0x80 mask
// cbw_tag is in the same bit format as it came
static inline void check_uf2_handover(uint8_t *buffer, uint32_t blocks_remaining, uint8_t ep_in,
                                      uint8_t ep_out, uint32_t cbw_tag) {
    if (!is_uf2_block(buffer))
        return;

    const char *board_info = UF2_BINFO->info_uf2;
    UF2_MSC_Handover_Handler fn = UF2_BINFO->handoverMSC;

    if (in_uf2_bootloader_space(board_info) && in_uf2_bootloader_space((const void *)fn) &&
        ((uint32_t)fn & 1)) {
        UF2_HandoverArgs hand = {
            1, ep_in, ep_out, 0, cbw_tag, blocks_remaining, buffer,
        };
        // Pass control to bootloader; never returns
        fn(&hand);
    }
}
#endif

#endif

#ifndef ARRAYSIZE2_H
#define ARRAYSIZE2_H

#ifndef __has_feature
    #define __has_feature(x) 0 // Compatibility with non-clang compilers.
#endif

#if __cplusplus >= 199711L
    #pragma message "using Ivan J. Johnson's ARRAY_SIZE2"

    // Works on older compilers, even Visual C++ 6....
    // Created by Ivan J. Johnson, March 06, 2007
    // See http://drdobbs.com/cpp/197800525?pgno=1
    //
    // Pseudocode:
    // if x is not an array
    //   issue a compile-time error
    // else
    //   use the traditional (non-typesafe) C99 COUNTOF expression
    //
    // If the argument is any of:
    //    object of class type, such as an std::vector
    //    floating-point type
    //    function pointer
    //    pointer-to-member
    // then the first reinterpret_cast<> is not legal (compiler error)
    //
    // The type for check1 is chosen and named to help understand
    // the cause of the error, because the class name is likely to
    // appear in the compiler error message.
    //
    // If check1 succeeds, then the argument must be one of:
    //    an integral type
    //    an enumerated type
    //    a pointer to an object
    //    an array
    //
    // Check2 expands approximately to sizeof(check_type(x, &x)),
    // where check_type is an overloaded function.
    // Because this is purely a compile-time computation,
    // the function is never really called or even implemented,
    // but it lets the compiler apply overload resolution,
    // which allows further type discrimination.
    // There are three possibilities to consider:
    //    x is an integral type or enumerated type.
    //      In this case, neither of the two function overloads
    //      is a match, resulting in a compiler error.
    //    x is a pointer to an object.
    //      In this case, the first argument to check_type()
    //      is a pointer and the second one is a pointer-to-pointer.
    //      The best function match is the first overload of check_type,
    //      the one that returns an incomplete type (Is_pointer).
    //      However, because Is_pointer is an incomplete type,
    //      sizeof(Is_pointer) is not a valid expression,
    //      resulting in a compiler error.
    //    x is an array.
    //      In this case, the first argument to check_type()
    //      is an array and the second is a pointer-to-array.
    //      A pointer-to-array is *NOT* convertible to a
    //      pointer-to-pointer, so the first overload of
    //      check_type() is not a match.
    //      However, an array IS convertible to a pointer,
    //      and a pointer-to-array already is a pointer.
    //      Any pointer is convertible to a void*,
    //      so the second function overload is a match.
    //      That overload returns a complete type (Is_array).
    //      Because it's a complete type,
    //      sizeof(Is_array) is a valid expression.
    // Thus, the compiler has EXCLUDED every possible type
    // except arrays via compilation errors before reaching
    // the third line.
    // Moreover, check1 and check2 are reduced to the value zero,
    // while the third line is the old type-unsafe C-style macro,
    // now made entirely type-safe.
    // 
    // Additional benefits:
    // The result is itself constexpr
    // 
    //
    #define ARRAY_SIZE2(arr) ( \
       0 * sizeof(reinterpret_cast<const ::Bad_arg_to_COUNTOF*>(arr)) + /*check1*/ \
       0 * sizeof(::Bad_arg_to_COUNTOF::check_type((arr), &(arr)))    + /*check2*/ \
       sizeof(arr) / sizeof((arr)[0])                                   /* eval */ \
       )

    struct Bad_arg_to_COUNTOF {
       class Is_pointer; // incomplete
       class Is_array {};
       template <typename T>
       static Is_pointer check_type(const T*, const T* const*);
       static Is_array check_type(const void*, const void*);
    };

#elif __cplusplus >= 201103L ||  /* any compiler claiming C++11 support */ \
    _MSC_VER >= 1900 ||          /* Visual C++ 2015 or higher           */ \
    __has_feature(cxx_constexpr) /* CLang versions supporting constexp  */

    #pragma message "C++11 version ARRAY_SIZE2"

    namespace detail
    {
        template <typename T, std::size_t N>
        constexpr std::size_t countof(T const (&)[N]) noexcept
        {
            return N;
        }
    } // namespace detail
    #define ARRAY_SIZE2(arr) detail::countof(arr)

#elif _MSC_VER // Visual C++ fallback

    #pragma message "using Microsoft Visual C++ intrinsic ARRAY_SIZE2"
    #define ARRAY_SIZE2(arr) _countof(arr)

#elif __cplusplus >= 199711L && ( /* C++ 98 trick */ \
    defined(__INTEL_COMPILER) ||                     \
    defined(__clang__) ||                            \
    (defined(__GNUC__) && (                          \
        (__GNUC__ > 4) ||                            \
        (__GNUC__ == 4 && __GNUC_MINOR__ >= 4)       \
    )))

    #pragma message "C++98 version ARRAY_SIZE2"

    template <typename T, std::size_t N>
    char(&_ArraySizeHelperRequiresArray(T(&)[N]))[N];
    #define ARRAY_SIZE2(x) sizeof(_ArraySizeHelperRequiresArray(x))

#else

    #pragma message "Using type-unsafe version of ARRAY_SIZE2"
    // This is the worst-case scenario macro.
    // While it is valid C, it is NOT typesafe.
    // For example, if the parameter arr is a pointer instead of array,
    // the compiler will SILENTLY give a (likely) incorrect result. 
    #define ARRAY_SIZE2(arr) sizeof(arr) / sizeof(arr[0])

#endif


#endif // ARRAYSIZE2_H


#ifndef COMPILE_DATE_H
#define COMPILE_DATE_H

#define __YEAR_INT__ ((( \
  (__DATE__ [ 7u] - '0')  * 10u + \
  (__DATE__ [ 8u] - '0')) * 10u + \
  (__DATE__ [ 9u] - '0')) * 10u + \
  (__DATE__ [10u] - '0'))

#define __MONTH_INT__ ( \
  (__DATE__ [2u] == 'n' && __DATE__ [1u] == 'a') ?  1u  /*Jan*/ \
: (__DATE__ [2u] == 'b'                        ) ?  2u  /*Feb*/ \
: (__DATE__ [2u] == 'r' && __DATE__ [1u] == 'a') ?  3u  /*Mar*/ \
: (__DATE__ [2u] == 'r'                        ) ?  4u  /*Apr*/ \
: (__DATE__ [2u] == 'y'                        ) ?  5u  /*May*/ \
: (__DATE__ [2u] == 'n'                        ) ?  6u  /*Jun*/ \
: (__DATE__ [2u] == 'l'                        ) ?  7u  /*Jul*/ \
: (__DATE__ [2u] == 'g'                        ) ?  8u  /*Jul*/ \
: (__DATE__ [2u] == 'p'                        ) ?  9u  /*Jul*/ \
: (__DATE__ [2u] == 't'                        ) ? 10u  /*Jul*/ \
: (__DATE__ [2u] == 'v'                        ) ? 11u  /*Jul*/ \
:                                                  12u  /*Dec*/ )

#define __DAY_INT__ ( \
   (__DATE__ [4u] == ' ' ? 0u : __DATE__ [4u] - '0') * 10u \
 + (__DATE__ [5u] - '0')                                   )

#define __DOSDATE__ ( \
	((__YEAR_INT__  - 1980u) << 9u) | \
	( __MONTH_INT__          << 5u) | \
                    ( __DAY_INT__            << 0u) )


#endif // COMPILE_DATE_H

