/**

The MIT License (MIT)

Copyright (c)

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
: (__DATE__ [2u] == 'g'                        ) ?  8u  /*Aug*/ \
: (__DATE__ [2u] == 'p'                        ) ?  9u  /*Sep*/ \
: (__DATE__ [2u] == 't'                        ) ? 10u  /*Oct*/ \
: (__DATE__ [2u] == 'v'                        ) ? 11u  /*Nov*/ \
:                                                  12u  /*Dec*/ )

#define __DAY_INT__ ( \
   (__DATE__ [4u] == ' ' ? 0u : __DATE__ [4u] - '0') * 10u \
 + (__DATE__ [5u] - '0')                                   )

// __TIME__ expands to an eight-character string constant
// "23:59:01", or (if cannot determine time) "??:??:??" 
#define __HOUR_INT__ ( \
   (__TIME__ [0u] == '?' ? 0u : __TIME__ [0u] - '0') * 10u \
 + (__TIME__ [1u] == '?' ? 0u : __TIME__ [1u] - '0')       )

#define __MINUTE_INT__ ( \
   (__TIME__ [3u] == '?' ? 0u : __TIME__ [3u] - '0') * 10u \
 + (__TIME__ [4u] == '?' ? 0u : __TIME__ [4u] - '0')       )

#define __SECONDS_INT__ ( \
   (__TIME__ [6u] == '?' ? 0u : __TIME__ [6u] - '0') * 10u \
 + (__TIME__ [7u] == '?' ? 0u : __TIME__ [7u] - '0')       )


#define __DOSDATE__ ( \
	((__YEAR_INT__  - 1980u) << 9u) | \
	( __MONTH_INT__          << 5u) | \
	( __DAY_INT__            << 0u) )

#define __DOSTIME__ ( \
	( __HOUR_INT__    << 11u) | \
	( __MINUTE_INT__  <<  5u) | \
	( __SECONDS_INT__ <<  0u) )

#endif // COMPILE_DATE_H

