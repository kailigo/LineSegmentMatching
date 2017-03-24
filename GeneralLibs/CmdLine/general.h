// Copyright (C) 2003--2004 Ronan Collobert (collober@idiap.ch)
//                
// This file is part of Torch 3.1.
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. The name of the author may not be used to endorse or promote products
//    derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
// OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef GENERAL_INC
#define GENERAL_INC

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <limits.h>
#include <stdarg.h>
#include <time.h>
#include <float.h>

// Old systems need that to define FLT_MAX and DBL_MAX
#ifndef DBL_MAX
#include <values.h>
#endif

namespace Torch {

#ifdef _MSC_VER
#ifndef for
#define for if (0) {} else for
#endif
#define M_PI 3.14159265358979323846
#define popen(x,y) _popen(x,y)
#define pclose(x) _pclose(x)
#define isnan(x) _isnan(x)
#define log1p(x) log(1+(x))
#endif

#ifdef USE_DOUBLE
#define INF DBL_MAX
#define REAL_EPSILON DBL_EPSILON
#define real double
#else
#define INF FLT_MAX
#define REAL_EPSILON FLT_EPSILON
#define real float
#endif

//-----------------------------------

/** Text outputs functions.

    These functions are like #printf()#, but you should
    use them instead. Note that you should never try to
    print a message larger than 10000 characters.

    @author Ronan Collobert (collober@idiap.ch)
 */
//@{

/// Print an error message. The program will exit.
void error(const char* msg, ...);
/// Print a warning message.
void warning(const char* msg, ...);
/// Print a message.
void message(const char* msg, ...);
/// Like printf.
void print(const char* msg, ...);

/** Print a control bar [\#\#\#\#\#\#\#\#\#\#].
    
    First time, you can it with #level=-1#.
    It'll print the control bar at each time
    you will call that.

    #max_level# is the value of the last #level#
    you'll call this function.

    @author Ronan Collobert (collober@idiap.ch)
 */
void controlBar(int level, int max_level);
//@}

//-----------------------------------

#ifndef min
/// The min function
#define	min(a,b) ((a) > (b) ? (b) : (a))
#endif

#ifndef max
/// The max function
#define	max(a,b) ((a) > (b) ? (a) : (b))
#endif

}

#endif
