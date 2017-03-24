// Copyright (C) 2003--2004 Johnny Mariethoz (Johnny.Mariethoz@idiap.ch)
//                and Ronan Collobert (collober@idiap.ch)
//                and Samy Bengio (bengio@idiap.ch)
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

#ifndef STRING_UTILS_INC
#define STRING_UTILS_INC

#include <string.h>

namespace Torch {

/** Some simple functions for string operations.

    @author Samy Bengio (bengio@idiap.ch)
    @author Johnny Mariethoz (Johnny.Mariethoz@idiap.ch)
    @author Ronan Collobert (collober@idiap.ch)
*/
//@{
/** Returns the name of a file without leading pathname.
    (It's not a new string, but a pointer in the given string)
 */
char *strBaseName(char *filename);

/** Returns a fresh copy of the name of a file without suffix.
    (Trailing chars after c) You have to free the memory!
*/
char *strRemoveSuffix(char *filename, char c='.');

/** Returns the concatenation #n# strings.
    The strings are the parameters given after #n#;
    You have to free the memory!
*/
char *strConcat(int n, ...);

//@}

}

#endif
