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

#ifndef X_FILE_INC
#define X_FILE_INC

#include "Object.h"

namespace Torch {

/** XFile. A File which could be anything.
    The syntax (and results) for method is very similar
    to C FILE. (Except for the FILE pointer which is not
    given in the parameters!).

    @author Ronan Collobert (collober@idiap.ch)
*/
class XFile : public Object
{
  public:

    ///
    XFile();

    /// Read something.
    virtual int read(void *ptr, int block_size, int n_blocks) = 0;

    /// Write.
    virtual int write(void *ptr, int block_size, int n_blocks) = 0;

    /** Read and check the tag/the size. To be used with #taggedWrite()#.
        If the tag and the size readed doesn't correspond to the given
        tag and size, an error will occur.
    */
    int taggedRead(void *ptr, int block_size, int n_blocks, const char *tag);

    /// Write and write the tag/the size.
    int taggedWrite(void *ptr, int block_size, int n_blocks, const char *tag);

    /// Are we at the end ?
    virtual int eof() = 0;

    /// Flush the file.
    virtual int flush() = 0;

    /// Seek.
    virtual int seek(long offset, int whence) = 0;

    /// Tell me where am I...
    virtual long tell() = 0;
    
    /// Rewind.
    virtual void rewind() = 0;

    /// Print some text.
    virtual int printf(const char *format, ...) = 0;

    /// Scan some text.
    virtual int scanf(const char *format, void *ptr) = 0;

    /// Get one line (read at most #size_# characters).
    virtual char *gets(char *dest, int size_) = 0;

    //-----

    virtual ~XFile();
};

}

#endif
