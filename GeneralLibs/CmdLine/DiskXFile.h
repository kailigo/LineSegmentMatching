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

#ifndef DISK_X_FILE_INC
#define DISK_X_FILE_INC

#include "XFile.h"

namespace Torch {

/** A file on the disk.
    
    @author Ronan Collobert (collober@idiap.ch)
*/
class DiskXFile : public XFile
{
  private:
    static bool is_native_mode;
    char *buffer_block;
    int buffer_block_size;
    void reverseMemory(void *ptr_, int block_size, int n_blocks);
    
  public:
    FILE *file;
    bool is_opened;
    bool its_a_pipe;

    /// Open "file_name" with the flags #open_flags#
    DiskXFile(const char *file_name, const char *open_flags);

    /// Use the given file...
    DiskXFile(FILE *file_);

    //-----

    /// Returns #true# if the processor uses the little endian coding format.
    static bool isLittleEndianProcessor();

    /// Returns #true# if the processor uses the big endian coding format.
    static bool isBigEndianProcessor();

    /// Returns #true# if we'll load/save using the native mode.
    static bool isNativeMode();

    /** We'll load/save using native mode.
        We use little endian iff the computer uses little endian.
        We use big endian iff the computer uses big endian.
    */
    static void setNativeMode();

    /** We'll load/save using little endian mode.
        It means that if the computer doesn't use Little Endian,
        data will be converted.
    */
    static void setLittleEndianMode();

    /** We'll load/save using big endian mode.
        It means that if the computer doesn't use Big Endian,
        data will be converted.
    */
    static void setBigEndianMode();

    //-----
    
    virtual int read(void *ptr, int block_size, int n_blocks);
    virtual int write(void *ptr, int block_size, int n_blocks);
    virtual int eof();
    virtual int flush();
    virtual int seek(long offset, int whence);
    virtual long tell();
    virtual void rewind();
    virtual int printf(const char *format, ...);
    virtual int scanf(const char *format, void *ptr) ;
    virtual char *gets(char *dest, int size_);

    //-----

    virtual ~DiskXFile();
};

}

#endif
