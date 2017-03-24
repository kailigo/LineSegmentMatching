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

#ifndef FILE_LIST_CMD_OPTION_INC
#define FILE_LIST_CMD_OPTION_INC

#include "CmdOption.h"

namespace Torch {

/** This class take a file name in the command line,
    and reads a list of files contained in this
    file.

    In fact, there is a special case: it checks first
    if "-one_file" the current argument on the command
    line. If true, then it reads the next argument which
    will be the only file in the list.

    @author Ronan Collobert (collober@idiap.ch)
    @see CmdLine
*/
class FileListCmdOption : public CmdOption
{
  public:
    /// Contains the file names after reading the command line.
    char **file_names;

    /// Number of files that have been read.
    int n_files;

    ///
    FileListCmdOption(const char *name_, const char *help_="", bool save_=false);

    virtual void read(int *argc_, char ***argv_);
    virtual void loadXFile(XFile *file);
    virtual void saveXFile(XFile *file);
    ~FileListCmdOption();
};

}

#endif
