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

#include "FileListCmdOption.h"
#include "DiskXFile.h"

namespace Torch {

FileListCmdOption::FileListCmdOption(const char *name_, const char *help_, bool save_)
  : CmdOption(name_, "<[-one_file] file_name>", help_, save_)
{
  n_files = 0;
  file_names = NULL;
}

void FileListCmdOption::read(int *argc_, char ***argv_)
{
  char **argv = *argv_;

  if(*argc_ == 0)
    error("FileListCmdOption: cannot correctly set <%s>", name);

  // Special case...
  if(!strcmp("-one_file", argv[0]))
  {
    (*argc_)--;
    (*argv_)++;

    argv = *argv_;
    if(*argc_ == 0)
      error("FileListCmdOption: cannot correctly set <%s>", name);
    
    n_files = 1;
    file_names = (char **)allocator->alloc(sizeof(char *));
    file_names[0] = (char *)allocator->alloc(strlen(argv[0])+1);
    strcpy(file_names[0], argv[0]);

    (*argc_)--;
    (*argv_)++;
    return;
  }

  /// Read the contents of the file...
  DiskXFile file_(argv[0], "r");

  char *melanie = (char *)allocator->alloc(1024);
  file_.read(melanie, 1, 1024);
  melanie[1023] = '\0';
  file_.rewind();

  char* endp_;
  strtol(melanie, &endp_, 10);
  if( (*endp_ != '\0') && (*endp_ != '\n') )
  {
    do
    {
      file_.gets(melanie, 1024);
      n_files++;
    } while (!file_.eof());
    n_files--;
    file_.rewind();
  }
  else
    file_.scanf("%d", &n_files);

  message("FileListCmdOption: %d files detected", n_files);

  file_names = (char **)allocator->alloc(sizeof(char *)*n_files);
  for(int i = 0; i < n_files; i++)
  {
    file_.scanf("%s", melanie);
    file_names[i] = (char *)allocator->alloc(strlen(melanie)+1);
    strcpy(file_names[i], melanie);
  }

  allocator->free(melanie);

  ////////////////////////////////////

  (*argc_)--;
  (*argv_)++;
}

void FileListCmdOption::loadXFile(XFile *file)
{
  file->taggedRead(&n_files, sizeof(int), 1, "NFILES");
  file_names = (char **)allocator->alloc(sizeof(char *)*n_files);  
  for(int i = 0; i < n_files; i++)
  {
    int melanie;
    file->taggedRead(&melanie, sizeof(int), 1, "SIZE");
    file_names[i] = (char *)allocator->alloc(melanie);
    file->taggedRead(file_names[i], 1, melanie, "FILE");
  }
}

void FileListCmdOption::saveXFile(XFile *file)
{
  file->taggedWrite(&n_files, sizeof(int), 1, "NFILES");
  for(int i = 0; i < n_files; i++)
  {
    int melanie = strlen(file_names[i])+1;
    file->taggedWrite(&melanie, sizeof(int), 1, "SIZE");
    file->taggedWrite(file_names[i], 1, melanie, "FILE");
  }
}

FileListCmdOption::~FileListCmdOption()
{
}

}
