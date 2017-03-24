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

#include "DiskXFile.h"
#include "string_utils.h"
#ifdef _MSC_VER
#include <fcntl.h>
#endif

namespace Torch {

bool DiskXFile::is_native_mode = true;

DiskXFile::DiskXFile(const char *file_name, const char *open_flags)
{
#ifdef _MSC_VER
  _fmode = _O_BINARY;
#endif

  its_a_pipe = false;
  bool zipped_file = false;
  if(!strcmp(open_flags, "r"))
  {
    if(strlen(file_name) > 3)
    {
      if(strcmp(file_name+strlen(file_name)-3, ".gz"))
        zipped_file = false;
      else
        zipped_file = true;
    }
    else
      zipped_file = false;

    if(zipped_file)
    {
      char *cmd_buffer = strConcat(2, "zcat ", file_name);
      file = fopen(file_name, "r");
      if(!file)
        error("DiskXFile: cannot open the file <%s> for reading", file_name);
      fclose(file);

      file = popen(cmd_buffer, open_flags);
      if(!file)
        error("DiskXFile: cannot execute the command <%s>", file_name, cmd_buffer);
      free(cmd_buffer);
    }
  }

  if(!zipped_file)
  {
    file = fopen(file_name, open_flags);
    if(!file)
      error("DiskXFile: cannot open <%s> in mode <%s>. Sorry.", file_name, open_flags);
  }
  is_opened = true;

  // Buffer
  buffer_block = NULL;
  buffer_block_size = 0;
}

DiskXFile::DiskXFile(FILE *file_)
{
  file = file_;
  is_opened = false;
  its_a_pipe = false;

  // Buffer
  buffer_block = NULL;
  buffer_block_size = 0;
}

int DiskXFile::read(void *ptr, int block_size, int n_blocks)
{
  int melanie = fread(ptr, block_size, n_blocks, file);

  if(!is_native_mode)
    reverseMemory(ptr, block_size, n_blocks);

  return(melanie);
}

int DiskXFile::write(void *ptr, int block_size, int n_blocks)
{
  if(!is_native_mode)
    reverseMemory(ptr, block_size, n_blocks);

  int melanie = fwrite(ptr, block_size, n_blocks, file);

  if(!is_native_mode)
    reverseMemory(ptr, block_size, n_blocks);

  return(melanie);
}

int DiskXFile::eof()
{
  return(feof(file));
}

int DiskXFile::flush()
{
  return(fflush(file));
}

int DiskXFile::seek(long offset, int whence)
{
  return(fseek(file, offset, whence));
}

long DiskXFile::tell()
{
  return(ftell(file));
}

void DiskXFile::rewind()
{
  ::rewind(file);
}

int DiskXFile::printf(const char *format, ...)
{
  va_list args;
  va_start(args, format);
  int res = vfprintf(file, format, args);
  va_end(args);
  return(res);
}

int DiskXFile::scanf(const char *format, void *ptr)
{
  int res = fscanf(file, format, ptr);
  return(res);
}

char *DiskXFile::gets(char *dest, int size_)
{
  return(fgets(dest, size_, file));
}

//-----

bool DiskXFile::isLittleEndianProcessor()
{
  int x = 7;
  char *ptr = (char *)&x;

  if(ptr[0] == 0)
    return(false);
  else
    return(true);
}

bool DiskXFile::isBigEndianProcessor()
{
  return(!isLittleEndianProcessor());
}

bool DiskXFile::isNativeMode()
{
  return(is_native_mode);
}

void DiskXFile::setNativeMode()
{
  is_native_mode = true;
}

void DiskXFile::setLittleEndianMode()
{
  if(isLittleEndianProcessor())
    is_native_mode = true;
  else
    is_native_mode = false;
}

void DiskXFile::setBigEndianMode()
{
  if(isBigEndianProcessor())
    is_native_mode = true;
  else
    is_native_mode = false;
}

void DiskXFile::reverseMemory(void *ptr_, int block_size, int n_blocks)
{
  if(block_size == 1)
    return;

  char *ptr = (char *)ptr_;
  char *ptrr, *ptrw;

  if(block_size > buffer_block_size)
  {
    allocator->free(buffer_block);
    buffer_block = (char *)allocator->alloc(block_size);
  }

  for(int i = 0; i < n_blocks; i++)
  {
    ptrr = ptr + ((i+1)*block_size);
    ptrw = buffer_block;

    for(int j = 0; j < block_size; j++)
    {
      ptrr--;
      *ptrw++ = *ptrr;
    }

    ptrr = buffer_block;
    ptrw = ptr + (i*block_size);
    for(int j = 0; j < block_size; j++)
      *ptrw++ = *ptrr++;
  }
}

//-----

DiskXFile::~DiskXFile()
{
  if(is_opened)
  {
    if(its_a_pipe)
      pclose(file);
    else
      fclose(file);
  }
}

}
