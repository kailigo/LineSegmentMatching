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

#include "XFile.h"

namespace Torch {

XFile::XFile()
{
}

int XFile::taggedRead(void *ptr, int block_size, int n_blocks, const char *tag)
{
  int tag_size;
  this->read(&tag_size, sizeof(int), 1);
  if(tag_size != (int)strlen(tag))
    error("XFile: sorry, the tag <%s> cannot be read!", tag);

  char *tag_ = (char *)Allocator::sysAlloc(tag_size+1);
  tag_[tag_size] = '\0';
  this->read(tag_, 1, tag_size);

  if(strcmp(tag, tag_))
    error("XFile: tag <%s> not found!", tag);
  free(tag_);

  int block_size_;
  int n_blocks_;
  this->read(&block_size_, sizeof(int), 1);
  this->read(&n_blocks_, sizeof(int), 1);

  if( (block_size_ != block_size) || (n_blocks_ != n_blocks) )
    error("XFile: tag <%s> has a corrupted size!", tag);

  return this->read(ptr, block_size, n_blocks);
}

int XFile::taggedWrite(void *ptr, int block_size, int n_blocks, const char *tag){
  int tag_size = strlen(tag);
  this->write(&tag_size, sizeof(int), 1);
  this->write((char *)tag, 1, tag_size);
  this->write(&block_size, sizeof(int), 1);
  this->write(&n_blocks, sizeof(int), 1);
  return this->write(ptr, block_size, n_blocks);
}

XFile::~XFile()
{
}

}
