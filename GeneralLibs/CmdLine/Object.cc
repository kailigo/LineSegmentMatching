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

#include "Object.h"
#include "DiskXFile.h"
#include "XFile.h"

namespace Torch {

Object::Object()
{
  n_options = 0;
  options = NULL;
  allocator = new Allocator;
}

void Object::addOption(const char *name, int size, void *ptr, const char *help)
{
  options = (Option *)allocator->realloc((void *)options, (n_options+1)*sizeof(Option));

  Option *optr = options+n_options;

  optr->name = (char *)allocator->alloc(strlen(name)+1);
  optr->help = (char *)allocator->alloc(strlen(help)+1);
  strcpy(optr->name, name);
  strcpy(optr->help, help);
  optr->size = size;
  optr->ptr = ptr;
  n_options++;
}

void Object::addIOption(const char *name, int *ptr,  int init_value,  const char *help)
{
  *ptr = init_value;
  addOption(name, sizeof(int), ptr, help);
}

void Object::addROption(const char *name, real *ptr, real init_value, const char *help)
{
  *ptr = init_value;
  addOption(name, sizeof(real), ptr, help);
}

void Object::addBOption(const char *name, bool *ptr, bool init_value, const char *help)
{
  *ptr = init_value;
  addOption(name, sizeof(bool), ptr, help);
}

void Object::addOOption(const char *name, Object **ptr, Object *init_value, const char *help)
{
  *ptr = init_value;
  addOption(name, sizeof(Object *), ptr, help);  
}

void Object::setOption(const char *name, void *ptr)
{
  Option *optr = options;

  bool flag = false;
  for(int i = 0; i < n_options; i++, optr++)
  {
    if( !strcmp(optr->name, name) )
    {
      flag = true;
      break;
    }
  }

  if(!flag)
    error("Object: option doesn't exist: %s", name);

  char *odata = (char *)optr->ptr;
  char *odatao = (char *)ptr;

  for(int i = 0; i < optr->size; i++)
  {
    *odata = *odatao;
    odata++;
    odatao++;
  }
}

void Object::setIOption(const char *name, int option)
{
  setOption(name, (void *)&option);
}

void Object::setROption(const char *name, real option)
{
  setOption(name, (void *)&option);
}

void Object::setBOption(const char *name, bool option)
{
  setOption(name, (void *)&option);
}

void Object::setOOption(const char *name, Object *option)
{
  setOption(name, (void *)&option);
}

void Object::load(const char *filename)
{
  DiskXFile file(filename, "r");
  loadXFile(&file);
}

void Object::save(const char *filename)
{
  DiskXFile file(filename, "w");
  saveXFile(&file);
}

void Object::loadXFile(XFile *file)
{
}

void Object::saveXFile(XFile *file)
{
}

void* Object::operator new(size_t size, Allocator *allocator_)
{
  if(allocator_)
    return(allocator_->alloc(size, 1));
  else
    return(Allocator::sysAlloc(size));
}

void* Object::operator new(size_t size, Allocator *allocator_, void *ptr_)
{
  allocator_->retain(ptr_, 2);
  return ptr_;
}

void Object::operator delete(void *ptr)
{
  free(ptr);
}

Object::~Object()
{
  delete allocator;
}

}
