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

#include "Allocator.h"
#include "Object.h"

namespace Torch {

Allocator::Allocator()
{
  ptrs = NULL;
}

void *Allocator::alloc(size_t size, int object_style)
{
  // Are you stupid ?
  if(size <= 0)
    return(NULL);

  // Allocate what you need
  void *ptr = sysAlloc(size);
  if(!ptr)
    error("Allocator: not enough memory. Buy new ram.");

  // Save the pointer
  retain(ptr, object_style);
  return(ptr);
}

void *Allocator::realloc(void *ptr, size_t size)
{
  // Free it ?
  if(size <= 0)
  {
    Allocator::free(ptr);
    return NULL;
  }
  
  // Find the node
  bool is_mine = false;
  AllocatorNode *ptrs_ = ptrs;
  
  if(ptr == NULL)
    is_mine = true;
  else
  {    
    while(ptrs_)
    {
      if(ptrs_->ptr == ptr)
      {
        is_mine = true;
        break;
      }
      ptrs_ = ptrs_->next;
    }
  }

  if(!is_mine)
    error("Allocator: cannot realloc a pointer which is not mine.");

  if(ptr)
  {
    // Reallocate (Here, size > 0 for sure)
    void *ptrx = sysRealloc(ptr, size);
    if(!ptrx)
      error("Allocator: not enough memory. Buy new ram.");
    
    // Save the new pointer
    ptrs_->ptr = ptrx;
    return(ptrx);
  }
  else
  {
    // Forcement pas un objet
    return(this->alloc(size, 0));
  }
}

void Allocator::free(void *ptr)
{
  // Gni?
  if(!ptr)
    return;
  
  // Release the pointer
  int object_style = release(ptr);

  // Free it
//  message("Allocator [%p] free %p mode %d", this, ptr, object_style);
  if(object_style == 0)
    ::free(ptr);
  else
  {
    if(object_style == 1)
      delete (Object *)ptr;
    else
      ((Object *)ptr)->~Object();
  }
}

void Allocator::retain(void *ptr, int object_style)
{
#ifdef DEBUG
  AllocatorNode *dbg_ptr = isMine(ptr);
  if(dbg_ptr)
  {
    if(object_style != 2)
      error("Allocator [debug mode]: try to retain a previously retained pointer! You'll destruct an inexistant object.");    
    if(object_style == dbg_ptr->object_style)
      error("Allocator [debug mode]: try to retain a previously retained pointer with same mode [%d]!", object_style);
  }
#endif

  // Create a new node to be placed *before* the root
  AllocatorNode *ptrs_ = (AllocatorNode *)sysAlloc(sizeof(AllocatorNode));
  if(!ptrs_)
    error("Allocator: not enough memory. Buy new ram.");
  ptrs_->prev = NULL;
  ptrs_->next = ptrs;
  if(ptrs)
    ptrs->prev = ptrs_;
  
  // Save the root
  ptrs = ptrs_;

  // Save the pointer
  ptrs->ptr = ptr;
  ptrs->object_style = object_style;
}

int Allocator::release(void *ptr)
{
  // Find the node (Note: Start with the beginning... recent is faster!)
  bool is_mine = false;
  AllocatorNode *ptrs_ = ptrs;
  while(ptrs_)
  {
    if(ptrs_->ptr == ptr)
    {
      is_mine = true;
      break;
    }
    ptrs_ = ptrs_->next;
  }

  if(!is_mine)
    error("Allocator: cannot release a pointer which is not mine.");

  // Check the links
  if(ptrs_->next)
    ptrs_->next->prev = ptrs_->prev;

  if(ptrs_->prev)
    ptrs_->prev->next = ptrs_->next;
  else
    // Viens-t-on de scrapper le root ?
    ptrs = ptrs_->next;

  // Free the node and return if object or not
  int object_style = ptrs_->object_style;
  ::free(ptrs_);
  return(object_style);
}

void Allocator::steal(void *ptr, Allocator *allocator)
{
  int object_style = allocator->release(ptr);
  retain(ptr, object_style);
}

void Allocator::steal(Allocator *allocator)
{
  while(allocator->ptrs)
    steal(allocator->ptrs->ptr, allocator);
}

AllocatorNode *Allocator::isMine(void *ptr)
{
  AllocatorNode *ptrs_ = ptrs;
  while(ptrs_)
  {
    if(ptrs_->ptr == ptr)
      return ptrs_;
    ptrs_ = ptrs_->next;
  }

  return NULL;
}

void Allocator::freeAll()
{
  while(ptrs)
    this->free(ptrs->ptr);
}


void *Allocator::sysAlloc(int size)
{
  if(size <= 0)
    return(NULL);
  void *ptr = malloc(size);
  if(!ptr)
    error("Allocator: not enough memory. Buy new ram");
  return(ptr);
}

void *Allocator::sysRealloc(void *ptr, int size)
{
  void *ptr_ = NULL;
  if(size <= 0)
    ::free(ptr);
  else
  {
    ptr_ = ::realloc(ptr, size);
    if(!ptr_)
      error("Allocator: not enough memory. Buy new ram");
  }
  return(ptr_);
}

Allocator::~Allocator()
{
  freeAll();
}

}
