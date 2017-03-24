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

#ifndef ALLOCATOR_INC
#define ALLOCATOR_INC

#include "general.h"

namespace Torch {

struct AllocatorNode
{
    void *ptr;
    int object_style;
    AllocatorNode *prev;
    AllocatorNode *next;
};

/** Class do easily allocate/deallocate memory in Torch.
    The memory allocated by an allocator will be destroyed
    when the allocator will be destroyed.

    @see Object
    @author Ronan Collobert (collober@idiap.ch)
*/
class Allocator
{
  public:
    AllocatorNode *ptrs;

    /// Create a new allocator.
    Allocator();

    /** Returns #size# bytes of allocated memory.
        \begin{itemize}
          \item If #object_style# is 0, the allocated memory is
          considered as if it wasn't for an object. At the destruction
          of the allocator the memory will be freed, and that's all.
          \item If 1, the #Object# destructor will be called
          at the destruction and the memory will be freed.
          \item If 2, the destructor will be called, but the memory
          won't be freed.
        \end{itemize}
    */
    void *alloc(size_t size, int object_style=0);

    /** Reallocate a part of the memory which has been already
        allocated with alloc (and #object_style#=0).
        Same behaviour as the #realloc()# system function.
    */
    void *realloc(void *ptr, size_t size);

    /** Force given pointer to memory to be freed now.
        It considers the #object_style# given by #alloc()#
        and calls the Object destructor, if needed.
    */
    void free(void *ptr);

    /** Tells to the allocator that it should handle the memory
        given by #ptr#. Take in account the #object_style#.
    */
    void retain(void *ptr, int object_style=0);

    /** Tells to the allocator to stop taking in account the
        memory given by #ptr#. The memory will not be released.
     */
    int release(void *ptr);

    /** Handles the memory given by #ptr# which was previouly
        handled by #allocator#. #allocator# looses the control
        of this memory */
    void steal(void *ptr, Allocator *allocator);

    /// Steals all pointers contained in #allocator#.
    void steal(Allocator *allocator);

    /// Returns true iff ptr is handled by the allocator.
    AllocatorNode *isMine(void *ptr);

    /** Force all pointers contained in the allocator to be freed now.
        It considers the #object_style# given by #alloc()#
        and calls the Object destructor, if needed.
    */
    void freeAll();

    /** System allocation.
        As system malloc function, but do an error if there is no more memory.
    */
    static void *sysAlloc(int size);

    /** System reallocation.
        As system realloc function, but do an error if there is no more memory.
    */
    static void *sysRealloc(void *ptr, int size);

    ~Allocator();
};

}

#endif
