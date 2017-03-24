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

#ifndef OBJECT_INC
#define OBJECT_INC

#include "general.h"
#include "Allocator.h"

namespace Torch {

//-----

class XFile;

typedef struct Option_
{
    char *name;
    char *help;
    int size;
    void *ptr;
} Option;

//-----

/** Almost all classes in Torch should be a sub-class of this class.
    It provides two interesting things: first a useful interface to
    manage option, and second the "allocator trick".
    
    Indeed, in Torch, a class which allocate memory should desallocate
    this memory when it's destroyed. The idea is the following: in the
    constructor of Object, an Allocator object is created. You can use
    it in your subclasses to allocate memory. This memory will be destroyed
    when the Object is destroyed.

    "new" operator have been defined to be used with Allocator.

    @see Allocator
    @author Ronan Collobert (collober@idiap.ch)
*/
class Object
{
  public:
    int n_options;
    Option *options;

    /// Allocator associated to the Object.
    Allocator *allocator;

    ///
    Object();
    
    //-----

    /** Add the option #name#.
        This one has a pointer on #ptr# and has the size #size#.
        You can provide an help in #help#.
        (Note that if #help# is empty, its an empty string and not NULL).
        If the option can be changed at any time, you can set
        #is_allowed_after_init# to #true#.
    */
    void addOption(const char *name, int size, void *ptr, const char *help="");

    /** Several wrappers of #addOption()# for
        Int, Real and Bool options.
        Be carefull: there is no type checking.
    */
    void addIOption(const char *name, int *ptr,  int init_value,  const char *help="");

    ///
    void addROption(const char *name, real *ptr, real init_value, const char *help="");

    ///
    void addBOption(const char *name, bool *ptr, bool init_value, const char *help="");

    ///
    void addOOption(const char *name, Object **ptr, Object *init_value, const char *help="");

    /// Set the option #name# to the value contained at #ptr#.
    void setOption(const char *name, void *ptr);

    /** Several wrappers of #setOption()# for
        Int, Real and Bool options.
        Be carefull: there is no type checking.
    */
    void setIOption(const char *name, int option);

    ///
    void setROption(const char *name, real option);

    ///
    void setBOption(const char *name, bool option);

    ///
    void setOOption(const char *name, Object *option);

    /// Load the object from a file pointer (\emph{not the options})
    virtual void loadXFile(XFile *file);

    /// Save the object to a file pointer (\emph{not the options})
    virtual void saveXFile(XFile *file);

    /// Load the machine from a file name (\emph{not the options})
    void load(const char *filename);

    /// Save the machine to a file name (\emph{not the options})
    void save(const char *filename);

    /** Allocate the object using #allocator_#. The object will be
        destroyed and freed when the allocator will be destroyed.
        If no allocator is provided, the new will be similar than
        the standard new.
    */
    void* operator new(size_t size, Allocator *allocator_=NULL);

    /** Allocate the object using the memory given by #ptr_#.
        The object will be destroyed (but not freed!) when the
        allocator will be destroyed.
    */
    void* operator new(size_t size, Allocator *allocator_, void *ptr_);

    /// Delete an object.
    void  operator delete(void *ptr);

    //-----

    virtual ~Object();
};

}

#endif
