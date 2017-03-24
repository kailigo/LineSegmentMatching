// Copyright (C) 2003--2004 Johnny Mariethoz (Johnny.Mariethoz@idiap.ch)
//                and Ronan Collobert (collober@idiap.ch)
//                and Samy Bengio (bengio@idiap.ch)
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

#include "general.h"
#include "string_utils.h"
#include "Allocator.h"
#include <stdarg.h>

namespace Torch {

char *strBaseName(char *filename) {
   char *p = strrchr(filename, '/');
   return p ? (p+1) : filename;
}

char *strRemoveSuffix(char *filename, char c)
{
  char *copy = NULL;
  int len = strlen(filename);
  char *p = filename + len - 1;
  int i=len-1;
  while (*p != c && i-- >0) p--;
  if (i>0) {
    //*p = '\0';
    copy = (char*)Allocator::sysAlloc(sizeof(char)*(i+1));
    strncpy(copy,filename,i);
    copy[i] = '\0';
  } else {
    copy = (char*)Allocator::sysAlloc(sizeof(char)*(len+1));
    strcpy(copy,filename);
  }
  return copy;
}

char *strConcat(int n, ...)
{
  char **strs = (char **)Allocator::sysAlloc(sizeof(char *)*n);

  int taille = 0;
  va_list args;
  va_start(args, n);
  for(int i = 0; i < n; i++)
  {
    strs[i] = va_arg(args, char *);
    taille += strlen(strs[i]);
  }
  va_end(args);
  taille++; // Pour le truc de fin

  char *the_concat = (char *)Allocator::sysAlloc(sizeof(char)*taille);
  the_concat[0] = '\0';

  for(int i = 0; i < n; i++)
    strcat(the_concat, strs[i]);

  free(strs);

  return(the_concat);
}

}

