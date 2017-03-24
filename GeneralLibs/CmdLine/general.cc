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

#include "general.h"

namespace Torch {

char xxpetit_message_pour_melanie[10000];

void error(const char* msg, ...)
{
  va_list args;
  va_start(args,msg);
  vsprintf(xxpetit_message_pour_melanie, msg, args);
  printf("\n$ Error: %s\n\n", xxpetit_message_pour_melanie);
  fflush(stdout);
  va_end(args);
  exit(-1);
}

void warning(const char* msg, ...)
{
  va_list args;
  va_start(args,msg);
  vsprintf(xxpetit_message_pour_melanie, msg, args);
  printf("! Warning: %s\n", xxpetit_message_pour_melanie);
  fflush(stdout);
  va_end(args);
}

void message(const char* msg, ...)
{
  va_list args;
  va_start(args,msg);
  vsprintf(xxpetit_message_pour_melanie, msg, args);
  printf("# %s\n", xxpetit_message_pour_melanie);
  fflush(stdout);
  va_end(args);
}

void print(const char* msg, ...)
{
  va_list args;
  va_start(args,msg);
  vsprintf(xxpetit_message_pour_melanie, msg, args);
  printf("%s", xxpetit_message_pour_melanie);
  fflush(stdout);
  va_end(args);
}

void controlBar(int level, int max_level)
{
  if(level == -1)
    print("[");
  else
  {
    if(max_level < 10)
      print(".");
    else
    {
      if( !(level % (max_level/10) ) )
        print(".");
    }
  
    if(level == max_level-1)
      print("]\n");
  }
}

}
