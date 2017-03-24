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

#include "CmdLine.h"
#include <time.h>

namespace Torch {

// Oy. J'ai fait le menage dans c'te classe.
// Pleins de features non documentees dans le tutorial!

CmdLine::CmdLine()
{
  n_master_switches = 1; // the default!
  n_cmd_options = (int *)allocator->alloc(sizeof(int));
  cmd_options = (CmdOption ***)allocator->alloc(sizeof(CmdOption **));
  n_cmd_options[0] = 0;
  cmd_options[0] = NULL;
  text_info = NULL;
  working_directory = (char *)allocator->alloc(2);
  strcpy(working_directory, ".");
  associated_files = NULL;
  n_associated_files = 0;
  master_switch = -1;
  program_name = (char *)allocator->alloc(1);
  *program_name = '\0';

  addBOption("write log", &write_log, true, "Should I output the cmd.log file ?");
}

void CmdLine::info(const char *text)
{
  if(text_info)
    allocator->free(text_info);

  text_info = (char *)allocator->alloc(strlen(text)+1);

  strcpy(text_info, text);
}

void CmdLine::addCmdOption(CmdOption *option)
{
  if(option->isMasterSwitch())
  {
    n_cmd_options = (int *)allocator->realloc(n_cmd_options, sizeof(int)*(n_master_switches+1));
    cmd_options = (CmdOption ***)allocator->realloc(cmd_options, sizeof(CmdOption **)*(n_master_switches+1));
    n_cmd_options[n_master_switches] = 0;
    cmd_options[n_master_switches] = NULL;
    n_master_switches++;
  }

  int n = n_master_switches-1;
  cmd_options[n] = (CmdOption **)allocator->realloc(cmd_options[n], (n_cmd_options[n]+1)*sizeof(CmdOption *));
  cmd_options[n][n_cmd_options[n]] = option;
  
  n_cmd_options[n]++;
}

void CmdLine::addMasterSwitch(const char *text)
{
  CmdOption *option = new(allocator) CmdOption(text, "", "", false);
  option->isMasterSwitch(true);
  addCmdOption(option);
}

void CmdLine::addICmdOption(const char *name, int *ptr, int init_value, const char *help, bool save_it)
{
  IntCmdOption *option = new(allocator) IntCmdOption(name, ptr, init_value, help, save_it);
  addCmdOption(option);
}

void CmdLine::addBCmdOption(const char *name, bool *ptr, bool init_value, const char *help, bool save_it)
{
  BoolCmdOption *option = new(allocator) BoolCmdOption(name, ptr, init_value, help, save_it);
  addCmdOption(option);
}

void CmdLine::addRCmdOption(const char *name, real *ptr, real init_value, const char *help, bool save_it)
{
  RealCmdOption *option = new(allocator) RealCmdOption(name, ptr, init_value, help, save_it);
  addCmdOption(option);
}

void CmdLine::addSCmdOption(const char *name, char **ptr, const char *init_value, const char *help, bool save_it)
{
  StringCmdOption *option = new(allocator) StringCmdOption(name, ptr, init_value, help, save_it);
  addCmdOption(option);
}

void CmdLine::addICmdArg(const char *name, int *ptr, const char *help, bool save_it)
{
  IntCmdOption *option = new(allocator) IntCmdOption(name, ptr, 0, help, save_it);
  option->isArgument(true);
  addCmdOption(option);
}

void CmdLine::addBCmdArg(const char *name, bool *ptr, const char *help, bool save_it)
{
  BoolCmdOption *option = new(allocator) BoolCmdOption(name, ptr, false, help, save_it);
  option->isArgument(true);
  addCmdOption(option);
}

void CmdLine::addRCmdArg(const char *name, real *ptr, const char *help, bool save_it)
{
  RealCmdOption *option = new(allocator) RealCmdOption(name, ptr, 0., help, save_it);
  option->isArgument(true);
  addCmdOption(option);
}

void CmdLine::addSCmdArg(const char *name, char **ptr, const char *help, bool save_it)
{
  StringCmdOption *option = new(allocator) StringCmdOption(name, ptr, "", help, save_it);
  option->isArgument(true);
  addCmdOption(option);
}

void CmdLine::addText(const char *text)
{
  CmdOption *option = new(allocator) CmdOption(text, "", "", false);
  option->isText(true);
  addCmdOption(option);
}

int CmdLine::read(int argc_, char **argv_)
{
  allocator->free(program_name);
  program_name = (char *)allocator->alloc(strlen(argv_[0])+1);
  strcpy(program_name, argv_[0]);
  argv = argv_+1;
  argc = argc_-1;
  
  // Look for help request and the Master Switch
  master_switch = 0;
  if(argc >= 1)
  {
    if( ! (strcmp(argv[0], "-h") && strcmp(argv[0], "-help") && strcmp(argv[0], "--help")) )
      help();

    for(int i = 1; i < n_master_switches; i++)
    {
      if(cmd_options[i][0]->isCurrent(&argc, &argv))
      {
        master_switch = i;
        break;
      }
    }
  }
  
  CmdOption **cmd_options_ = cmd_options[master_switch];
  int n_cmd_options_ = n_cmd_options[master_switch];

  // Initialize the options.
  for(int i = 0; i < n_cmd_options_; i++)
    cmd_options_[i]->initValue();

  while(argc > 0)
  {
    // First, check the option.
    int current_option = -1;    
    for(int i = 0; i < n_cmd_options_; i++)
    {
      if(cmd_options_[i]->isCurrent(&argc, &argv))
      {
        current_option = i;
        break;
      }
    }

    if(current_option >= 0)
    {
      if(cmd_options_[current_option]->is_setted)
        error("CmdLine: option %s is setted twice", cmd_options_[current_option]->name);
      cmd_options_[current_option]->read(&argc, &argv);
      cmd_options_[current_option]->is_setted = true;
    }
    else
    {
      // Check for arguments
      for(int i = 0; i < n_cmd_options_; i++)
      {
        if(cmd_options_[i]->isArgument() && (!cmd_options_[i]->is_setted))
        {
          current_option = i;
          break;
        }
      }
       
      if(current_option >= 0)
      {
        cmd_options_[current_option]->read(&argc, &argv);
        cmd_options_[current_option]->is_setted = true;        
      }
      else
        error("CmdLine: parse error near <%s>. Too many arguments.", argv[0]);
    }    
  }

  // Check for empty arguments
  for(int i = 0; i < n_cmd_options_; i++)
  {
    if(cmd_options_[i]->isArgument() && (!cmd_options_[i]->is_setted))
    {
      message("CmdLine: not enough arguments!\n");
      help();
    }
  }

  if(write_log)
  {
    DiskXFile cmd_log("cmd.log", "w");
    writeLog(&cmd_log, false);
  }
  return master_switch;
}

// RhhAHha AH AHa hha hahaAH Ha ha ha

void CmdLine::help()
{
  if(text_info)
    print("%s\n", text_info);

  for(int master_switch_ = 0; master_switch_ < n_master_switches; master_switch_++)
  {
    int n_cmd_options_ = n_cmd_options[master_switch_];
    CmdOption **cmd_options_ = cmd_options[master_switch_];

    int n_real_options = 0;
    for(int i = 0; i < n_cmd_options_; i++)
    {
      if(cmd_options_[i]->isOption())
        n_real_options++;
    }

    if(master_switch_ == 0)
    {
      print("#\n");
      print("# usage: %s", program_name);
      if(n_real_options > 0)
        print(" [options]");
    }
    else
    {
      print("\n#\n");
      print("# or: %s %s", program_name, cmd_options_[0]->name);
      if(n_real_options > 0)
        print(" [options]");
    }

    for(int i = 0; i < n_cmd_options_; i++)
    {
      if(cmd_options_[i]->isArgument())
        print(" <%s>", cmd_options_[i]->name);
    }
    print("\n#\n");

    // Cherche la longueur max du param
    int long_max = 0;
    for(int i = 0; i < n_cmd_options_; i++)
    {
      int laurence = 0;
      if(cmd_options_[i]->isArgument())
        laurence = strlen(cmd_options_[i]->name)+2;

      if(cmd_options_[i]->isOption())
        laurence = strlen(cmd_options_[i]->name)+strlen(cmd_options_[i]->type_name)+1;
      
      if(long_max < laurence)
        long_max = laurence;
    }

    for(int i = 0; i < n_cmd_options_; i++)
    {
      int z = 0;
      if(cmd_options_[i]->isText())
      {
        z = -1;
        print("%s", cmd_options_[i]->name);
      }

      if(cmd_options_[i]->isArgument())
      {
        z = strlen(cmd_options_[i]->name)+2;
        print("  ");
        print("<%s>", cmd_options_[i]->name);
      }
      
      if(cmd_options_[i]->isOption())
      {
        z = strlen(cmd_options_[i]->name)+strlen(cmd_options_[i]->type_name)+1;
        print("  ");
        print("%s", cmd_options_[i]->name);
        print(" %s", cmd_options_[i]->type_name);
      }

      if(z >= 0)
      {
        for(int i = 0; i < long_max+1-z; i++)
          print(" ");
      }
      
      if( cmd_options_[i]->isOption() || cmd_options_[i]->isArgument() )
        print("-> %s", cmd_options_[i]->help);
    
      if(cmd_options_[i]->isArgument())
        print(" (%s)", cmd_options_[i]->type_name);

      if(cmd_options_[i]->isOption())
      {
        DiskXFile std_out(stdout);
        print(" ");
        cmd_options_[i]->printValue(&std_out);
      }

      if(!cmd_options_[i]->isMasterSwitch())
        print("\n");
    }
  }  
  exit(-1);
}

void CmdLine::setWorkingDirectory(const char* dirname)
{
  allocator->free(working_directory);
  working_directory = (char *)allocator->alloc(strlen(dirname)+1);
  strcpy(working_directory, dirname);
}

char *CmdLine::getPath(const char *filename)
{
  associated_files = (char **)allocator->realloc(associated_files, sizeof(char *)*(n_associated_files+1));
  char *path_ = (char *)allocator->alloc(strlen(working_directory)+strlen(filename)+2);
  strcpy(path_, working_directory);
  strcat(path_, "/");
  strcat(path_, filename);
  associated_files[n_associated_files] = (char *)allocator->alloc(strlen(filename)+1);
  strcpy(associated_files[n_associated_files], filename);
  n_associated_files++;
  return path_;
}

DiskXFile *CmdLine::getXFile(const char *filename)
{
  char *full_file_name = this->getPath(filename);
  DiskXFile *file_ = new(allocator) DiskXFile(full_file_name, "w");
  return file_;
}

void CmdLine::saveXFile(XFile *file)
{
  if(master_switch < 0)
    error("CmdLine: nothing to save!");

  writeLog(file, true);

  file->taggedWrite(&master_switch, sizeof(int), 1, "MASTER_SWITCH");
  CmdOption **cmd_options_ = cmd_options[master_switch];
  int n_cmd_options_ = n_cmd_options[master_switch];
  for(int i = 0; i < n_cmd_options_; i++)
  {
    if(cmd_options_[i]->save)
      cmd_options_[i]->saveXFile(file);
  }
}

void CmdLine::writeLog(XFile *file, bool write_associated_files)
{
  // Header
  time_t time_ = time(NULL);
  file->printf("# Date: %s", ctime(&time_));
  file->printf("# Program: %s\n", program_name);
  if(master_switch < 0)
    file->printf("\n# CmdLine not read\n");
  if(master_switch == 0)
    file->printf("\n# Mode: default\n");
  if(master_switch > 0)
    file->printf("\n# Mode: <%s>\n", cmd_options[master_switch][0]->name);

  CmdOption **cmd_options_ = cmd_options[master_switch];
  int n_cmd_options_ = n_cmd_options[master_switch];

  // Cherche la longueur max du param
  int long_max = 0;
  for(int i = 0; i < n_cmd_options_; i++)
  {
    int z = 0;
    if(cmd_options_[i]->isArgument())
      z = strlen(cmd_options_[i]->name)+2;
    
    if(cmd_options_[i]->isOption())
      z = strlen(cmd_options_[i]->name)+strlen(cmd_options_[i]->type_name)+1;
    
    if(long_max < z)
      long_max = z;
  }

  file->printf("\n# Arguments:\n");
  for(int i = 0; i < n_cmd_options_; i++)
  {
    if(!cmd_options_[i]->isArgument())
      continue;

    int z = strlen(cmd_options_[i]->name)+2;
    file->printf("    ");
    file->printf("%s", cmd_options_[i]->name);

    if(z >= 0)
    {
      for(int i = 0; i < long_max+1-z; i++)
        file->printf(" ");
    }

    cmd_options_[i]->printValue(file);
    file->printf("\n");
  }  

  file->printf("\n# Options:\n");
  for(int i = 0; i < n_cmd_options_; i++)
  {
    if(!cmd_options_[i]->isOption())
      continue;

    int z = strlen(cmd_options_[i]->name)+2;
    if(cmd_options_[i]->is_setted)
      file->printf(" *  ");
    else
      file->printf("    ");
    
    file->printf("%s", cmd_options_[i]->name);

    if(z >= 0)
    {
      for(int i = 0; i < long_max+1-z; i++)
        file->printf(" ");
    }

    cmd_options_[i]->printValue(file);
    file->printf("\n");
  }  

  if(write_associated_files)
  {
    file->printf("\n# Associated files:\n");
    for(int i = 0; i < n_associated_files; i++)
      file->printf("    %s\n", associated_files[i]);
  }

  file->printf("\n<#>\n\n");
}


void CmdLine::loadXFile(XFile *file)
{
  // Skip the header
  int header_end = 0;
  while( (header_end != 3) && (!file->eof()) )
  {
    char c;
    file->scanf("%c", &c);
    if(c == '<')
      header_end = 1;
    else
    {
      if(c == '#')
      {
        if(header_end == 1)
          header_end = 2;
        else
          header_end = 0;
      }
      else
      {
        if(c == '>')
        {
          if(header_end == 2)
          {
            header_end = 3;
            // the return-lines
            file->scanf("%c", &c);
            file->scanf("%c", &c);
          }
          else
            header_end = 0;
        }
        else
          header_end = 0;
      }
    }
  }
  
  if(header_end != 3)
    error("CmdLine: cannot find the end of the header!");

  //////////////////

  int old_master_switch;
  file->taggedRead(&old_master_switch, sizeof(int), 1, "MASTER_SWITCH");
  CmdOption **cmd_options_ = cmd_options[old_master_switch];
  int n_cmd_options_ = n_cmd_options[old_master_switch];
  for(int i = 0; i < n_cmd_options_; i++)
  {
    if(cmd_options_[i]->save)
      cmd_options_[i]->loadXFile(file);
  }
}

CmdLine::~CmdLine()
{
}

}
