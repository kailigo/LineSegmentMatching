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

#ifndef CMD_OPTION_INC
#define CMD_OPTION_INC

#include "Object.h"

namespace Torch {

/** This class defines an option for the command line.
    If you need special command line arguments/options,
    you have to create a new children of this class.

    @author Ronan Collobert (collober@idiap.ch)
    @see CmdLine
*/
class CmdOption : public Object
{
  private:
    // Special flags.
    bool is_option;
    bool is_argument;
    bool is_text;
    bool is_master_switch;

  public:
    /// Name of the option.
    char *name;

    /// Type name of the option.
    char *type_name;

    /** An help string.
        Cannot be NULL.
    */
    char *help;

    /** True is the option has to be saved
        when saving the command line.
    */
    bool save;

    /** True is the option has been setted after
        reading the command-line.
    */
    bool is_setted;

    //////////////////////

    ///
    CmdOption(const char *name_, const char *type_name_, const char *help_="", bool save_=false);

    /// Initialize the value of the option.
    virtual void initValue();

    /// If #is_setted# is true, print the current value, else the init value.
    virtual void printValue(XFile *file_);

    /** Read the option on the command line.
        argv_ and argc_ have to point of the next
        option after that.
    */
    virtual void read(int *argc_, char ***argv_);

    /* Return true if the option is on the command line.
       Decrements argc_ and increment argv_ if true.
    */
    bool isCurrent(int *argc_, char ***argv_);

    /** Returns true if it's an optional argument.
        If #set_# is true, set it to an optional argument.
    */
    bool isOption(bool set_=false);

    /** Returns true if it's a required argument.
        If #set_# is true, set it to a required argument.
    */
    bool isArgument(bool set_=false);

    /** Returns true if it's just text to be displayed in the command line.
        If #set_# is true, set it to text mode.
    */
    bool isText(bool set_=false);

    /** Returns true if it's a master switch.
        If #set_# is true, set it to a master switch.
    */
    bool isMasterSwitch(bool set_=false);
    
    ~CmdOption();
};

/** This class defines a integer command-line option.

    @author Ronan Collobert (collober@idiap.ch)
    @see CmdLine
*/
class IntCmdOption : public CmdOption
{
  public:
    int *ptr;
    int init_value;

    ///
    IntCmdOption(const char *name_, int *ptr_, int init_value_, const char *help_="", bool save_=false);

    virtual void initValue();
    virtual void printValue(XFile *file_);
    virtual void read(int *argc_, char ***argv_);
    virtual void loadXFile(XFile *file);
    virtual void saveXFile(XFile *file);
    ~IntCmdOption();
};

/** This class defines a real command-line option.

    @author Ronan Collobert (collober@idiap.ch)
    @see CmdLine
*/
class RealCmdOption : public CmdOption
{
  public:
    real *ptr;
    real init_value;

    ///
    RealCmdOption(const char *name_, real *ptr_, real init_value_, const char *help_="", bool save_=false);

    virtual void initValue();
    virtual void printValue(XFile *file_);
    virtual void read(int *argc_, char ***argv_);
    virtual void loadXFile(XFile *file);
    virtual void saveXFile(XFile *file);
    ~RealCmdOption();
};

/** This class defines a bool command-line option.

    @author Ronan Collobert (collober@idiap.ch)
    @see CmdLine
*/
class BoolCmdOption : public CmdOption
{
  public:
    bool *ptr;
    bool init_value;

    ///
    BoolCmdOption(const char *name_, bool *ptr_, bool init_value_, const char *help_="", bool save_=false);

    virtual void initValue();
    virtual void read(int *argc_, char ***argv_);
    virtual void loadXFile(XFile *file);
    virtual void saveXFile(XFile *file);
    ~BoolCmdOption();
};

/** This class defines a string command-line option.

    @author Ronan Collobert (collober@idiap.ch)
    @see CmdLine
*/
class StringCmdOption : public CmdOption
{
  public:
    char **ptr;
    char *init_value;

    ///
    StringCmdOption(const char *name_, char **ptr_, const char *init_value_, const char *help_="", bool save_=false);

    virtual void initValue();
    virtual void printValue(XFile *file_);
    virtual void read(int *argc_, char ***argv_);
    virtual void loadXFile(XFile *file);
    virtual void saveXFile(XFile *file);
    ~StringCmdOption();
};

}

#endif
