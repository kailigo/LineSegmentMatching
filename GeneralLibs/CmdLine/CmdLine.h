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

#ifndef CMD_LINE_INC
#define CMD_LINE_INC

#include "Object.h"
#include "CmdOption.h"
#include "DiskXFile.h"

#ifdef WIN32
	#ifdef CMDLINE_LIB
		#define CMDLINE_EXPORT __declspec(dllexport)
	#else
		#define CMDLINE_EXPORT __declspec(dllimport)
	#endif /* CMDLINE_EXPORTS */
#else /* __linux */
	#define CMDLINE_EXPORT
#endif /* #ifdef WIN32 */

namespace Torch {

/** This class provides a useful interface for the user,
    to easily read some arguments/options from the command-line.
    
    Note that here, we make a difference between:
    \begin{itemize}
      \item {\bf options} which are not required.
      \item {\bf arguments} which are required.
    \end{itemize}

    Options:
    \begin{tabular}{lcll}
      "write log"  &  bool  &  Should I output the cmd.log file ? & [true]
    \end{tabular}

    @author Ronan Collobert (collober@idiap.ch)
    @see CmdOption
*/
class CMDLINE_EXPORT CmdLine : public Object
{
  public:
    char *program_name;
    bool write_log;
    int n_master_switches;
    int *n_cmd_options;
    CmdOption ***cmd_options;
    char *text_info;

    char *working_directory;
    char **associated_files;
    int n_associated_files;
    int master_switch;

    char **argv;
    int argc;

    // -----

    ///
    CmdLine();

    /** Read the command-line.
        Call this function {\bf after} adding options/arguments
        that you need, with the help of the following functions.
    */
    int read(int argc_, char **argv_);

    /** Print the help.
        Call this function {\bf after} adding options/arguments
        that you need, with the help of the following functions.
    */
    void help();

    //-----

    /** Functions for adding options.
        The calling order of the following functions will
        define the text order associated when you will call #help()#.
        
        Add an option (Int, Bool, Real, String).
        \begin{itemize}
          \item #name# the name of the option (must be unique).
          \item #ptr# is the pointer on the optional variable.
          \item #init_value# is the initialization value.
          \item #help# is the help text for this option.
        \end{itemize}
        
        The option will be setted to #value# in the command-line
        by printing "#name# #value#"
    */
    ///
    void addICmdOption(const char *name, int *ptr, int init_value, const char *help="", bool save_it=false);
    ///
    void addBCmdOption(const char *name, bool *ptr, bool init_value, const char *help="", bool save_it=false);
    ///
    void addRCmdOption(const char *name, real *ptr, real init_value, const char *help="", bool save_it=false);
    ///
    void addSCmdOption(const char *name, char **ptr, const char *init_value, const char *help="", bool save_it=false);

    /** Functions for adding an argument.
        The argument will be setted to #value# in the command-line
        by writting "#value#" {\bf after} all the options.
        If there are N arguments, you have to write
        "#value1# #value2# #value3# ... #valueN#" to set them in the
        command-line.
    */
    ///
    void addICmdArg(const char *name, int *ptr, const char *help="", bool save_it=false);
    ///
    void addBCmdArg(const char *name, bool *ptr, const char *help="", bool save_it=false);
    ///
    void addRCmdArg(const char *name, real *ptr, const char *help="", bool save_it=false);
    ///
    void addSCmdArg(const char *name, char **ptr, const char *help="", bool save_it=false);

    /// Add a text line in the help message.
    void addText(const char *text);

    /// Add a text at the beginnig of the help.
    void info(const char *text);

    /** Add a master switch.
        It creates an another type of command line.
        If the #text# is the first argument of the user command line,
        only the options corresponding to this new command line will be considered.
    */
    void addMasterSwitch(const char *text);

    /** Set the working directory.
        Use it with #getPath()# and #getXFile()#.
    */
    void setWorkingDirectory(const char* dirname);

    /** Get a full path.
        It adds the #working_directory# before the #filename#.
        This path will be deleted by CmdLine. */
    char *getPath(const char *filename);

    /** Get a DiskXFile.
        It adds the #working_directory# before the #filename# and opens the file.
        This XFile will be deleted by CmdLine. */
    DiskXFile *getXFile(const char *filename);
    
    /// Load the object from a file pointer (\emph{not the options})
    virtual void loadXFile(XFile *file);

    /// Save the object to a file pointer (\emph{not the options})
    virtual void saveXFile(XFile *file);

    //-----

    /** Add an option to the command line. Use this method
        if the wrappers that are provided are not sufficient.
    */
    void addCmdOption(CmdOption *option);

    /** Write a log in #file#.
        If desired, the associated files can be printed.
    */
    void writeLog(XFile *file, bool write_associated_files);

    virtual ~CmdLine();
};

}

#endif
