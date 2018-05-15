/* 
 * debug_macros.cpp
 *
 * By: Ashton Stephens
 */

#ifndef DEBUG
#define DEBUG // necessary for debug_macros.o to compile correctly
#endif

#include "debug_macros.h"

// prints out the file, function, and line number provided
//
void debug_print (const std::string file, const std::string func,
                  const int line)
{
    size_t file_hash = std::hash<std::string>{}(file);
    size_t func_hash = std::hash<std::string>{}(func);
    size_t line_hash = std::hash<int>{}(line);
    std::cerr   << "\033[0;33;2m[\033[0;33mDBG\033[2m]\033[10m --> \033[0m"
                << "\033[" << file_hash % 7 + 31 << "m"
                << file << ":"
                << "\033[" << func_hash % 7 + 31 << "m"
                << "(" << func << ")"
                << "\033[" << line_hash % 7 + 31 << "m"
                << "[" << line << "]\033[0m"  
                << std::endl;
}

// kills the program if the assertion is false
//
bool debug_assert (const bool assertion,   const std::string expression,
                   const std::string file, const std::string func,
                   const int line,         const bool kill) 
{
    if (!assertion) {
        std::cerr  << "\033[31;1m" << "[ASSERTION FAILED : "
                   << "\033[0;31m(" <<  expression <<  ")\033[1m ]";
        debug_print(file,func,line); 
        if (kill) exit(1);
        return true;
    } 
    return false;
}
