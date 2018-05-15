/* 
 * debug_macros.h 
 * 
 * Purpose: adds easily visible debug functions only to .cpp files
 *          that need it.
 *
 * By: Ashton Stephens         
 * 
 */

/* ~~ ONLY INCLUDE THIS IN *.cpp FILES ~~ */

#ifndef DEBUG_MACROS_H__
#define DEBUG_MACROS_H__

#ifdef DEBUG

/* ---------------------------------------------------------------------------*/

#include <string>
#include <iostream>     
#include <functional>   // hashes

/* prints the file, function, and line number */
#define DBG \
    debug_print(__FILE__,__func__,__LINE__)

/* prints the location, the expression, and kills the program on false */

#define DBG_ASSERT_HARD(expr) \
    debug_assert(expr,#expr,__FILE__,__func__,__LINE__,true)

#define DBG_ASSERT_SOFT(expr) \
    debug_assert(expr,#expr,__FILE__,__func__,__LINE__,false)

#define DBG_ASSERT(expr) \
    DBG_ASSERT_HARD(expr)

extern void debug_print (const std::string file, const std::string func,
                         const int line_number);

extern bool debug_assert (const bool assertion, const std::string expression,
                   const std::string file, const std::string func,
                   const int line,         const bool kill);

/* ---------------------------------------------------------------------------*/

#else 

#define DBG
#define DBG_ASSERT_HARD(expr)
#define DBG_ASSERT_SOFT(expr)
#define DBG_ASSERT(expr)

#endif

/*  USAGE: this file can be included at the top in any .CPP file,
 *  and will not add any features or include any libraries. 
 *
 *  In order to add desired functionality, one must add "#define DEBUG" on 
 *  any line before this header is included. This restricts the debuging 
 *  functions to only apply for the files in which the 'DEBUG' macro is defined.
 *
 *  tldr: |------ this prints -------|------- this does not --------|
 *        |                          |                              |
 *        |#define DEBUG             |#include "debug_macros.h"     |
 *        |#include "debug_macros.h" |#define DEBUG                 |
 *        |                          |                              |
 *        |int main() {              |int main() {                  |
 *        |                          |                              |
 *        |DBG;                      |DBG;                          |
 *        |                          |                              |
 *        |}                         |}                             |
 *        |                          |                              |
 */

#endif
