#pragma once
//Enumerates all symbols in the currently loaded excutable.
//Don't forget to compile with -rdynamic!

//Return 0 to continue search.  1 to stop.
typedef int (*SymEnumeratorCallback)( const char * path, const char * name, void * location, long size );

int EnumerateSymbols( SymEnumeratorCallback cb );
