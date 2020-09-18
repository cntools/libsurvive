#include "symbol_enumerator.h"

#if defined( WIN32 ) || defined( WINDOWS ) || defined( USE_WINDOWS ) || defined( _WIN32 )
#include <stdio.h>
#include <windows.h>

#include <dbghelp.h>

BOOL CALLBACK mycb(PSYMBOL_INFO pSymInfo, ULONG SymbolSize, PVOID UserContext) {
	SymEnumeratorCallback cb = (SymEnumeratorCallback)UserContext;
	return !cb("", &pSymInfo->Name[0], (void *)pSymInfo->Address, (long)pSymInfo->Size);
  }
  
int EnumerateSymbols( SymEnumeratorCallback cb )
{
	HANDLE proc = GetCurrentProcess();
	if( !SymInitialize( proc, 0, 1 ) ) return -1;
	if( !SymEnumSymbols( proc, 0, "*!*", &mycb, (void*)cb ) )
	{
		fprintf(stderr, "SymEnumSymbols returned %d\n", GetLastError());
		SymCleanup(proc);
		return -2;
	}
	SymCleanup(proc);
	return 0;
}

#elif __linux__
#define _GNU_SOURCE

#include <stdio.h>
#include <dlfcn.h>
#include <stdint.h>
#include <limits.h>
#include <string.h>

#ifndef __GNUC__
#define __int128_t long long long
#endif

#ifndef __WORDSIZE
#define __WORDSIZE (sizeof(size_t) * 8)
#endif

#include <link.h>
#include <elf.h>

#define UINTS_PER_WORD (__WORDSIZE / (CHAR_BIT * sizeof (unsigned int)))

#ifndef _GNU_SOURCE
#ifndef ANDROID
struct dl_phdr_info {
	ElfW(Addr) dlpi_addr;		  /* Base address of object */
	const char *dlpi_name;		  /* (Null-terminated) name of
									 object */
	const ElfW(Phdr) * dlpi_phdr; /* Pointer to array of
									 ELF program headers
									 for this object */
	ElfW(Half) dlpi_phnum;		  /* # of items in dlpi_phdr */
    };

void dl_iterate_phdr( void*, void*);
#endif
#endif
static ElfW(Word) gnu_hashtab_symbol_count(const unsigned int *const table)
{
    const unsigned int *const bucket = table + 4 + table[2] * (unsigned int)(UINTS_PER_WORD);
    unsigned int              b = table[0];
    unsigned int              max = 0U;

    while (b-->0U)
        if (bucket[b] > max)
            max = bucket[b];

    return (ElfW(Word))max;
}

static void *dynamic_pointer(const ElfW(Addr) addr, const ElfW(Addr) base, const ElfW(Phdr) *const header,
							 const ElfW(Half) headers) {
	if (addr) {
        ElfW(Half) h;

        for (h = 0; h < headers; h++)
            if (header[h].p_type == PT_LOAD)
                if (addr >= base + header[h].p_vaddr &&
                    addr <  base + header[h].p_vaddr + header[h].p_memsz)
                    return (void *)addr;
    }

    return NULL;
}

//Mostly based off of http://stackoverflow.com/questions/29903049/get-names-and-addresses-of-exported-functions-from-in-linux
static int callback(struct dl_phdr_info *info,
               size_t size, void *data)
{
#pragma GCC diagnostic ignored "-Wpedantic"
	SymEnumeratorCallback cb = (SymEnumeratorCallback)data;
	const ElfW(Addr)                 base = info->dlpi_addr;
	const ElfW(Phdr) *const          header = info->dlpi_phdr;
	const ElfW(Half)                 headers = info->dlpi_phnum;
	const char                      *libpath, *libname;
	ElfW(Half)                       h;

	if (info->dlpi_name && info->dlpi_name[0])
		libpath = info->dlpi_name;
	else
		libpath = "";

	libname = strrchr(libpath, '/');

	if (libname && libname[0] == '/' && libname[1])
		libname++;
	else
		libname = libpath;

	for (h = 0; h < headers; h++)
	{
		if (header[h].p_type == PT_DYNAMIC)
		{
			const ElfW(Dyn)  *entry = (const ElfW(Dyn) *)(base + header[h].p_vaddr);
			const ElfW(Word) *hashtab;
			const ElfW(Sym)  *symtab = NULL;
			const char       *strtab = NULL;
			ElfW(Word)        symbol_count = 0;

			for (; entry->d_tag != DT_NULL; entry++)
			{
				switch (entry->d_tag)
				{
				case DT_HASH:
					hashtab = dynamic_pointer(entry->d_un.d_ptr, base, header, headers);
					if (hashtab)
						symbol_count = hashtab[1];
					break;
				case DT_GNU_HASH:
					hashtab = dynamic_pointer(entry->d_un.d_ptr, base, header, headers);
					if (hashtab)
					{
						ElfW(Word) count = gnu_hashtab_symbol_count(hashtab);
						if (count > symbol_count)
							symbol_count = count;
					}
					break;
				case DT_STRTAB:
					strtab = dynamic_pointer(entry->d_un.d_ptr, base, header, headers);
					break;
				case DT_SYMTAB:
					symtab = dynamic_pointer(entry->d_un.d_ptr, base, header, headers);
					break;
				}
			}

			if (symtab && strtab && symbol_count > 0) {
				ElfW(Word)  s;

				for (s = 0; s < symbol_count; s++) {
					const char *name;
					void *const ptr = dynamic_pointer(base + symtab[s].st_value, base, header, headers);
					int         result;

					if (!ptr)
						continue;

					if (symtab[s].st_name)
						name = strtab + symtab[s].st_name;
					else
						name = "";

					result = cb( libpath, name, ptr, symtab[s].st_size );
					if( result ) return result; //Bail early.
				}
			}
		}
	}
	return 0;
}


int EnumerateSymbols( SymEnumeratorCallback cb )
{
	dl_iterate_phdr( callback, cb );
	return 0;
}

#endif
