// This file is meant to tackle the dependencies found in PX4
// that have not been implemented in the VxWorks SDK yet.

#pragma once

// for VxWorks, DKMs are not supported, so we use the
#ifdef NONE
#undef NONE
#endif

#ifdef ERROR
#undef ERROR
#endif

// for VxWorks DKMs are not supported, so we use the
#ifdef _WRS_KERNEL
#ifndef STDIN_FILENO
#  define STDIN_FILENO 0
#endif
#ifndef STDOUT_FILENO
#  define STDOUT_FILENO 1
#endif
#ifndef STDERR_FILENO
#  define STDERR_FILENO 2
#endif
#endif