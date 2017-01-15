
#if defined(__CYGWIN__)

#include "../CORE_URUS.h"
#include "../CORE_URUS_NAMESPACE.h"
#include "CORE_URUS_CYGWIN.h"

#include "CoreUrusTimers_Cygwin.h"
#include "CoreUrusScheduler_Cygwin.h"
#include <stdio.h>

static CLCoreUrusTimers_Cygwin coreTimers;
static CLCoreUrusScheduler_Cygwin coreScheduler;

CORE_CYGWIN::CORE_CYGWIN() :
    NSCORE_URUS::CLCORE_URUS(
        &coreTimers,
        &coreScheduler)
{}

void CORE_CYGWIN::init_core() const
{
#if 0
    printf("Cygwin Core Started!\n");
#endif
}

const NSCORE_URUS::CLCORE_URUS& NSCORE_URUS::get_CORE()
{
    static const CORE_CYGWIN _urus_core;
    return _urus_core;
}

#endif // __CYGWIN__
