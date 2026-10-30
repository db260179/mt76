/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Use a unique name for define_trace's second inclusion.  mac80211 also
 * installs a trace.h in its exported include directory, so using the generic
 * name may re-read mac80211's trace definitions instead of mt76's.
 */

#include "trace.h"
