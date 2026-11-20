/* hangdump.h -- hang reporter: writes a thread/memory report to the SD card when the
 * game stalls, or when ZL+ZR+L+R are held for two seconds.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE file for details.
 */

#ifndef __HANGDUMP_H__
#define __HANGDUMP_H__

#include <stdint.h>

/* Start the watchdog. The bases are the load addresses of libil2cpp.so and libunity.so,
 * so report addresses can be turned into offsets. Call once from the main thread. */
void hangdump_init(uintptr_t il2cpp_base, uintptr_t unity_base);

/* Call once per Unity frame (from the TimeManager hook); it is the watchdog's heartbeat. */
void hangdump_frame(void);

/* Call at the start and end of every thread created through pthread_create_fake. */
void hangdump_thread_register(void);
void hangdump_thread_unregister(void);

#endif
