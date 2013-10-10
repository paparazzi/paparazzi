/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*------------------------------------------------------------------------*/
/* Sample code of OS dependent controls for FatFs R0.08b                  */
/* (C)ChaN, 2011                                                          */
/*------------------------------------------------------------------------*/

#include "ch.h"
#include "ff.h"

#if _FS_REENTRANT
/*------------------------------------------------------------------------*/
/* Static array of Synchronization Objects                                */
/*------------------------------------------------------------------------*/
static Semaphore ff_sem[_VOLUMES];

/*------------------------------------------------------------------------*/
/* Create a Synchronization Object                                        */
/*------------------------------------------------------------------------*/
int ff_cre_syncobj(BYTE vol, _SYNC_t *sobj) {

  *sobj = &ff_sem[vol];
  chSemInit(*sobj, 1);
  return TRUE;
}

/*------------------------------------------------------------------------*/
/* Delete a Synchronization Object                                        */
/*------------------------------------------------------------------------*/
int ff_del_syncobj(_SYNC_t sobj) {

  chSemReset(sobj, 0);
  return TRUE;
}

/*------------------------------------------------------------------------*/
/* Request Grant to Access the Volume                                     */
/*------------------------------------------------------------------------*/
int ff_req_grant(_SYNC_t sobj) {

  msg_t msg = chSemWaitTimeout(sobj, (systime_t)_FS_TIMEOUT);
  return msg == RDY_OK;
}

/*------------------------------------------------------------------------*/
/* Release Grant to Access the Volume                                     */
/*------------------------------------------------------------------------*/
void ff_rel_grant(_SYNC_t sobj) {

  chSemSignal(sobj);
}
#endif /* _FS_REENTRANT */

#if _USE_LFN == 3	/* LFN with a working buffer on the heap */
/*------------------------------------------------------------------------*/
/* Allocate a memory block                                                */
/*------------------------------------------------------------------------*/
void *ff_memalloc(UINT size) {

  return chHeapAlloc(NULL, size);
}

/*------------------------------------------------------------------------*/
/* Free a memory block                                                    */
/*------------------------------------------------------------------------*/
void ff_memfree(void *mblock) {

  chHeapFree(mblock);
}
#endif /* _USE_LFN == 3 */
