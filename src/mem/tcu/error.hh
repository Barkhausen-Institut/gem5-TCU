/*
 * Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
 * Copyright (C) 2019-2021 Nils Asmussen, Barkhausen Institut
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 */

#ifndef __MEM_TCU_ERROR_HH__
#define __MEM_TCU_ERROR_HH__

enum class TcuError
{
    NONE                = 0,
    NO_MEP              = 1,
    NO_SEP              = 2,
    NO_REP              = 3,
    FOREIGN_EP          = 4,
    SEND_REPLY_EP       = 5,
    RECV_GONE           = 6,
    RECV_NO_SPACE       = 7,
    REPLIES_DISABLED    = 8,
    OUT_OF_BOUNDS       = 9,
    NO_CREDITS          = 10,
    NO_PERM             = 11,
    INV_MSG_OFF         = 12,
    TRANSLATION_FAULT   = 13,
    ABORT               = 14,
    UNKNOWN_CMD         = 15,
    RECV_OUT_OF_BOUNDS  = 16,
    RECV_INV_RPL_EPS    = 17,
    SEND_INV_CRD_EP     = 18,
    SEND_INV_MSG_SZ     = 19,
    TIMEOUT_MEM         = 20,
    TIMEOUT_NOC         = 21,
    PAGE_BOUNDARY       = 22,
    MSG_UNALIGNED       = 23,
    TLB_MISS            = 24,
    TLB_FULL            = 25,
};

#endif // __MEM_TCU_ERROR_HH__
