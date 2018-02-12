/*
 * Copyright (c) 2016, Nils Asmussen
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

#ifndef __CPU_DTU_ACCEL_STREAM_ALGORITHM_TOUPPER_HH__
#define __CPU_DTU_ACCEL_STREAM_ALGORITHM_TOUPPER_HH__

#include "cpu/dtu-accel-stream/algorithm.hh"

class DtuAccelStreamAlgoToUpper : public DtuAccelStreamAlgo
{
  public:

    const char *name() const override { return "ToUpper"; }

    size_t execute(uint8_t *dst, const uint8_t *src, size_t len) override
    {
        for (size_t i = 0; i < len; ++i)
        {
            if (src[i] >= 'a' && src[i] <= 'z')
                dst[i] = src[i] + ('A' - 'a');
            else
                dst[i] = src[i];
        }
        return len;
    }

    Cycles getDelay(Cycles, size_t len) override
    {
        const Cycles BLOCK_TIME      = Cycles(1);
        const size_t BLOCK_SIZE      = 64;

        return Cycles((BLOCK_TIME * len) / BLOCK_SIZE);
    }
};

#endif // __CPU_DTU_ACCEL_STREAM_ALGORITHM_TOUPPER_HH__
