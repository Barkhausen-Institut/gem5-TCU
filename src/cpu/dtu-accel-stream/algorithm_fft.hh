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

#ifndef __CPU_DTU_ACCEL_STREAM_ALGORITHM_FFT_HH__
#define __CPU_DTU_ACCEL_STREAM_ALGORITHM_FFT_HH__

#include "cpu/dtu-accel-stream/algorithm.hh"

class DtuAccelStreamAlgoFFT : public DtuAccelStreamAlgo
{
  public:

    const char *name() const override { return "FFT"; }

    void execute(uint8_t *dst, const uint8_t *src, size_t len) override
    {
        // only pretend to do fft
        memcpy(dst, src, len);
    }

    Cycles getDelay(Cycles time, size_t len) override
    {
        // the time for one 2048 block for 2D-FFT; determined by ALADDIN and
        // picking the sweet spot between area, power and performance.
        // 732 cycles for the FFT function. we have two loops in FFT2D with
        // 16 iterations each. we unroll both 4 times, leading to
        // (4 + 4) * 732 = 5856.
        const Cycles BLOCK_TIME      = time != 0 ? time : Cycles(5856);
        const size_t BLOCK_SIZE      = 2048;

        return Cycles((BLOCK_TIME * len) / BLOCK_SIZE);
    }
};

#endif // __CPU_DTU_ACCEL_STREAM_ALGORITHM_FFT_HH__
