/*
 * Copyright (c) 2021 Stephan Gerhold
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <cassert>
#include <cstring>

#include "kecacc-xkcp.hh"

extern "C" {
    #include "xkcp/KeccakP-1600-SnP.h"
}

struct KecAccState
{
    uint64_t state[KeccakP1600_stateSizeInBytes / sizeof(uint64_t)];
    uint8_t rate_bytes;
    uint8_t pad_delim;
    uint8_t offset;
};

#ifndef KeccakF1600_FastLoop_supported
#error KeccakF1600_FastLoop_Absorb() is required!
#endif

namespace {
// XKCP does currently not have a KeccakF1600_FastLoop_Squeeze()
// Use a (slightly slower) replacement for now
size_t
KeccakF1600_NotSoFastLoop_Squeeze(void *state, unsigned int laneCount,
                                  unsigned char *data, size_t dataByteLen)
{
    size_t originalDataByteLen = dataByteLen;
    unsigned int rateBytes = laneCount * sizeof(uint64_t);

    while (dataByteLen > rateBytes) {
        KeccakP1600_ExtractBytes(state, data, 0, rateBytes);
        KeccakP1600_Permute_24rounds(state);
        data += rateBytes;
        dataByteLen -= rateBytes;
    }
    return originalDataByteLen - dataByteLen;
}

struct KeccakHashType
{
    constexpr KeccakHashType(unsigned int rate_bits, uint8_t pad_delim)
        : rate_bytes(rate_bits / 8), pad_delim(pad_delim) {}
    uint8_t rate_bytes;
    uint8_t pad_delim;
};

constexpr uint8_t HASH_TYPE_COUNT = 7;
constexpr KeccakHashType HASH_TYPES[HASH_TYPE_COUNT] = {
    {   0, 0x00}, // RESET
    {1152, 0x06}, // SHA3-224
    {1088, 0x06}, // SHA3-256
    { 832, 0x06}, // SHA3-384
    { 576, 0x06}, // SHA3-512
    {1344, 0x1F}, // SHAKE128
    {1088, 0x1F}, // SHAKE128
};

template<auto KeccakP1600_HandleBytes, auto KeccakF1600_FastLoop, typename B>
size_t
sponge(struct KecAccState &s, B *buf, size_t nbytes)
{
    if (nbytes == 0)
        return 0;

    size_t permutations = nbytes / s.rate_bytes;

    if (s.offset) {
        // Handle remaining bytes for incomplete block
        size_t remaining = s.rate_bytes - s.offset;
        if (nbytes < remaining) {
            // Not enough bytes to make block complete
            KeccakP1600_HandleBytes(s.state, buf, s.offset, nbytes);
            s.offset += nbytes;
            return 0;
        }

        KeccakP1600_HandleBytes(s.state, buf, s.offset, remaining);
        KeccakP1600_Permute_24rounds(s.state);
        s.offset = 0;
        buf += remaining;
        nbytes -= remaining;
        permutations++;
    }

    // Handle many full blocks efficiently
    assert(reinterpret_cast<uintptr_t>(buf) % alignof(uint64_t) == 0);
    auto n = KeccakF1600_FastLoop(s.state, s.rate_bytes / sizeof(uint64_t),
                                  buf, nbytes);
    buf += n;
    nbytes -= n;

    if (nbytes > 0) {
        // Handle remaining bytes for next incomplete block
        KeccakP1600_HandleBytes(s.state, buf, 0, nbytes);
        s.offset = nbytes;
    }

    return permutations;
}
} // namespace

static_assert(sizeof(KecAccState) <= KECACC_STATE_SIZE, "State too large");

extern "C" {
uint8_t
kecacc_rate_bytes(const struct KecAccState *s)
{
    // Check if state is initialized properly
    if (!s->rate_bytes || s->rate_bytes % alignof(uint64_t) != 0)
        return 0;

    return s->rate_bytes;
}

uint8_t
kecacc_alignment_offset(const struct KecAccState *s)
{
    return s->offset % alignof(uint64_t);
}

bool
kecacc_init(struct KecAccState *s, uint8_t hash_type)
{
    if (hash_type >= HASH_TYPE_COUNT)
        return false;

    std::memset(s, 0, KECACC_STATE_SIZE);
    s->rate_bytes = HASH_TYPES[hash_type].rate_bytes;
    s->pad_delim = HASH_TYPES[hash_type].pad_delim;
    return true;
}

size_t
kecacc_absorb(struct KecAccState *s, const uint8_t *buf, size_t nbytes)
{
    return sponge<KeccakP1600_AddBytes, KeccakF1600_FastLoop_Absorb>
        (*s, buf, nbytes);
}

size_t
kecacc_squeeze(struct KecAccState *s, uint8_t *buf, size_t nbytes)
{
    return sponge<KeccakP1600_ExtractBytes, KeccakF1600_NotSoFastLoop_Squeeze>
        (*s, buf, nbytes);
}

void
kecacc_pad(struct KecAccState *s)
{
    KeccakP1600_AddByte(s->state, s->pad_delim, s->offset);
    KeccakP1600_AddByte(s->state, 0x80, s->rate_bytes - 1);
    KeccakP1600_Permute_24rounds(s->state);
    s->offset = 0;
}
} // extern "C"
