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

#ifndef __DEV_KECACC_XKCP_HH__
#define __DEV_KECACC_XKCP_HH__

#include <cstddef>
#include <cstdint>

#define KECACC_STATE_SIZE  256 /* bytes */

// Also provide a C interface for usage in Rust.
// This is used as fallback when gem5 is missing.
extern "C" {
/**
 * Opaque representation of state, should be backed by 64-bit aligned
 * array of at least KECACC_STATE_SIZE bytes.
 */
struct KecAccState;

uint8_t kecacc_rate_bytes(const struct KecAccState *s);
uint8_t kecacc_alignment_offset(const struct KecAccState *s);
bool kecacc_init(struct KecAccState *s, uint8_t hash_type);
size_t kecacc_absorb(struct KecAccState *s, const uint8_t *buf, size_t nbytes);
size_t kecacc_squeeze(struct KecAccState *s, uint8_t *buf, size_t nbytes);
void kecacc_pad(struct KecAccState *s);

} // extern "C"

class KecAccXKCP
{
  public:
    KecAccXKCP() : state() {}

    /**
     * Returns if the current bit rate (block size) configured for the state.
     * Might be 0 if invalid or if not initialized with valid configuration.
     */
    uint8_t
    rate_bytes() const { return kecacc_rate_bytes(get_state()); }

    /**
     * Returns the alignment offset in bytes that should be used for input
     * and output buffers given to absorb() and squeeze(). The reason for this
     * is that full blocks must be 64-bit aligned in XKCP for best performance.
     */
    uint8_t
    alignment_offset() const { return kecacc_alignment_offset(get_state()); }

    bool
    init(uint8_t hash_type)
    {
        return kecacc_init(get_state(), hash_type);
    }

    size_t
    absorb(const uint8_t *buf, size_t nbytes)
    {
        return kecacc_absorb(get_state(), buf, nbytes);
    }

    size_t
    squeeze(uint8_t *buf, size_t nbytes)
    {
        return kecacc_squeeze(get_state(), buf, nbytes);
    }

    void pad()
    {
        return kecacc_pad(get_state());
    }

    uint64_t state[KECACC_STATE_SIZE / sizeof(uint64_t)];

  private:
    struct KecAccState *get_state() {
        return reinterpret_cast<struct KecAccState*>(state);
    }

    const struct KecAccState *get_state() const {
        return reinterpret_cast<const struct KecAccState*>(state);
    }
};

#endif // __DEV_KECACC_XKCP_HH__
