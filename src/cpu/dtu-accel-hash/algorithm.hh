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

#ifndef __CPU_DTU_ACCEL_HASH_ALGORITHM_HH__
#define __CPU_DTU_ACCEL_HASH_ALGORITHM_HH__

#include "base/types.hh"

#include <openssl/sha.h>

class DtuAccelHashAlgorithm
{
  public:

    enum Type
    {
        SHA1,
        SHA224,
        SHA256,
        SHA384,
        SHA512,
    };

  private:

    bool autonomous;
    Type type;
    union
    {
        SHA_CTX sha1;
        SHA256_CTX sha224;
        SHA256_CTX sha256;
        SHA512_CTX sha384;
        SHA512_CTX sha512;
    } ctx;
    char pad[32];

  public:

    DtuAccelHashAlgorithm() : autonomous(true), type(SHA1), ctx() {}

    bool isAutonomous() const
    {
        return autonomous;
    }

    void start(bool _autonomous, Type _type)
    {
        autonomous = _autonomous;
        type = _type;
        switch(type)
        {
            case SHA1:      SHA1_Init(&ctx.sha1); break;
            case SHA224:    SHA224_Init(&ctx.sha224); break;
            case SHA256:    SHA256_Init(&ctx.sha256); break;
            case SHA384:    SHA384_Init(&ctx.sha384); break;
            case SHA512:    SHA512_Init(&ctx.sha512); break;
        }
    }

    void update(const void *data, size_t len)
    {
        switch(type)
        {
            case SHA1:      SHA1_Update(&ctx.sha1, data, len); break;
            case SHA224:    SHA224_Update(&ctx.sha224, data, len); break;
            case SHA256:    SHA256_Update(&ctx.sha256, data, len); break;
            case SHA384:    SHA384_Update(&ctx.sha384, data, len); break;
            case SHA512:    SHA512_Update(&ctx.sha512, data, len); break;
        }
    }

    size_t get(uint8_t *res)
    {
        switch(type)
        {
            case SHA1:      SHA1_Final(res, &ctx.sha1); return 20;
            case SHA224:    SHA224_Final(res, &ctx.sha224); return 28;
            case SHA256:    SHA256_Final(res, &ctx.sha256); return 32;
            case SHA384:    SHA384_Final(res, &ctx.sha384); return 48;
            case SHA512:    SHA512_Final(res, &ctx.sha512); return 64;
        }
        return 0;
    }
};

#endif // __CPU_DTU_ACCEL_HASH_ALGORITHM_HH__
