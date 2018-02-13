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

    bool _autonomous;
    Type _type;
    union
    {
        SHA_CTX sha1;
        SHA256_CTX sha224;
        SHA256_CTX sha256;
        SHA512_CTX sha384;
        SHA512_CTX sha512;
    } _ctx;
    uint32_t _memOff;
    uint32_t _dataSize;
    uint32_t _dataOff;
    char _pad[20];

  public:

    DtuAccelHashAlgorithm()
        : _autonomous(true), _type(SHA1), _ctx(), _memOff(), _dataSize(), _dataOff() {}

    uint32_t memOffset() const { return _memOff; }
    uint32_t dataSize() const { return _dataSize; }
    uint32_t dataOffset() const { return _dataOff; }

    bool interrupted() const
    {
        return autonomous() && dataOffset() != dataSize();
    }

    bool autonomous() const
    {
        return _autonomous;
    }

    void start(bool autonomous, Type type)
    {
        _autonomous = autonomous;
        _type = type;
        switch(type)
        {
            case SHA1:      SHA1_Init(&_ctx.sha1); break;
            case SHA224:    SHA224_Init(&_ctx.sha224); break;
            case SHA256:    SHA256_Init(&_ctx.sha256); break;
            case SHA384:    SHA384_Init(&_ctx.sha384); break;
            case SHA512:    SHA512_Init(&_ctx.sha512); break;
        }
    }

    void startUpdate(uint32_t memOff, uint32_t dataSize, uint32_t dataOff)
    {
        _memOff = memOff;
        _dataSize = dataSize;
        _dataOff = dataOff;
    }

    void incOffset(uint32_t amount)
    {
        _dataOff += amount;
    }

    void finishUpdate()
    {
        _dataSize = 0;
    }

    void update(const void *data, size_t len)
    {
        switch(_type)
        {
            case SHA1:      SHA1_Update(&_ctx.sha1, data, len); break;
            case SHA224:    SHA224_Update(&_ctx.sha224, data, len); break;
            case SHA256:    SHA256_Update(&_ctx.sha256, data, len); break;
            case SHA384:    SHA384_Update(&_ctx.sha384, data, len); break;
            case SHA512:    SHA512_Update(&_ctx.sha512, data, len); break;
        }
    }

    size_t get(uint8_t *res)
    {
        switch(_type)
        {
            case SHA1:      SHA1_Final(res, &_ctx.sha1); return 20;
            case SHA224:    SHA224_Final(res, &_ctx.sha224); return 28;
            case SHA256:    SHA256_Final(res, &_ctx.sha256); return 32;
            case SHA384:    SHA384_Final(res, &_ctx.sha384); return 48;
            case SHA512:    SHA512_Final(res, &_ctx.sha512); return 64;
        }
        return 0;
    }
};

#endif // __CPU_DTU_ACCEL_HASH_ALGORITHM_HH__
