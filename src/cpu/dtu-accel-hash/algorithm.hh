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

    virtual void start() = 0;

    virtual void update(const void *data, size_t len) = 0;

    virtual size_t get(uint8_t *res) = 0;
};

class DtuAccelHashAlgorithmSHA1 : public DtuAccelHashAlgorithm
{
  private:

    SHA_CTX ctx;

  public:

    void start() override
    {
        SHA1_Init(&ctx);
    }

    void update(const void *data, size_t len) override
    {
        SHA1_Update(&ctx, data, len);
    }

    size_t get(uint8_t *res) override
    {
        SHA1_Final(res, &ctx);
        return 20;
    }
};

class DtuAccelHashAlgorithmSHA224 : public DtuAccelHashAlgorithm
{
  private:

    SHA256_CTX ctx;

  public:

    void start() override
    {
        SHA224_Init(&ctx);
    }

    void update(const void *data, size_t len) override
    {
        SHA224_Update(&ctx, data, len);
    }

    size_t get(uint8_t *res) override
    {
        SHA224_Final(res, &ctx);
        return 28;
    }
};

class DtuAccelHashAlgorithmSHA256 : public DtuAccelHashAlgorithm
{
  private:

    SHA256_CTX ctx;

  public:

    void start() override
    {
        SHA256_Init(&ctx);
    }

    void update(const void *data, size_t len) override
    {
        SHA256_Update(&ctx, data, len);
    }

    size_t get(uint8_t *res) override
    {
        SHA256_Final(res, &ctx);
        return 32;
    }
};

class DtuAccelHashAlgorithmSHA384 : public DtuAccelHashAlgorithm
{
  private:

    SHA512_CTX ctx;

  public:

    void start() override
    {
        SHA384_Init(&ctx);
    }

    void update(const void *data, size_t len) override
    {
        SHA384_Update(&ctx, data, len);
    }

    size_t get(uint8_t *res) override
    {
        SHA384_Final(res, &ctx);
        return 48;
    }
};

class DtuAccelHashAlgorithmSHA512 : public DtuAccelHashAlgorithm
{
  private:

    SHA512_CTX ctx;

  public:

    void start() override
    {
        SHA512_Init(&ctx);
    }

    void update(const void *data, size_t len) override
    {
        SHA512_Update(&ctx, data, len);
    }

    size_t get(uint8_t *res) override
    {
        SHA512_Final(res, &ctx);
        return 64;
    }
};

#endif // __CPU_DTU_ACCEL_HASH_ALGORITHM_HH__
