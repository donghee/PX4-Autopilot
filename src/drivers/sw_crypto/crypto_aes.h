#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <tomcrypt.h>

#define AES_BLOCK_SIZE 16

typedef struct {
   ulong32 eK[60], dK[60];
   int Nr;
} crypto_aes_key;

__BEGIN_DECLS

__EXPORT int crypto_aes_setup(const unsigned char *key, int keylen, int num_rounds, crypto_aes_key *skey);
__EXPORT int crypto_aes_encrypt(const unsigned char *pt, unsigned char *ct, crypto_aes_key *skey);

__END_DECLS
