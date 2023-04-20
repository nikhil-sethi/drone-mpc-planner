#include "hash.h"
#include<fstream>
#include <openssl/evp.h>

unsigned char getFileHash(std::string file) {
    std::ifstream fileStream(file, std::ios::binary);

    EVP_MD_CTX *new_hash;
    unsigned char *result;
    unsigned int md5_digest_len = EVP_MD_size(EVP_md5());
    char b;

    new_hash = EVP_MD_CTX_new();
    EVP_DigestInit_ex(new_hash, EVP_md5(), NULL);

    while (fileStream.get(b))
    {
        EVP_DigestUpdate(new_hash, &b, 1);
    }

    result = (unsigned char *)OPENSSL_malloc(md5_digest_len);
    EVP_DigestFinal_ex(new_hash, result, &md5_digest_len);
    EVP_MD_CTX_free(new_hash);

    fileStream.close();

    return *result;
}
