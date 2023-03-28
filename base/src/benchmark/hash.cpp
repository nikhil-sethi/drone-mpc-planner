#include "hash.h"
#include<fstream>
#include <openssl/md5.h>

unsigned char getFileHash(std::string file) {
    std::ifstream fileStream(file, std::ios::binary);

    MD5_CTX new_hash;
    unsigned char result;
    char b;

    MD5_Init(&new_hash);

    while (fileStream.get(b))
    {
        MD5_Update(&new_hash, &b, 1);
    }
    MD5_Final(&result, &new_hash);
    fileStream.close();

    return result;
}
