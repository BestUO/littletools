#pragma once
#include <unistd.h>

#define crush_hashmix(a, b, c) do {			\
	a = a-b;  a = a-c;  a = a^(c>>13);	\
	b = b-c;  b = b-a;  b = b^(a<<8);	\
	c = c-a;  c = c-b;  c = c^(b>>13);	\
	a = a-b;  a = a-c;  a = a^(c>>12);	\
	b = b-c;  b = b-a;  b = b^(a<<16);	\
	c = c-a;  c = c-b;  c = c^(b>>5);	\
	a = a-b;  a = a-c;  a = a^(c>>3);	\
	b = b-c;  b = b-a;  b = b^(a<<10);	\
	c = c-a;  c = c-b;  c = c^(b>>15);	\
} while (0)

//from Ceph
inline uint32_t CephHashFun(uint32_t a)
{
	uint32_t hash = 0x4E67C6A7  ^ a;//	uint32_t crush_hash_seed = 0x4E67C6A7;
	uint32_t b = a;
	uint32_t x = 231232;
	uint32_t y = 1232;
	crush_hashmix(b, x, hash);
	crush_hashmix(y, a, hash);
	return hash;
}

//from Jenkins hash function
inline uint32_t JenkinsHashFun(uint8_t *key, size_t length)
{
	size_t i = 0;
	uint32_t hash = 0;
	while (i != length) 
	{
		hash += key[i++];
		hash += hash << 10;
		hash ^= hash >> 6;
	}
	hash += hash << 3;
	hash ^= hash >> 11;
	hash += hash << 15;
	return hash;
}

//from DPDK
inline uint32_t public_align32pow2(uint32_t x)
{
	x |= x >> 1;
	x |= x >> 2;
	x |= x >> 4;
	x |= x >> 8;
	x |= x >> 16;

	return x;
}

inline int set_cpu(int i)  
{  
    cpu_set_t mask;  
    CPU_ZERO(&mask);  
  
    CPU_SET(i,&mask);  
  
    printf("thread %u, i = %d\n", (uint16_t)pthread_self(), i);  
    if(-1 == pthread_setaffinity_np(pthread_self() ,sizeof(mask),&mask))  
    {  
        fprintf(stderr, "pthread_setaffinity_np erro\n");  
        return -1;  
    }  
    return 0;  
} 