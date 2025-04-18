import os
import argparse

def create_random_file(filename, size_bytes, chunk_size=4096):
    with open(filename, 'wb') as f:
        remaining = size_bytes
        while remaining > 0:
            data = os.urandom(min(chunk_size, remaining))
            f.write(data)
            remaining -= len(data)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='create a random file')
    parser.add_argument('-s', '--size', required=True, type=int, help='file size in MB')
    args = parser.parse_args()
    print(f"Creating file of size: {args.size} MB")
    file_size = args.size*1024*1024
    filename = 'random_file_{}M.bin'.format(args.size)
    create_random_file(filename, file_size)
    print(f"Created file: {filename} with size: {file_size} bytes")