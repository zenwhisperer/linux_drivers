#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>


int main(int argc, char** argv)
{
    char *filename = argv[1];
    int ret = 0;
    int fd = open(filename, 0, O_RDWR);
    if (fd < 0) {
        printf("open file failed\n");
        return -1;
    }

    char readbuf[100];
    ret = read(fd, readbuf, sizeof(readbuf));
    if (ret < 0) {
        printf("read file failed\n");
        return -1;
    }
    printf("%s", readbuf);

    char writebuf[] = "hello my device!";
    ret = write(fd, writebuf, sizeof(writebuf));
    if (ret < 0) {
        printf("write file failed\n");
        return -1;
    }

    ret = close(fd);
    if (ret < 0) {
        printf("close file failed\n");
        return -1;
    }


    return 0;
}