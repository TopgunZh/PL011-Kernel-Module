#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#define SERIAL_RESET_COUNTER 0
#define SERIAL_GET_COUNTER 1

int main(void)
{
    int fd, ret;

    fd = open("/dev/feserial-3f201000", O_RDWR);
    if (fd < 0)
    {
        fprintf(stderr, "Unable to open /dev/feserial-3f201000\n");
        exit(1);
    }

    ret = ioctl(fd, SERIAL_RESET_COUNTER);
    if (ret < 0)
    {
        fprintf(stderr, "Unable to reset counter\n");
        exit(1);
    }

    printf("Counter reset\n");
    return 0;
}
