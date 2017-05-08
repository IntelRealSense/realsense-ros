/* Find and reset all RealSense cameras */
// Andy Zelenak
// Copyright 2017 <Andy Zelenak>

#include <cstdlib>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>
#include <unistd.h>

int reset_all_devices(std::string device_type)
{
    int fd = 0;
    int rc = 0;

    FILE *usb_line;
    char path[1035];

    // Intel is manufacturer 8086
    std::string list_devices = "lsusb -d 8086:" + device_type;

    usb_line = popen(list_devices.c_str(), "r");

    // Display the cameras we found
    while (fgets(path, sizeof(path)-1, usb_line) != NULL)
    {
      printf("%s", path);

      // Parse to get the bus/device identifiers
      char bus[3];
      bus[0] = path[4];
      bus[1] = path[5];
      bus[2] = path[6];

      char device[3];
      device[0] = path[15];
      device[1] = path[16];
      device[2] = path[17];

      // Concatenate the full USB bus/device

     // Example: const char *filename = "/dev/bus/usb/004/003";
      char filename [21]= "/dev/bus/usb/";  // Elements 0-12
      filename[13] = bus[0];
      filename[14] = bus[1];
      filename[15] = bus[2];
      filename[16] = '/';
      filename[17] = device[0];
      filename[18] = device[1];
      filename[19] = device[2];

      printf("%s\n", filename);

      fd = open(filename, O_WRONLY);
      if (fd < 0)
      {
          perror("Error opening output file");
          return 1;
      }

      printf("Resetting USB device %s\n", filename);
      rc = ioctl(fd, USBDEVFS_RESET, 0);
      if (rc < 0)
      {
          perror("Error in ioctl");
          return 1;
      }
      printf("Reset successful\n");
    }

    close(fd);
    return 0;
}


int main(int argc, char **argv)
{
    ///////////////////////////////////
    // Kill any zombie camera processes
    ///////////////////////////////////
    if ( !system("killall nodelet") )
      printf("Could not kill nodelet processes.");


    //////////////////////////////////
    // Find and reset RealSense USB devices
    //////////////////////////////////

    // Find all R200's
    reset_all_devices("0a80");
    // SR300's
    reset_all_devices("0aa5");
    reset_all_devices("0ab3");
    // ZR300's
    reset_all_devices("0ad0");
    reset_all_devices("0ACB");

    return 0;
}
