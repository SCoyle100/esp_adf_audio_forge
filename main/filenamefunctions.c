
#include "periph_sdcard.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h> // Include this header for bool type

#define MOUNT_POINT "/sdcard"

FILE *file;
DIR *dir;
struct dirent *ent;
int numberOfFiles;

bool fileSeekCallback(unsigned long position) {
    return (fseek(file, position, SEEK_SET) == 0);
}

unsigned long filePositionCallback(void) {
    return ftell(file);
}

int fileReadCallback(void) {
    return fgetc(file);
}

int fileReadBlockCallback(void *buffer, int numberOfBytes) {
    return fread(buffer, 1, numberOfBytes, file);
}

int fileSizeCallback(void) {
    struct stat st;
    if (fstat(fileno(file), &st) != 0) {
        return -1;
    }
    return st.st_size;
}

int initFileSystem(int chipSelectPin) {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    const char mount_point[] = MOUNT_POINT;
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, NULL);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            printf("Failed to mount filesystem. "
                   "If you want the card to be formatted, set format_if_mount_failed = true.\n");
        } else {
            printf("Failed to initialize the card (%s). "
                   "Make sure SD card lines have pull-up resistors in place.\n", esp_err_to_name(ret));
        }
        return -1;
    }

    return 0;
}

bool isAudioFile(const char filename[]) {
    char *ext = strrchr(filename, '.');
    if (ext == NULL) {
        return false;
    }

    if (ext[0] == '_' || ext[0] == '~' || ext[0] == '.') {
        return false;
    }

    return (strcasecmp(ext, ".wav") == 0);
}

int enumerateAudioFiles(const char *directoryName, bool displayFilenames) {
    numberOfFiles = 0;

    dir = opendir(directoryName);
    if (!dir) {
        printf("Failed to open directory\n");
        return -1;
    }

    while ((ent = readdir(dir)) != NULL) {
        if (isAudioFile(ent->d_name)) {
            numberOfFiles++;
            if (displayFilenames) {
                printf("%d: %s\n", numberOfFiles, ent->d_name);
            }
        } else if (displayFilenames) {
            printf("%s\n", ent->d_name);
        }
    }

    closedir(dir);
    return numberOfFiles;
}

void getAudioFilenameByIndex(const char *directoryName, int index, char *pnBuffer) {
    if (index < 0 || index >= numberOfFiles) {
        printf("Index out of range\n");
        return;
    }

    dir = opendir(directoryName);
    if (!dir) {
        printf("Failed to open directory\n");
        return;
    }

    int fileIndex = 0;
    while ((ent = readdir(dir)) != NULL) {
        if (isAudioFile(ent->d_name)) {
            if (fileIndex == index) {
                strcpy(pnBuffer, directoryName);
                if (pnBuffer[strlen(pnBuffer) - 1] != '/') {
                    strcat(pnBuffer, "/");
                }
                strcat(pnBuffer, ent->d_name);
                printf("Constructed Pathname: %s\n", pnBuffer);
                break;
            }
            fileIndex++;
        }
    }

    closedir(dir);
}

int openAudioFilenameByIndex(const char *directoryName, int index) {
    char pathname[255];

    getAudioFilenameByIndex(directoryName, index, pathname);

    printf("Pathname: %s\n", pathname);

    if (file) {
        fclose(file);
    }

    file = fopen(pathname, "rb");
    if (!file) {
        printf("Error opening audio file\n");
        return -1;
    }

    printf("Audio file opened successfully\n");
    return 0;
}

