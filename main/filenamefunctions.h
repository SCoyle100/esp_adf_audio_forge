#ifndef FILENAME_FUNCTIONS_H
#define FILENAME_FUNCTIONS_H

#include <stdbool.h> // Include this header for bool type

int enumerateAudioFiles(const char *directoryName, bool displayFilenames);
bool fileSeekCallback(unsigned long position);
bool isAudioFile(const char filename[]);
void getAudioFilenameByIndex(const char *directoryName, int index, char *pnBuffer);
int openAudioFilenameByIndex(const char *directoryName, int index);
int initFileSystem(int chipSelectPin);

#endif // FILENAME_FUNCTIONS_H

