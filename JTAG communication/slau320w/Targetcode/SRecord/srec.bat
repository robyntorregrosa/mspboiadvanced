@echo off

REM ********************************************************************************
REM This batch file can be used to convert a TI-txt file into various output file
REM formats using the open source tool SRecord - http://srecord.sourceforge.net
REM
REM After downloading SRecord, be sure to put the required executables (srec_cat.exe
REM and srec_info.exe) in the Targetcode\SRecord directory.
REM
REM Input %1: The filename (without extension) of the TI-txt file to be converted
REM Output:   Assembler files for CCE and IAR
REM           C-Header file including a C-Array
REM ********************************************************************************

set SRCAT=srec_cat.exe
set SRINF=srec_info.exe

echo -------------------------------------------------------------------------------
echo Source file attributes:
echo Name:   %1.txt
%SRINF% %1.txt -guess
echo -------------------------------------------------------------------------------
echo Add padding bytes (0xFF) for word alignment (only if necessary)
echo and generate temporary source file (%1.tmp.txt).
%SRCAT% %1.txt -guess -fill 0xFF -within %1.txt -guess -range-padding 2 -o %1.tmp.txt -ti_txt
echo -------------------------------------------------------------------------------
echo Temporary source file attributes:
echo Name:   %1.tmp.txt
%SRINF% %1.tmp.txt -guess
echo -------------------------------------------------------------------------------
echo Convert to various output formats:
echo A430:    %1.s43
%SRCAT% %1.tmp.txt -guess -o %1.s43 -asm -ow -a430
echo CL430:   %1.asm
%SRCAT% %1.tmp.txt -guess -o %1.asm -asm -ow -cl430
echo C-Array: %1.h
%SRCAT% %1.tmp.txt -guess -o %1.h -ca -ow -c_comp
echo -------------------------------------------------------------------------------
echo Conversion done.
echo -------------------------------------------------------------------------------