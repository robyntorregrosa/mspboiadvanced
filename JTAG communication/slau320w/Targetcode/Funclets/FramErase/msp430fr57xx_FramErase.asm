;*******************************************************************************
;   MSP430FR57xx Erase FRAM Memory
;
;   Description: This code is used to erase MSP430 FRAM Memory
;
;   F. Berenbrinker
;   Texas Instruments, Inc
;   2010
;*******************************************************************************
;*
;* msp430fr57xx_FramErase.asm
;*
;* Version 1.0
;* 
;* Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/ 
;* 
;*  Redistribution and use in source and binary forms, with or without 
;*  modification, are permitted provided that the following conditions 
;*  are met:
;*
;*    Redistributions of source code must retain the above copyright 
;*    notice, this list of conditions and the following disclaimer.
;*
;*    Redistributions in binary form must reproduce the above copyright
;*    notice, this list of conditions and the following disclaimer in the 
;*    documentation and/or other materials provided with the   
;*    distribution.
;*
;*    Neither the name of Texas Instruments Incorporated nor the names of
;*    its contributors may be used to endorse or promote products derived
;*    from this software without specific prior written permission.
;*
;*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
;*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
;*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
;*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
;*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
;*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
;*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
;*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
;*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
;*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
;*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "msp430.h"

;-------------------------------------------------------------------------------
            NAME    FramErase
;-------------------------------------------------------------------------------
; Exported Symbols
PUBLIC      MyEraseAddr
PUBLIC      MyEraseLenght
;-------------------------------------------------------------------------------
            RSEG    MYVARS
;-------------------------------------------------------------------------------
MyProgStart
MyStart         DW      PageErase - MyProgStart
MyEnd           DW      TheEnd - MyProgStart
MyEraseAddr     DL      0xDEADBEEF
MyEraseLenght   DL      0xDEADBABE
MyR10           DL      0x000BDEAD
MyR11           DL      0x000BDEAD

;-------------------------------------------------------------------------------
            RSEG    MYCODE
;-------------------------------------------------------------------------------
;--- [1] Erase Routine Start ---------------------------------------------------
PageErase
StopWDT     MOVX.W   #WDTPW+WDTHOLD,&WDTCTL   ; Make sure watchdog timer is stopped

;--- [2] Save Context Section --------------------------------------------------
SaveR10    ; save R10 & R11 & R12
            MOVX.A  R10,MyR10
            MOVX.A  R11,MyR11
;--- [3] Actual Write Sequence -------------------------------------------------
DoTask
            MOVX.A   MyEraseAddr,R10      ; The start address to write to
            MOVX.A   MyEraseLenght,R11      ; the number of words to be written
                         
W1          MOVX.W   #0xFFFF,0(R10)      ; Yes, move data into Fram
            BIC     #JMBIN0FG,&SYSJMBC    ; clear the mailbox input flag
            
            INCDX.A R10
            DECX.A  R11
            JNZ     W1

;--- [4] Restore Context Section -----------------------------------------------
RestoreR10
            MOVX.A  MyR10,R10
RestoreR11
            MOVX.A  MyR11,R11
            
;--- [5] Erase Routine End -----------------------------------------------------
NotifyDriverStop
            MOV.W     #0xCAFE,&SYSJMBO1
            MOV.W     #0xBABE,&SYSJMBO0
TheEnd
            JMP     TheEnd

            END
