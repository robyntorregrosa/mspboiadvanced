;*******************************************************************************
;   MSP430x5xx Write Flash Memory
;
;   Description: This code is used to write MSP430 Flash Memory
;
;   W. Lutsch
;   Texas Instruments, Inc
;   March 2008
;*******************************************************************************
;*
;* msp430x5xx_FlashWrite.asm
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

#include "msp430x54x.h"
;-------------------------------------------------------------------------------
            NAME    FlashWordWrite
;-------------------------------------------------------------------------------
; Exported Symbols
PUBLIC      MyWriteAddr
PUBLIC      MyWriteSize
PUBLIC      MyWriteLock
PUBLIC      WordWrite
;-------------------------------------------------------------------------------
            RSEG    MYVARS
;-------------------------------------------------------------------------------
MyProgStart
MyStart     DW      WordWrite - MyProgStart
MyEnd       DW      TheEnd - MyProgStart
MyWriteAddr DL      0xDEADBEEF
MyWriteSize DL      0xDEADBEEF
MyWriteLock DW      FWKEY+WAIT
MyLockTst   DW      FWKEY+WAIT
MyFlash1    DW      0xA500
MyFlash3    DW      0xA500
MyR10       DL      0x000BDEAD
MyR11       DL      0x000BDEAD
;-------------------------------------------------------------------------------
            RSEG    MYCODE
;-------------------------------------------------------------------------------
;--- [1] Write Routine Start ---------------------------------------------------
WordWrite
StopWDT     
            MOV.W   #WDTPW+WDTHOLD,&WDTCTL   ; Make sure watchdog timer is stopped
NotifyDriverStart
            MOV     #0xABAD,&SYSJMBO1
            MOV     #0xBABE,&SYSJMBO0
            
;--- [2] Save Context Section --------------------------------------------------
SaveFCTL    ; save Flash Controller Configuration
            MOV     &FCTL1,MyFlash1
            MOV     &FCTL3,MyFlash3
SaveR10    ; save R10 & R11 & R12
            MOVX.A  R10,MyR10
            MOVX.A  R11,MyR11

;--- [3] Actual Write Sequence -------------------------------------------------
L1          BIT     #BUSY,&FCTL3
            JNZ     L1
HandleLockedSegment
            MOV     MyWriteLock,&FCTL3    ; Assign value passed by driver
            MOV     &FCTL3,MyLockTst      ; Read out register again
            CMP.B   MyWriteLock,MyLockTst ; Is content as expected?
            JEQ     DoTask                ; Yes, continue
ToggleLockA
            MOV     MyWriteLock,R10       ; No, set LOCKA bit to be toggled
            BIS     #LOCKA,R10
            MOV     R10,&FCTL3            ; Assign value again - LOCKA will be toggled now
DoTask
            MOVX.A   MyWriteAddr,R10      ; The start address to write to
            MOVX.A   MyWriteSize,R11      ; the number of words to be written

            MOV     #FWKEY+WRT,&FCTL1     ; configure word-write access

            MOV     #JMBCLR0OFF+JMBMODE,&SYSJMBC  ; disable auto-clear feature, 16bit mode
W1          BIT     #JMBIN0FG,&SYSJMBC    ; Data available in mailbox?
            JZ      W1                    ; No, wait for data
            MOV.W   &SYSJMBI0,0(R10)      ; Yes, move data into Flash
            BIC     #JMBIN0FG,&SYSJMBC    ; clear the mailbox input flag

L2          BIT     #BUSY,&FCTL3
            JNZ     L2
            INCDX.A R10
            DECX.A  R11
            JNZ     W1

;--- [4] Restore Context Section -----------------------------------------------
RestoreR10
            MOVX.A  MyR10,R10
RestoreR11
            MOVX.A  MyR11,R11
RestoreFCTL ; restore Flash Controller Configuration
            XOR     #0x3300,MyFlash1      ; restore password
            XOR     #0x3300,MyFlash3      ; restore password
            MOV     MyFlash1,&FCTL1
            MOV     MyFlash3,&FCTL3
RestoreLockState
            MOV     &FCTL3,MyLockTst      ; Read out register again
            CMP.B   MyFlash3,MyLockTst    ; Is content as expected?
            JEQ     NotifyDriverStop      ; Yes, continue
            BIS     #LOCKA,MyFlash3       ; No, set LOCKA bit to be toggled
            MOV     MyFlash3,&FCTL3       ; Assign value again - LOCKA will be toggled now

;--- [5] Write Routine End -----------------------------------------------------
NotifyDriverStop
            MOV     #0xCAFE,&SYSJMBO1
            MOV     #0xBABE,&SYSJMBO0
TheEnd
            JMP     TheEnd

            END
