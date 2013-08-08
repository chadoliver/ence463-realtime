
	.cdecls "FreeRTOSConfig.h"

	.thumb

	.def vPortSetInterruptMask
	.def vPortClearInterruptMask
	.def vPortSVCHandler
	.def vPortStartFirstTask
	.def xPortPendSVHandler

	.ref pxCurrentTCB          
    .ref vTaskSwitchContext

	.text

NVIC_VTABLE_R: .word  	0xE000ED08

;---------------------------------------------------------

vPortSetInterruptMask:
	
	push { r0 }
	mov r0, #configMAX_SYSCALL_INTERRUPT_PRIORITY                     
	msr basepri, r0     
	pop { r0 }
	bx r14
	
;---------------------------------------------------------

vPortClearInterruptMask:

	push { r0 }
	mov r0, #0
	msr basepri, r0
	pop { r0 }
	bx r14
	

;---------------------------------------------------------

xPortPendSVHandler:

   mrs r0, psp                      

   ldr r3, pxCurrentTCBConst        ; Get the location of the current TCB.
   ldr r2, [r3]                     

   stmdb r0!, {r4-r11}              ; Save the remaining registers. 
   str r0, [r2]                     ; Save the new top of stack into the first member of the TCB. 

   stmdb sp!, {r3, r14}             
   mov r0, #configMAX_SYSCALL_INTERRUPT_PRIORITY                       
   msr basepri, r0                  
   bl vTaskSwitchContext           
   mov r0, #0                       
   msr basepri, r0                  
   ldmia sp!, {r3, r14}             														    
                                    ; Restore the context, including the critical nesting count.
   ldr r1, [r3]                     
   ldr r0, [r1]                     ; The first item in pxCurrentTCB is the task top of stack. 
   ldmia r0!, {r4-r11}              ; Pop the registers. 
   msr psp, r0                         
   bx r14                              

   .align 2                            
pxCurrentTCBConst: .word pxCurrentTCB 

;---------------------------------------------------------

vPortSVCHandler:
   	                                
   	ldr r3, pxCurrentTCBConst2      ; Restore the context.  
   	ldr r1, [r3]                    ; Use pxCurrentTCBConst to get the pxCurrentTCB address.  
   	ldr r0, [r1]                    ; The first item in pxCurrentTCB is the task top of stack. 
   	ldmia r0!, {r4-r11}             ; Pop the registers that are not automatically saved on exception entry and the critical nesting count.  
   	msr psp, r0                     ; Restore the task stack pointer. 
   	mov r0, #0                      
   	msr basepri, r0                 
   	orr r14, #0xd                   
   	bx r14                          
   	                                
   	.align 2                        
pxCurrentTCBConst2: .word pxCurrentTCB


;---------------------------------------------------------

vPortStartFirstTask:
                                       
 	ldr r0, NVIC_VTABLE_R			; Use the NVIC offset register to locate the stack. 
 	ldr r0, [r0]         
 	ldr r0, [r0]         
 	msr msp, r0          			; Set the msp back to the start of the stack. 
 	svc #0               			; System call to start first task. 

	.end
		