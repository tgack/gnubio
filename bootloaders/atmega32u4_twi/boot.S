/*
 * GNU BIO Bootloader interrupt vector table
 */

#if (__GNUC__ < 3) || (__GNUC__ == 3 && __GNUC_MINOR__ < 3)
#error "GCC version >= 3.3 required"
#endif

#define __RESET_VECTOR__   (0x3800 << 1)
#define __TWI_VECTOR__ (0x3848 << 1)

	.section .boot
	.global __boot_vectors
	.func __boot_vectors
__boot_vectors:
	jmp	__RESET_VECTOR__		/* Vector No. 1 */
	jmp	__RESET_VECTOR__		/* Vector No. 2 */
	jmp	__RESET_VECTOR__		/* Vector No. 3 */
	jmp	__RESET_VECTOR__		/* Vector No. 4 */
	jmp	__RESET_VECTOR__		/* Vector No. 5 */
	jmp	__RESET_VECTOR__		/* Vector No. 6 */
	jmp	__RESET_VECTOR__		/* Vector No. 7 */
	jmp	__RESET_VECTOR__		/* Vector No. 8 */
	jmp	__RESET_VECTOR__		/* Vector No. 9 */
	jmp	__RESET_VECTOR__		/* Vector No. 10 */
	jmp	__RESET_VECTOR__		/* Vector No. 11 */
	jmp	__RESET_VECTOR__		/* Vector No. 12 */
	jmp	__RESET_VECTOR__		/* Vector No. 13 */
	jmp	__RESET_VECTOR__		/* Vector No. 14 */
	jmp	__RESET_VECTOR__		/* Vector No. 15 */
	jmp	__RESET_VECTOR__		/* Vector No. 16 */
	jmp	__RESET_VECTOR__		/* Vector No. 17 */
	jmp	__RESET_VECTOR__		/* Vector No. 18 */
	jmp	__RESET_VECTOR__		/* Vector No. 19 */
	jmp	__RESET_VECTOR__		/* Vector No. 20 */
	jmp	__RESET_VECTOR__		/* Vector No. 21 */
	jmp	__RESET_VECTOR__		/* Vector No. 22 */
	jmp	__RESET_VECTOR__		/* Vector No. 23 */
	jmp	__RESET_VECTOR__		/* Vector No. 24 */
	jmp	__RESET_VECTOR__		/* Vector No. 25 */
	jmp	__RESET_VECTOR__		/* Vector No. 26 */
	jmp	__RESET_VECTOR__		/* Vector No. 27 */
	jmp	__RESET_VECTOR__		/* Vector No. 28 */
	jmp	__RESET_VECTOR__		/* Vector No. 29 */
	jmp	__RESET_VECTOR__		/* Vector No. 30 */
	jmp	__RESET_VECTOR__		/* Vector No. 31 */
	jmp	__RESET_VECTOR__		/* Vector No. 32 */
	jmp	__RESET_VECTOR__		/* Vector No. 33 */
	jmp	__RESET_VECTOR__		/* Vector No. 34 */
	jmp	__RESET_VECTOR__		/* Vector No. 35 */
	jmp	__RESET_VECTOR__		/* Vector No. 36 */
	jmp	__TWI_VECTOR__			/* Vector No. 37 */
	jmp	__RESET_VECTOR__		/* Vector No. 38 */
	jmp	__RESET_VECTOR__		/* Vector No. 39 */
	jmp	__RESET_VECTOR__		/* Vector No. 40 */
	jmp	__RESET_VECTOR__		/* Vector No. 41 */
	jmp	__RESET_VECTOR__		/* Vector No. 42 */
	jmp	__RESET_VECTOR__		/* Vector No. 43 */
	.endfunc