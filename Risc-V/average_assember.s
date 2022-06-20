_start:	
	li t0, 639
	sw t0, 256(zero)
	li t0, 2534
	sw t0, 260(zero)
	li t0, 2600
	sw t0, 264(zero)
	li t0, 1156
	sw t0, 268(zero)
	li t0, 3141
	sw t0, 272(zero)
	li t0, 1092
	sw t0, 276(zero)
	li t0, 2999
	sw t0, 280(zero)	
	
	j main

filter:
	li a0, 4
	li t0, 0
add: 	
	lw t1, 0(a1)		# load from mem
	add t0, t0, t1	 	# accumulate data in t0
	addi a1, a1, 4    	# increment pointer into source
	addi a0, a0, -1		# decrease counter
	blt zero, a0, add	
divide:
	srli t0, t0, 2
saturation:
	li t2, 2048
	blt t0, t2, no-overflow
overflow:
	li t0, 0		# when overflow saturate result
no-overflow:	
	sw t0, 0(a2)
	addi a2, a2, 4		# increment pointer into sink
	addi a1, a1, -12	# set pointer to next element in source
	ret

main:
	li a1, 256
	li a2, 512

	li t4, 0
	li t5, 4
main-loop:	
	jal filter
	addi t4, t4, 1
	blt t4, t5, main-loop
	
exit:	
	li t0 25	
	sw t0, 100(zero)	# end condition for testbench
