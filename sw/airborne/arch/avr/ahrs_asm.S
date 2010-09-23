/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id$
 *
 * Assembly file to layout the AHRS data.
 * We want to be sure that things are aligned and packed as
 * closely as possible, as well as alias several things.  This
 * is the easiest way to do it.
 */
.section .bss

.global X
X:
.global quat
quat:
.global q0
q0:
.space 4
.global q1
q1:
.space 4
.global q2
q2:
.space 4
.global q3
q3:
.space 4

.global	bias
bias:
.global	bias_p
bias_p:
.space	4
.global	bias_q
bias_q:
.space	4
.global	bias_r
bias_r:
.space	4


.global	C
C:
.global	Qdot
Qdot:
.space	16


.global A
A:
.global	PCt
PCt:
.space	7 * 4
.global	K
K:
.space	7 * 4
.global E
E:
.space	1 * 4

/* And the rest of A */
.space (4*7 - 7 - 7 - 1) * 4


.global end_bss
end_bss:
