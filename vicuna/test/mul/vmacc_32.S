# Copyright TU Wien
# Licensed under the Solderpad Hardware License v2.1, see LICENSE.txt for details
# SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1


    .text
    .global main
main:
    la              a0, vdata_start

    li              t0, 4
    vsetvli         t0, t0, e32,m1,tu,mu

    li              t0, 3
    vmv.v.i         v0, 7
    vle32.v         v4, (a0)
    vmacc.vx        v0, t0, v4
    vse32.v         v0, (a0)

    addi            a0, a0, 64
    li              t0, 0x74cd5a46
    vmv.v.x         v0, t0
    li              t0, 0xfffffffd
    vmacc.vx        v0, t0, v4
    vse32.v         v0, (a0)

    la              a0, vdata_start
    la              a1, vdata_end
    j               spill_cache


    .data
    .align 10
    .global vdata_start
    .global vdata_end
vdata_start:
    .word           0x323b3f47
    .word           0x47434b3a
    .word           0x302f2e32
    .word           0xe8404a51
    .word           0x3f44383b
    .word           0x37424d54
    .word           0x5e4b5049
    .word           0x4c4c4c4a
    .word           0x4a505f3e
    .word           0x485e5455
    .word           0x4d4c4a41
    .word           0x373b5451
    .word           0x41454c45
    .word           0x3a3e3738
    .word           0x312f2e2f
    .word           0x3d433f45
    .word           0x46424949
    .word           0x494a4d51
    .word           0x49413c38
    .word           0x3e514143
    .word           0x47525353
    .word           0x514e5052
    .word           0x525a5b58
    .word           0x5e575254
    .word           0x56545058
    .word           0x5a534947
    .word           0x4744544f
    .word           0x4e515051
    .word           0x5a4b4545
    .word           0x454c4342
    .word           0x40504a3f
    .word           0x4448535a
vdata_end:

    .align 10
    .global vref_start
    .global vref_end
vref_start:
    .word           0x96b1bddc
    .word           0xd5c9e1b5
    .word           0x908d8a9d
    .word           0xb8c0defa
    .word           0x3f44383b
    .word           0x37424d54
    .word           0x5e4b5049
    .word           0x4c4c4c4a
    .word           0x4a505f3e
    .word           0x485e5455
    .word           0x4d4c4a41
    .word           0x373b5451
    .word           0x41454c45
    .word           0x3a3e3738
    .word           0x312f2e2f
    .word           0x3d433f45
    .word           0xde1b9c71
    .word           0x9f037898
    .word           0xe43fcfb0
    .word           0xbc0c7b53
    .word           0x47525353
    .word           0x514e5052
    .word           0x525a5b58
    .word           0x5e575254
    .word           0x56545058
    .word           0x5a534947
    .word           0x4744544f
    .word           0x4e515051
    .word           0x5a4b4545
    .word           0x454c4342
    .word           0x40504a3f
    .word           0x4448535a
vref_end:
