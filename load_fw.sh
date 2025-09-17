#!/bin/bash
probe-rs download cyw43-firmware/43439A0.bin --binary-format bin --chip RP235x --base-address 0x101C6000
probe-rs download cyw43-firmware/43439A0_clm.bin --binary-format bin --chip RP235x --base-address 0x101FF000