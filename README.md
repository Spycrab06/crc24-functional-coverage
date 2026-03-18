# CRC-24/BLE Functional Verification Testbench

SystemVerilog testbench for a CRC-24/BLE hardware engine — the CRC used in Bluetooth Low Energy packets.

CRC (Cyclic Redundancy Check) is an error-detection algorithm used in digital communication. The sender runs data through a polynomial and appends the remainder to the packet. The receiver repeats the calculation and checks for a match. BLE uses a 24-bit CRC (polynomial 0x00065B, LSB-first) on every packet.

The spec is a ChatGPT-generated UVM test plan with 10 directed tests, a randomization strategy, scoreboard requirements, and coverage bins. Verification follows three steps:

1. **Stimulus** — directed and randomized packets exercise normal operation, boundary lengths, gap cycles, mid-transaction reset, back-to-back transactions, and configurable CRC init values
2. **Checking** — a software golden model computes the expected CRC for every packet; the scoreboard compares it to the DUT output and flags mismatches
3. **Coverage** — a functional covergroup tracks which scenarios have been hit; the run is complete when all bins are closed

**Result:** 100% functional coverage across 9 coverpoints and 5 cross coverpoints, zero scoreboard mismatches.

Built using AI agents — spec from ChatGPT, implementation and coverage closure iterated with Claude.

**Run it:** [EDA Playground](https://edaplayground.com/x/frN4) (VCS / Xcelium, no external dependencies)
