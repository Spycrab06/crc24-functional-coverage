**CRC-24/BLE Functional Verification Testbench**<br>
SystemVerilog testbench for a CRC-24/BLE hardware engine — the CRC used in Bluetooth Low Energy packets.<br>Covers RTL design, a software golden model, directed tests, and functional coverage closure.

Verified: BLE polynomial, LSB-first bit ordering, configurable init values, reset scenarios, gap cycles, back-to-back transactions, and two hand-computed known vectors.<br>Result: 100% functional coverage across 9 coverpoints and 5 cross coverpoints.

Built using AI agents — spec from ChatGPT, implementation and coverage closure with Claude.

Runs on EDA Playground under VCS or Xcelium with no external dependencies.

https://edaplayground.com/x/frN4
