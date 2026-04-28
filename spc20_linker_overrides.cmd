/*
 * spc20_linker_overrides.cmd — project-local linker overrides
 * ============================================================
 *
 * Sourced AFTER the SysConfig-generated `Debug/device_linker.cmd`
 * so anything specified here wins (TI Arm linker option resolution
 * is last-wins for `--stack_size=`).
 *
 * Why this file exists:
 *   The MSPM0 SDK template hardcodes `StackSizeOptions["MSPM0G3507"] = 512`
 *   in `source/ti/project_config/.meta/linker/LINKERMSPM0options.js`,
 *   and that value is baked into `device_linker.cmd` on every SysConfig
 *   regeneration. Editing the generated file directly is non-durable —
 *   the next syscfg run reverts it. This fragment is tracked in git and
 *   added to the linker invocation via the LIBRARY list in `.cproject`,
 *   so the override survives any number of SysConfig regenerations.
 *
 * Why 1024 B:
 *   The HardFault post-mortem path (added in v0.14) prints nine 32-bit
 *   words via blocking UART writes — its stack frame is small but it
 *   may be entered with the existing stack already deep into a logger
 *   call (snprintf in log_measurements consumes ~200 B). 512 B leaves
 *   no comfortable margin against an unexpected nesting; 1024 B does.
 *   Cost is 512 B of SRAM out of 32 KB total — negligible.
 */

--stack_size=1024
