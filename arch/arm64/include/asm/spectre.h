/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Interface for managing mitigations for Spectre vulnerabilities.
 *
 * Copyright (C) 2020 Google LLC
 * Author: Will Deacon <will@kernel.org>
 */

#ifndef __ASM_SPECTRE_H
#define __ASM_SPECTRE_H

#include <asm/cpufeature.h>

/* Watch out, ordering is important here. */
enum mitigation_state {
	SPECTRE_UNAFFECTED,
	SPECTRE_MITIGATED,
	SPECTRE_VULNERABLE,
};

struct pt_regs;
struct task_struct;

enum mitigation_state arm64_get_spectre_v2_state(void);
bool has_spectre_v2(const struct arm64_cpu_capabilities *cap, int scope);
void spectre_v2_enable_mitigation(const struct arm64_cpu_capabilities *__unused);

enum mitigation_state arm64_get_spectre_v4_state(void);
bool has_spectre_v4(const struct arm64_cpu_capabilities *cap, int scope);
void spectre_v4_enable_mitigation(const struct arm64_cpu_capabilities *__unused);
void spectre_v4_enable_task_mitigation(struct task_struct *tsk);

enum mitigation_state arm64_get_spectre_bhb_state(void);
bool is_spectre_bhb_affected(const struct arm64_cpu_capabilities *entry, int scope);
extern bool __nospectre_bhb;
u8 get_spectre_bhb_loop_value(void);
bool is_spectre_bhb_fw_mitigated(void);
void spectre_bhb_enable_mitigation(const struct arm64_cpu_capabilities *__unused);
bool try_emulate_el1_ssbs(struct pt_regs *regs, u32 instr);
#endif	/* __ASM_SPECTRE_H */
