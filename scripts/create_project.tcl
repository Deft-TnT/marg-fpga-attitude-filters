# Generic Vivado project generator for the algorithm-only RTL release.
# Usage: vivado -mode batch -source scripts/create_project.tcl -tclargs rgrsf

set design "rgrsf"
if {[llength $argv] > 0} {
    set design [lindex $argv 0]
}

set script_dir [file dirname [file normalize [info script]]]
set root [file normalize [file join $script_dir ..]]
set part_name "xc7z020clg400-2"
set project_dir [file join $root "vivado_${design}"]

set saam_common [list \
    [file join $root rtl common marg_fixed_pkg.sv] \
    [file join $root rtl common mul_signed32_seq.sv] \
    [file join $root rtl common udiv64_by35_seq.sv] \
    [file join $root rtl common sqrt_q30_seq.sv] \
    [file join $root rtl common rational_normalizer.sv]]

set conventional_common [list \
    [file join $root rtl mahony_madgwick ahrs_fixed_pkg.sv] \
    [file join $root rtl mahony_madgwick standard_mul_signed32_seq.sv] \
    [file join $root rtl mahony_madgwick standard_div_u64_u32_seq.sv] \
    [file join $root rtl mahony_madgwick standard_sqrt_q30_seq.sv] \
    [file join $root rtl mahony_madgwick standard_normalizer_q30_shared.sv]]

switch -- $design {
    fixed_beta {
        set top marg_zynq7000_top
        set sources [concat $saam_common [list \
            [file join $root rtl fixed_beta marg_estimator_core.sv] \
            [file join $root rtl fixed_beta marg_zynq7000_top.sv]]]
    }
    dynamic_beta {
        set top dynamic_beta_stream_top
        set sources [concat $saam_common [list \
            [file join $root rtl dynamic_beta marg_estimator_dynamic_beta_core_frozen.sv] \
            [file join $root rtl dynamic_beta dynamic_beta_stream_top.sv]]]
    }
    rgrsf {
        set top rgrsf_top
        set sources [concat $saam_common [list \
            [file join $root rtl rgrsf marg_estimator_three_mode_core_v2.sv] \
            [file join $root rtl rgrsf rgrsf_top.sv]]]
    }
    mahony -
    madgwick {
        set top "${design}_marg_top"
        set sources [concat $conventional_common [list \
            [file join $root rtl mahony_madgwick standard_marg_core_shared.sv] \
            [file join $root rtl mahony_madgwick standard_filter_stream_top.sv]]]
    }
    mekf_diag {
        set top mekf_diag_top
        set sources [concat $conventional_common [list \
            [file join $root rtl mekf_diag mekf_diag_core.sv] \
            [file join $root rtl mekf_diag mekf_diag_stream_top.sv]]]
    }
    default {
        error "Unknown design '$design'. Choose fixed_beta, dynamic_beta, rgrsf, mahony, madgwick, or mekf_diag."
    }
}

foreach source $sources {
    if {![file exists $source]} {
        error "Required source is missing: $source"
    }
}

create_project $design $project_dir -part $part_name -force
add_files -norecurse $sources
add_files -fileset constrs_1 -norecurse [file join $root constraints marg_core_50mhz.xdc]
set_property top $top [current_fileset]
update_compile_order -fileset sources_1
puts "PROJECT_READY=$project_dir"
puts "TOP_MODULE=$top"
close_project
