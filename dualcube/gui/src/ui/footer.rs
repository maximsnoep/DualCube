use super::theme::{sized_text, text_format, to_color32, LIGHT_RED, RED, TEXT_COLOR};
use super::widgets::timer_animation;
use crate::colors;
use crate::controls::InteractiveMode;
use crate::jobs::JobState;
use crate::resources::Configuration;
use bevy::diagnostic::{
    DiagnosticPath, DiagnosticsStore, FrameTimeDiagnosticsPlugin,
    SystemInformationDiagnosticsPlugin,
};
use bevy::prelude::*;
use bevy_egui::egui::{text, Align, Color32, Layout, TopBottomPanel};
use dualcube::prelude::*;

const STAT_SIZE: f32 = 8.0;

fn usage_color(value: f64) -> Color32 {
    if value < 70.0 {
        TEXT_COLOR
    } else if value < 90.0 {
        LIGHT_RED
    } else {
        RED
    }
}

/// Shows the bottom panel.
pub fn show(
    mut egui_ctx: bevy_egui::EguiContexts,
    conf: Res<Configuration>,
    diagnostics: Res<DiagnosticsStore>,
    job_state: Res<JobState>,
    time: Res<Time>,
) -> Result<(), BevyError> {
    TopBottomPanel::bottom("footer")
        .show_separator_line(false)
        .show(egui_ctx.ctx_mut()?, |ui| {
            ui.add_space(5.);
            ui.separator();
            ui.add_space(5.);

            ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                    ui.add_space(30.);

                    // The right-handed coordinate tripod, colored per axis.
                    let mut tripod = text::LayoutJob::default();
                    tripod.append("right-hand: ", 0.0, text_format(STAT_SIZE, TEXT_COLOR));
                    for (i, direction) in DIRECTIONS.into_iter().enumerate() {
                        if i > 0 {
                            tripod.append(", ", 0.0, text_format(STAT_SIZE, TEXT_COLOR));
                        }
                        let color = to_color32(colors::from_direction(
                            direction,
                            Some(Perspective::Primal),
                            None,
                        ));
                        tripod.append(["+X", "+Y", "+Z"][i], 0.0, text_format(STAT_SIZE, color));
                    }
                    ui.label(tripod);

                    // Performance, mode, and job status.
                    let measure = |path: &DiagnosticPath| {
                        diagnostics
                            .get(path)
                            .and_then(|d| d.smoothed())
                            .unwrap_or(0.0)
                    };
                    let usage = |label: &str, path: &DiagnosticPath| {
                        let value = measure(path);
                        (format!("{label} {value:>3.0}%"), usage_color(value))
                    };

                    let fps = measure(&FrameTimeDiagnosticsPlugin::FPS);
                    let fps_color = if fps < 30.0 {
                        RED
                    } else if fps < 50.0 {
                        LIGHT_RED
                    } else {
                        TEXT_COLOR
                    };

                    let mode = match conf.interactive_mode {
                        InteractiveMode::None => "automatic",
                        InteractiveMode::LoopModification => "manual loops",
                        InteractiveMode::SegmentationModification => "manual seg",
                    };
                    let job_status = job_state.request.map_or_else(
                        || "idle".to_string(),
                        |request| format!("{request}  {}", timer_animation(&time)),
                    );

                    let entries = [
                        (format!("fps {fps:>3.0}"), fps_color),
                        usage(
                            "scpu",
                            &SystemInformationDiagnosticsPlugin::SYSTEM_CPU_USAGE,
                        ),
                        usage(
                            "smem",
                            &SystemInformationDiagnosticsPlugin::SYSTEM_MEM_USAGE,
                        ),
                        usage(
                            "pcpu",
                            &SystemInformationDiagnosticsPlugin::PROCESS_CPU_USAGE,
                        ),
                        usage(
                            "pmem",
                            &SystemInformationDiagnosticsPlugin::PROCESS_MEM_USAGE,
                        ),
                        (mode.to_string(), TEXT_COLOR),
                        (job_status, TEXT_COLOR),
                    ];

                    let mut stats = text::LayoutJob::default();
                    for (entry, color) in entries {
                        stats.append("  |  ", 0.0, text_format(9.0, TEXT_COLOR));
                        stats.append(&entry, 0.0, text_format(STAT_SIZE, color));
                    }
                    ui.label(stats);
                });

                ui.with_layout(Layout::right_to_left(Align::TOP), |ui| {
                    ui.add_space(30.);
                    ui.label(sized_text("DualCube by snoep", 9.0, TEXT_COLOR));
                });
            });

            ui.add_space(5.);
        });

    Ok(())
}
